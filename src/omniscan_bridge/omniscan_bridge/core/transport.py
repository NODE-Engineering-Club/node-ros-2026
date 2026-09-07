"""Network transport for the Omniscan 3D.

The sonar sits on Ethernet and talks Ping Protocol on port 62312. Two rules
shape this module:

1. **Never block the ROS executor on a socket read.** A reader thread fills a
   bounded queue; the node drains it from a timer. If the sonar stops talking,
   the node keeps running and says so, rather than freezing the whole process.
2. **Reconnect without being asked.** Nobody is aboard the vessel. A cable that
   glitches at 200 m offshore must recover on its own.

The queue is bounded on purpose. If the consumer ever falls behind, the right
failure is to drop the oldest data and count it — a stale point cloud is worse
than a missing one, and unbounded growth on a Jetson mid-mission is worse than
both.
"""

from __future__ import annotations

import queue
import socket
import threading
import time
from dataclasses import dataclass

from .ping_protocol import DEFAULT_PORT

#: Blue Robotics device discovery: broadcast a request, devices answer with a
#: descriptive text blob.
DISCOVERY_PORT = 51200
DISCOVERY_REQUEST = b"DISCOVER"


@dataclass
class TransportStats:
    bytes_received: int = 0
    datagrams_received: int = 0
    #: Chunks dropped because the consumer fell behind. Never silent.
    chunks_dropped: int = 0
    connect_attempts: int = 0
    reconnects: int = 0
    last_rx_monotonic: float = 0.0


class SonarTransport:
    """Base class. Subclasses implement the socket specifics only."""

    def __init__(
        self,
        host: str,
        port: int = DEFAULT_PORT,
        queue_size: int = 256,
        reconnect_interval_s: float = 2.0,
        rx_timeout_s: float = 3.0,
    ) -> None:
        self.host = host
        self.port = port
        self.reconnect_interval_s = reconnect_interval_s
        self.rx_timeout_s = rx_timeout_s

        self.stats = TransportStats()
        self._queue: queue.Queue[bytes] = queue.Queue(maxsize=queue_size)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._sock: socket.socket | None = None
        self._lock = threading.Lock()

    # -- lifecycle --------------------------------------------------------

    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name=f"sonar-rx-{self.host}", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        with self._lock:
            sock, self._sock = self._sock, None
        if sock:
            try:
                sock.close()
            except OSError:
                pass
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

    @property
    def connected(self) -> bool:
        """Socket open *and* data arriving recently.

        An open socket that has gone quiet is not a working sonar, and reporting
        it as connected would be exactly the kind of comfortable lie this GUI
        must not tell.
        """
        if self._sock is None:
            return False
        if self.stats.last_rx_monotonic == 0.0:
            return False
        return (time.monotonic() - self.stats.last_rx_monotonic) < self.rx_timeout_s

    @property
    def seconds_since_data(self) -> float:
        if self.stats.last_rx_monotonic == 0.0:
            return float("inf")
        return time.monotonic() - self.stats.last_rx_monotonic

    # -- data -------------------------------------------------------------

    def read(self, max_chunks: int = 64) -> list[bytes]:
        """Drain what has arrived. Never blocks."""
        out: list[bytes] = []
        for _ in range(max_chunks):
            try:
                out.append(self._queue.get_nowait())
            except queue.Empty:
                break
        return out

    def _publish(self, data: bytes) -> None:
        self.stats.bytes_received += len(data)
        self.stats.datagrams_received += 1
        self.stats.last_rx_monotonic = time.monotonic()
        try:
            self._queue.put_nowait(data)
        except queue.Full:
            # Drop the oldest, keep the newest, and count it.
            try:
                self._queue.get_nowait()
                self._queue.put_nowait(data)
            except (queue.Empty, queue.Full):
                pass
            self.stats.chunks_dropped += 1

    def send(self, data: bytes) -> bool:
        """Send a command frame. Returns whether it went out."""
        with self._lock:
            sock = self._sock
        if sock is None:
            return False
        try:
            self._send_on(sock, data)
            return True
        except OSError:
            return False

    # -- to be implemented by subclasses ----------------------------------

    def _connect(self) -> socket.socket:
        raise NotImplementedError

    def _receive(self, sock: socket.socket) -> bytes | None:
        """Return data, ``None`` for "nothing this cycle", or ``b''`` for
        "the peer closed the connection". The three cases are distinct and
        collapsing any two of them causes a spurious reconnect loop."""
        raise NotImplementedError

    def _send_on(self, sock: socket.socket, data: bytes) -> None:
        raise NotImplementedError

    # -- reader thread ----------------------------------------------------

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                self.stats.connect_attempts += 1
                sock = self._connect()
            except OSError:
                self._sleep(self.reconnect_interval_s)
                continue

            with self._lock:
                if self._sock is not None:
                    self.stats.reconnects += 1
                self._sock = sock

            try:
                while not self._stop.is_set():
                    data = self._receive(sock)
                    if data:
                        self._publish(data)
                    elif data is not None:
                        break  # b'' means the peer closed
            except socket.timeout:
                continue
            except OSError:
                pass
            finally:
                with self._lock:
                    if self._sock is sock:
                        self._sock = None
                try:
                    sock.close()
                except OSError:
                    pass
                self._sleep(self.reconnect_interval_s)

    def _sleep(self, seconds: float) -> None:
        self._stop.wait(seconds)


class UdpTransport(SonarTransport):
    """Ping Protocol over UDP — the Omniscan's normal mode.

    Each datagram carries whole frames, but the parser is byte-stream oriented
    anyway, so a device that splits a frame across datagrams still works.
    """

    def __init__(self, *args, keepalive_interval_s: float = 1.0, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.keepalive_interval_s = keepalive_interval_s
        self._last_keepalive = 0.0

    def _connect(self) -> socket.socket:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(0.5)
        # UDP has no handshake; the device learns our address from the first
        # packet we send, so an empty datagram is how we announce ourselves.
        sock.sendto(b"", (self.host, self.port))
        self._last_keepalive = time.monotonic()
        return sock

    def _receive(self, sock: socket.socket) -> bytes | None:
        now = time.monotonic()
        if now - self._last_keepalive > self.keepalive_interval_s:
            self._last_keepalive = now
            try:
                sock.sendto(b"", (self.host, self.port))
            except OSError:
                pass
        try:
            data, _ = sock.recvfrom(65535)
            return data
        except socket.timeout:
            return None  # nothing this cycle; UDP has no connection to lose
        except ConnectionRefusedError:
            # ICMP port unreachable: the device is not there yet.
            raise OSError("connection refused")

    def _send_on(self, sock: socket.socket, data: bytes) -> None:
        sock.sendto(data, (self.host, self.port))

    def _run(self) -> None:
        # UDP has no connection to lose, so an empty read is normal rather than
        # a disconnect. Override the base loop to reflect that.
        while not self._stop.is_set():
            try:
                self.stats.connect_attempts += 1
                sock = self._connect()
            except OSError:
                self._sleep(self.reconnect_interval_s)
                continue

            with self._lock:
                if self._sock is not None:
                    self.stats.reconnects += 1
                self._sock = sock

            try:
                while not self._stop.is_set():
                    try:
                        data = self._receive(sock)
                    except OSError:
                        break
                    if data:
                        self._publish(data)
            finally:
                with self._lock:
                    if self._sock is sock:
                        self._sock = None
                try:
                    sock.close()
                except OSError:
                    pass
                if not self._stop.is_set():
                    self._sleep(self.reconnect_interval_s)


class TcpTransport(SonarTransport):
    """Ping Protocol over TCP, for devices or bridges configured that way."""

    def _connect(self) -> socket.socket:
        sock = socket.create_connection((self.host, self.port), timeout=2.0)
        sock.settimeout(0.5)
        return sock

    def _receive(self, sock: socket.socket) -> bytes | None:
        try:
            return sock.recv(65535)   # b'' here genuinely means "peer closed"
        except socket.timeout:
            return None

    def _send_on(self, sock: socket.socket, data: bytes) -> None:
        sock.sendall(data)


def discover(
    timeout_s: float = 1.0,
    broadcast_address: str = "255.255.255.255",
    port: int = DISCOVERY_PORT,
) -> list[dict[str, str]]:
    """Blue Robotics device discovery.

    Broadcasts a request and collects the descriptive replies. Used to fill in
    the sonar's address when it is not configured, and by the pre-flight check
    to say *"the sonar is on the network but not answering"* rather than the far
    less useful *"sonar: ERROR"*.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.settimeout(0.2)
    found: list[dict[str, str]] = []
    try:
        sock.sendto(DISCOVERY_REQUEST, (broadcast_address, port))
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                data, addr = sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            found.append(_parse_discovery_reply(data, addr[0]))
    finally:
        sock.close()
    return found


def _parse_discovery_reply(data: bytes, source_ip: str) -> dict[str, str]:
    """Replies are ``Key: value`` lines. Unknown keys are kept, not discarded."""
    info: dict[str, str] = {"source_ip": source_ip}
    for line in data.decode("ascii", "replace").splitlines():
        key, sep, value = line.partition(":")
        if sep:
            info[key.strip()] = value.strip()
    return info
