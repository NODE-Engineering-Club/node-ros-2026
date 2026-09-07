"""A fake Omniscan 3D on the network.

This is the piece that makes ``sim:=true`` honest. Rather than having
``omniscan_bridge`` read from a simulator object in sim mode and from a socket
in real mode — two code paths, one of which never gets tested — the simulator
puts real Ping Protocol frames on a real UDP socket. ``omniscan_bridge`` is then
byte-for-byte identical in both modes, and the socket handling, the resync
logic and the packet-loss accounting are all genuinely exercised.

It speaks the parts of the protocol the bridge uses:

* streams ``OS3D_POINT_SET``, ``END_PING_INFO`` and ``ATTITUDE_REPORT``;
* accepts ``OS3D_SET_PING_PARAMETERS`` and applies range / gain / ping rate;
* accepts ``SET_NTP_URL`` and records it, so the bridge can verify it was sent;
* answers the Blue Robotics discovery request with a plausible reply.

Run standalone, no ROS required::

    python3 -m asket_sim.core.fake_sonar_server --port 62312
"""

from __future__ import annotations

import argparse
import socket
import struct
import threading
import time
from dataclasses import dataclass, field

from omniscan_bridge.core import ping_protocol as pp

from .raw_stream import encode_sim_ping
from .world import SimWorld, WorldConfig

#: Blue Robotics discovery listens here and replies with a text blob.
DISCOVERY_PORT = 51200
DISCOVERY_REQUEST = b"DISCOVER"


@dataclass
class FakeSonarServer:
    world: SimWorld
    host: str = "127.0.0.1"
    port: int = pp.DEFAULT_PORT
    #: Wall-clock seconds per simulated second. >1 runs the world faster.
    time_scale: float = 1.0
    step_dt: float = 0.05
    device_id: int = 1
    serve_discovery: bool = True

    ntp_url: str | None = field(default=None, init=False)
    clients: set[tuple[str, int]] = field(default_factory=set, init=False)
    _stop: threading.Event = field(default_factory=threading.Event, init=False)
    _threads: list[threading.Thread] = field(default_factory=list, init=False)
    _sock: socket.socket | None = field(default=None, init=False)
    _lock: threading.Lock = field(default_factory=threading.Lock, init=False)
    frames_sent: int = field(default=0, init=False)

    # -- lifecycle --------------------------------------------------------

    def start(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.host, self.port))
        self._sock.settimeout(0.2)

        self._threads = [
            threading.Thread(target=self._rx_loop, name="fake-sonar-rx", daemon=True),
            threading.Thread(target=self._tx_loop, name="fake-sonar-tx", daemon=True),
        ]
        if self.serve_discovery:
            self._threads.append(
                threading.Thread(
                    target=self._discovery_loop, name="fake-sonar-disc", daemon=True
                )
            )
        for t in self._threads:
            t.start()

    def stop(self) -> None:
        self._stop.set()
        for t in self._threads:
            t.join(timeout=1.0)
        if self._sock:
            self._sock.close()
            self._sock = None

    @property
    def bound_port(self) -> int:
        return self._sock.getsockname()[1] if self._sock else self.port

    # -- inbound ----------------------------------------------------------

    def _rx_loop(self) -> None:
        parser = FrameCollector()
        while not self._stop.is_set():
            try:
                data, addr = self._sock.recvfrom(4096)
            except (socket.timeout, OSError):
                continue
            with self._lock:
                self.clients.add(addr)
            for frame in parser.feed(data):
                self._handle(frame)

    def _handle(self, frame: pp.Frame) -> None:
        if frame.message_id == pp.MSG_OS3D_SET_PING_PARAMETERS:
            range_mm, gain, rate = struct.unpack_from("<IB3xf", frame.payload, 0)
            cfg = self.world.cfg.sonar
            cfg.range_setting_m = range_mm / 1000.0
            cfg.gain = gain
            cfg.ping_rate_hz = max(0.1, rate)
        elif frame.message_id == pp.MSG_SET_NTP_URL:
            self.ntp_url = frame.payload.rstrip(b"\x00").decode("ascii", "replace")

    # -- outbound ---------------------------------------------------------

    def _tx_loop(self) -> None:
        last = time.monotonic()
        pings = 0
        while not self._stop.is_set():
            now = time.monotonic()
            elapsed = (now - last) * self.time_scale
            last = now

            steps = max(1, int(elapsed / self.step_dt))
            for _ in range(min(steps, 40)):
                self.world.step(self.step_dt)

            ping = self.world.take_ping()
            if ping is not None:
                pings += 1
                valid = sum(1 for p in ping.points if p.pt_type != 0)
                self._send(encode_sim_ping(ping))
                self._send(
                    pp.encode_end_ping_info(
                        pp.EndPingInfo(
                            ping.ping_number,
                            valid,
                            1.0 / max(0.1, self.world.cfg.sonar.ping_rate_hz),
                        ),
                        src_device_id=self.device_id,
                    )
                )
                if pings % 2 == 0:
                    snap = self.world.snapshot()
                    self._send(
                        pp.encode_attitude_report(
                            pp.AttitudeReport(snap.sonar_pitch_deg, snap.sonar_roll_deg),
                            src_device_id=self.device_id,
                        )
                    )
            time.sleep(self.step_dt / max(0.01, self.time_scale))

    def _send(self, data: bytes) -> None:
        if not self._sock:
            return
        with self._lock:
            targets = list(self.clients)
        for addr in targets:
            try:
                self._sock.sendto(data, addr)
                self.frames_sent += 1
            except OSError:
                pass

    # -- discovery --------------------------------------------------------

    def _discovery_loop(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind((self.host, DISCOVERY_PORT))
        except OSError:
            return  # port busy: discovery is optional, streaming is not
        sock.settimeout(0.2)
        reply = (
            "DeviceName: Omniscan 3D (simulated)\r\n"
            "Manufacturer: Cerulean Sonar\r\n"
            f"DeviceID: {self.device_id}\r\n"
            "MACAddress: 00-00-00-00-00-01\r\n"
            f"IPAddress: {self.host}\r\n"
        ).encode("ascii")
        while not self._stop.is_set():
            try:
                _, addr = sock.recvfrom(1024)
            except (socket.timeout, OSError):
                continue
            try:
                sock.sendto(reply, addr)
            except OSError:
                pass
        sock.close()


class FrameCollector:
    """Minimal inbound framing for the fake device.

    The *bridge* has the robust parser; the device side only needs enough to
    understand our own outbound commands, so this stays deliberately small.
    """

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes) -> list[pp.Frame]:
        self._buf += data
        out: list[pp.Frame] = []
        while True:
            start = self._buf.find(pp.START_BYTES)
            if start < 0:
                self._buf.clear()
                return out
            if start:
                del self._buf[:start]
            if len(self._buf) < pp.HEADER_SIZE:
                return out
            _, length, msg_id, src, dst = pp._HEADER_FMT.unpack_from(self._buf, 0)
            total = pp.HEADER_SIZE + length + pp.CHECKSUM_SIZE
            if len(self._buf) < total:
                return out
            payload = bytes(self._buf[pp.HEADER_SIZE : pp.HEADER_SIZE + length])
            out.append(pp.Frame(msg_id, src, dst, payload))
            del self._buf[:total]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Serve a simulated Omniscan 3D.")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=pp.DEFAULT_PORT)
    ap.add_argument("--time-scale", type=float, default=1.0)
    ap.add_argument("--ping-rate", type=float, default=5.0)
    args = ap.parse_args(argv)

    cfg = WorldConfig()
    cfg.sonar.ping_rate_hz = args.ping_rate
    server = FakeSonarServer(
        SimWorld(cfg), host=args.host, port=args.port, time_scale=args.time_scale
    )
    server.start()
    print(f"simulated Omniscan 3D on {args.host}:{server.bound_port} — Ctrl-C to stop")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
