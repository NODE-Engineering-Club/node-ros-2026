"""A fake Pico on a real serial port.

This is the piece that makes the *text layer* honest.

``PicoSim`` on its own hands ``gui_backend`` a structured ``PicoStatus``
message. That was the hole: the real path is firmware → text line → serial
framing → ``pico_bridge``'s line filter → ``parse_state_line`` → GUI, and the
simulated path skipped the middle four. So when the firmware said ``[STAT]``
and the Jetson filtered for ``STATE``, nothing failed — because the layer where
the two disagreed did not exist in simulation. Months of a dead uplink, and a
green test suite the whole time.

This closes it the same way ``fake_sonar_server`` closes it for the sonar: by
putting the real bytes on a real device. ``pico_bridge`` opens a pty with
pyserial and cannot tell the difference, so its line splitting, its ``STATE``
filter, its reject counter and the parser downstream are all genuinely
exercised — in ``sim:=true``, on a laptop, with no boat.

It speaks exactly what ``firmware/pico-node_v4/pico-node_v4.ino`` speaks:

* ``[VER] pico-node 4`` once on open, then a ``STATE key=value`` line at 4 Hz;
* accepts ``L,R`` motor commands, ``PING``, ``MODE AUTO`` and ``MODE MANUAL``;
* answers the mode verbs with ``[ACK] ...``, and anything else with
  ``Invalid cmd: ...``;
* refuses to grant autonomy unless channel 8 is in the high zone **and** the
  heartbeat is fresh, which is the firmware's arbitration and the one rule the
  GUI must never be able to talk its way around.

Run standalone, no ROS required::

    python3 -m asket_sim.core.fake_pico_serial

It prints the port it created; point ``pico_bridge`` at that path, or just
``cat`` it to watch the lines go by.
"""

from __future__ import annotations

import os
import pty
import termios
import threading
import tty
import time
from dataclasses import dataclass, field

from .pico import (
    CH7_ARM_MIN,
    CH8_ESTOP_MAX,
    CH8_MANUAL_MAX,
    MODE_AUTONOMOUS,
    MODE_ESTOP,
    MODE_MANUAL,
    PicoSim,
)

#: The firmware emits one status line every 250 ms, from loop().
STATUS_PERIOD_S = 0.25

#: HEARTBEAT_TIMEOUT_MS in the firmware. Past this with nothing from the host,
#: the Pico stops considering the link live and refuses to grant autonomy.
HEARTBEAT_TIMEOUT_S = 0.6

#: SERIAL_FAILSAFE_TIMEOUT_MS. Past this without a motor command while
#: autonomous, thrust goes to neutral — without opening the relay.
SERIAL_FAILSAFE_TIMEOUT_S = 0.5


@dataclass
class FakePicoSerial:
    """A pty that behaves like a Pico running pico-node_v4."""

    pico: PicoSim
    #: Wall-clock seconds per simulated second. >1 runs faster than real time.
    time_scale: float = 1.0
    step_dt: float = 0.02

    #: Filesystem path of the slave side. Open this with pyserial.
    port: str = field(default="", init=False)

    #: Last thrust the host commanded, in normalised units. Exposed so a test
    #: can assert that a command sent down the wire actually arrived.
    last_thrust: tuple[float, float] | None = field(default=None, init=False)
    #: Lines the host sent that the firmware did not understand.
    invalid_lines: list[str] = field(default_factory=list, init=False)

    _master: int = field(default=-1, init=False)
    _slave: int = field(default=-1, init=False)
    _stop: threading.Event = field(default_factory=threading.Event, init=False)
    _thread: threading.Thread | None = field(default=None, init=False)
    _rx: bytes = field(default=b"", init=False)
    _last_host_line: float = field(default=0.0, init=False)
    _last_motor_cmd: float = field(default=0.0, init=False)

    # -- lifecycle --------------------------------------------------------

    def start(self) -> str:
        self._master, self._slave = pty.openpty()
        # Raw mode, or the pty behaves like a terminal rather than a wire: in
        # its default canonical mode the line discipline echoes everything
        # written back to the writer, so the Pico reads its own STATE lines,
        # answers "Invalid cmd:", reads that back, and the buffer fills with
        # ever-longer nonsense within a second. It also translates newlines,
        # which would quietly corrupt the framing this exists to test.
        tty.setraw(self._master)
        tty.setraw(self._slave)
        for fd in (self._master, self._slave):
            attrs = termios.tcgetattr(fd)
            attrs[3] &= ~termios.ECHO  # lflag
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
        os.set_blocking(self._master, False)
        self.port = os.ttyname(self._slave)
        now = time.monotonic()
        self._last_host_line = now
        self._last_motor_cmd = now
        # Machine-readable identity first, exactly as setup() does it, so a
        # host that opens the port at boot learns the version immediately
        # rather than waiting for the first STATE line.
        self._write(f"[VER] pico-node {self.pico.firmware_version}")
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self.port

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        for fd in (self._master, self._slave):
            try:
                os.close(fd)
            except OSError:
                pass

    def __enter__(self) -> FakePicoSerial:
        self.start()
        return self

    def __exit__(self, *exc) -> None:
        self.stop()

    # -- the loop ---------------------------------------------------------

    def _run(self) -> None:
        next_status = time.monotonic()
        while not self._stop.is_set():
            self._drain_host()
            self.pico.step(self.step_dt)
            self._apply_arbitration()

            now = time.monotonic()
            if now >= next_status:
                self._write(self.pico.state_line())
                next_status = now + STATUS_PERIOD_S / max(self.time_scale, 1e-6)
            time.sleep(self.step_dt / max(self.time_scale, 1e-6))

    def _apply_arbitration(self) -> None:
        """The firmware's ``update_state()``, in Python.

        Written out rather than approximated, because the GUI's whole safety
        story rests on it: channel 8 decides the zone, and the host's request
        is consulted only inside the high zone and only while its heartbeat is
        fresh. A simulator that granted autonomy on request alone would let the
        GUI pass a test the boat would fail.
        """
        p = self.pico
        now = time.monotonic()
        p.link_live = (now - self._last_host_line) < HEARTBEAT_TIMEOUT_S

        ch8 = p.rc_channel8_raw
        if ch8 < CH8_ESTOP_MAX:
            mode = MODE_ESTOP
        elif ch8 < CH8_MANUAL_MAX:
            mode = MODE_MANUAL
        else:
            mode = MODE_AUTONOMOUS if (p.serial_wants_auto and p.link_live) else MODE_MANUAL

        armed = p.rc_channel7_raw > CH7_ARM_MIN and mode != MODE_ESTOP
        if not armed or mode == MODE_ESTOP:
            p.estop_latched = mode == MODE_ESTOP

        p.mode = mode
        p.armed = armed
        p.relay_states = [armed] * len(p.relay_states)
        p.esc_status = [0 if armed else 1] * len(p.esc_status)

        # Serial failsafe: neutral, relay still closed.
        if (
            p.mode == MODE_AUTONOMOUS
            and p.armed
            and (now - self._last_motor_cmd) > SERIAL_FAILSAFE_TIMEOUT_S
        ):
            self.last_thrust = (0.0, 0.0)

    # -- the wire ---------------------------------------------------------

    def _write(self, line: str) -> None:
        try:
            os.write(self._master, (line + "\n").encode())
        except OSError:
            pass  # host closed its end; the Pico would not notice either

    def _drain_host(self) -> None:
        try:
            chunk = os.read(self._master, 4096)
        except (BlockingIOError, OSError):
            return
        if not chunk:
            return
        self._rx += chunk
        while b"\n" in self._rx:
            raw, self._rx = self._rx.split(b"\n", 1)
            line = raw.decode(errors="replace").strip()
            if line:
                self._on_host_line(line)

    def _on_host_line(self, line: str) -> None:
        # Any valid line counts as a heartbeat, which is what makes PING work.
        self._last_host_line = time.monotonic()
        up = line.upper()

        if up == "PING":
            return
        if up == "MODE AUTO":
            self.pico.serial_wants_auto = True
            self._write("[ACK] MODE AUTO requested")
            return
        if up == "MODE MANUAL":
            self.pico.serial_wants_auto = False
            self._write("[ACK] MODE MANUAL requested")
            return

        thrust = _parse_motor_command(line)
        if thrust is None:
            self.invalid_lines.append(line)
            self._write(f"Invalid cmd: {line}")
            return

        self._last_motor_cmd = time.monotonic()
        # Applied only where the firmware would apply it. Accepting the command
        # as a heartbeat but not acting on it is the firmware's behaviour, and
        # the difference is the whole of "requested is not confirmed".
        if self.pico.armed and self.pico.mode == MODE_AUTONOMOUS:
            self.last_thrust = thrust


def _parse_motor_command(line: str) -> tuple[float, float] | None:
    """``process_motor_command()`` from the firmware, in Python.

    Same tolerance: brackets, commas and semicolons are separators, one value
    applies to both motors, and values within +-1.5 are normalised while
    anything larger is raw microseconds.
    """
    cleaned = line
    for ch in "[];,":
        cleaned = cleaned.replace(ch, " ")
    parts = cleaned.split()
    try:
        values = [float(p) for p in parts[:2]]
    except ValueError:
        return None
    if not values:
        return None

    if all(abs(v) <= 1.5 for v in values):
        left = max(-1.0, min(1.0, values[0]))
        right = max(-1.0, min(1.0, values[1])) if len(values) == 2 else left
        return (left, right)

    def us_to_norm(us: float) -> float:
        return max(-1.0, min(1.0, (max(1000.0, min(2000.0, us)) - 1500.0) / 500.0))

    left = us_to_norm(values[0])
    right = us_to_norm(values[1]) if len(values) == 2 else left
    return (left, right)


def main() -> None:  # pragma: no cover - a convenience entry point
    import argparse

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seconds", type=float, default=0.0, help="0 = run until Ctrl-C")
    args = ap.parse_args()

    with FakePicoSerial(PicoSim()) as fake:
        print(f"fake Pico on {fake.port}")
        print(f"  cat {fake.port}")
        deadline = time.monotonic() + args.seconds if args.seconds else None
        try:
            while deadline is None or time.monotonic() < deadline:
                time.sleep(0.2)
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":  # pragma: no cover
    main()
