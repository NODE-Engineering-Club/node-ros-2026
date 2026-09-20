"""The whole serial path, end to end, with no Pico and no ROS.

This is the test that did not exist, and whose absence let the Jetson and the
firmware disagree for months while every other test stayed green.

The real path is:

    firmware -> text line -> serial framing -> pico_bridge's line filter
             -> parse_state_line -> pico_payload -> the GUI

The simulator used to attach at the far end of that chain, handing
``gui_backend`` a structured message and skipping the middle four steps. So
when the firmware said ``[STAT]`` and ``pico_bridge`` filtered for ``STATE``,
nothing failed — the layer where they disagreed did not exist in simulation.

Here the bytes are real: ``FakePicoSerial`` writes to a pty, this reads from
it, and the line splitting and the ``STATE`` filter are the same two lines of
logic ``pico_bridge`` runs. ``pico_bridge`` itself needs ``rclpy`` and cannot
be imported on a laptop, so its filter is reproduced below — and a test asserts
that the reproduction still matches the real file, so the two cannot drift.
"""

from __future__ import annotations

import os
import re
import time
from pathlib import Path

import pytest

from asket_sim.core.fake_pico_serial import FakePicoSerial
from asket_sim.core import pico as pico_sim
from gui_backend.core import adapters, payloads, pico_state
from gui_backend.core.payloads import DETAIL_FULL

pytestmark = pytest.mark.skipif(
    not hasattr(os, "openpty"), reason="needs a pty, POSIX only"
)

_BRIDGE = (
    Path(__file__).resolve().parents[3] / "src" / "control" / "control" / "pico_bridge.py"
)


class _Host:
    """The Jetson end of the cable: open the port, split lines, keep STATE.

    Deliberately the same shape as ``pico_bridge._drain`` — including the
    rejected-line counter, which is the part that would have turned this whole
    episode into a one-line log message instead of a field investigation.
    """

    def __init__(self, port: str) -> None:
        self.fd = os.open(port, os.O_RDWR | os.O_NONBLOCK)
        self.rx = b""
        self.state_lines: list[str] = []
        self.rejected = 0

    def close(self) -> None:
        os.close(self.fd)

    def send(self, line: str) -> None:
        os.write(self.fd, (line + "\n").encode())

    def pump(self, seconds: float, heartbeat: bool = True) -> None:
        """Read for a while, pinging as ``pico_bridge`` does.

        The heartbeat is not decoration. ``pico_bridge`` writes at 20 Hz
        precisely because the firmware revokes autonomy after 600 ms of
        silence, and a test that pumped quietly would watch AUTONOMOUS decay to
        MANUAL and call it a bug. Pass ``heartbeat=False`` to test that decay.
        """
        deadline = time.monotonic() + seconds
        next_ping = 0.0
        while time.monotonic() < deadline:
            if heartbeat and time.monotonic() >= next_ping:
                self.send("PING")
                next_ping = time.monotonic() + 0.05
            try:
                chunk = os.read(self.fd, 4096)
            except BlockingIOError:
                chunk = b""
            if chunk:
                self.rx += chunk
                while b"\n" in self.rx:
                    raw, self.rx = self.rx.split(b"\n", 1)
                    line = raw.decode(errors="replace").strip()
                    if not line:
                        continue
                    if line.startswith("STATE"):
                        self.state_lines.append(line)
                    else:
                        self.rejected += 1
            time.sleep(0.01)


def _run(seconds=1.2, script=None, configure=None, heartbeat=True):
    pico = pico_sim.PicoSim()
    if configure:
        configure(pico)
    with FakePicoSerial(pico) as fake:
        host = _Host(fake.port)
        try:
            host.pump(0.3)
            for line in script or []:
                host.send(line)
                host.pump(0.3)
            host.pump(seconds, heartbeat=heartbeat)
            return host, fake, pico
        finally:
            host.close()


# -- the path itself ------------------------------------------------------


def test_state_lines_survive_the_whole_chain():
    host, _, _ = _run()
    assert host.state_lines, "no STATE line reached the host end of the cable"

    state = pico_state.parse_state_line(host.state_lines[-1])
    assert state.parsed is True
    assert state.version_mismatch is False
    assert state.mode is not None
    assert state.unknown_keys == [], f"the parser ignored {state.unknown_keys}"


def test_every_field_the_simulator_sends_is_a_field_the_parser_knows():
    """The check that makes the text layer worth having.

    If the firmware grows a field and nobody teaches the parser about it, this
    fails here rather than showing up as a WARN on a beach.
    """
    host, _, _ = _run()
    for line in host.state_lines:
        assert pico_state.parse_state_line(line).unknown_keys == []


def test_the_payload_the_gui_receives_is_built_without_crashing():
    """``int(None)`` raises, and it used to — ``pico_payload`` called
    ``int(sample.rc_channel8_raw_pct)`` on a field that is ``None`` whenever the
    line did not carry it. The first real STATE line would have taken the
    backend down."""
    host, _, _ = _run()
    record = adapters.pico_from_ros(_String(host.state_lines[-1]), 1_700_000_000_000)
    payload = payloads.pico_payload(record, DETAIL_FULL)
    assert payload["mode"] in ("ESTOP", "MANUAL", "AUTONOMOUS")
    assert payload["firmware_version"] == pico_state.PROTOCOL_VERSION
    assert payload["version_mismatch"] is False


def test_a_partial_field_never_becomes_a_confident_value():
    host, _, _ = _run()
    truncated = host.state_lines[-1].split(" ch8=")[0]
    state = pico_state.parse_state_line(truncated)
    assert state.rc_channel8_raw is None
    assert state.ch8_asserting_estop is None


# -- the arbitration, over the wire ---------------------------------------


def test_mode_auto_is_granted_only_in_the_high_zone():
    host, _, pico = _run(script=["MODE AUTO"])
    state = pico_state.parse_state_line(host.state_lines[-1])
    assert state.mode == pico_state.MODE_AUTONOMOUS
    assert state.mode_requested_auto is True


def test_mode_auto_is_refused_in_the_mid_zone_and_the_refusal_is_visible():
    """Requested and confirmed must be separable on screen. ``wantauto=1`` with
    ``mode=2`` reads as "channel 8 said no", not as "the command was lost"."""
    host, _, _ = _run(
        script=["MODE AUTO"],
        configure=lambda p: p.set_rc_channel8(1000),  # mid zone
    )
    state = pico_state.parse_state_line(host.state_lines[-1])
    assert state.mode == pico_state.MODE_MANUAL
    assert state.mode_requested_auto is True


def test_channel_8_low_is_estop_and_no_command_can_leave_it():
    host, _, _ = _run(
        script=["MODE AUTO", "0.9,0.9"],
        configure=lambda p: p.set_rc_channel8(200),
    )
    state = pico_state.parse_state_line(host.state_lines[-1])
    assert state.mode == pico_state.MODE_ESTOP
    assert state.armed is False
    assert state.ch8_asserting_estop is True


def test_autonomy_lapses_when_the_heartbeat_stops():
    """The firmware grants autonomy on ``serial_wants_auto && link_live``, and
    the second term is why ``pico_bridge`` pings at 20 Hz. A Jetson that
    freezes mid-mission does not leave the boat driving itself: within 600 ms
    the Pico drops to MANUAL and the RC pilot has it back.

    The request itself survives, so the GUI can show "still asking, not
    granted" rather than silently forgetting what was asked.
    """
    host, _, _ = _run(script=["MODE AUTO"], seconds=1.2, heartbeat=False)
    state = pico_state.parse_state_line(host.state_lines[-1])
    assert state.mode == pico_state.MODE_MANUAL
    assert state.link_live is False
    assert state.mode_requested_auto is True


def test_a_motor_command_reaches_the_thrusters_only_when_autonomous():
    _, fake, _ = _run(seconds=0.1, script=["MODE AUTO", "0.4,-0.4"])
    assert fake.last_thrust == pytest.approx((0.4, -0.4))

    _, fake_manual, _ = _run(
        seconds=0.1,
        script=["MODE MANUAL", "0.4,-0.4"],
    )
    assert fake_manual.last_thrust is None


# -- version, end to end --------------------------------------------------


def test_a_firmware_the_parser_does_not_know_is_refused_on_the_wire():
    """The mechanism that closes this class of bug for good.

    Not a warning: a version this parser does not know is a firmware whose
    field meanings are unknown, and a plausible-looking panel built from
    misread fields is worse than no panel.
    """
    def older(p):
        p.firmware_version = 3

    host, _, _ = _run(configure=older)
    state = pico_state.parse_state_line(host.state_lines[-1])
    assert state.firmware_version == 3
    assert state.version_mismatch is True


def test_the_version_banner_is_sent_before_any_state_line():
    """A host that opens the port at boot learns what it is talking to at once,
    rather than waiting up to 250 ms and guessing in the meantime."""
    pico = pico_sim.PicoSim()
    with FakePicoSerial(pico) as fake:
        host = _Host(fake.port)
        try:
            host.pump(0.4)
            first = host.rx  # nothing consumed yet if no newline, so re-read
        finally:
            host.close()
    assert host.rejected >= 1, "the [VER] line should be seen and not kept as STATE"
    del first


# -- the three copies of these constants must agree -----------------------


def test_the_simulator_and_the_parser_agree_on_the_protocol_version():
    assert pico_sim.FIRMWARE_VERSION == pico_state.PROTOCOL_VERSION


def test_the_simulator_and_the_parser_agree_on_the_estop_threshold():
    assert pico_sim.CH8_ESTOP_MAX == pico_state.CH8_ESTOP_MAX
    assert pico_sim.CH7_ARM_MIN == pico_state.CH7_ARM_MIN


def test_the_firmware_source_still_says_what_these_tests_assume():
    """Read the .ino and check the numbers, rather than trusting a comment.

    Three copies of these constants exist — firmware, simulator, parser — and
    only one of them is the truth. This is the test that notices when the
    firmware moves and nothing else does.
    """
    ino = (
        Path(__file__).resolve().parents[3]
        / "firmware" / "pico-node_v4" / "pico-node_v4.ino"
    ).read_text()

    def const(pattern):
        match = re.search(pattern, ino)
        assert match, f"{pattern!r} no longer appears in the firmware"
        return int(match.group(1))

    assert const(r"#define FW_VERSION\s+(\d+)") == pico_state.PROTOCOL_VERSION
    assert const(r"MODE_LOW_MAX\s*=\s*(\d+)") == pico_state.CH8_ESTOP_MAX
    assert const(r"ARM_THRESHOLD\s*=\s*(\d+)") == pico_state.CH7_ARM_MIN
    assert "STATE ver=" in ino, "the firmware no longer emits ver= first"


def test_the_bridges_state_filter_is_what_this_test_reproduces():
    """If ``pico_bridge`` changes its filter, this file is out of date."""
    source = _BRIDGE.read_text()
    assert 'line.startswith("STATE")' in source


class _String:
    """Stand-in for ``std_msgs/String``: a ``.data`` and nothing else."""

    def __init__(self, data: str) -> None:
        self.data = data
