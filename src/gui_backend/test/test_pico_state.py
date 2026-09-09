"""The Pico status line parser, against the real firmware format.

The format is transcribed from ``pico-node_v3.ino`` (the flashed v3 firmware),
not guessed. So unlike the provisional version this replaces, these tests assert
*spelling* as well as behaviour: exact field names, exact value vocabularies, and
the exact line the firmware prints.

The behavioural guarantees still hold and still have their own tests, because
they are what keeps a parser bug from becoming a safety problem:

* never raise, on anything a serial link can produce
* never invent a value — absent stays absent
* an unreadable line is reported as unreadable, not quietly turned into a mode
"""

import pytest

from gui_backend.core import pico_state
from gui_backend.core.pico_state import (
    ARM_THRESHOLD,
    EVENT_ARMED,
    EVENT_COMMAND_ACK,
    EVENT_DISARMED,
    EVENT_ESTOP_TRIGGERED,
    EVENT_INVALID_COMMAND,
    EVENT_MODE_ESTOP,
    MODE_AUTONOMOUS,
    MODE_ESTOP,
    MODE_MANUAL,
    PicoState,
    is_status_line,
    parse_event_line,
    parse_state_line,
    rc_mode_from_sbus,
    sbus_to_pct,
)

#: Captured from the firmware source: the exact line `loop()` prints every
#: 250 ms. Disarmed, ESTOP on the switch, sticks centred.
REAL_LINE = (
    "[STAT] Mode:2 Armed:Y Relay:ON "
    "Thr(Ch3):991 Yaw(Ch4):991 Arm(Ch7):172 Mode(Ch8):172"
)


def test_format_is_no_longer_provisional():
    """The format came out of the firmware source. The runtime check that live
    lines still parse lives in system_test, not in this flag."""
    assert pico_state.FORMAT_VERIFIED is True


def test_the_real_line_parses_completely():
    state = parse_state_line(REAL_LINE)

    assert state.parsed is True
    assert state.mode == MODE_MANUAL
    assert state.mode_number == 2
    assert state.armed is True
    assert state.relay_on is True
    assert state.ch_throttle == 991
    assert state.ch_yaw == 991
    assert state.ch_arm == 172
    assert state.ch_mode == 172
    assert state.unknown_keys == []


# -- the trap --------------------------------------------------------------


def test_mode_and_mode_ch8_are_not_conflated():
    """`Mode` and `Mode(Ch8)` are different fields carrying different units.

    Splitting the line naively on ':' and taking a trailing `Mode` would put a
    raw SBUS count (172-1811) into the mode enum, or the enum into the channel.
    Here Mode:1 is ESTOP while Ch8 reads 1811 (AUTONOMOUS on the switch) — the
    two disagree on purpose, so a parser that mixed them up cannot pass.
    """
    line = (
        "[STAT] Mode:1 Armed:N Relay:OFF "
        "Thr(Ch3):991 Yaw(Ch4):991 Arm(Ch7):172 Mode(Ch8):1811"
    )
    state = parse_state_line(line)

    assert state.mode == MODE_ESTOP, "firmware mode must come from `Mode:`"
    assert state.mode_number == 1
    assert state.ch_mode == 1811, "channel must come from `Mode(Ch8):`"
    assert state.rc_mode == MODE_AUTONOMOUS, "and the switch says AUTONOMOUS"
    assert state.mode != state.rc_mode


def test_the_disagreement_is_reported_as_a_software_clamp():
    """Firmware more restrictive than the switch means software is holding the
    vessel down. The operator must be able to tell that from a fault."""
    clamped = parse_state_line(
        "[STAT] Mode:2 Armed:Y Relay:ON "
        "Thr(Ch3):991 Yaw(Ch4):991 Arm(Ch7):1811 Mode(Ch8):1811"
    )
    assert clamped.mode == MODE_MANUAL
    assert clamped.rc_mode == MODE_AUTONOMOUS
    assert clamped.software_clamp_active is True

    agreeing = parse_state_line(
        "[STAT] Mode:3 Armed:Y Relay:ON "
        "Thr(Ch3):991 Yaw(Ch4):991 Arm(Ch7):1811 Mode(Ch8):1811"
    )
    assert agreeing.software_clamp_active is False


# -- field vocabularies ----------------------------------------------------


@pytest.mark.parametrize(
    "number,expected",
    [(1, MODE_ESTOP), (2, MODE_MANUAL), (3, MODE_AUTONOMOUS)],
)
def test_every_firmware_mode_number(number, expected):
    line = f"[STAT] Mode:{number} Armed:N Relay:OFF Mode(Ch8):991"
    assert parse_state_line(line).mode == expected


@pytest.mark.parametrize("text,expected", [("Y", True), ("N", False)])
def test_armed_vocabulary(text, expected):
    assert parse_state_line(f"[STAT] Armed:{text}").armed is expected


@pytest.mark.parametrize("text,expected", [("ON", True), ("OFF", False)])
def test_relay_vocabulary(text, expected):
    assert parse_state_line(f"[STAT] Relay:{text}").relay_on is expected


def test_an_unknown_mode_number_is_not_guessed_at():
    """A number outside 1-3 must not be rounded to the nearest plausible mode."""
    state = parse_state_line("[STAT] Mode:7 Armed:N")
    assert state.mode is None
    assert state.mode_number == 7
    assert "Mode:7" in state.unknown_keys


# -- derived RC values -----------------------------------------------------


@pytest.mark.parametrize(
    "raw,expected",
    [
        (172, MODE_ESTOP),
        (699, MODE_ESTOP),
        (700, MODE_MANUAL),
        (991, MODE_MANUAL),
        (1399, MODE_MANUAL),
        (1400, MODE_AUTONOMOUS),
        (1811, MODE_AUTONOMOUS),
    ],
)
def test_rc_mode_thresholds_match_the_firmware(raw, expected):
    assert rc_mode_from_sbus(raw) == expected


def test_rc_mode_is_absent_when_the_channel_is():
    assert rc_mode_from_sbus(None) is None
    assert parse_state_line("[STAT] Mode:2").rc_mode is None


@pytest.mark.parametrize("raw,expected", [(172, 0), (991, 50), (1811, 100)])
def test_channel_percentage(raw, expected):
    assert sbus_to_pct(raw) == expected


def test_channel_percentage_clamps_rather_than_going_out_of_range():
    assert sbus_to_pct(0) == 0
    assert sbus_to_pct(4000) == 100
    assert sbus_to_pct(None) is None


@pytest.mark.parametrize(
    "raw,expected", [(172, False), (1000, False), (1001, True), (1811, True)]
)
def test_arm_switch_threshold(raw, expected):
    assert parse_state_line(f"[STAT] Arm(Ch7):{raw}").rc_arm_high is expected


# -- behaviour under garbage ----------------------------------------------


@pytest.mark.parametrize(
    "line",
    [
        "",
        "   ",
        "[STAT]",
        "[STAT] ",
        "\x00\xff garbage",
        "[STAT] Mode: Armed:",
        "[STAT] Mode:abc Armed:maybe Relay:perhaps Thr(Ch3):x",
        "[STAT] Mode:2 Mode:3 Mode:1",
        "=== Asket EC Pico Controller (rebuilt) ===",
        ">>> MODE: E-STOP",
        "[STAT] " + "A:1 " * 500,
        "[STAT] Mode:99999999999999999999",
    ],
)
def test_never_raises(line):
    """Anything a serial link can produce, including a half-written line."""
    state = parse_state_line(line)
    assert isinstance(state, PicoState)


def test_a_truncated_line_yields_what_it_carried_and_no_more():
    """Serial lines get cut. What survived is still usable; what did not is
    absent rather than defaulted."""
    state = parse_state_line("[STAT] Mode:3 Armed:Y Relay:ON Thr(Ch3):9")
    assert state.mode == MODE_AUTONOMOUS
    assert state.armed is True
    assert state.ch_throttle == 9
    assert state.ch_yaw is None
    assert state.ch_mode is None
    assert state.rc_mode is None


def test_an_unrecognised_line_is_reported_not_interpreted():
    state = parse_state_line("Invalid cmd: 0.5 0.5")
    assert state.parsed is False
    assert state.mode is None
    assert state.raw == "Invalid cmd: 0.5 0.5"


def test_unparseable_values_leave_fields_absent():
    state = parse_state_line("[STAT] Armed:maybe Relay:perhaps Thr(Ch3):lots")
    assert state.armed is None
    assert state.relay_on is None
    assert state.ch_throttle is None
    assert state.parsed is False


def test_unknown_keys_are_reported_rather_than_dropped():
    """An unknown key is usually the field that matters — a firmware that grew
    a battery reading should surface it, not swallow it."""
    state = parse_state_line("[STAT] Mode:2 Vbat:25.9 Faults:3")
    assert state.mode == MODE_MANUAL
    assert "Vbat" in state.unknown_keys
    assert "Faults" in state.unknown_keys


def test_non_string_input_is_survived():
    for value in (None, 42, b"[STAT] Mode:2", object()):
        assert parse_state_line(value).parsed is False


def test_fields_the_firmware_does_not_report_stay_absent():
    """The firmware has no battery sense. Absent must not become zero — '0.0 V'
    would read as a flat battery and ground a healthy vessel."""
    state = parse_state_line(REAL_LINE)
    assert state.battery_voltage is None
    assert state.battery_current is None
    assert state.rc_link_ok is None


# -- prefixes --------------------------------------------------------------


def test_both_prefixes_are_accepted():
    """`[STAT]` is what v3 emits. `STATE` is accepted too — a prefix mismatch is
    precisely what stopped /pico/status publishing anything at all."""
    assert is_status_line("[STAT] Mode:2")
    assert is_status_line("STATE Mode:2")
    assert parse_state_line("STATE Mode:2 Armed:Y").mode == MODE_MANUAL


@pytest.mark.parametrize(
    "line", ["", ">>> MODE: E-STOP", "Invalid cmd: x", "=== Asket EC ==="]
)
def test_non_status_lines_are_not_status_lines(line):
    assert is_status_line(line) is False


# -- event lines -----------------------------------------------------------


@pytest.mark.parametrize(
    "line,kind,detail",
    [
        (">>> MODE: E-STOP", EVENT_MODE_ESTOP, "E-STOP"),
        (">>> ARMED: MANUAL", EVENT_ARMED, "MANUAL"),
        (">>> ARMED: AUTONOMOUS", EVENT_ARMED, "AUTONOMOUS"),
        (">>> DISARMED (manual)", EVENT_DISARMED, "manual"),
        (">>> DISARMED (auto)", EVENT_DISARMED, "auto"),
        ("[E-STOP] SBUS timeout", EVENT_ESTOP_TRIGGERED, "SBUS timeout"),
        ("[E-STOP] power feedback LOW", EVENT_ESTOP_TRIGGERED, "power feedback LOW"),
        ("Invalid cmd: WOBBLE", EVENT_INVALID_COMMAND, "WOBBLE"),
        ("[ACK] MODE ESTOP accepted", EVENT_COMMAND_ACK, "MODE ESTOP accepted"),
    ],
)
def test_every_firmware_event_line(line, kind, detail):
    event = parse_event_line(line)
    assert event is not None
    assert event.kind == kind
    assert event.detail == detail
    assert event.raw == line


def test_status_lines_are_not_events():
    """They arrive four times a second and would drown the event log."""
    assert parse_event_line(REAL_LINE) is None


@pytest.mark.parametrize("line", ["", "   ", "something else", None, 42])
def test_non_events_return_none(line):
    assert parse_event_line(line) is None


def test_arm_threshold_is_the_firmware_value():
    assert ARM_THRESHOLD == 1000


# -- the ESC arming window -------------------------------------------------


def test_relay_edge_is_timed_across_lines():
    """The status line says ON or OFF, never when. The edge has to be watched."""
    from gui_backend.core.pico_state import RelayEdgeTracker

    t = RelayEdgeTracker()
    assert t.observe(False, 1000) is None          # open: no window
    assert t.observe(True, 2000) == 0              # the edge
    assert t.observe(True, 2500) == 500
    assert t.observe(True, 4500) == 2500           # past the 2 s window
    assert t.observe(False, 5000) is None          # opened again


def test_a_relay_already_closed_when_we_arrive_starts_the_clock_then():
    """Connecting mid-flight must not claim the relay closed at the epoch, nor
    that it closed long ago. The first sighting is the best we honestly have."""
    from gui_backend.core.pico_state import RelayEdgeTracker

    t = RelayEdgeTracker()
    assert t.observe(True, 9000) == 0
    assert t.observe(True, 9750) == 750


def test_a_line_without_the_relay_field_does_not_reset_the_window():
    """A truncated line is missing information, not evidence the relay opened."""
    from gui_backend.core.pico_state import RelayEdgeTracker

    t = RelayEdgeTracker()
    t.observe(True, 1000)
    assert t.observe(None, 1500) == 500
    assert t.observe(True, 1800) == 800


def test_reclosing_restarts_the_window():
    """Every OFF->ON edge is a fresh 2 s of ESCs booting."""
    from gui_backend.core.pico_state import RelayEdgeTracker

    t = RelayEdgeTracker()
    t.observe(True, 1000)
    t.observe(False, 3000)
    assert t.observe(True, 4000) == 0
