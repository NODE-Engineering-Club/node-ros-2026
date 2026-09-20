"""Parsing the Pico's STATE line.

The format here is transcribed from ``firmware/pico-node_v4/pico-node_v4.ino``,
field by field, so these tests can assert real spellings — unlike the previous
round, where the format was a guess and the tests could only pin behaviour.

What they still pin is the behaviour that must hold regardless: never raise,
never invent, absent stays absent, an unreadable line is reported as unreadable
rather than quietly becoming a mode, and a version this parser does not know is
refused rather than half-read.

``FORMAT_VERIFIED`` stays False until somebody captures a line off real
hardware. Reading the firmware source is not the same as reading its output.
"""

import pytest
from gui_backend.core import pico_state
from gui_backend.core.pico_state import (
    MODE_AUTONOMOUS,
    MODE_ESTOP,
    MODE_MANUAL,
    PROTOCOL_VERSION,
    parse_state_line,
)

#: Copied from the firmware's own loop(), in field order. Every test that needs
#: a well-formed line starts from this one.
V4_LINE = (
    "STATE ver=4 mode=3 armed=1 relay=1 wantauto=1 link=1 estoplatch=0 "
    "thr=991 yaw=991 ch7=1811 ch8=1811 "
    "sbusok=2400 sbusbad=0 sbusfs=0 sbuslost=0"
)


def _line(**overrides):
    """V4_LINE with some fields replaced, keeping the real field order."""
    parts = V4_LINE.split()
    out = [parts[0]]
    for token in parts[1:]:
        key = token.split("=")[0]
        out.append(f"{key}={overrides[key]}" if key in overrides else token)
    return " ".join(out)


def test_the_format_is_still_marked_unverified():
    """If this fails, either somebody captured a real line — in which case
    delete this test — or somebody flipped the flag without doing so, which is
    the thing it exists to catch."""
    assert pico_state.FORMAT_VERIFIED is False


# -- robustness -----------------------------------------------------------


@pytest.mark.parametrize("line", [
    "",
    "   ",
    "not a state line",
    "STATEMENT mode=AUTO",
    "\x00\xff binary noise",
    "STATE " + "x" * 10000,
    "STATE ver=4 mode= armed=",
    "STATE=1",
    "STATE ver=4 mode=3 armed=1 " + "=" * 500,
])
def test_nothing_a_serial_link_can_produce_makes_it_raise(line):
    state = parse_state_line(line)
    assert isinstance(state.parsed, bool)


@pytest.mark.parametrize("value", [None, 42, b"STATE ver=4 mode=3 armed=1", object()])
def test_a_non_string_is_not_a_line(value):
    assert parse_state_line(value).parsed is False


def test_a_v3_line_is_refused_rather_than_half_read():
    """The bug this whole round exists to close.

    ``pico-node_v3`` emitted ``[STAT] Mode:3 Armed:Y ...``. It must come back
    unparsed, so the pre-flight fails and says so, rather than yielding a
    half-populated panel.
    """
    state = parse_state_line("[STAT] Mode:3 Armed:Y Relay:ON Thr(Ch3):991 Mode(Ch8):1811")
    assert state.parsed is False
    assert state.mode is None
    assert state.raw  # the text is kept, so the operator can see what arrived


# -- the fields -----------------------------------------------------------


def test_a_real_v4_line_reads_every_field():
    state = parse_state_line(V4_LINE)
    assert state.parsed is True
    assert state.unknown_keys == []
    assert state.firmware_version == PROTOCOL_VERSION
    assert state.version_mismatch is False
    assert state.mode == MODE_AUTONOMOUS
    assert state.armed is True
    assert state.relay_closed is True
    assert state.mode_requested_auto is True
    assert state.link_live is True
    assert state.estop_latched is False
    assert state.rc_throttle_raw == 991
    assert state.rc_yaw_raw == 991
    assert state.rc_channel7_raw == 1811
    assert state.rc_channel8_raw == 1811
    assert state.sbus_frames_ok == 2400
    assert state.sbus_frames_bad == 0
    assert state.sbus_failsafe is False
    assert state.sbus_frame_lost is False


@pytest.mark.parametrize("raw,expected", [
    ("1", MODE_ESTOP),
    ("2", MODE_MANUAL),
    ("3", MODE_AUTONOMOUS),
])
def test_mode_is_an_integer_on_the_wire(raw, expected):
    """The firmware sends ``enum OperationMode { ESTOP=1, MANUAL=2, AUTO=3 }``.

    The previous parser looked for a *word*, found a number, and left mode as
    None — producing exactly the symptom of the prefix bug, which made the two
    indistinguishable in the field.
    """
    assert parse_state_line(_line(mode=raw)).mode == expected


@pytest.mark.parametrize("word,expected", [
    ("MANUAL", MODE_MANUAL), ("AUTO", MODE_AUTONOMOUS),
    ("autonomous", MODE_AUTONOMOUS), ("ESTOP", MODE_ESTOP),
])
def test_mode_words_are_accepted_too(word, expected):
    """Nothing sends these, but a human reproducing a bug by hand does."""
    assert parse_state_line(_line(mode=word)).mode == expected


def test_an_unknown_mode_number_does_not_become_a_mode():
    state = parse_state_line(_line(mode="9"))
    assert state.mode is None
    assert state.parsed is False
    assert "mode=9" in state.unknown_keys


# -- channel 8, on the scale it is actually sent on -----------------------


def test_channel_8_is_a_raw_sbus_count_not_a_percentage():
    assert parse_state_line(_line(ch8="1811")).rc_channel8_raw == 1811


@pytest.mark.parametrize("ch8,asserting", [
    ("1811", False),   # top of travel: autonomy permitted
    ("1000", False),   # middle: manual forced, but not ESTOP
    ("700", False),    # MODE_LOW_MAX itself: the firmware compares with `<`
    ("699", True),     # one count below, and the zone changes
    ("200", True),     # bottom of travel: ESTOP
    ("172", True),     # the protocol minimum
])
def test_the_estop_threshold_fires_on_the_sbus_scale(ch8, asserting):
    """The defect this replaces: the field was named ``_raw_pct`` and alarmed
    below 25, so a raw count of 200 — the bottom of the travel, the one
    position the alarm exists for — read as "200%" and never fired."""
    assert parse_state_line(_line(ch8=ch8)).ch8_asserting_estop is asserting


def test_channel_8_absent_is_unknown_not_safe():
    line = " ".join(t for t in V4_LINE.split() if not t.startswith("ch8="))
    assert parse_state_line(line).ch8_asserting_estop is None


# -- version --------------------------------------------------------------


def test_a_future_firmware_is_a_mismatch_not_a_guess():
    state = parse_state_line(_line(ver="5"))
    assert state.firmware_version == 5
    assert state.version_mismatch is True


def test_a_line_with_no_version_is_not_reported_as_a_mismatch():
    """Absent is not wrong. It is a different failure — an older firmware, or a
    truncated line — and the check layer says so differently."""
    line = " ".join(t for t in V4_LINE.split() if not t.startswith("ver="))
    state = parse_state_line(line)
    assert state.firmware_version is None
    assert state.version_mismatch is False


def test_version_is_the_first_field_so_it_survives_truncation():
    """A line cut short by a serial glitch still carries its version, which is
    what lets the check layer say 'wrong firmware' instead of 'unreadable'."""
    truncated = V4_LINE[:20]
    assert truncated.startswith("STATE ver=4 ")
    assert parse_state_line(truncated).firmware_version == PROTOCOL_VERSION


# -- parsed means parsed --------------------------------------------------


def test_one_understood_field_is_not_enough():
    """``armed=1`` alone used to mark a line parsed. A line whose mode was
    misread then reported success, and the operator saw a confident panel."""
    state = parse_state_line("STATE ver=4 armed=1")
    assert state.armed is True
    assert state.parsed is False


def test_mode_and_armed_together_are_enough():
    assert parse_state_line("STATE ver=4 mode=2 armed=0").parsed is True


def test_state_alone_carries_nothing():
    assert parse_state_line("STATE").parsed is False


# -- absent stays absent --------------------------------------------------


def test_unknown_keys_are_reported_not_dropped():
    state = parse_state_line(V4_LINE + " newfield=7")
    assert "newfield" in state.unknown_keys
    assert state.parsed is True  # a new field does not invalidate the rest


def test_an_unreadable_value_leaves_the_field_absent():
    state = parse_state_line(_line(sbusok="banana"))
    assert state.sbus_frames_ok is None
    assert "sbus_frames_ok=banana" in state.unknown_keys


def test_rc_link_follows_the_receivers_own_failsafe_bit():
    assert parse_state_line(_line(sbusfs="0")).rc_link_ok is True
    assert parse_state_line(_line(sbusfs="1")).rc_link_ok is False


def test_rc_link_is_unknown_when_the_bit_is_not_sent():
    line = " ".join(t for t in V4_LINE.split() if not t.startswith("sbusfs="))
    assert parse_state_line(line).rc_link_ok is None
