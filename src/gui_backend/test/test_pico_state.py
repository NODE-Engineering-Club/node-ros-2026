"""Parsing the Pico's STATE line.

The format is **PROVISIONAL**: nobody has captured a real line from firmware v3
(docs/open_questions.md Q7). So these tests deliberately do *not* assert that
any particular spelling is correct — that would bake a guess in and make it look
verified.

What they pin is the behaviour that must hold whatever the format turns out to
be: never raise, never invent, absent stays absent, and an unreadable line is
reported as unreadable rather than quietly becoming a mode.

When somebody captures a real line, paste it in as a new test and set
FORMAT_VERIFIED.
"""

import pytest
from gui_backend.core import pico_state
from gui_backend.core.pico_state import (
    MODE_AUTONOMOUS,
    MODE_MANUAL,
    parse_state_line,
)


def test_the_format_is_still_marked_unverified():
    """If this fails, either somebody captured a real line — in which case
    delete this test — or somebody flipped the flag without doing so, which is
    the thing it exists to catch."""
    assert pico_state.FORMAT_VERIFIED is False


@pytest.mark.parametrize("line", [
    "",
    "   ",
    "not a state line",
    "STATEMENT mode=AUTO",
    "\x00\xff binary noise",
    "STATE " + "x" * 10000,
])
def test_nothing_a_serial_link_can_produce_makes_it_raise(line):
    """This runs on every message from a physical serial link. A line half-eaten
    by a cable makes the vessel panel go blank; an exception takes the GUI down
    with it."""
    state = parse_state_line(line)
    assert isinstance(state, pico_state.PicoState)


def test_an_unreadable_line_is_reported_as_unreadable():
    """The dangerous alternative is defaulting to a mode. A panel that says
    MANUAL because the parser gave up is claiming a confirmed vessel state that
    nobody confirmed."""
    state = parse_state_line("STATE 0x41 0x00 0x1f")
    assert state.parsed is False
    assert state.mode is None
    # And the raw text survives, because whoever fixes the parser needs it.
    assert state.raw == "STATE 0x41 0x00 0x1f"


def test_fields_the_line_does_not_carry_stay_absent():
    """Absent is not zero and not False. Reporting a healthy RC link as lost
    because a text field was missing is exactly the lie this GUI avoids."""
    state = parse_state_line("STATE mode=MANUAL")
    assert state.mode == MODE_MANUAL
    assert state.armed is None
    assert state.rc_link_ok is None
    assert state.rc_channel8_raw_pct is None


def test_the_pico_word_for_autonomous_is_auto():
    """pico_bridge speaks AUTO; the GUI says AUTONOMOUS. The translation lives
    here so nothing downstream has to know both."""
    assert parse_state_line("STATE mode=AUTO").mode == MODE_AUTONOMOUS
    assert parse_state_line("STATE AUTO").mode == MODE_AUTONOMOUS


def test_a_value_that_will_not_parse_leaves_the_field_absent():
    state = parse_state_line("STATE armed=maybe ch8=lots vbat=n/a")
    assert state.armed is None
    assert state.rc_channel8_raw_pct is None
    assert state.battery_voltage is None


def test_an_unknown_mode_word_is_not_silently_dropped():
    """If the firmware ever reports a third mode, the GUI must not decide it is
    one of the two it knows."""
    state = parse_state_line("STATE mode=FAILSAFE")
    assert state.mode is None
    assert any("FAILSAFE" in k for k in state.unknown_keys)


def test_unrecognised_keys_are_kept_rather_than_ignored():
    """The key this parser does not know is usually the one that matters."""
    state = parse_state_line("STATE mode=MANUAL faults=3 batt_mv=25900")
    assert set(state.unknown_keys) == {"faults", "batt_mv"}


def test_a_plausible_line_reads_end_to_end():
    """A guess at the shape, and marked as one. Its value is proving the pieces
    fit together, not that this is what firmware v3 emits."""
    state = parse_state_line("STATE mode=AUTO armed=1 rc=1 ch8=100 vbat=25.9 ibat=4.2")
    assert state.parsed
    assert state.mode == MODE_AUTONOMOUS
    assert state.armed is True
    assert state.rc_link_ok is True
    assert state.rc_channel8_raw_pct == 100
    assert state.battery_voltage == pytest.approx(25.9)
    assert state.unknown_keys == []
