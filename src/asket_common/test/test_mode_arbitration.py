"""Behaviour of the Python arbitration mirror, and of the two explanations.

``test_firmware_arbitration_matches.py`` proves this module agrees with the
firmware. It does not prove the module says anything *useful* — two
implementations can agree and both be unreadable. This file covers the parts an
operator actually meets: why autonomy is not engaged, and why the propellers
will not turn.

These tests need no compiler and no firmware, so they run on a laptop.
"""

import pytest

from asket_common.mode_arbitration import (
    ARM_BLOCKED_CONTRADICTORY,
    ARM_BLOCKED_LATCHED,
    ARM_BLOCKED_RC_LOW,
    ARM_BLOCKED_UNKNOWN,
    ARM_THRESHOLD,
    MODE_AUTONOMOUS,
    MODE_ESTOP,
    MODE_MANUAL,
    REASON_LINK_STALE,
    REASON_NOT_REQUESTED,
    REASON_RC_ESTOP,
    REASON_RC_MANUAL,
    REASON_SBUS_FAILSAFE,
    arbitrate,
    arming_block_reason,
    autonomy_block_reason,
    reason_text,
)

ESTOP_ZONE = 400
MANUAL_ZONE = 991
AUTO_ZONE = 1811
ARM_UP = 1811
ARM_DOWN = 172


def _auto(**kw):
    """The normal mission case: switch up, armed, requested, link alive."""
    params = dict(serial_wants_auto=True, link_live=True)
    params.update(kw)
    return arbitrate(AUTO_ZONE, ARM_UP, **params)


# -- the three zones --------------------------------------------------------


def test_the_low_zone_is_estop_whatever_else_is_true():
    arb = arbitrate(ESTOP_ZONE, ARM_UP, serial_wants_auto=True, link_live=True)
    assert arb.mode == MODE_ESTOP
    assert arb.armed is False
    assert arb.autonomy_blocked_by == REASON_RC_ESTOP


def test_the_middle_zone_forces_manual_over_a_standing_request():
    arb = arbitrate(MANUAL_ZONE, ARM_UP, serial_wants_auto=True, link_live=True)
    assert arb.mode == MODE_MANUAL
    assert arb.autonomy_permitted is False
    assert arb.autonomy_blocked_by == REASON_RC_MANUAL


def test_the_top_zone_permits_autonomy_without_granting_it():
    """The distinction the whole v4 design rests on."""
    arb = arbitrate(AUTO_ZONE, ARM_UP)
    assert arb.autonomy_permitted is True
    assert arb.mode == MODE_MANUAL
    assert arb.autonomy_blocked_by == REASON_NOT_REQUESTED


def test_a_request_engages_autonomy_in_the_top_zone():
    arb = _auto()
    assert arb.mode == MODE_AUTONOMOUS
    assert arb.armed is True
    assert arb.autonomy_blocked_by == ""


def test_a_stale_heartbeat_withdraws_autonomy_and_says_so():
    """Distinct from "never asked". The operator needs to know the transmitter
    is still permitting it and the link is what failed."""
    arb = _auto(link_live=False)
    assert arb.mode == MODE_MANUAL
    assert arb.autonomy_permitted is True
    assert arb.autonomy_blocked_by == REASON_LINK_STALE


def test_a_receiver_failsafe_beats_every_zone():
    for ch8 in (ESTOP_ZONE, MANUAL_ZONE, AUTO_ZONE):
        arb = arbitrate(
            ch8, ARM_UP, serial_wants_auto=True, link_live=True, sbus_failsafe=True
        )
        assert arb.mode == MODE_ESTOP, ch8
        assert arb.armed is False
        assert arb.autonomy_permitted is False
        assert arb.autonomy_blocked_by == REASON_SBUS_FAILSAFE


# -- the boundaries ---------------------------------------------------------


@pytest.mark.parametrize(
    "ch8,expected",
    [(699, MODE_ESTOP), (700, MODE_MANUAL), (1399, MODE_MANUAL), (1400, MODE_AUTONOMOUS)],
)
def test_the_zone_boundaries_are_where_the_firmware_puts_them(ch8, expected):
    assert _auto().mode == MODE_AUTONOMOUS  # guard: the helper still means what it says
    arb = arbitrate(ch8, ARM_UP, serial_wants_auto=True, link_live=True)
    assert arb.mode == expected


@pytest.mark.parametrize("ch7,armed", [(ARM_THRESHOLD, False), (ARM_THRESHOLD + 1, True)])
def test_arming_is_strictly_above_the_threshold(ch7, armed):
    assert _auto(**{}).armed is True
    assert arbitrate(AUTO_ZONE, ch7, serial_wants_auto=True, link_live=True).armed is armed


# -- the latch --------------------------------------------------------------


def test_a_latched_estop_keeps_the_vessel_disarmed():
    arb = _auto(estop_latched=True)
    assert arb.mode == MODE_AUTONOMOUS
    assert arb.armed is False, "a latched e-stop must not arm"
    assert arb.estop_latched is True


def test_disarming_clears_the_latch():
    arb = arbitrate(AUTO_ZONE, ARM_DOWN, estop_latched=True)
    assert arb.estop_latched is False


def test_selecting_estop_clears_the_latch():
    arb = arbitrate(ESTOP_ZONE, ARM_UP, estop_latched=True)
    assert arb.estop_latched is False


# -- the explanations -------------------------------------------------------


def test_autonomy_block_reason_is_none_when_autonomous():
    assert autonomy_block_reason(_auto()) is None


def test_every_reason_code_has_wording():
    for ch8, kw in (
        (ESTOP_ZONE, {}),
        (MANUAL_ZONE, {}),
        (AUTO_ZONE, {}),
        (AUTO_ZONE, {"serial_wants_auto": True}),
        (AUTO_ZONE, {"sbus_failsafe": True}),
    ):
        arb = arbitrate(ch8, ARM_UP, **kw)
        text = autonomy_block_reason(arb)
        assert text, arb.autonomy_blocked_by
        assert not text.startswith("autonomy not engaged ("), (
            f"{arb.autonomy_blocked_by} has no wording, only a code"
        )


def test_an_unknown_reason_code_is_quoted_rather_than_dropped():
    assert "weather" in reason_text("weather")


# -- arming legibility ------------------------------------------------------
#
# The state this exists for: a mission request accepted, arbitrated and
# confirmed in the STATE line, and the boat moves nothing because the arm
# switch is down. An operator who cannot see why is left guessing whether the
# boat ignored them or the switch did.


def test_channel_seven_down_is_named_as_the_cause():
    assert arming_block_reason(False, rc_arm_high=False) == ARM_BLOCKED_RC_LOW
    assert "channel 7" in ARM_BLOCKED_RC_LOW


def test_a_latched_estop_is_named_as_the_cause():
    reason = arming_block_reason(False, rc_arm_high=True, estop_latched=True)
    assert reason == ARM_BLOCKED_LATCHED
    assert "arm switch" in reason, "the operator needs to be told the way out"


def test_a_contradiction_gets_its_own_wording():
    """Channel 7 up and the vessel disarmed is not the same problem as channel
    7 down, and flattening the two would send somebody to the wrong switch."""
    reason = arming_block_reason(False, rc_arm_high=True, estop_latched=False)
    assert reason == ARM_BLOCKED_CONTRADICTORY
    assert reason != ARM_BLOCKED_RC_LOW


def test_a_disarmed_vessel_says_so_even_when_the_cause_is_unknown():
    assert arming_block_reason(False) == ARM_BLOCKED_UNKNOWN


def test_an_armed_vessel_is_never_blocked():
    """The vessel is the authority on whether its own propellers may turn."""
    assert arming_block_reason(True) is None
    assert arming_block_reason(True, rc_arm_high=False, estop_latched=True) is None


def test_not_knowing_is_not_evidence_of_a_block():
    """A vessel that has not told us its arming state gets no red line. This is
    the case every panel starts in, before the first STATE line arrives."""
    assert arming_block_reason(None) is None
    assert arming_block_reason(None, rc_arm_high=False, estop_latched=True) is None
