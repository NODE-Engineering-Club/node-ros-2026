"""The mode arbitration decision table.

This is the test that stands in for hardware. The firmware change it mirrors
cannot be run here — it is flashed on a Pico — so every RC x request x
configuration combination is enumerated explicitly rather than sampled, and the
expected effective mode is written out by hand from the rule rather than
computed by re-implementing the function under test.

The rule, once more:

    The RC transmitter is sovereign. The effective mode is the more restrictive
    of (RC channel 8, active software request). Software can only take authority
    away, never add it.
"""

import itertools

import pytest

from asket_common.mode_arbitration import (
    ARM_BLOCKED_CONTRADICTORY,
    ARM_BLOCKED_LATCHED,
    ARM_BLOCKED_RC_LOW,
    ARM_BLOCKED_UNKNOWN,
    MODE_AUTONOMOUS,
    MODE_ESTOP,
    MODE_MANUAL,
    MODE_RANK,
    REASON_RC_CLAMP,
    REASON_UNKNOWN_MODE,
    REASON_UPWARD_DISABLED,
    SOFTWARE_UPWARD_REQUESTS_ALLOWED,
    arbitrate,
    arming_block_reason,
    more_restrictive,
    requestable_modes,
)

MODES = (MODE_ESTOP, MODE_MANUAL, MODE_AUTONOMOUS)

# (rc, request, allow_upward) -> (effective, accepted, reason)
#
# Written out longhand on purpose. Twenty-four rows is not too many to read, and
# a table generated from the same logic it is checking proves nothing.
TABLE = {
    # --- downward-only (SOFTWARE_UPWARD_REQUESTS_ALLOWED = 0) --------------
    (MODE_ESTOP, None, False): (MODE_ESTOP, True, ""),
    (MODE_ESTOP, MODE_ESTOP, False): (MODE_ESTOP, True, ""),
    (MODE_ESTOP, MODE_MANUAL, False): (MODE_ESTOP, False, REASON_RC_CLAMP),
    (MODE_ESTOP, MODE_AUTONOMOUS, False): (MODE_ESTOP, False, REASON_UPWARD_DISABLED),
    (MODE_MANUAL, None, False): (MODE_MANUAL, True, ""),
    (MODE_MANUAL, MODE_ESTOP, False): (MODE_ESTOP, True, ""),
    (MODE_MANUAL, MODE_MANUAL, False): (MODE_MANUAL, True, ""),
    (MODE_MANUAL, MODE_AUTONOMOUS, False): (MODE_MANUAL, False, REASON_UPWARD_DISABLED),
    (MODE_AUTONOMOUS, None, False): (MODE_AUTONOMOUS, True, ""),
    (MODE_AUTONOMOUS, MODE_ESTOP, False): (MODE_ESTOP, True, ""),
    (MODE_AUTONOMOUS, MODE_MANUAL, False): (MODE_MANUAL, True, ""),
    (MODE_AUTONOMOUS, MODE_AUTONOMOUS, False): (MODE_AUTONOMOUS, False, REASON_UPWARD_DISABLED),
    # --- upward requests enabled (the default build) -----------------------
    (MODE_ESTOP, None, True): (MODE_ESTOP, True, ""),
    (MODE_ESTOP, MODE_ESTOP, True): (MODE_ESTOP, True, ""),
    (MODE_ESTOP, MODE_MANUAL, True): (MODE_ESTOP, False, REASON_RC_CLAMP),
    (MODE_ESTOP, MODE_AUTONOMOUS, True): (MODE_ESTOP, False, REASON_RC_CLAMP),
    (MODE_MANUAL, None, True): (MODE_MANUAL, True, ""),
    (MODE_MANUAL, MODE_ESTOP, True): (MODE_ESTOP, True, ""),
    (MODE_MANUAL, MODE_MANUAL, True): (MODE_MANUAL, True, ""),
    (MODE_MANUAL, MODE_AUTONOMOUS, True): (MODE_MANUAL, False, REASON_RC_CLAMP),
    (MODE_AUTONOMOUS, None, True): (MODE_AUTONOMOUS, True, ""),
    (MODE_AUTONOMOUS, MODE_ESTOP, True): (MODE_ESTOP, True, ""),
    (MODE_AUTONOMOUS, MODE_MANUAL, True): (MODE_MANUAL, True, ""),
    (MODE_AUTONOMOUS, MODE_AUTONOMOUS, True): (MODE_AUTONOMOUS, True, ""),
}


def test_table_covers_every_combination():
    """No combination may be left untested by accident."""
    expected = set(itertools.product(MODES, (None,) + MODES, (False, True)))
    assert set(TABLE) == expected
    assert len(TABLE) == 24


@pytest.mark.parametrize("key", sorted(TABLE, key=str))
def test_decision_table(key):
    rc, request, allow_upward = key
    effective, accepted, reason = TABLE[key]

    result = arbitrate(rc, request, allow_upward=allow_upward)

    assert result.effective == effective
    assert result.accepted is accepted
    assert result.reason == reason
    if not accepted:
        assert result.detail, "a rejection must say why — the GUI shows this"


# -- the properties that matter more than any single row --------------------


@pytest.mark.parametrize("rc,requested", list(itertools.product(MODES, MODES)))
@pytest.mark.parametrize("allow_upward", (False, True))
def test_software_never_makes_the_vessel_more_permissive(rc, requested, allow_upward):
    """The whole point. No input may produce an effective mode above RC."""
    result = arbitrate(rc, requested, allow_upward=allow_upward)
    assert MODE_RANK[result.effective] <= MODE_RANK[rc]


@pytest.mark.parametrize("rc", MODES)
@pytest.mark.parametrize("allow_upward", (False, True))
def test_estop_is_always_accepted(rc, allow_upward):
    """'Cut propulsion' must work from any state, in any build."""
    result = arbitrate(rc, MODE_ESTOP, allow_upward=allow_upward)
    assert result.accepted is True
    assert result.effective == MODE_ESTOP


@pytest.mark.parametrize("rc", MODES)
def test_no_request_means_rc_alone_decides(rc):
    """An expired or absent request hands authority straight back to RC."""
    assert arbitrate(rc, None).effective == rc


@pytest.mark.parametrize("allow_upward", (False, True))
def test_autonomous_is_only_requestable_when_upward_is_enabled(allow_upward):
    result = arbitrate(MODE_AUTONOMOUS, MODE_AUTONOMOUS, allow_upward=allow_upward)
    assert result.accepted is allow_upward


def test_rc_dropping_below_a_held_request_wins():
    """A MANUAL request accepted under AUTONOMOUS must not survive the operator
    moving channel 8 to ESTOP."""
    assert arbitrate(MODE_AUTONOMOUS, MODE_MANUAL).effective == MODE_MANUAL
    assert arbitrate(MODE_ESTOP, MODE_MANUAL).effective == MODE_ESTOP


def test_rc_rising_does_not_release_a_held_clamp():
    """The operator moving channel 8 up must not silently clear a software
    clamp — the request is still held and still applies."""
    result = arbitrate(MODE_AUTONOMOUS, MODE_MANUAL)
    assert result.effective == MODE_MANUAL
    assert result.accepted is True


def test_clamped_by_rc_is_reported_when_upward_is_enabled():
    """With upward enabled, a request RC will not honour is refused outright
    rather than quietly downgraded."""
    result = arbitrate(MODE_MANUAL, MODE_AUTONOMOUS, allow_upward=True)
    assert result.accepted is False
    assert result.reason == REASON_RC_CLAMP
    assert "MANUAL" in result.detail


# -- garbage in ------------------------------------------------------------


def test_unreadable_rc_mode_falls_back_to_estop():
    """An RC mode we cannot read is not a licence to pick a permissive one."""
    result = arbitrate("WOBBLE", None)
    assert result.effective == MODE_ESTOP
    assert result.accepted is False
    assert result.reason == REASON_UNKNOWN_MODE


def test_unknown_request_is_rejected_without_changing_the_mode():
    result = arbitrate(MODE_MANUAL, "TURBO")
    assert result.effective == MODE_MANUAL
    assert result.accepted is False
    assert result.reason == REASON_UNKNOWN_MODE


# -- ordering and the GUI's button list ------------------------------------


def test_ordering_is_estop_manual_autonomous():
    assert MODE_RANK[MODE_ESTOP] < MODE_RANK[MODE_MANUAL] < MODE_RANK[MODE_AUTONOMOUS]


def test_more_restrictive_is_symmetric():
    for a, b in itertools.product(MODES, MODES):
        assert more_restrictive(a, b) == more_restrictive(b, a)


def test_requestable_modes_follows_the_constant():
    assert requestable_modes(allow_upward=False) == [MODE_ESTOP, MODE_MANUAL]
    assert requestable_modes(allow_upward=True) == [MODE_ESTOP, MODE_MANUAL, MODE_AUTONOMOUS]


def test_default_build_lets_the_gui_request_every_mode():
    """The GUI is the primary way this boat is driven, missions included.

    Changing this should be a deliberate act that breaks this test and makes
    somebody think — in either direction.
    """
    assert SOFTWARE_UPWARD_REQUESTS_ALLOWED is True
    assert requestable_modes() == [MODE_ESTOP, MODE_MANUAL, MODE_AUTONOMOUS]


def test_the_constant_does_not_let_software_overrule_rc():
    """Worth stating separately now that the permissive build is the default.

    Allowing upward requests widens what software may *ask for*. It does not
    widen what channel 8 will allow, and the table above still refuses every
    request that would be less restrictive than the switch.
    """
    assert arbitrate(MODE_MANUAL, MODE_AUTONOMOUS, allow_upward=True).accepted is False
    assert arbitrate(MODE_ESTOP, MODE_MANUAL, allow_upward=True).accepted is False


# -- arming is a separate authority ----------------------------------------


def test_nothing_in_arbitration_speaks_to_arming():
    """Every result is a mode. Whether the propellers may turn is channel 7, and
    no combination of inputs here can raise it."""
    result = arbitrate(MODE_AUTONOMOUS, MODE_AUTONOMOUS)
    assert result.effective == MODE_AUTONOMOUS
    assert not hasattr(result, "armed")


def test_a_disarmed_vessel_in_autonomous_names_the_switch():
    """The state most likely to be misread: request accepted, mode confirmed,
    nothing moves."""
    assert arming_block_reason(armed=False, rc_arm_high=False) == ARM_BLOCKED_RC_LOW
    assert "channel 7" in ARM_BLOCKED_RC_LOW


def test_an_armed_vessel_reports_nothing_blocking():
    assert arming_block_reason(armed=True, rc_arm_high=True) is None
    # The vessel is the authority on its own arming. If it says armed, that
    # settles it, whatever the other inputs look like.
    assert arming_block_reason(armed=True, rc_arm_high=False) is None


def test_unknown_arming_makes_no_claim():
    """Not knowing is not evidence of a block. Inventing one would put a red
    line under a healthy vessel."""
    assert arming_block_reason(armed=None) is None
    assert arming_block_reason(armed=None, rc_arm_high=False) is None


def test_a_latch_is_named_as_a_latch():
    reason = arming_block_reason(armed=False, rc_arm_high=True, estop_latched=True)
    assert reason == ARM_BLOCKED_LATCHED
    assert "arm switch" in reason


def test_a_disarmed_vessel_with_no_identifiable_cause_still_says_so():
    """An incomplete diagnosis is not a reason to stay silent about a vessel
    that cannot move."""
    assert arming_block_reason(armed=False) == ARM_BLOCKED_UNKNOWN


def test_a_contradiction_is_reported_as_a_contradiction():
    """Channel 7 up but the vessel disarmed, with no latch. Something is wrong
    and guessing which side would be worse than saying they disagree."""
    reason = arming_block_reason(armed=False, rc_arm_high=True, estop_latched=False)
    assert reason == ARM_BLOCKED_CONTRADICTORY
    assert "disagree" in reason


def test_every_block_reason_says_propulsion_cannot_start():
    """Whatever the cause, the operator needs the same fact first."""
    for reason in (
        ARM_BLOCKED_RC_LOW,
        ARM_BLOCKED_LATCHED,
        ARM_BLOCKED_UNKNOWN,
        ARM_BLOCKED_CONTRADICTORY,
    ):
        assert "propulsion cannot start" in reason
