"""Command lifecycle. These tests are safety rules 4 and 5 written as code."""

from gui_backend.core.commands import (
    CMD_SET_MODE,
    STATUS_CONFIRMED,
    STATUS_FAILED,
    STATUS_PENDING,
    CommandManager,
    mode_confirmed,
    ping_parameters_confirmed,
    propulsion_cut_confirmed,
)


def test_a_command_starts_pending_not_successful():
    """Nothing displayed may change because a command was sent."""
    mgr = CommandManager()
    cmd = mgr.issue(CMD_SET_MODE, {"mode": "AUTONOMOUS"}, 1000,
                    confirm=mode_confirmed("AUTONOMOUS"))
    assert cmd.status == STATUS_PENDING


def test_confirmation_comes_from_the_vessel_not_from_the_request():
    mgr = CommandManager()
    cmd = mgr.issue(CMD_SET_MODE, {"mode": "AUTONOMOUS"}, 1000,
                    confirm=mode_confirmed("AUTONOMOUS"))

    # The vessel still says MANUAL. The command stays pending.
    assert mgr.update({"pico": {"mode": "MANUAL"}}, 1500) == []
    assert cmd.status == STATUS_PENDING

    changed = mgr.update({"pico": {"mode": "AUTONOMOUS"}}, 2000)
    assert changed == [cmd]
    assert cmd.status == STATUS_CONFIRMED


def test_an_unconfirmed_command_fails_and_says_the_vessel_did_not_change():
    """The message matters. 'Timed out' invites a second attempt; 'the vessel
    has NOT changed state' tells the operator what is true."""
    mgr = CommandManager(timeout_s=3.0)
    cmd = mgr.issue(CMD_SET_MODE, {"mode": "AUTONOMOUS"}, 1000,
                    confirm=mode_confirmed("AUTONOMOUS"))
    mgr.update({"pico": {"mode": "MANUAL"}}, 1000 + 3500)
    assert cmd.status == STATUS_FAILED
    assert "has NOT changed state" in cmd.detail


def test_cutting_propulsion_needs_both_the_mode_and_the_latch():
    """ESTOP without the latch, or the latch without ESTOP, is a half-applied
    state and must not read as confirmed."""
    assert not propulsion_cut_confirmed({"pico": {"mode": "ESTOP", "estop_latched": False}})
    assert not propulsion_cut_confirmed({"pico": {"mode": "MANUAL", "estop_latched": True}})
    assert propulsion_cut_confirmed({"pico": {"mode": "ESTOP", "estop_latched": True}})


def test_a_rejected_command_fails_immediately():
    mgr = CommandManager()
    cmd = mgr.issue(CMD_SET_MODE, {"mode": "SIDEWAYS"}, 1000, confirm=mode_confirmed("SIDEWAYS"))
    mgr.fail(cmd, "unknown mode", 1000)
    assert cmd.status == STATUS_FAILED
    assert cmd not in mgr.pending


def test_commands_with_nothing_to_observe_settle_immediately():
    """A profile change has no vessel-side effect to wait for. That is an
    explicit decision here, not a default to optimism."""
    mgr = CommandManager()
    cmd = mgr.issue("set_profile", {"profile": "reduced"}, 1000)
    assert cmd.status == STATUS_CONFIRMED


def test_sonar_parameters_are_confirmed_by_the_sonar():
    check = ping_parameters_confirmed(25.0, 3, 10.0)
    assert not check({"sonar": {"range_setting_m": 30.0, "gain_setting": 4,
                                "commanded_ping_rate_hz": 5.0}})
    assert check({"sonar": {"range_setting_m": 25.0, "gain_setting": 3,
                            "commanded_ping_rate_hz": 10.0}})


def test_history_is_bounded():
    mgr = CommandManager()
    for i in range(200):
        mgr.issue("set_profile", {"n": i}, i)
    assert len(mgr.history) <= 50


# -- refusals ---------------------------------------------------------------
#
# A command the firmware refuses must say why. Before the vessel had a real
# arbitration path there was nothing to refuse a command *with*, so a clamped
# request looked exactly like a slow one: the button sat there and then reported
# that nothing had happened, which was true and useless.


def _ack(mode, reason, utc_ms, accepted=False):
    return {
        "subject": f"MODE {mode}",
        "mode": mode,
        "accepted": accepted,
        "reason": reason,
        "raw": f"[ACK] MODE {mode} {'accepted' if accepted else 'rejected ' + reason}",
        "utc_ms": utc_ms,
    }


def test_a_refused_mode_fails_immediately_with_the_reason():
    mgr = CommandManager()
    cmd = mgr.issue("set_mode", {"mode": "AUTONOMOUS"}, 1000,
                    confirm=mode_confirmed("AUTONOMOUS"))

    changed = mgr.update({"pico_command_ack": _ack("AUTONOMOUS", "rc_clamp", 1100)}, 1200)

    assert [c.id for c in changed] == [cmd.id]
    assert cmd.status == STATUS_FAILED
    assert "channel 8" in cmd.detail
    # Not the generic timeout message: that one teaches nothing.
    assert "no confirmation" not in cmd.detail


def test_an_upward_refusal_says_which_constant_is_in_the_way():
    mgr = CommandManager()
    cmd = mgr.issue("set_mode", {"mode": "AUTONOMOUS"}, 1000,
                    confirm=mode_confirmed("AUTONOMOUS"))
    mgr.update({"pico_command_ack": _ack("AUTONOMOUS", "upward_disabled", 1100)}, 1200)
    assert cmd.status == STATUS_FAILED
    assert "AUTONOMOUS" in cmd.detail


def test_an_ack_from_before_the_command_does_not_fail_it():
    """Otherwise the answer to the previous press fails the next one, before the
    firmware has even seen it."""
    mgr = CommandManager()
    cmd = mgr.issue("set_mode", {"mode": "MANUAL"}, 2000, confirm=mode_confirmed("MANUAL"))
    mgr.update({"pico_command_ack": _ack("MANUAL", "rc_clamp", 1000)}, 2100)
    assert cmd.status == STATUS_PENDING


def test_a_refusal_of_a_different_mode_does_not_fail_this_one():
    mgr = CommandManager()
    cmd = mgr.issue("set_mode", {"mode": "MANUAL"}, 1000, confirm=mode_confirmed("MANUAL"))
    mgr.update({"pico_command_ack": _ack("AUTONOMOUS", "upward_disabled", 1100)}, 1200)
    assert cmd.status == STATUS_PENDING


def test_an_accepted_ack_is_not_a_confirmation():
    """Safety rule 4. The firmware taking the request is not the vessel having
    changed state — only the status stream settles that."""
    mgr = CommandManager()
    cmd = mgr.issue("set_mode", {"mode": "MANUAL"}, 1000, confirm=mode_confirmed("MANUAL"))
    mgr.update({"pico_command_ack": _ack("MANUAL", "", 1100, accepted=True)}, 1200)
    assert cmd.status == STATUS_PENDING

    mgr.update({"pico": {"mode": "MANUAL"}}, 1300)
    assert cmd.status == STATUS_CONFIRMED


def test_an_unknown_reason_code_is_quoted_rather_than_swallowed():
    mgr = CommandManager()
    cmd = mgr.issue("set_mode", {"mode": "MANUAL"}, 1000, confirm=mode_confirmed("MANUAL"))
    mgr.update({"pico_command_ack": _ack("MANUAL", "gremlins", 1100)}, 1200)
    assert cmd.status == STATUS_FAILED
    assert "gremlins" in cmd.detail


# -- arming is a separate authority -----------------------------------------
#
# The mode changes; the propellers still cannot turn, because arming is channel
# 7 and no software request can raise it. "Confirmed by the vessel" on its own
# is a button appearing to succeed.


def test_a_confirmed_mode_says_when_propulsion_still_cannot_start():
    mgr = CommandManager()
    cmd = mgr.issue("set_mode", {"mode": "AUTONOMOUS"}, 1000,
                    confirm=mode_confirmed("AUTONOMOUS"))

    mgr.update(
        {"pico": {"mode": "AUTONOMOUS", "armed": False,
                  "arming_block": "RC channel 7 disarmed, propulsion cannot start"}},
        1100,
    )

    assert cmd.status == STATUS_CONFIRMED
    assert "confirmed by the vessel" in cmd.detail
    assert "channel 7" in cmd.detail


def test_a_confirmed_mode_on_an_armed_vessel_says_nothing_extra():
    mgr = CommandManager()
    cmd = mgr.issue("set_mode", {"mode": "AUTONOMOUS"}, 1000,
                    confirm=mode_confirmed("AUTONOMOUS"))
    mgr.update({"pico": {"mode": "AUTONOMOUS", "armed": True, "arming_block": None}}, 1100)
    assert cmd.detail == "confirmed by the vessel"


def test_a_propulsion_cut_is_not_annotated_with_the_arming_block():
    """A disarmed vessel is what that command was for. Appending 'propulsion
    cannot start' would read as a fault instead of as success."""
    mgr = CommandManager()
    cmd = mgr.issue("cut_propulsion", {}, 1000, confirm=propulsion_cut_confirmed)
    mgr.update(
        {"pico": {"mode": "ESTOP", "estop_latched": True, "armed": False,
                  "arming_block": "e-stop latched, propulsion cannot start until "
                                  "the arm switch is cycled"}},
        1100,
    )
    assert cmd.status == STATUS_CONFIRMED
    assert cmd.detail == "confirmed by the vessel"
