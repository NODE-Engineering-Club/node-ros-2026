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
