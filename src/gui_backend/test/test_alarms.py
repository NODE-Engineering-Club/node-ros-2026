"""Alarms: few, actionable, and never crying wolf."""

from gui_backend.core.alarms import AlarmThresholds, Alarm, diff, evaluate

T = AlarmThresholds()


def keys(state):
    return {a.key for a in evaluate(state, T)}


def test_a_healthy_vessel_raises_nothing():
    assert keys({"state_of_charge": 0.8, "heading_valid": True, "rc_link_ok": True}) == set()


def test_every_alarm_carries_a_remedy():
    """'CLOCK DRIFT' on a beach costs an hour. Saying what to do costs a minute."""
    state = {
        "state_of_charge": 0.05, "heading_valid": False, "rc_link_ok": False,
        "sonar_expected": True, "sonar_connected": False, "clock_offset_ms": 2000,
        "roll_deg": 35.0, "recording": True, "disk_free_bytes": 1024,
        "link_age_s": 30.0, "geofence_distance_m": 5.0,
    }
    for alarm in evaluate(state, T):
        assert alarm.remedy, f"{alarm.key} has no remedy"
        assert alarm.message


def test_battery_escalates_from_warning_to_alarm():
    warn = [a for a in evaluate({"state_of_charge": 0.2}, T) if a.key == "battery_low"]
    crit = [a for a in evaluate({"state_of_charge": 0.05}, T) if a.key == "battery_critical"]
    assert warn and warn[0].severity == "warn"
    assert crit and crit[0].severity == "alarm"


def test_disk_only_alarms_while_actually_recording():
    """A nearly full disk is not urgent when nothing is being written to it."""
    low_disk = {"disk_free_bytes": 1024}
    assert "disk_low" not in keys({**low_disk, "recording": False})
    assert "disk_low" in keys({**low_disk, "recording": True})


def test_clock_drift_escalates_and_says_what_it_costs():
    warn = evaluate({"sonar_expected": True, "sonar_connected": True,
                     "clock_offset_ms": 400}, T)
    alarm = evaluate({"sonar_expected": True, "sonar_connected": True,
                      "clock_offset_ms": 2000}, T)
    assert [a.severity for a in warn if a.key == "clock_drift"] == ["warn"]
    bad = next(a for a in alarm if a.key == "clock_drift")
    assert bad.severity == "alarm"
    assert "cannot be georeferenced" in bad.remedy


def test_invalid_heading_says_the_data_is_compromised():
    """The operator needs to know to re-run those lines, not just that a flag
    went red."""
    alarm = next(a for a in evaluate({"heading_valid": False}, T) if a.key == "heading_invalid")
    assert "compromised" in alarm.remedy


def test_obstacles_are_not_an_alarm():
    """Avoidance is the navigation stack's job; the GUI informs, it does not
    alert. An alarm here would be one the operator learns to dismiss."""
    assert keys({"nearest_obstacle_m": 1.0}) == set()


def test_divergence_only_alarms_when_it_is_meaningful():
    assert "heading_divergence" not in keys(
        {"heading_valid": True, "heading_divergence_suspicious": False,
         "heading_divergence_deg": 40.0}
    )
    assert "heading_divergence" in keys(
        {"heading_valid": True, "heading_divergence_suspicious": True,
         "heading_divergence_deg": 40.0}
    )


def test_sonar_alarms_are_suppressed_when_no_sonar_is_expected():
    assert keys({"sonar_expected": False, "sonar_connected": False}) == set()


def test_diff_reports_transitions_only():
    a = Alarm("x", "warn", "m")
    b = Alarm("y", "alarm", "m")
    raised, cleared = diff([a], [a, b])
    assert raised == [b] and cleared == []

    raised, cleared = diff([a, b], [b])
    assert raised == [] and cleared == ["x"]


def test_a_changed_message_on_the_same_key_is_re_raised():
    """Battery 24% and battery 13% are different facts under one key."""
    raised, _ = diff([Alarm("battery_low", "warn", "Battery at 24%")],
                     [Alarm("battery_low", "warn", "Battery at 13%")])
    assert len(raised) == 1


def test_a_recorder_that_stopped_still_raises_an_alarm():
    """The pre-emptive disk_low alarm depends on `recording`, so it goes quiet
    the moment the recorder stops because the disk filled — at exactly the
    moment it matters most. This one does not depend on the state it reports."""
    stopped = {"recording": False, "recording_error": "stopped: only 8 MB left"}
    alarms = {a.key: a for a in evaluate(stopped, T)}
    assert "recording_stopped" in alarms
    assert alarms["recording_stopped"].severity == "alarm"
    assert "no longer being logged" in alarms["recording_stopped"].remedy


def test_no_recording_alarm_when_nothing_went_wrong():
    assert "recording_stopped" not in keys({"recording": False, "recording_error": None})
