"""Pre-flight checks.

The requirement these are written against: on a Namibian beach, identify the
faulty link in thirty seconds rather than an hour.
"""

import pytest
from system_test.core.checks import (
    CHECKS,
    FAIL,
    PASS,
    SKIPPED,
    WARN,
    Thresholds,
    run_checks,
)

HEALTHY = dict(
    pico_age_s=0.1, rc_link_ok=True, rc_channel8_raw_pct=100,
    num_sats=14, gnss_fix_type=3, hdop=0.8,
    heading_valid=True, heading_source="gnss_compass", heading_accuracy_deg=0.2,
    heading_divergence_deg=2.0, heading_divergence_meaningful=True,
    roll_deg=1.0, pitch_deg=0.5,
    lidar_rotation_hz=10.0, lidar_points_per_revolution=400,
    sonar_connected=True, sonar_ping_rate_hz=5.0, sonar_points_per_ping=256,
    clock_offset_ms=12,
    disk_free_bytes=200 * 1024**3, disk_write_mbps=120.0,
    state_of_charge=0.9, battery_voltage=28.0,
    link_rtt_ms=25.0, link_active="wifi",
    expected_nodes=["omniscan_bridge"], present_nodes=["omniscan_bridge"],
)


def item(report, check_id):
    return next(i for i in report.items if i.id == check_id)


def test_a_healthy_vessel_is_go():
    report = run_checks(HEALTHY)
    assert report.go
    assert report.summary == "GO — everything checks out"


def test_every_failing_check_says_what_to_do_about_it():
    """A red word is not an instruction. 'GPS: ERROR' costs an hour."""
    broken = dict(
        HEALTHY, num_sats=2, gnss_fix_type=1, sonar_connected=False, clock_offset_ms=5000,
        disk_free_bytes=1024**3, disk_write_mbps=3.0, battery_voltage=19.0,
        rc_link_ok=False, lidar_rotation_hz=0.0, heading_valid=False,
        present_nodes=[], pico_age_s=30.0,
    )
    report = run_checks(broken)
    assert not report.go
    for result in report.items:
        if result.status in (FAIL, WARN, SKIPPED):
            assert result.remedy, f"{result.id} has no remedy"
            assert len(result.message) > 10


def test_messages_carry_the_number_and_the_requirement():
    """Not 'GPS: ERROR' but '4 satellites, 6 required'."""
    report = run_checks(dict(HEALTHY, num_sats=4))
    gnss = item(report, "gnss.fix")
    assert gnss.status == FAIL
    assert "4 satellites" in gnss.message
    assert "6 required" in gnss.message


def test_nothing_reporting_is_no_go_not_go():
    """'GO — 14 checks could not run' is the most dangerous sentence this panel
    could produce."""
    report = run_checks({})
    assert not report.go
    assert "cannot confirm" in report.summary
    assert all(i.status == SKIPPED for i in report.items)


def test_a_skipped_non_critical_check_does_not_ground_the_vessel():
    without_link = {k: v for k, v in HEALTHY.items() if k != "link_rtt_ms"}
    report = run_checks(without_link)
    assert report.go
    assert item(report, "link.quality").status == SKIPPED


def test_a_sonar_on_the_network_and_a_sonar_absent_are_different_faults():
    """They send you to different cables, so they must be different messages."""
    absent = item(run_checks(dict(HEALTHY, sonar_connected=False, sonar_discovered=False)),
                  "sonar.link")
    silent = item(run_checks(dict(HEALTHY, sonar_connected=False, sonar_discovered=True)),
                  "sonar.link")

    assert absent.status == silent.status == FAIL
    assert absent.message != silent.message
    assert "Ethernet cable" in absent.remedy
    assert "Power-cycle" in silent.remedy


def test_clock_drift_is_a_hard_fail_and_says_what_it_costs():
    """Post-mission fusion pairs on this timestamp. Drift does not degrade the
    survey, it destroys it."""
    result = item(run_checks(dict(HEALTHY, clock_offset_ms=5000)), "sonar.clock")
    assert result.status == FAIL
    assert "un-georeferenceable" in result.remedy
    assert "NTP" in result.remedy


def test_heading_accuracy_is_reported_as_seabed_error():
    """The number that makes heading matter."""
    result = item(
        run_checks(dict(HEALTHY, heading_source="magnetometer", heading_accuracy_deg=15.0)),
        "heading.valid",
    )
    assert result.status == WARN
    assert " m of seabed error at 50 m" in result.message


def test_divergence_is_skipped_rather_than_failed_when_stationary():
    """Sitting still with a 40 degree divergence is not a fault."""
    result = item(
        run_checks(dict(HEALTHY, heading_divergence_deg=40.0,
                        heading_divergence_meaningful=False)),
        "heading.divergence",
    )
    assert result.status == SKIPPED
    assert "Too slow" in result.message


def test_disk_space_is_expressed_in_hours_of_survey():
    """'42 GB free' means nothing on a beach. 'About 12 hours' does."""
    result = item(run_checks(HEALTHY), "disk.space")
    assert "hours of survey" in result.message


def test_a_slow_disk_fails_because_the_sonar_will_outrun_it():
    result = item(run_checks(dict(HEALTHY, disk_write_mbps=5.0)), "disk.speed")
    assert result.status == FAIL
    assert "SSD" in result.remedy


def test_a_low_battery_warns_and_points_at_the_endurance_estimate():
    result = item(run_checks(dict(HEALTHY, state_of_charge=0.3)), "battery.charge")
    assert result.status == WARN
    assert "endurance" in result.remedy


def test_a_flat_battery_fails_outright():
    result = item(run_checks(dict(HEALTHY, battery_voltage=19.0)), "battery.charge")
    assert result.status == FAIL
    assert "Do not launch" in result.remedy


def test_the_killswitch_channel_being_asserted_is_a_warning_not_a_failure():
    """It is the safe state. Failing it would teach people to ignore the panel."""
    result = item(run_checks(dict(HEALTHY, rc_channel8_raw_pct=0)), "rc.link")
    assert result.status == WARN
    assert "Release the killswitch" in result.remedy


def test_a_missing_node_names_the_node():
    result = item(
        run_checks(dict(HEALTHY, expected_nodes=["omniscan_bridge", "mission_recorder"],
                        present_nodes=["omniscan_bridge"])),
        "ros.nodes",
    )
    assert result.status == FAIL
    assert "mission_recorder" in result.message


def test_running_a_subset_runs_only_that_subset():
    report = run_checks(HEALTHY, only=["gnss.fix", "sonar.clock"])
    assert {i.id for i in report.items} == {"gnss.fix", "sonar.clock"}


def test_the_verdict_leads_with_the_worst_problem():
    report = run_checks(dict(HEALTHY, num_sats=2, state_of_charge=0.3))
    assert report.summary.startswith("NO-GO")
    assert "satellites" in report.summary


@pytest.mark.parametrize("registered", CHECKS, ids=lambda c: c.id)
def test_every_check_survives_a_completely_empty_state(registered):
    """A check that throws on missing data takes the whole pre-flight with it,
    on the day the pre-flight is most needed."""
    result = registered.fn({}, Thresholds())
    assert result.status in (PASS, WARN, FAIL, SKIPPED)
    assert result.message
