"""Sonar health accounting, especially the clock offset."""

import pytest
from omniscan_bridge.core.status import (
    CLOCK_OFFSET_ALARM_MS,
    CLOCK_OFFSET_WARN_MS,
    SonarHealthTracker,
)


def feed_pings(tracker, count, period_s=0.2, offset_ms=0, start=100.0, utc0=1_700_000_000_000):
    for i in range(count):
        now = start + i * period_s
        wall = utc0 + int(i * period_s * 1000)
        tracker.on_point_set(
            utc_msec=wall + offset_ms,
            num_points=256,
            num_valid=200,
            speed_of_sound=1500.0,
            now_monotonic=now,
            now_utc_ms=wall,
        )
    return start + (count - 1) * period_s


def test_ping_rate_is_measured_not_assumed():
    t = SonarHealthTracker()
    now = feed_pings(t, 20, period_s=0.2)
    assert t.ping_rate_hz(now) == pytest.approx(5.0, rel=0.05)


def test_ping_rate_falls_to_zero_when_pings_stop():
    """An open socket that has gone quiet is not a working sonar."""
    t = SonarHealthTracker(window_s=2.0)
    now = feed_pings(t, 10, period_s=0.1)
    assert t.ping_rate_hz(now) > 5.0
    assert t.ping_rate_hz(now + 30.0) == 0.0


def test_commanded_versus_actual_divergence_is_visible():
    """The sonar reduces its own rate when the range setting demands it. That is
    a real symptom, not a bug, and the operator needs to see it."""
    t = SonarHealthTracker()
    t.on_parameters_commanded(range_m=30.0, gain=4, ping_rate_hz=20.0)
    now = feed_pings(t, 20, period_s=0.2)     # actually 5 Hz
    health = t.health(True, 0.0, 100, 0, 0, 0.1, now_monotonic=now)
    assert health.commanded_ping_rate_hz == 20.0
    assert health.actual_ping_rate_hz == pytest.approx(5.0, rel=0.05)
    assert not health.ping_rate_ok


def test_a_synchronised_clock_reports_no_offset():
    t = SonarHealthTracker()
    now = feed_pings(t, 10, offset_ms=0)
    assert t.clock_offset_ms() == 0
    assert t.health(True, 0, 0, 0, 0, 0.1, now).clock_ok


def test_clock_drift_is_detected_and_escalates():
    """A bad clock does not degrade the survey, it destroys it — and nobody
    finds out until the data is opened back home."""
    warn = SonarHealthTracker()
    feed_pings(warn, 10, offset_ms=CLOCK_OFFSET_WARN_MS + 50)
    h = warn.health(True, 0, 0, 0, 0, 0.1, 200.0)
    assert not h.clock_ok
    assert not h.clock_compromised

    bad = SonarHealthTracker()
    feed_pings(bad, 10, offset_ms=CLOCK_OFFSET_ALARM_MS + 500)
    assert bad.health(True, 0, 0, 0, 0, 0.1, 200.0).clock_compromised


def test_one_late_packet_cannot_raise_a_clock_alarm_on_its_own():
    """The offset is a median precisely so a single outlier does not cry wolf."""
    t = SonarHealthTracker()
    feed_pings(t, 15, offset_ms=10)
    t.on_point_set(1_700_000_099_999, 256, 200, 1500.0, 200.0, 1_700_000_000_000)
    assert abs(t.clock_offset_ms()) < CLOCK_OFFSET_WARN_MS


def test_an_unset_sonar_clock_is_not_averaged_in_as_a_huge_drift():
    """A zero timestamp is a different fault from a drifting clock, and must not
    be reported as a 1.7-trillion-millisecond error."""
    t = SonarHealthTracker()
    t.on_point_set(0, 256, 200, 1500.0, 100.0, 1_700_000_000_000)
    assert t.clock_offset_ms() == 0


def test_attitude_is_carried_through():
    t = SonarHealthTracker()
    t.on_attitude(pitch_deg=-2.5, roll_deg=7.25)
    h = t.health(True, 0, 0, 0, 0, 0.1, 100.0)
    assert (h.pitch_deg, h.roll_deg) == (-2.5, 7.25)
