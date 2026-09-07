"""ROS adapters, tested without ROS.

The adapters take ROS messages, so the tests feed them stand-ins with the same
attributes. That keeps the *shared* payload path — the one both sim and field
use — under test on a machine with no ROS installed.
"""

import math
from types import SimpleNamespace

import pytest
from gui_backend.core import adapters, payloads
from gui_backend.core.streams import DETAIL_FULL


def header(sec, nanosec=0):
    return SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec))


def quaternion(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0):
    r, p, y = (math.radians(v) / 2 for v in (roll_deg, pitch_deg, yaw_deg))
    return SimpleNamespace(
        x=math.sin(r) * math.cos(p) * math.cos(y) - math.cos(r) * math.sin(p) * math.sin(y),
        y=math.cos(r) * math.sin(p) * math.cos(y) + math.sin(r) * math.cos(p) * math.sin(y),
        z=math.cos(r) * math.cos(p) * math.sin(y) - math.sin(r) * math.sin(p) * math.cos(y),
        w=math.cos(r) * math.cos(p) * math.cos(y) + math.sin(r) * math.sin(p) * math.sin(y),
    )


def test_quaternion_to_euler_round_trip():
    roll, pitch, yaw = adapters.quaternion_to_euler_deg(quaternion(12.0, -5.0, 40.0))
    assert roll == pytest.approx(12.0, abs=1e-6)
    assert pitch == pytest.approx(-5.0, abs=1e-6)
    assert yaw == pytest.approx(40.0, abs=1e-6)


def test_vessel_takes_the_age_of_its_stalest_component():
    """MAVROS spreads position, velocity and attitude across topics with
    independent stamps. Reporting the freshest would let a frozen GNSS hide
    behind a live IMU."""
    fix = SimpleNamespace(header=header(1000), latitude=-22.9, longitude=14.5,
                          altitude=0.0, status=SimpleNamespace(status=0))
    vel = SimpleNamespace(header=header(1005),
                          twist=SimpleNamespace(linear=SimpleNamespace(x=1.0, y=1.0)))
    imu = SimpleNamespace(header=header(1009), orientation=quaternion())

    record = adapters.vessel_from_ros(fix, 42.0, vel, imu)
    assert record.utc_ms == 1000 * 1000


def test_course_over_ground_comes_from_the_velocity_vector():
    fix = SimpleNamespace(header=header(1), latitude=0.0, longitude=0.0, altitude=0.0,
                          status=SimpleNamespace(status=0))
    # 1 m/s east, 1 m/s north -> 045 degrees at 1.41 m/s.
    vel = SimpleNamespace(header=header(1),
                          twist=SimpleNamespace(linear=SimpleNamespace(x=1.0, y=1.0)))
    record = adapters.vessel_from_ros(fix, 40.0, vel, None)
    assert record.cog_deg == pytest.approx(45.0)
    assert record.sog_ms == pytest.approx(math.sqrt(2))


def test_a_stationary_vessel_reports_no_course_rather_than_a_random_one():
    fix = SimpleNamespace(header=header(1), latitude=0.0, longitude=0.0, altitude=0.0,
                          status=SimpleNamespace(status=0))
    vel = SimpleNamespace(header=header(1),
                          twist=SimpleNamespace(linear=SimpleNamespace(x=0.001, y=0.0)))
    assert adapters.vessel_from_ros(fix, 40.0, vel, None).cog_deg == 0.0


def test_laser_scan_infinities_become_absent_not_zero():
    """Infinity means no return. Carried through as a number it puts an obstacle
    at the edge of the world; carried through as zero it puts one on the hull."""
    scan = SimpleNamespace(
        header=header(5),
        angle_min=0.0,
        angle_increment=math.radians(1.0),
        range_min=0.2,
        range_max=30.0,
        scan_time=0.1,
        ranges=[float("inf"), 5.0, float("nan"), 0.05, 100.0] + [5.1] * 5,
    )
    record = adapters.lidar_from_ros(scan)
    assert record.ranges_m[0] is None
    assert record.ranges_m[1] == 5.0
    assert record.ranges_m[2] is None
    assert record.ranges_m[3] is None      # below range_min
    assert record.ranges_m[4] is None      # above range_max
    assert record.rotation_hz == pytest.approx(10.0)


def test_battery_endurance_is_conservative_when_current_reads_zero():
    """An infinite endurance estimate is the most dangerous number this panel
    could show."""
    msg = SimpleNamespace(header=header(1), voltage=25.0, current=0.0, percentage=0.5)
    record = adapters.battery_from_ros(msg, capacity_wh=1200.0, hotel_load_w=85.0)
    assert math.isfinite(record.endurance_s)
    assert record.endurance_s == pytest.approx(600.0 / 85.0 * 3600.0)


def test_battery_discharge_current_sign_is_normalised():
    """MAVROS reports discharge as negative; the panel shows a magnitude."""
    msg = SimpleNamespace(header=header(1), voltage=25.0, current=-8.0, percentage=0.9)
    assert adapters.battery_from_ros(msg, 1200.0).current == 8.0


def test_adapted_records_feed_the_same_payload_builders_as_the_simulator():
    """The whole reason the adapters exist: one payload definition, both modes."""
    pico = SimpleNamespace(
        header=header(3), mode=2, armed=True, estop_latched=False,
        relay_states=[True, True], esc_status=[0, 0], rc_link_ok=True,
        rc_channel8_raw_pct=100,
    )
    payload = payloads.pico_payload(adapters.pico_from_ros(pico), DETAIL_FULL)
    assert payload["mode"] == "AUTONOMOUS"
    assert payload["armed"] is True
    assert payload["relay_states"] == [True, True]
