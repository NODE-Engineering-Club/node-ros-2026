"""Sensor frame to vessel frame.

The sonar reports angles and times. Everything about where those detections
actually are is asserted here.
"""

import math

import pytest
from omniscan_bridge.core.geometry import (
    SonarMounting,
    point_set_to_vessel_frame,
    range_from_tof,
    sensor_to_vessel,
    swath_extent,
)
from omniscan_bridge.core.ping_protocol import Point, PointSet

FLUSH = dict(lever_x_m=0.0, lever_y_m=0.0, lever_z_m=0.0)


def test_time_of_flight_is_two_way():
    """Forgetting the factor of two doubles every depth in the survey."""
    assert range_from_tof(0.02, 1500.0) == pytest.approx(15.0)


def test_the_sonars_own_speed_of_sound_is_used_not_an_assumed_one():
    """Assuming 1500 when the sonar used 1520 puts a 20 m bottom 27 cm out,
    systematically, over the entire survey."""
    ps = PointSet(1, speed_of_sound=1520.0, utc_msec=0, points=[Point(0.0, 0.02, 1.0, 1)])
    (_, _, z, _), = point_set_to_vessel_frame(ps, SonarMounting(tilt_deg=0.0, **FLUSH))
    assert -z == pytest.approx(15.2)


def test_zero_tilt_points_straight_down():
    x, y, z = sensor_to_vessel(0.0, 20.0, SonarMounting(tilt_deg=0.0, **FLUSH))
    assert (x, y) == (0.0, 0.0)
    assert z == pytest.approx(-20.0), "z is up in REP-103, so down is negative"


def test_mounting_tilt_swings_the_boresight_to_starboard():
    m = SonarMounting(tilt_deg=35.0, **FLUSH)
    x, y, z = sensor_to_vessel(0.0, 20.0, m)
    assert x == 0.0
    assert y == pytest.approx(-20.0 * math.sin(math.radians(35.0)))  # -y = starboard
    assert z == pytest.approx(-20.0 * math.cos(math.radians(35.0)))
    assert math.hypot(y, z) == pytest.approx(20.0), "tilt must not change the range"


def test_a_port_install_puts_the_swath_on_the_other_side():
    stbd = sensor_to_vessel(0.0, 20.0, SonarMounting(tilt_deg=35.0, **FLUSH))
    port = sensor_to_vessel(0.0, 20.0, SonarMounting(tilt_deg=-35.0, **FLUSH))
    assert stbd[1] < 0 < port[1]


def test_the_lever_arm_is_applied():
    """A systematic offset no post-processing will discover for you."""
    m = SonarMounting(tilt_deg=0.0, lever_x_m=-0.2, lever_y_m=-0.35, lever_z_m=-0.15)
    x, y, z = sensor_to_vessel(0.0, 0.0, m)
    assert (x, y, z) == pytest.approx((-0.2, -0.35, -0.15))


def test_vessel_roll_moves_the_swath():
    """Roll is why a flat seabed can look like a slope. It must be corrected,
    and that correction must be exercised."""
    m = SonarMounting(tilt_deg=30.0, **FLUSH)
    level = sensor_to_vessel(0.0, 20.0, m, roll_deg=0.0)
    rolled = sensor_to_vessel(0.0, 20.0, m, roll_deg=10.0)
    assert abs(rolled[1] - level[1]) > 1.0
    assert math.hypot(*rolled[1:]) == pytest.approx(math.hypot(*level[1:]))


def test_roll_of_exactly_the_tilt_angle_puts_the_beam_flat_out_sideways():
    m = SonarMounting(tilt_deg=45.0, **FLUSH)
    x, y, z = sensor_to_vessel(0.0, 10.0, m, roll_deg=-45.0)
    assert z == pytest.approx(0.0, abs=1e-9)
    assert y == pytest.approx(-10.0)


def test_beams_with_no_detection_are_dropped_not_placed_on_the_hull():
    """A zero-range point would sit under the boat and read as a boulder."""
    ps = PointSet(1, 1500.0, 0, [Point(0.0, 0.0, 0.0, 0), Point(0.1, 0.02, 0.8, 1)])
    out = point_set_to_vessel_frame(ps, SonarMounting())
    assert len(out) == 1


def test_a_zero_speed_of_sound_yields_nothing_rather_than_garbage():
    ps = PointSet(1, 0.0, 0, [Point(0.0, 0.02, 1.0, 1)])
    assert point_set_to_vessel_frame(ps, SonarMounting()) == []


def test_points_beyond_the_range_setting_are_dropped():
    ps = PointSet(1, 1500.0, 0, [Point(0.0, 0.02, 1.0, 1), Point(0.0, 0.2, 1.0, 1)])
    out = point_set_to_vessel_frame(ps, SonarMounting(), max_range_m=30.0)
    assert len(out) == 1


def test_swath_extent_reports_what_was_ensonified_not_what_was_possible():
    """The coverage overlay paints this. If it painted the range setting
    instead, every gap would be hidden."""
    m = SonarMounting(tilt_deg=35.0, **FLUSH)
    ps = PointSet(
        1, 1500.0, 0,
        [Point(a, 20.0 * 2 / 1500.0, 1.0, 1) for a in (-0.3, 0.0, 0.3)],
    )
    points = point_set_to_vessel_frame(ps, m)
    near, far, depth = swath_extent(points)
    assert 0.0 < near < far
    assert depth > 0.0


def test_an_empty_ping_reports_zero_extent_so_the_overlay_shows_a_gap():
    assert swath_extent([]) == (0.0, 0.0, 0.0)
