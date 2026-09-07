"""Lidar simulation, and the filter the GUI lets an operator toggle."""

from asket_sim.core.lidar import LidarConfig, LidarSim, Obstacle, filter_scan


def test_finds_an_obstacle_at_the_right_bearing_and_range():
    sim = LidarSim(
        [Obstacle(0.0, 20.0, 2.0, "buoy")],
        LidarConfig(dropout_probability=0.0, wave_clutter_probability=0.0, range_noise_m=0.0),
    )
    scan = sim.scan(0, 0.0, 0.0, heading_deg=0.0)
    nearest = scan.nearest(use_filtered=True)
    assert nearest is not None
    range_m, bearing = nearest
    assert abs(range_m - 18.0) < 0.5      # 20 m to centre, 2 m radius
    assert abs(bearing) < 2.0 or abs(bearing - 360.0) < 2.0


def test_bearing_is_relative_to_the_hull_not_to_north():
    sim = LidarSim(
        [Obstacle(0.0, 20.0, 2.0)],
        LidarConfig(dropout_probability=0.0, wave_clutter_probability=0.0, range_noise_m=0.0),
    )
    scan = sim.scan(0, 0.0, 0.0, heading_deg=90.0)  # pointing east
    range_m, bearing = scan.nearest()
    # An obstacle due north is now on the port beam.
    assert abs(bearing - 270.0) < 2.0


def test_absent_returns_are_none_never_zero():
    """A zero range would render as an obstacle on the hull. Absence is absence."""
    sim = LidarSim([], LidarConfig(dropout_probability=1.0))
    scan = sim.scan(0, 0.0, 0.0, 0.0)
    assert all(r is None for r in scan.ranges_m)
    assert scan.nearest() is None


def test_filter_removes_isolated_clutter_but_keeps_real_returns():
    ranges = [None] * 20
    ranges[5] = 1.2                       # lone spike: wave clutter
    for i in range(10, 16):
        ranges[i] = 12.0 + 0.05 * i       # a real, extended object
    out = filter_scan(ranges, angle_increment_deg=0.5)
    assert out[5] is None
    assert all(out[i] is not None for i in range(10, 16))


def test_roll_produces_clutter_that_the_filter_then_removes():
    """The raw/filtered toggle only earns its place if the two differ."""
    sim = LidarSim(
        [Obstacle(0.0, 15.0, 3.0)],
        LidarConfig(dropout_probability=0.0, wave_clutter_probability=0.4),
    )
    scan = sim.scan(0, 0.0, 0.0, 0.0, roll_deg=12.0)
    raw = sum(1 for r in scan.ranges_m if r is not None)
    filtered = sum(1 for r in scan.filtered_m if r is not None)
    assert raw > filtered
