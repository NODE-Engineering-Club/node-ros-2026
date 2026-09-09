"""The adapters, against the Njord stack's real message shapes.

No ROS here: the adapters are plain functions over anything with the right
attributes, which is what lets them be tested on a laptop. The stand-ins below
mirror the real messages field for field.
"""

import math
from types import SimpleNamespace

import pytest
from gui_backend.core import adapters


def header(sec=1_800_000_000, nsec=0):
    return SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nsec))


def quat_from_yaw(yaw_deg):
    half = math.radians(yaw_deg) / 2.0
    return SimpleNamespace(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


def fix(lat=-22.9576, lon=14.5053, cov0=4.0, status=0):
    return SimpleNamespace(
        header=header(), latitude=lat, longitude=lon, altitude=0.0,
        position_covariance=[cov0, 0, 0, 0, cov0, 0, 0, 0, 9.0],
        status=SimpleNamespace(status=status),
    )


def odom(yaw_deg=0.0, vx=0.0, vy=0.0):
    return SimpleNamespace(
        header=header(),
        pose=SimpleNamespace(pose=SimpleNamespace(orientation=quat_from_yaw(yaw_deg))),
        twist=SimpleNamespace(twist=SimpleNamespace(
            linear=SimpleNamespace(x=vx, y=vy, z=0.0))),
    )


# -- heading and course ---------------------------------------------------


@pytest.mark.parametrize("yaw_deg, expected_bearing", [
    (0.0, 90.0),      # ENU yaw 0 = pointing east = 090
    (90.0, 0.0),      # yaw 90 = north = 000
    (180.0, 270.0),   # west
    (-90.0, 180.0),   # south
])
def test_enu_yaw_becomes_a_compass_bearing(yaw_deg, expected_bearing):
    """ROS is counter-clockwise from east; a bearing is clockwise from north.
    Getting this backwards puts the whole survey on the wrong side of the boat,
    plausibly, and nothing on screen would look wrong."""
    record = adapters.vessel_from_odometry(fix(), odom(yaw_deg=yaw_deg))
    assert record.heading_deg == pytest.approx(expected_bearing, abs=1e-6)


def test_course_over_ground_comes_from_the_velocity_not_the_heading():
    """Heading and course must be derived independently or comparing them says
    nothing — and that comparison is the row the heading panel exists for.

    Bow pointing north, drifting east: the two must differ."""
    record = adapters.vessel_from_odometry(fix(), odom(yaw_deg=90.0, vx=0.0, vy=-2.0))
    assert record.heading_deg == pytest.approx(0.0, abs=1e-6)
    assert record.cog_deg == pytest.approx(90.0, abs=1e-6)


def test_course_is_not_computed_from_noise_at_rest():
    record = adapters.vessel_from_odometry(fix(), odom(yaw_deg=45.0, vx=0.01, vy=0.0))
    assert record.sog_ms < 0.05
    assert record.cog_deg == 0.0


def test_speed_is_the_magnitude_of_the_body_velocity():
    record = adapters.vessel_from_odometry(fix(), odom(vx=3.0, vy=4.0))
    assert record.sog_ms == pytest.approx(5.0)


def test_the_record_is_stamped_with_its_oldest_component():
    """A frozen GNSS must not hide behind a live EKF."""
    old = fix()
    old.header = header(sec=1_800_000_000)
    fresh = odom()
    fresh.header = header(sec=1_800_000_030)
    record = adapters.vessel_from_odometry(old, fresh)
    assert record.utc_ms == 1_800_000_000_000


# -- what this stack does not report --------------------------------------


def test_a_missing_satellite_count_is_absent_rather_than_zero():
    """Neither NavSatFix nor Odometry carries one, and this stack has no MAVROS
    GPSRAW. "0 satellites" would read as a GNSS failure and ground a healthy
    vessel; the pre-flight reports SKIPPED instead."""
    assert adapters.vessel_from_odometry(fix(), odom()).num_sats is None


def test_position_accuracy_comes_from_the_covariance_when_there_is_one():
    assert adapters.vessel_from_odometry(fix(cov0=4.0), odom()).hdop == pytest.approx(2.0)


def test_no_covariance_means_no_figure_invented():
    assert adapters.vessel_from_odometry(fix(cov0=0.0), odom()).hdop is None


def test_heading_is_absent_without_the_ekf_rather_than_defaulting_to_north():
    record = adapters.vessel_from_odometry(fix(), None)
    assert math.isnan(record.heading_deg)


# -- the Pico, as a string ------------------------------------------------


def test_a_string_status_is_parsed_and_stamped_with_our_receipt_time():
    """std_msgs/String has no header, so there is no sample time to use. The
    receipt time is honest about being ours."""
    msg = SimpleNamespace(
        data="[STAT] Mode:2 Armed:N Relay:OFF "
             "Thr(Ch3):991 Yaw(Ch4):991 Arm(Ch7):172 Mode(Ch8):991"
    )
    record = adapters.pico_from_ros(msg, received_utc_ms=1_800_000_000_000)
    assert record.utc_ms == 1_800_000_000_000
    assert record.mode == 1          # MANUAL
    assert record.armed is False
    assert record.state_format_verified is True


def test_the_single_relay_is_reported_as_one_relay():
    """One relay on GPIO21 cuts ESC power. The '4' the GUI used to show came
    from the simulator's placeholder, not from hardware."""
    record = adapters.pico_from_ros(SimpleNamespace(data="[STAT] Relay:ON"), 0)
    assert record.relay_states == [True]
    record = adapters.pico_from_ros(SimpleNamespace(data="[STAT] Relay:OFF"), 0)
    assert record.relay_states == [False]


def test_the_rc_switch_position_is_carried_alongside_the_confirmed_mode():
    """So an operator can see a request being clamped by the transmitter rather
    than reading it as a failure."""
    record = adapters.pico_from_ros(
        SimpleNamespace(
            data="[STAT] Mode:2 Armed:Y Relay:ON "
                 "Thr(Ch3):991 Yaw(Ch4):991 Arm(Ch7):1811 Mode(Ch8):1811"
        ),
        0,
    )
    assert record.mode == 1                      # firmware says MANUAL
    assert record.rc_mode == "AUTONOMOUS"        # the switch says otherwise
    assert record.software_clamp_active is True
    assert record.rc_channels["mode"] == 1811


def test_the_status_line_cannot_report_a_latch_it_does_not_carry():
    """The firmware latches internally but never prints it. False would be a
    claim about something never sent; None says we do not know."""
    record = adapters.pico_from_ros(SimpleNamespace(data="[STAT] Mode:3"), 0)
    assert record.estop_latched is None
    assert record.hardware_killswitch_engaged is None


def test_an_unreadable_status_line_yields_no_mode_at_all():
    record = adapters.pico_from_ros(SimpleNamespace(data="garbage"), 0)
    assert record.mode is None
    assert record.state_parsed is False


def test_a_structured_picostatus_still_works():
    """asket_sim publishes the proper message; both shapes reach the same GUI."""
    msg = SimpleNamespace(
        header=header(), mode=2, armed=True, estop_latched=False,
        relay_states=[True, True], esc_status=[0, 0], rc_link_ok=True,
        rc_channel8_raw_pct=100, hardware_killswitch_engaged=False,
    )
    record = adapters.pico_from_ros(msg)
    assert record.mode == 2
    assert record.state_format_verified is True


# -- obstacles ------------------------------------------------------------


def cloud(points):
    import struct
    fields = [SimpleNamespace(name="x", offset=0, datatype=7),
              SimpleNamespace(name="y", offset=4, datatype=7),
              SimpleNamespace(name="z", offset=8, datatype=7)]
    data = b"".join(struct.pack("<fff", x, y, 0.0) for x, y in points)
    return SimpleNamespace(header=header(), fields=fields, point_step=12,
                           is_bigendian=False, data=data)


def test_a_point_cloud_becomes_bearings_and_ranges():
    """x forward, y left, bearing clockwise from the bow."""
    pairs = adapters.obstacles_from_pointcloud(cloud([(10.0, 0.0), (0.0, -5.0)]))
    assert pairs[0] == [0.0, 10.0]      # dead ahead
    assert pairs[1] == [90.0, 5.0]      # abeam to starboard


def test_nan_padding_is_dropped_rather_than_placed_on_the_hull():
    """A NaN read as zero puts a boulder underneath the boat."""
    pairs = adapters.obstacles_from_pointcloud(cloud([(float("nan"), 0.0), (4.0, 0.0)]))
    assert pairs == [[0.0, 4.0]]


def test_a_cloud_without_x_and_y_yields_nothing_rather_than_guessing():
    empty = SimpleNamespace(header=header(), fields=[], point_step=12,
                            is_bigendian=False, data=b"")
    assert adapters.obstacles_from_pointcloud(empty) == []
