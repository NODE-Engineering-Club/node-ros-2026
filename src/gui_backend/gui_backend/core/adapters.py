"""ROS messages to the shapes :mod:`gui_backend.core.payloads` expects.

The payload builders are shared between the simulated and the real source. That
is the point: there is exactly one definition of what a `vessel` frame looks
like on the wire, so the GUI cannot work in sim and be subtly different in the
field.

To make that possible, these adapters convert ROS messages into the same simple
attribute-bearing records the simulator produces. They are plain functions over
plain data and are tested without ROS, by feeding them stand-ins with the same
attributes.

**None of them names a topic.** Topic names, types and which adapter to use all
come from ``config/topics.yaml``, because the existing ``pico_bridge`` message
is unknown to this repository and that package must not be modified
(docs/open_questions.md Q7).
"""

from __future__ import annotations

import math
from types import SimpleNamespace


def _stamp_to_utc_ms(header) -> int:
    """A ROS header stamp as UTC milliseconds."""
    stamp = header.stamp
    return int(stamp.sec) * 1000 + int(stamp.nanosec) // 1_000_000


def quaternion_to_euler_deg(q) -> tuple[float, float, float]:
    """(roll, pitch, yaw) in degrees, from a ROS quaternion."""
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.degrees(math.atan2(sinr, cosr))

    sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
    pitch = math.degrees(math.asin(sinp))

    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.degrees(math.atan2(siny, cosy))
    return roll, pitch, yaw


def vessel_from_ros(fix, compass_hdg, gps_vel, imu, gps_raw=None, extra=None):
    """Assemble a vessel record from the several MAVROS topics that carry it.

    MAVROS spreads position, heading, velocity and attitude across four topics
    with independent timestamps. The record takes the **oldest** of them as its
    own timestamp, so the age shown on screen is the age of the stalest
    component rather than the freshest. Showing the freshest would let a frozen
    GNSS hide behind a live IMU.
    """
    roll = pitch = 0.0
    if imu is not None:
        roll, pitch, _ = quaternion_to_euler_deg(imu.orientation)

    cog_deg = 0.0
    sog_ms = 0.0
    if gps_vel is not None:
        east = gps_vel.twist.linear.x
        north = gps_vel.twist.linear.y
        sog_ms = math.hypot(east, north)
        if sog_ms > 0.05:
            cog_deg = math.degrees(math.atan2(east, north)) % 360.0

    stamps = [_stamp_to_utc_ms(m.header) for m in (fix, gps_vel, imu) if m is not None]
    utc_ms = min(stamps) if stamps else 0

    hdop = float("nan")
    num_sats = 0
    if gps_raw is not None:
        hdop = getattr(gps_raw, "eph", 0) / 100.0 or float("nan")
        num_sats = int(getattr(gps_raw, "satellites_visible", 0))

    return SimpleNamespace(
        utc_ms=utc_ms,
        lat=fix.latitude if fix else float("nan"),
        lon=fix.longitude if fix else float("nan"),
        alt=fix.altitude if fix else float("nan"),
        heading_deg=float(compass_hdg) if compass_hdg is not None else float("nan"),
        heading_source=(extra or {}).get("heading_source", "magnetometer"),
        heading_valid=(extra or {}).get("heading_valid", compass_hdg is not None),
        heading_accuracy_deg=(extra or {}).get("heading_accuracy_deg", float("nan")),
        cog_deg=cog_deg,
        sog_ms=sog_ms,
        roll_deg=roll,
        pitch_deg=pitch,
        gnss_fix_type=int(getattr(getattr(fix, "status", None), "status", -1)) + 3
        if fix is not None
        else 0,
        num_sats=num_sats,
        hdop=hdop,
        distance_travelled_m=(extra or {}).get("distance_travelled_m", 0.0),
        on_survey=(extra or {}).get("on_survey", False),
    )


def pico_from_ros(msg):
    """Adapt a Pico status message.

    Written against ``asket_interfaces/PicoStatus`` (what the simulator
    publishes). The real ``pico_bridge`` message is not known here, so this is
    the one function that changes when Q7 is answered — deliberately isolated
    so that change is a few lines and not a hunt.
    """
    return SimpleNamespace(
        utc_ms=_stamp_to_utc_ms(msg.header),
        mode=int(msg.mode),
        armed=bool(msg.armed),
        estop_latched=bool(msg.estop_latched),
        relay_states=list(msg.relay_states),
        esc_status=list(msg.esc_status),
        rc_link_ok=bool(msg.rc_link_ok),
        rc_channel8_raw_pct=int(msg.rc_channel8_raw_pct),
        hardware_killswitch_engaged=bool(getattr(msg, "hardware_killswitch_engaged", False)),
    )


def battery_from_ros(msg, capacity_wh: float, hotel_load_w: float = 85.0):
    """Adapt ``sensor_msgs/BatteryState``.

    MAVROS reports discharge current as negative; endurance is computed from the
    magnitude of recent power draw, falling back to the hotel load so the
    estimate is conservative rather than infinite when current reads zero.
    """
    voltage = float(msg.voltage)
    current = abs(float(msg.current))
    soc = float(msg.percentage) if msg.percentage == msg.percentage else 0.0
    power_w = max(voltage * current, hotel_load_w)
    remaining_wh = capacity_wh * soc
    return SimpleNamespace(
        utc_ms=_stamp_to_utc_ms(msg.header),
        voltage=voltage,
        current=current,
        power_w=power_w,
        state_of_charge=soc,
        remaining_wh=remaining_wh,
        consumed_wh=max(0.0, capacity_wh - remaining_wh),
        endurance_s=remaining_wh / power_w * 3600.0 if power_w > 1.0 else float("inf"),
    )


def lidar_from_ros(msg):
    """Adapt ``sensor_msgs/LaserScan``.

    Infinities and NaNs become ``None``. A LaserScan says "no return" with
    infinity; carrying that through as a number would put an obstacle at the
    edge of the world, and carrying it through as zero would put one on the hull.
    """
    from asket_sim.core.lidar import filter_scan  # pure function, no ROS

    increment_deg = math.degrees(msg.angle_increment)
    ranges: list[float | None] = []
    for value in msg.ranges:
        if value != value or math.isinf(value):
            ranges.append(None)
        elif value < msg.range_min or value > msg.range_max:
            ranges.append(None)
        else:
            ranges.append(float(value))

    scan_time = msg.scan_time or 0.1
    return SimpleNamespace(
        utc_ms=_stamp_to_utc_ms(msg.header),
        angle_min_deg=math.degrees(msg.angle_min),
        angle_increment_deg=increment_deg,
        ranges_m=ranges,
        filtered_m=filter_scan(ranges, increment_deg),
        rotation_hz=1.0 / scan_time,
        points_per_revolution=sum(1 for r in ranges if r is not None),
        nearest=lambda use_filtered=True: _nearest(
            filter_scan(ranges, increment_deg) if use_filtered else ranges,
            math.degrees(msg.angle_min),
            increment_deg,
        ),
    )


def _nearest(ranges, angle_min_deg, increment_deg):
    best = None
    for i, r in enumerate(ranges):
        if r is None:
            continue
        if best is None or r < best[0]:
            best = (r, angle_min_deg + i * increment_deg)
    return best


def sonar_from_ros(msg):
    """Adapt ``asket_interfaces/SonarStatus`` into the health record shape."""
    from omniscan_bridge.core.status import SonarHealth

    return SonarHealth(
        connected=bool(msg.connected),
        actual_ping_rate_hz=float(msg.actual_ping_rate_hz),
        commanded_ping_rate_hz=float(msg.commanded_ping_rate_hz),
        points_per_ping=int(msg.points_per_ping),
        valid_points_per_ping=int(msg.points_per_ping),
        speed_of_sound=float(msg.speed_of_sound),
        range_setting_m=float(msg.range_setting_m),
        gain_setting=int(msg.gain_setting),
        packet_loss_ratio=float(msg.packet_loss_ratio),
        clock_offset_ms=int(msg.clock_offset_ms),
        pitch_deg=float(msg.pitch_deg),
        roll_deg=float(msg.roll_deg),
        packets_parsed=int(msg.packets_parsed),
        checksum_errors=int(msg.checksum_errors),
        bytes_discarded=int(msg.bytes_discarded),
        seconds_since_data=0.0 if msg.connected else float("inf"),
    )


def link_from_measurements(active_link: str, quality: float, rtt_ms: float, capacity: float):
    """Build a link record from what the backend has measured itself."""
    return SimpleNamespace(
        active_link=active_link,
        quality=quality,
        rtt_ms=rtt_ms,
        capacity_bytes_per_s=capacity,
        distance_m=float("nan"),
    )
