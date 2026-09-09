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

from .pico_state import parse_state_line


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


#: The GUI's mode vocabulary, as integers, matching asket_interfaces/PicoStatus.
_MODE_VALUES = {"ESTOP": 0, "MANUAL": 1, "AUTONOMOUS": 2}


def pico_from_ros(msg, received_utc_ms: int = 0):
    """Adapt whatever ``/pico/status`` carries into a vessel-state record.

    Two shapes are accepted, because two exist:

    * ``std_msgs/String`` — what the real ``pico_bridge`` publishes. It reads
      lines off the serial link and republishes any ``STATE ...`` line verbatim,
      unparsed, so the text is parsed here through
      :mod:`gui_backend.core.pico_state`. That format is **PROVISIONAL**.
    * ``asket_interfaces/PicoStatus`` — what ``asket_sim`` publishes, a proper
      structured message.

    A String has no header, so it has no timestamp of its own. ``received_utc_ms``
    is when this process saw it, which is the best available and is honest about
    what it is: the age on screen is then the age of our receipt, not of the
    sample. That is a slight over-estimate of freshness by the serial latency,
    and it is the direction to err in.
    """
    if hasattr(msg, "data") and not hasattr(msg, "mode"):
        return _pico_from_state_line(str(msg.data), received_utc_ms)

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
        state_line="",
        state_parsed=True,
        state_format_verified=True,
    )


def _pico_from_state_line(line: str, received_utc_ms: int):
    """A raw firmware line into the same record shape.

    Every field the line does not carry stays ``None``. It travels to the GUI as
    ``null`` and renders as "not sent" — not as zero, and not as a fault. The
    difference matters most for ``rc_link_ok``: reporting a healthy RC link as
    lost because a text field was missing is exactly the lie this GUI exists to
    avoid.
    """
    state = parse_state_line(line)
    return SimpleNamespace(
        utc_ms=received_utc_ms,
        mode=_MODE_VALUES.get(state.mode) if state.mode else None,
        armed=state.armed,
        # There is no ESTOP in the firmware bridge's vocabulary, so the Pico
        # cannot report one latched. Absent, not False: claiming "not latched"
        # would be claiming knowledge of something never reported.
        estop_latched=None,
        relay_states=[],
        esc_status=[],
        rc_link_ok=state.rc_link_ok,
        rc_channel8_raw_pct=state.rc_channel8_raw_pct,
        hardware_killswitch_engaged=None,
        battery_voltage=state.battery_voltage,
        battery_current=state.battery_current,
        state_line=state.raw,
        state_parsed=state.parsed,
        state_unknown_keys=state.unknown_keys,
        state_format_verified=state.format_verified,
    )


def vessel_from_odometry(fix, odom_filtered, imu=None, extra=None):
    """Assemble a vessel record from the club stack's topics.

    Where the values come from, and why:

    * **Position** from ``/gps_driver/gps_raw`` (``NavSatFix``). The map-frame
      pose on ``/odometry/gps`` is the same fix transformed; the GUI draws on a
      geographic map, so it takes the geographic one and avoids depending on the
      datum being set.
    * **Heading, speed and attitude** from ``/odometry/filtered``, the EKF
      output. That is a fused heading rather than a raw magnetometer, which is
      why its nominal accuracy is better than the magnetometer figure.
    * **Course over ground** from the same message's twist, rotated out of the
      body frame into the map frame. Heading and course must be independently
      derived or comparing them says nothing — and that comparison is the row
      the heading panel exists for.

    ROS is ENU (x east, y north, yaw counter-clockwise from east); a compass
    bearing is clockwise from north. Hence ``90 - yaw``.
    """
    roll = pitch = 0.0
    heading_deg = None
    sog_ms = 0.0
    cog_deg = 0.0

    if odom_filtered is not None:
        roll, pitch, yaw = quaternion_to_euler_deg(odom_filtered.pose.pose.orientation)
        heading_deg = (90.0 - yaw) % 360.0

        # Odometry twist is in child_frame_id — the body frame. Rotate it into
        # the map frame before taking a course from it.
        vx = odom_filtered.twist.twist.linear.x
        vy = odom_filtered.twist.twist.linear.y
        sog_ms = math.hypot(vx, vy)
        if sog_ms > 0.05:
            rad = math.radians(yaw)
            east = vx * math.cos(rad) - vy * math.sin(rad)
            north = vx * math.sin(rad) + vy * math.cos(rad)
            cog_deg = math.degrees(math.atan2(east, north)) % 360.0

    if imu is not None and odom_filtered is None:
        roll, pitch, _ = quaternion_to_euler_deg(imu.orientation)

    stamps = [_stamp_to_utc_ms(m.header) for m in (fix, odom_filtered, imu) if m is not None]
    utc_ms = min(stamps) if stamps else 0

    # Neither NavSatFix nor Odometry carries a satellite count, and this stack
    # has no equivalent of MAVROS's GPSRAW. Absent, not zero: "0 satellites"
    # would read as a GNSS failure and ground a perfectly healthy vessel.
    hdop = None
    if fix is not None and fix.position_covariance[0] > 0.0:
        # Horizontal standard deviation in metres. Reported as itself rather
        # than converted to an HDOP by a made-up UERE.
        hdop = math.sqrt(fix.position_covariance[0])

    return SimpleNamespace(
        utc_ms=utc_ms,
        lat=fix.latitude if fix else float("nan"),
        lon=fix.longitude if fix else float("nan"),
        alt=fix.altitude if fix else float("nan"),
        heading_deg=heading_deg if heading_deg is not None else float("nan"),
        heading_source=(extra or {}).get("heading_source", "ekf"),
        heading_valid=(extra or {}).get("heading_valid", heading_deg is not None),
        heading_accuracy_deg=(extra or {}).get("heading_accuracy_deg", float("nan")),
        cog_deg=cog_deg,
        sog_ms=sog_ms,
        roll_deg=roll,
        pitch_deg=pitch,
        gnss_fix_type=int(getattr(getattr(fix, "status", None), "status", -1)) + 3
        if fix is not None
        else 0,
        num_sats=None,
        hdop=hdop,
        distance_travelled_m=(extra or {}).get("distance_travelled_m", 0.0),
        on_survey=(extra or {}).get("on_survey", False),
    )


def obstacles_from_pointcloud(msg, max_points: int = 720):
    """``sensor_msgs/PointCloud2`` of obstacle points into bearing/range pairs.

    ``/obstacles/lidar`` and ``/obstacles/fused`` are what the perception stack
    already produces, so the GUI consumes those rather than re-filtering a raw
    scan itself and risking a second, disagreeing answer about where the
    obstacles are.

    Points are read with the stdlib only — no numpy, no ``sensor_msgs_py`` — so
    this stays testable without ROS installed like everything else in ``core``.
    """
    import struct

    offsets = {f.name: (f.offset, f.datatype) for f in msg.fields}
    if "x" not in offsets or "y" not in offsets:
        return []

    fmt = "<f" if not msg.is_bigendian else ">f"
    step = msg.point_step
    data = bytes(msg.data)
    count = min(len(data) // step if step else 0, max_points)

    out = []
    for i in range(count):
        base = i * step
        try:
            x = struct.unpack_from(fmt, data, base + offsets["x"][0])[0]
            y = struct.unpack_from(fmt, data, base + offsets["y"][0])[0]
        except struct.error:
            break
        if x != x or y != y:          # NaN padding
            continue
        rng = math.hypot(x, y)
        if rng < 0.05:
            continue
        # Body frame: x forward, y left. Bearing is clockwise from the bow.
        bearing = math.degrees(math.atan2(-y, x)) % 360.0
        out.append([round(bearing, 1), round(rng, 2)])
    return out


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
        # SonarStatus does not distinguish the source of the rate; the bridge
        # already preferred the device's figure when it had one.
        rate_from_device=bool(msg.connected),
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
