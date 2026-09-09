"""ROS 2 node publishing the simulated vessel.

Publishes on the **same topics the real stack publishes**, so that
``sim:=true`` swaps sources and nothing downstream changes. The sonar is not
here: it is served as real Ping Protocol frames on a real socket by
``fake_sonar_node``, so ``omniscan_bridge`` is identical in both modes.

This file is a thin adapter. All the behaviour lives in ``asket_sim.core`` and
is tested there, without ROS.
"""

from __future__ import annotations

import math

import rclpy
from asket_common.heading import (
    SOURCE_GNSS_COMPASS,
    SOURCE_MAGNETOMETER,
    evaluate_heading,
)
from asket_interfaces.msg import HeadingStatus, PicoStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import BatteryState, Imu, LaserScan, NavSatFix, NavSatStatus
from std_msgs.msg import Float64, String

from asket_sim.core.faults import FAULTS
from asket_sim.core.vessel import HEADING_SOURCE_GNSS
from asket_sim.core.world import SimWorld, WorldConfig

SENSOR_QOS = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
LATCHED_QOS = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def _quaternion_from_euler(roll: float, pitch: float, yaw: float):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class SimNode(Node):
    def __init__(self) -> None:
        super().__init__("asket_sim")

        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("time_scale", 1.0)
        self.declare_parameter("origin_lat", WorldConfig.origin_lat)
        self.declare_parameter("origin_lon", WorldConfig.origin_lon)
        self.declare_parameter("survey_lines", WorldConfig.survey_num_lines)
        self.declare_parameter("line_length_m", WorldConfig.survey_line_length_m)
        self.declare_parameter("line_spacing_m", WorldConfig.survey_line_spacing_m)
        self.declare_parameter("heading_source", "magnetometer")
        self.declare_parameter("seed", 1)

        cfg = WorldConfig(
            origin_lat=self.get_parameter("origin_lat").value,
            origin_lon=self.get_parameter("origin_lon").value,
            survey_num_lines=int(self.get_parameter("survey_lines").value),
            survey_line_length_m=float(self.get_parameter("line_length_m").value),
            survey_line_spacing_m=float(self.get_parameter("line_spacing_m").value),
            seed=int(self.get_parameter("seed").value),
        )
        cfg.vessel.heading_source = self.get_parameter("heading_source").value
        self.world = SimWorld(cfg)
        self.time_scale = float(self.get_parameter("time_scale").value)

        # Publishers, on the real stack's topic names.
        self.pub_fix = self.create_publisher(
            NavSatFix, "/mavros/global_position/global", SENSOR_QOS
        )
        self.pub_hdg = self.create_publisher(
            Float64, "/mavros/global_position/compass_hdg", SENSOR_QOS
        )
        self.pub_vel = self.create_publisher(
            TwistStamped, "/mavros/global_position/raw/gps_vel", SENSOR_QOS
        )
        self.pub_imu = self.create_publisher(Imu, "/mavros/imu/data", SENSOR_QOS)
        self.pub_batt = self.create_publisher(BatteryState, "/mavros/battery", SENSOR_QOS)
        self.pub_scan = self.create_publisher(LaserScan, "/scan", SENSOR_QOS)
        self.pub_pico = self.create_publisher(PicoStatus, "/pico/status", 10)
        self.pub_heading = self.create_publisher(HeadingStatus, "/asket/heading", 10)
        self.pub_diag = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.pub_faults = self.create_publisher(String, "/sim/faults", LATCHED_QOS)

        # Fault injection: a topic rather than a service, so it can be driven
        # from the command line with ros2 topic pub during a demo.
        self.create_subscription(String, "/sim/inject_fault", self._on_inject, 10)
        self.create_subscription(String, "/sim/clear_fault", self._on_clear, 10)
        # Mode requests. In the real system these go to pico_bridge; here they
        # go to the simulated Pico, which confirms them a moment later.
        self.create_subscription(String, "/pico/mode_request", self._on_mode, 10)

        rate = float(self.get_parameter("rate_hz").value)
        self._dt = 1.0 / rate
        self.create_timer(self._dt, self._tick)
        self.create_timer(1.0, self._publish_diagnostics)
        self._publish_faults()

        self.get_logger().info(
            f"simulating {cfg.survey_num_lines} survey lines from "
            f"{cfg.origin_lat:.5f},{cfg.origin_lon:.5f} at {rate:g} Hz"
        )

    # -- inputs -----------------------------------------------------------

    def _on_inject(self, msg: String) -> None:
        name, _, duration = msg.data.partition(":")
        try:
            self.world.inject_fault(name.strip(), float(duration) if duration else None)
        except (KeyError, ValueError) as exc:
            self.get_logger().warn(f"fault request {msg.data!r} rejected: {exc}")
            return
        self.get_logger().warn(f"injected fault {name!r}")
        self._publish_faults()

    def _on_clear(self, msg: String) -> None:
        if msg.data.strip() in ("*", "all"):
            self.world.faults.clear_all()
        else:
            self.world.clear_fault(msg.data.strip())
        self._publish_faults()

    def _on_mode(self, msg: String) -> None:
        from asket_sim.core.pico import MODE_VALUES

        mode = MODE_VALUES.get(msg.data.strip().upper())
        if mode is None:
            self.get_logger().warn(f"unknown mode request {msg.data!r}")
            return
        self.world.pico.request_mode(mode)

    def _publish_faults(self) -> None:
        self.pub_faults.publish(String(data=",".join(self.world.faults.names())))

    # -- output -----------------------------------------------------------

    def _stamp(self, utc_ms: int):
        msg_time = rclpy.time.Time(nanoseconds=utc_ms * 1_000_000)
        return msg_time.to_msg()

    def _tick(self) -> None:
        self.world.step(self._dt * self.time_scale)
        snap = self.world.snapshot()
        stamp = self._stamp(snap.utc_ms)
        v = snap.vessel

        fix = NavSatFix()
        fix.header.stamp = stamp
        fix.header.frame_id = "gps"
        fix.status.status = (
            NavSatStatus.STATUS_FIX if v.gnss_fix_type >= 3 else NavSatStatus.STATUS_NO_FIX
        )
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude, fix.longitude, fix.altitude = v.lat, v.lon, v.alt
        # Diagonal covariance from HDOP; approximate but honest about scale.
        sigma = v.hdop * 2.5
        fix.position_covariance = [
            sigma**2, 0.0, 0.0, 0.0, sigma**2, 0.0, 0.0, 0.0, (sigma * 2) ** 2
        ]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.pub_fix.publish(fix)

        self.pub_hdg.publish(Float64(data=float(v.heading_deg)))

        vel = TwistStamped()
        vel.header.stamp = stamp
        vel.header.frame_id = "map"
        cog = math.radians(v.cog_deg)
        vel.twist.linear.x = v.sog_ms * math.sin(cog)   # east
        vel.twist.linear.y = v.sog_ms * math.cos(cog)   # north
        self.pub_vel.publish(vel)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "base_link"
        qx, qy, qz, qw = _quaternion_from_euler(
            math.radians(v.roll_deg),
            math.radians(v.pitch_deg),
            math.radians(90.0 - v.heading_deg),  # compass -> ENU yaw
        )
        imu.orientation.x, imu.orientation.y = qx, qy
        imu.orientation.z, imu.orientation.w = qz, qw
        self.pub_imu.publish(imu)

        b = snap.battery
        batt = BatteryState()
        batt.header.stamp = stamp
        batt.voltage = float(b.voltage)
        batt.current = float(-b.current)  # discharge is negative, per the message
        batt.percentage = float(b.state_of_charge)
        batt.capacity = float(b.remaining_wh / max(1.0, b.voltage))
        batt.present = True
        self.pub_batt.publish(batt)

        if snap.lidar is not None:
            scan = LaserScan()
            scan.header.stamp = self._stamp(snap.lidar.utc_ms)
            scan.header.frame_id = "laser"
            scan.angle_min = 0.0
            scan.angle_increment = math.radians(snap.lidar.angle_increment_deg)
            scan.angle_max = scan.angle_increment * (len(snap.lidar.ranges_m) - 1)
            scan.range_min = self.world.cfg.lidar.min_range_m
            scan.range_max = self.world.cfg.lidar.max_range_m
            scan.scan_time = 1.0 / max(0.1, snap.lidar.rotation_hz)
            scan.time_increment = scan.scan_time / max(1, len(snap.lidar.ranges_m))
            # An absent return is infinity, per the LaserScan contract. Never 0:
            # zero is a measurement and would read as an obstacle on the hull.
            scan.ranges = [
                float("inf") if r is None else float(r) for r in snap.lidar.ranges_m
            ]
            self.pub_scan.publish(scan)

        p = snap.pico
        pico = PicoStatus()
        pico.header.stamp = stamp
        pico.mode = int(p.mode)
        pico.armed = bool(p.armed)
        pico.estop_latched = bool(p.estop_latched)
        pico.relay_states = list(p.relay_states)
        pico.esc_status = list(p.esc_status)
        pico.rc_link_ok = bool(p.rc_link_ok)
        pico.rc_channel8_raw_pct = int(p.rc_channel8_raw_pct)
        pico.battery_voltage = float(b.voltage)
        pico.battery_current = float(b.current)
        self.pub_pico.publish(pico)

        est = evaluate_heading(
            heading_deg=v.heading_deg,
            source=(
                SOURCE_GNSS_COMPASS
                if self.world.cfg.vessel.heading_source == HEADING_SOURCE_GNSS
                else SOURCE_MAGNETOMETER
            ),
            cog_deg=v.cog_deg,
            sog_ms=v.sog_ms,
            reported_accuracy_deg=v.heading_accuracy_deg,
            source_valid=v.heading_valid,
        )
        hs = HeadingStatus()
        hs.header.stamp = stamp
        hs.heading_deg = float(est.heading_deg)
        hs.source = est.source
        hs.valid = bool(est.valid)
        hs.accuracy_deg = float(est.accuracy_deg)
        hs.cog_deg = float(est.cog_deg)
        hs.sog_ms = float(est.sog_ms)
        hs.divergence_deg = float(0.0 if math.isnan(est.divergence_deg) else est.divergence_deg)
        hs.divergence_meaningful = bool(est.divergence_meaningful)
        self.pub_heading.publish(hs)

    def _publish_diagnostics(self) -> None:
        snap = self.world.snapshot()
        arr = DiagnosticArray()
        arr.header.stamp = self._stamp(snap.utc_ms)

        st = DiagnosticStatus()
        st.name = "asket_sim: simulated world"
        st.hardware_id = "sim"
        st.level = DiagnosticStatus.WARN if snap.active_faults else DiagnosticStatus.OK
        st.message = (
            "faults active: " + ", ".join(snap.active_faults)
            if snap.active_faults
            else "simulating normally"
        )
        st.values = [
            KeyValue(key="sim_time_s", value=f"{snap.sim_time_s:.1f}"),
            KeyValue(key="distance_travelled_m", value=f"{snap.vessel.distance_travelled_m:.1f}"),
            KeyValue(key="seabed_depth_m", value=f"{snap.seabed_depth_m:.1f}"),
            KeyValue(key="disk_free_gb", value=f"{snap.disk_free_bytes / 1024**3:.1f}"),
            KeyValue(key="available_faults", value=",".join(sorted(FAULTS))),
        ]
        arr.status.append(st)
        self.pub_diag.publish(arr)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
