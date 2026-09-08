"""ROS 2 node: Cerulean Omniscan 3D -> ROS 2.

A thin adapter. Framing, resynchronisation, packet-loss accounting, the
sensor-to-vessel conversion and the health maths all live in
``omniscan_bridge.core`` and are tested there without ROS, a socket or a sonar.

Two things this node must never do:

* **Block the executor on a socket read.** The transport runs its own reader
  thread; this node drains a queue from a timer. If the sonar goes silent the
  node keeps running and reports it, rather than freezing the process.
* **Georeference.** Points are published in the vessel frame. Position and
  heading are recorded alongside the raw stream by ``mission_recorder`` and
  merged afterwards (brief, section 5). Fusing here would make the point cloud
  depend on GNSS availability, so a fix dropout would corrupt the data instead
  of merely annotating it.
"""

from __future__ import annotations

import math
import struct

import rclpy
from asket_interfaces.msg import SonarStatus
from asket_interfaces.srv import SetPingParameters
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2, PointField
from std_msgs.msg import Header
from std_msgs.msg import UInt8MultiArray
from std_srvs.srv import Trigger

from omniscan_bridge.core import ping_protocol as pp
from omniscan_bridge.core.geometry import SonarMounting, point_set_to_vessel_frame
from omniscan_bridge.core.parser import PingParser
from omniscan_bridge.core.status import (
    CLOCK_OFFSET_ALARM_MS,
    CLOCK_OFFSET_WARN_MS,
    SonarHealthTracker,
)
from omniscan_bridge.core.transport import TcpTransport, UdpTransport

SENSOR_QOS = QoSProfile(depth=2, reliability=QoSReliabilityPolicy.BEST_EFFORT)

#: PointCloud2 layout: x, y, z as float32, then intensity. 16 bytes per point.
_POINT_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
]
_POINT_STEP = 16
_PACK_POINT = struct.Struct("<ffff")


class OmniscanBridge(Node):
    def __init__(self) -> None:
        super().__init__("omniscan_bridge")

        self.declare_parameter("host", "192.168.2.92")
        self.declare_parameter("port", pp.DEFAULT_PORT)
        self.declare_parameter("transport", "udp")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("ntp_url", "")
        self.declare_parameter("range_m", 30.0)
        self.declare_parameter("gain", 4)
        self.declare_parameter("ping_rate_hz", 5.0)
        self.declare_parameter("max_publish_rate_hz", 20.0)
        self.declare_parameter("drain_rate_hz", 100.0)
        # Mounting geometry. PROVISIONAL until measured — open question Q2.
        self.declare_parameter("mounting.tilt_deg", 35.0)
        self.declare_parameter("mounting.yaw_deg", 0.0)
        self.declare_parameter("mounting.pitch_deg", 0.0)
        self.declare_parameter("mounting.lever_x_m", -0.20)
        self.declare_parameter("mounting.lever_y_m", -0.35)
        self.declare_parameter("mounting.lever_z_m", -0.15)
        self.declare_parameter("mounting.side", "starboard")
        self.declare_parameter("use_vessel_attitude", True)
        # Republish the unmodified stream for mission_recorder. This is what
        # ends up in sonar_raw.bin, and it must be the bytes as they arrived:
        # anything reinterpreted before it reaches disk is something a
        # post-mission tool cannot reinterpret differently later.
        self.declare_parameter("publish_raw", True)

        self.frame_id = self.get_parameter("frame_id").value
        self.mounting = SonarMounting(
            tilt_deg=self.get_parameter("mounting.tilt_deg").value,
            yaw_deg=self.get_parameter("mounting.yaw_deg").value,
            pitch_deg=self.get_parameter("mounting.pitch_deg").value,
            lever_x_m=self.get_parameter("mounting.lever_x_m").value,
            lever_y_m=self.get_parameter("mounting.lever_y_m").value,
            lever_z_m=self.get_parameter("mounting.lever_z_m").value,
            side=self.get_parameter("mounting.side").value,
        )

        self.parser = PingParser()
        self.health = SonarHealthTracker()
        self.health.on_parameters_commanded(
            self.get_parameter("range_m").value,
            int(self.get_parameter("gain").value),
            self.get_parameter("ping_rate_hz").value,
        )

        transport_cls = (
            TcpTransport
            if str(self.get_parameter("transport").value).lower() == "tcp"
            else UdpTransport
        )
        self.transport = transport_cls(
            host=self.get_parameter("host").value,
            port=int(self.get_parameter("port").value),
        )

        self.pub_points = self.create_publisher(PointCloud2, "/sonar/points", SENSOR_QOS)
        self.pub_status = self.create_publisher(SonarStatus, "/sonar/status", 10)
        self.pub_attitude = self.create_publisher(Imu, "/sonar/attitude", SENSOR_QOS)
        self.pub_diag = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.pub_raw = (
            self.create_publisher(UInt8MultiArray, "/sonar/raw", SENSOR_QOS)
            if self.get_parameter("publish_raw").value
            else None
        )

        # Vessel attitude at the instant of the ping. Roll is what turns a flat
        # seabed into an apparent slope, so it is corrected here rather than
        # left for post-processing.
        self._roll_deg = 0.0
        self._pitch_deg = 0.0
        if self.get_parameter("use_vessel_attitude").value:
            self.create_subscription(Imu, "/mavros/imu/data", self._on_imu, SENSOR_QOS)

        self.srv_params = self.create_service(
            SetPingParameters, "~/set_ping_parameters", self._on_set_parameters
        )
        self.srv_start = self.create_service(Trigger, "~/start_pinging", self._on_start)
        self.srv_stop = self.create_service(Trigger, "~/stop_pinging", self._on_stop)

        self._pinging = True
        self._min_publish_interval_ns = int(
            1e9 / max(0.1, self.get_parameter("max_publish_rate_hz").value)
        )
        self._last_publish_ns = 0

        self.transport.start()
        self.create_timer(1.0 / self.get_parameter("drain_rate_hz").value, self._drain)
        self.create_timer(1.0, self._publish_status)
        # Configure the device once it answers, and again after any reconnect.
        self._configured_for_reconnects = -1
        self.create_timer(1.0, self._configure_if_needed)

        self.get_logger().info(
            f"connecting to Omniscan 3D at {self.transport.host}:{self.transport.port} "
            f"over {transport_cls.__name__.replace('Transport', '').lower()}"
        )

    # -- device configuration ---------------------------------------------

    def _configure_if_needed(self) -> None:
        """Send NTP and ping parameters on first contact and after a reconnect.

        A device that reboots mid-mission comes back with defaults. Silently
        surveying at the wrong range for the second half is exactly the kind of
        failure nobody notices until the data is opened back home.
        """
        if not self.transport.connected:
            return
        if self.transport.stats.reconnects == self._configured_for_reconnects:
            return
        self._configured_for_reconnects = self.transport.stats.reconnects
        self._send_ntp_url()
        self._send_ping_parameters(
            self.health.range_setting_m,
            self.health.gain_setting,
            self.health.commanded_ping_rate_hz,
        )

    def _send_ntp_url(self) -> None:
        url = self.get_parameter("ntp_url").value
        if not url:
            self.get_logger().warn(
                "no ntp_url configured — the sonar's clock will not be disciplined "
                "by the Jetson, and post-mission fusion depends on it"
            )
            return
        if self.transport.send(pp.encode_set_ntp_url(url)):
            self.health.ntp_url_sent = url
            self.get_logger().info(f"pointed the sonar at NTP server {url}")

    def _send_ping_parameters(self, range_m: float, gain: int, rate_hz: float) -> bool:
        ok = self.transport.send(pp.encode_set_ping_parameters(range_m, gain, rate_hz))
        if ok:
            self.health.on_parameters_commanded(range_m, gain, rate_hz)
        return ok

    # -- inputs -----------------------------------------------------------

    def _on_imu(self, msg: Imu) -> None:
        q = msg.orientation
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self._roll_deg = math.degrees(math.atan2(sinr, cosr))
        sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
        self._pitch_deg = math.degrees(math.asin(sinp))

    def _drain(self) -> None:
        """Move bytes from the reader thread into ROS. Never blocks."""
        for chunk in self.transport.read():
            if self.pub_raw is not None:
                # Published BEFORE parsing, so a chunk the parser rejects still
                # reaches the recording. A frame we could not decode is exactly
                # the frame somebody will want to look at afterwards.
                self.pub_raw.publish(UInt8MultiArray(data=list(chunk)))
            for frame, message in self.parser.feed_and_decode(chunk):
                self._dispatch(frame, message)

    def _dispatch(self, frame: pp.Frame, message) -> None:
        if isinstance(message, pp.PointSet):
            self._on_point_set(message)
        elif isinstance(message, pp.AttitudeReport):
            self._on_attitude(message)
        # END_PING_INFO carries no information the point set does not; it is
        # parsed for completeness and for the frame count, not consumed.

    def _on_point_set(self, ps: pp.PointSet) -> None:
        valid = sum(1 for p in ps.points if p.pt_type != 0)
        self.health.on_point_set(
            utc_msec=ps.utc_msec,
            num_points=len(ps.points),
            num_valid=valid,
            speed_of_sound=ps.speed_of_sound,
        )

        if ps.length_mismatch:
            # Throttled: if the layout assumption is wrong this fires every ping.
            self.get_logger().warn(
                "point set declared a different point count than it carried — "
                "verify the OS3D_POINT_SET layout against Cerulean sample data "
                "(docs/open_questions.md Q8)",
                throttle_duration_sec=30.0,
            )

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_publish_ns < self._min_publish_interval_ns:
            return  # rate limit: the health topic still sees every ping
        self._last_publish_ns = now_ns

        points = point_set_to_vessel_frame(
            ps, self.mounting, roll_deg=self._roll_deg, pitch_deg=self._pitch_deg
        )
        self.pub_points.publish(self._to_point_cloud(points, ps.utc_msec))

    def _on_attitude(self, att: pp.AttitudeReport) -> None:
        self.health.on_attitude(att.pitch_deg, att.roll_deg)

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "sonar"
        half_pitch = math.radians(att.pitch_deg) / 2.0
        half_roll = math.radians(att.roll_deg) / 2.0
        cp, sp = math.cos(half_pitch), math.sin(half_pitch)
        cr, sr = math.cos(half_roll), math.sin(half_roll)
        msg.orientation.x = sr * cp
        msg.orientation.y = cr * sp
        msg.orientation.z = -sr * sp
        msg.orientation.w = cr * cp
        # The sonar reports attitude but no rates or accelerations; -1 in the
        # first covariance element is the REP-145 way of saying "not provided".
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0
        self.pub_attitude.publish(msg)

    # -- outputs ----------------------------------------------------------

    def _to_point_cloud(self, points, utc_msec: int) -> PointCloud2:
        msg = PointCloud2()
        # Stamp with the SONAR's timestamp, not ours. It is the key everything
        # is paired on afterwards, and rewriting it here would throw away the
        # only evidence of a clock problem.
        msg.header = Header()
        msg.header.stamp = rclpy.time.Time(nanoseconds=utc_msec * 1_000_000).to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = len(points)
        msg.fields = _POINT_FIELDS
        msg.is_bigendian = False
        msg.point_step = _POINT_STEP
        msg.row_step = _POINT_STEP * len(points)
        msg.is_dense = True
        buf = bytearray(_POINT_STEP * len(points))
        for i, (x, y, z, power) in enumerate(points):
            _PACK_POINT.pack_into(buf, i * _POINT_STEP, x, y, z, power)
        msg.data = bytes(buf)
        return msg

    def _current_health(self):
        stats = self.parser.stats
        return self.health.health(
            connected=self.transport.connected,
            packet_loss_ratio=stats.packet_loss_ratio,
            packets_parsed=stats.frames_parsed,
            checksum_errors=stats.checksum_errors,
            bytes_discarded=stats.bytes_discarded,
            seconds_since_data=self.transport.seconds_since_data,
        )

    def _publish_status(self) -> None:
        h = self._current_health()

        msg = SonarStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.connected = h.connected
        msg.actual_ping_rate_hz = float(h.actual_ping_rate_hz)
        msg.commanded_ping_rate_hz = float(h.commanded_ping_rate_hz)
        msg.points_per_ping = int(h.points_per_ping)
        msg.speed_of_sound = float(h.speed_of_sound)
        msg.range_setting_m = float(h.range_setting_m)
        msg.gain_setting = int(h.gain_setting)
        msg.packet_loss_ratio = float(h.packet_loss_ratio)
        msg.clock_offset_ms = int(h.clock_offset_ms)
        msg.pitch_deg = float(h.pitch_deg)
        msg.roll_deg = float(h.roll_deg)
        msg.packets_parsed = int(h.packets_parsed)
        msg.checksum_errors = int(h.checksum_errors)
        msg.bytes_discarded = int(h.bytes_discarded)
        self.pub_status.publish(msg)

        self.pub_diag.publish(self._diagnostics(h))

    def _diagnostics(self, h) -> DiagnosticArray:
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()

        st = DiagnosticStatus()
        st.name = "omniscan_bridge: sonar"
        st.hardware_id = f"{self.transport.host}:{self.transport.port}"

        # Plain language, and say what to DO about it. "sonar: ERROR" on a
        # Namibian beach costs an hour; "no packets for 12 s — check the
        # Ethernet cable" costs a minute.
        if not h.connected:
            st.level = DiagnosticStatus.ERROR
            st.message = (
                f"no data for {h.seconds_since_data:.0f} s — check the sonar's "
                "Ethernet cable and power"
            )
        elif h.clock_compromised:
            st.level = DiagnosticStatus.ERROR
            st.message = (
                f"sonar clock is {h.clock_offset_ms} ms from the Jetson's — data "
                "recorded now cannot be georeferenced; check the NTP server"
            )
        elif not h.clock_ok:
            st.level = DiagnosticStatus.WARN
            st.message = (
                f"sonar clock drifting ({h.clock_offset_ms} ms); "
                f"warn at {CLOCK_OFFSET_WARN_MS} ms, data unusable past "
                f"{CLOCK_OFFSET_ALARM_MS} ms"
            )
        elif h.packet_loss_ratio > 0.05:
            st.level = DiagnosticStatus.WARN
            st.message = (
                f"losing {h.packet_loss_ratio * 100:.0f}% of pings — check the "
                "network path between the sonar and the Jetson"
            )
        elif not h.ping_rate_ok:
            st.level = DiagnosticStatus.WARN
            st.message = (
                f"pinging at {h.actual_ping_rate_hz:.1f} Hz, commanded "
                f"{h.commanded_ping_rate_hz:.1f} Hz — the range setting may be "
                "too long for this rate"
            )
        else:
            st.level = DiagnosticStatus.OK
            st.message = (
                f"{h.actual_ping_rate_hz:.1f} Hz, {h.valid_points_per_ping} of "
                f"{h.points_per_ping} points returning"
            )

        st.values = [
            KeyValue(key="connected", value=str(h.connected)),
            KeyValue(key="ping_rate_hz", value=f"{h.actual_ping_rate_hz:.2f}"),
            KeyValue(key="points_per_ping", value=str(h.points_per_ping)),
            KeyValue(key="valid_points_per_ping", value=str(h.valid_points_per_ping)),
            KeyValue(key="speed_of_sound_ms", value=f"{h.speed_of_sound:.1f}"),
            KeyValue(key="packet_loss_ratio", value=f"{h.packet_loss_ratio:.4f}"),
            KeyValue(key="clock_offset_ms", value=str(h.clock_offset_ms)),
            KeyValue(key="checksum_errors", value=str(h.checksum_errors)),
            KeyValue(key="bytes_discarded", value=str(h.bytes_discarded)),
            KeyValue(key="ntp_url_sent", value=h.ntp_url_sent or "(none)"),
            KeyValue(key="reconnects", value=str(self.transport.stats.reconnects)),
            KeyValue(key="rx_chunks_dropped", value=str(self.transport.stats.chunks_dropped)),
        ]
        arr.status.append(st)
        return arr

    # -- services ---------------------------------------------------------

    def _on_set_parameters(self, request, response):
        if not self.transport.connected:
            response.success = False
            response.message = "sonar not connected"
            return response
        ok = self._send_ping_parameters(
            float(request.range_m), int(request.gain), float(request.ping_rate_hz)
        )
        response.success = ok
        response.message = (
            f"requested {request.range_m:.1f} m, gain {request.gain}, "
            f"{request.ping_rate_hz:.1f} Hz"
            if ok
            else "failed to send to the sonar"
        )
        return response

    def _on_start(self, request, response):
        self._pinging = True
        ok = self._send_ping_parameters(
            self.health.range_setting_m,
            self.health.gain_setting,
            self.health.commanded_ping_rate_hz,
        )
        response.success = ok
        response.message = "pinging" if ok else "failed to send to the sonar"
        return response

    def _on_stop(self, request, response):
        self._pinging = False
        # Rate zero is how the device is told to stop without closing the link.
        ok = self._send_ping_parameters(
            self.health.range_setting_m, self.health.gain_setting, 0.0
        )
        response.success = ok
        response.message = "stopped" if ok else "failed to send to the sonar"
        return response

    def destroy_node(self) -> bool:
        self.transport.stop()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OmniscanBridge()
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
