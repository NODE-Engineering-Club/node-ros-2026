"""ROS 2 node: record a mission.

A thin adapter. The file layout, the manifest, the checksums, the disk-full
handling and the export verification all live in ``mission_recorder.core`` and
are tested there without ROS.

What this node adds is the subscriptions: the raw sonar stream, the trajectory
at 10 Hz, diagnostics at 1 Hz, and events. Which is to say — it is the thing
that makes the two halves of a survey land on the disk together, so they can be
merged afterwards.
"""

from __future__ import annotations

import math

import rclpy
from asket_common.heading import SOURCE_NONE, evaluate_heading
from asket_interfaces.msg import HeadingStatus, MissionState, PicoStatus, SonarStatus
from asket_interfaces.srv import (
    DeleteMission,
    ExportMission,
    ListMissions,
    StartMission,
    StopMission,
)
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import UInt8MultiArray

from mission_recorder.core.export import (
    delete_mission,
    detect_destinations,
    export_mission,
)
from mission_recorder.core.mission import (
    STATE_ERROR,
    STATE_IDLE,
    STATE_RECORDING,
    STATE_STOPPING,
    MissionRecorder,
    RecorderConfig,
    list_missions,
)

SENSOR_QOS = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

_STATE_VALUES = {
    STATE_IDLE: MissionState.IDLE,
    STATE_RECORDING: MissionState.RECORDING,
    STATE_STOPPING: MissionState.STOPPING,
    STATE_ERROR: MissionState.ERROR,
}


class MissionRecorderNode(Node):
    def __init__(self) -> None:
        super().__init__("mission_recorder")

        self.declare_parameter("missions_root", "/data/missions")
        self.declare_parameter("min_free_bytes", 1024**3)
        self.declare_parameter("flush_interval_s", 1.0)
        self.declare_parameter("trajectory_rate_hz", 10.0)
        self.declare_parameter("diagnostics_rate_hz", 1.0)
        self.declare_parameter("record_rosbag", False)
        self.declare_parameter("fast_path_globs", ["/media/*", "/mnt/usb*"])
        self.declare_parameter("exclude_roots", ["/data", "/"])

        self.recorder = MissionRecorder(
            RecorderConfig(
                missions_root=self.get_parameter("missions_root").value,
                min_free_bytes=int(self.get_parameter("min_free_bytes").value),
                flush_interval_s=float(self.get_parameter("flush_interval_s").value),
                record_rosbag=bool(self.get_parameter("record_rosbag").value),
            )
        )

        self._fix: NavSatFix | None = None
        self._vel: TwistStamped | None = None
        self._imu: Imu | None = None
        self._heading: HeadingStatus | None = None
        self._sonar: SonarStatus | None = None
        self._pico: PicoStatus | None = None
        self._last_mode: int | None = None

        self.create_subscription(NavSatFix, "/mavros/global_position/global",
                                 self._on_fix, SENSOR_QOS)
        self.create_subscription(TwistStamped, "/mavros/global_position/raw/gps_vel",
                                 self._on_vel, SENSOR_QOS)
        self.create_subscription(Imu, "/mavros/imu/data", self._on_imu, SENSOR_QOS)
        self.create_subscription(HeadingStatus, "/asket/heading", self._on_heading, 10)
        self.create_subscription(SonarStatus, "/sonar/status", self._on_sonar, 10)
        self.create_subscription(PicoStatus, "/pico/status", self._on_pico, 10)
        self.create_subscription(DiagnosticArray, "/diagnostics", self._on_diagnostics, 10)
        # The unmodified Ping Protocol stream, republished by omniscan_bridge
        # for exactly this purpose. Anything reinterpreted before it reaches
        # disk is something a post-mission tool cannot reinterpret differently.
        self.create_subscription(UInt8MultiArray, "/sonar/raw", self._on_sonar_raw, SENSOR_QOS)

        self.pub_state = self.create_publisher(MissionState, "/mission/state", 10)

        self.create_service(StartMission, "~/start_mission", self._on_start)
        self.create_service(StopMission, "~/stop_mission", self._on_stop)
        self.create_service(ListMissions, "~/list_missions", self._on_list)
        self.create_service(ExportMission, "~/export_mission", self._on_export)
        self.create_service(DeleteMission, "~/delete_mission", self._on_delete)

        self.create_timer(1.0 / float(self.get_parameter("trajectory_rate_hz").value),
                          self._write_trajectory)
        self.create_timer(1.0 / float(self.get_parameter("diagnostics_rate_hz").value),
                          self._write_diagnostics)
        self.create_timer(0.5, self._publish_state)

        self.get_logger().info(f"missions root: {self.recorder.cfg.missions_root}")

    # -- clock ------------------------------------------------------------

    def _utc_ms(self) -> int:
        return self.get_clock().now().nanoseconds // 1_000_000

    # -- inputs -----------------------------------------------------------

    def _on_fix(self, msg): self._fix = msg
    def _on_vel(self, msg): self._vel = msg
    def _on_imu(self, msg): self._imu = msg
    def _on_heading(self, msg): self._heading = msg
    def _on_sonar(self, msg): self._sonar = msg

    def _on_pico(self, msg: PicoStatus) -> None:
        if self._last_mode is not None and msg.mode != self._last_mode:
            self.recorder.write_event(
                self._utc_ms(), "mode_changed",
                {"from": int(self._last_mode), "to": int(msg.mode)},
            )
        self._last_mode = msg.mode
        self._pico = msg

    def _on_sonar_raw(self, msg: UInt8MultiArray) -> None:
        self.recorder.write_sonar(bytes(msg.data))

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        if not self.recorder.recording:
            return
        for status in msg.status:
            if status.level:
                self.recorder.write_event(
                    self._utc_ms(), "diagnostic",
                    {"name": status.name, "level": int(status.level),
                     "message": status.message},
                )

    # -- writing ----------------------------------------------------------

    def _write_trajectory(self) -> None:
        if not self.recorder.recording or self._fix is None:
            return
        self.recorder.write_trajectory(self._trajectory_record())

    def _trajectory_record(self) -> dict:
        fix, vel, imu, heading = self._fix, self._vel, self._imu, self._heading

        cog = sog = 0.0
        if vel is not None:
            east, north = vel.twist.linear.x, vel.twist.linear.y
            sog = math.hypot(east, north)
            if sog > 0.05:
                cog = math.degrees(math.atan2(east, north)) % 360.0

        roll = pitch = 0.0
        if imu is not None:
            q = imu.orientation
            roll = math.degrees(math.atan2(
                2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)))
            pitch = math.degrees(math.asin(
                max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))))

        estimate = evaluate_heading(
            heading_deg=heading.heading_deg if heading else None,
            source=heading.source if heading else SOURCE_NONE,
            cog_deg=cog, sog_ms=sog,
            reported_accuracy_deg=heading.accuracy_deg if heading else None,
            source_valid=bool(heading.valid) if heading else False,
        )

        return {
            "utc_ms": self._utc_ms(),
            "lat": fix.latitude, "lon": fix.longitude, "alt": fix.altitude,
            "heading_deg": estimate.heading_deg,
            "heading_source": estimate.source,
            "heading_valid": estimate.valid,
            "cog_deg": cog, "sog_ms": sog,
            "roll_deg": roll, "pitch_deg": pitch,
            "gnss_fix_type": int(fix.status.status) + 3,
            "num_sats": 0,
            "hdop": math.sqrt(fix.position_covariance[0]) / 2.5
            if fix.position_covariance[0] > 0 else float("nan"),
        }

    def _write_diagnostics(self) -> None:
        if not self.recorder.recording:
            return
        sonar = self._sonar
        self.recorder.write_diagnostics({
            "utc_ms": self._utc_ms(),
            # Logged every second. If the sonar's clock does drift, the damage
            # is at least visible in the recording rather than invisible in the
            # data.
            "clock_offset_ms": int(sonar.clock_offset_ms) if sonar else None,
            "sonar_connected": bool(sonar.connected) if sonar else None,
            "sonar_ping_rate_hz": float(sonar.actual_ping_rate_hz) if sonar else None,
            "packet_loss_ratio": float(sonar.packet_loss_ratio) if sonar else None,
            "heading_valid": bool(self._heading.valid) if self._heading else None,
            "battery_voltage": float(self._pico.battery_voltage) if self._pico else None,
            "mode": int(self._pico.mode) if self._pico else None,
        })

    def _publish_state(self) -> None:
        now_s = self.get_clock().now().nanoseconds / 1e9
        status = self.recorder.tick(now_s, self._utc_ms())

        msg = MissionState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = _STATE_VALUES.get(status.state, MissionState.IDLE)
        msg.mission_name = status.name
        msg.mission_dir = status.mission_dir
        msg.elapsed_s = float(status.elapsed_s)
        msg.bytes_written = int(status.bytes_written)
        msg.disk_free_bytes = int(status.disk_free_bytes)
        msg.estimated_remaining_s = (
            0.0 if math.isinf(status.estimated_remaining_s)
            else float(status.estimated_remaining_s)
        )
        msg.error_message = status.error_message
        self.pub_state.publish(msg)

    # -- services ---------------------------------------------------------

    def _config_snapshot(self) -> dict:
        return {
            "recorder": {
                "missions_root": str(self.recorder.cfg.missions_root),
                "trajectory_rate_hz": float(self.get_parameter("trajectory_rate_hz").value),
            },
            "sonar": {
                "range_setting_m": float(self._sonar.range_setting_m) if self._sonar else None,
                "gain": int(self._sonar.gain_setting) if self._sonar else None,
                "speed_of_sound": float(self._sonar.speed_of_sound) if self._sonar else None,
            },
            "heading_source": self._heading.source if self._heading else None,
        }

    def _on_start(self, request, response):
        status = self.recorder.start(
            request.name, self._utc_ms(),
            config_snapshot=self._config_snapshot(),
            record_rosbag=bool(request.record_rosbag),
        )
        if status.state == STATE_RECORDING and self._fix is not None:
            # Before any sonar byte, so the trajectory brackets the stream and
            # every ping can be interpolated afterwards.
            self.recorder.write_trajectory(self._trajectory_record())
        response.success = status.state == STATE_RECORDING
        response.mission_dir = status.mission_dir
        response.message = status.error_message or f"recording to {status.mission_dir}"
        return response

    def _on_stop(self, request, response):
        if not self.recorder.recording:
            response.success = False
            response.message = "no mission is recording"
            return response
        if self._fix is not None:
            self.recorder.write_trajectory(self._trajectory_record())
        status = self.recorder.stop(self._utc_ms())
        response.success = True
        response.mission_dir = status.mission_dir
        response.message = f"stopped after {status.elapsed_s:.0f} s"
        return response

    def _on_list(self, request, response):
        import json

        missions = list_missions(self.recorder.cfg.missions_root)
        response.names = [m.name for m in missions]
        response.manifests_json = [json.dumps(m.to_dict()) for m in missions]
        return response

    def _on_export(self, request, response):
        result = export_mission(
            request.name, request.destination,
            recording=self.recorder.recording,
            allowed_destinations=self._destinations(),
        )
        response.success = result.success
        response.message = result.message
        response.checksum_status = result.checksum_status
        return response

    def _on_delete(self, request, response):
        if not request.confirm:
            response.success = False
            response.message = "delete requires an explicit confirmation"
            return response
        ok, message = delete_mission(
            request.name,
            recording_dir=self.recorder.status.mission_dir
            if self.recorder.recording else None,
        )
        response.success = ok
        response.message = message
        return response

    def _destinations(self):
        return detect_destinations(
            globs=list(self.get_parameter("fast_path_globs").value),
            exclude_roots=list(self.get_parameter("exclude_roots").value),
        )

    def destroy_node(self) -> bool:
        if self.recorder.recording:
            # Never leave a mission half-written because a node was shut down.
            self.recorder.stop(self._utc_ms())
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionRecorderNode()
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
