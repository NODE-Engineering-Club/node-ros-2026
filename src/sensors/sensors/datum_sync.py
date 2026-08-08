"""Anchors navsat_transform_node's GPS datum to ArduPilot's own home position
(/mavros/home_position/home), so GPS-derived waypoints (/fromLL) and the
robot's actual odom-frame position agree on where (0, 0) is.

Without this, ArduPilot's local_position/odom origin (set internally by the
FC's EKF at boot/arm — the same point as home for this vehicle) and
navsat_transform_node's auto-initialized datum (set from whatever GPS fix
arrives first after navsat_transform_node starts) anchor to two different
points — map and odom are published as identity in njord.launch.py on the
assumption they're globally GPS-anchored the same way, which only holds if
both share this origin.

(GPS_GLOBAL_ORIGIN / gp_origin would be the more "correct" EKF-origin source,
but this FC/firmware never publishes it even on explicit MAV_CMD_REQUEST_MESSAGE
— home_position is the reliable equivalent here.)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from mavros_msgs.msg import HomePosition
from robot_localization.srv import SetDatum


class DatumSync(Node):
    def __init__(self):
        super().__init__("datum_sync")
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._cli = self.create_client(SetDatum, "/datum")
        self._synced = False
        self.create_subscription(HomePosition, "/mavros/home_position/home", self._origin_cb, qos)

    def _origin_cb(self, msg):
        if self._synced:
            return
        self._synced = True
        if not self._cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("SetDatum service unavailable — datum not synced to FC origin")
            self._synced = False
            return
        req = SetDatum.Request()
        req.geo_pose.position = msg.geo
        req.geo_pose.orientation.w = 1.0  # local_position/odom is already true-ENU; no extra rotation
        self._cli.call_async(req).add_done_callback(self._on_result)

    def _on_result(self, future):
        future.result()
        self.get_logger().info("Datum synced to FC local-position origin — GPS waypoints and odom now share the same reference")


def main(args=None):
    rclpy.init(args=args)
    node = DatumSync()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
