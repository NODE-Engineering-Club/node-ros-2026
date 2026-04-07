"""Sequences GPS waypoints through Nav2's NavigateToPose action."""

import rclpy
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from robot_localization.srv import FromLL

# (lat, lon) — replace with competition waypoints
WAYPOINTS = [
    (63.4305, 10.3951),
    (63.4310, 10.3955),
]


class MissionManager(Node):
    def __init__(self):
        super().__init__("mission_manager")
        self._nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._fromll = self.create_client(FromLL, "/fromLL")
        self._idx = 0
        self._navigating = False
        self.create_timer(1.0, self._tick)
        self.get_logger().info(f"Mission: {len(WAYPOINTS)} waypoints")

    def _tick(self):
        if self._navigating or self._idx >= len(WAYPOINTS):
            return
        if not self._nav.wait_for_server(timeout_sec=0.5):
            self.get_logger().info("Waiting for Nav2...", throttle_duration_sec=5.0)
            return
        if not self._fromll.wait_for_service(timeout_sec=0.5):
            self.get_logger().info("Waiting for /fromLL...", throttle_duration_sec=5.0)
            return
        self._send_next()

    def _send_next(self):
        lat, lon = WAYPOINTS[self._idx]
        self.get_logger().info(f"WP {self._idx + 1}/{len(WAYPOINTS)}: ({lat}, {lon})")
        self._navigating = True

        req = FromLL.Request()
        req.ll_point = GeoPoint(latitude=lat, longitude=lon, altitude=0.0)
        self._fromll.call_async(req).add_done_callback(self._on_ll)

    def _on_ll(self, future):
        pt = future.result().map_point
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position = pt
        self._nav.send_goal_async(goal).add_done_callback(self._on_accepted)

    def _on_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("Goal rejected")
            self._navigating = False
            return
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        self._idx += 1
        self._navigating = False
        if self._idx >= len(WAYPOINTS):
            self.get_logger().info("Mission complete")


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
