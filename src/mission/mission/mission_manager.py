"""Mission manager — idle until /mission/start is called via service."""

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from njord_msgs.srv import StartMission
from rclpy.action import ActionClient
from rclpy.node import Node
from robot_localization.srv import FromLL
from std_srvs.srv import Trigger


class MissionManager(Node):
    def __init__(self):
        super().__init__("mission_manager")

        self._nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._fromll = self.create_client(FromLL, "/fromLL")

        self.create_service(StartMission, "/mission/start", self._start_cb)
        self.create_service(Trigger, "/mission/abort", self._abort_cb)

        self._waypoints = []
        self._idx = 0
        self._navigating = False
        self._active = False
        self._nav_handle = None

        self.get_logger().info("Mission manager ready — call /mission/start to begin")

    # ── Service handlers ──────────────────────────────────────────────────────

    def _start_cb(self, req, res):
        if len(req.waypoints) == 0:
            res.success = False
            res.message = "waypoints must be non-empty"
            return res

        if self._active:
            res.success = False
            res.message = "Mission already running — call /mission/abort first"
            return res

        self._waypoints = req.waypoints
        self._idx = 0
        self._navigating = False
        self._active = True
        self.get_logger().info(f"Mission started: {len(self._waypoints)} waypoints")
        self.create_timer(1.0, self._tick)
        res.success = True
        res.message = f"{len(self._waypoints)} waypoints accepted"
        return res

    def _abort_cb(self, req, res):
        if not self._active:
            res.success = False
            res.message = "No mission running"
            return res
        self._cancel()
        res.success = True
        res.message = "Mission aborted"
        return res

    # ── Mission tick ──────────────────────────────────────────────────────────

    def _tick(self):
        if not self._active or self._navigating:
            return
        if self._idx >= len(self._waypoints):
            self.get_logger().info("Mission complete")
            self._active = False
            return
        if not self._nav.wait_for_server(timeout_sec=0.5):
            self.get_logger().info("Waiting for Nav2...", throttle_duration_sec=5.0)
            return
        if not self._fromll.wait_for_service(timeout_sec=0.5):
            self.get_logger().info("Waiting for /fromLL...", throttle_duration_sec=5.0)
            return
        self._send_next()

    def _send_next(self):
        wp = self._waypoints[self._idx]
        self.get_logger().info(f"WP {self._idx + 1}/{len(self._waypoints)}: ({wp.latitude}, {wp.longitude})")
        self._navigating = True
        req = FromLL.Request()
        req.ll_point = wp
        req.ll_point.altitude = 0.0
        self._fromll.call_async(req).add_done_callback(self._on_ll)

    def _on_ll(self, future):
        result = future.result()
        if result is None:
            self.get_logger().error("fromLL service call failed")
            self._navigating = False
            return
        pt = result.map_point
        self.get_logger().info(f"fromLL map_point: ({pt.x:.4f}, {pt.y:.4f})")
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
        self._nav_handle = handle
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        self._nav_handle = None
        self._idx += 1
        self._navigating = False

    def _cancel(self):
        self._active = False
        self._navigating = False
        if self._nav_handle is not None:
            self._nav_handle.cancel_goal_async()
            self._nav_handle = None
        self.get_logger().info("Mission aborted")


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
