"""Mission manager — idle until /mission/start is called via service."""

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from njord_msgs.msg import MissionStatus
from njord_msgs.srv import StartMission
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from robot_localization.srv import FromLL
from std_srvs.srv import Trigger


class MissionManager(Node):
    def __init__(self):
        super().__init__("mission_manager")

        self._nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._fromll = self.create_client(FromLL, "/fromLL")

        self.create_service(StartMission, "/mission/start", self._start_cb)
        self.create_service(Trigger, "/mission/abort", self._abort_cb)

        status_qos = QoSProfile(depth=1)
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._status_pub = self.create_publisher(
            MissionStatus,
            "/mission/status",
            status_qos,
        )

        self._waypoints = []
        self._idx = 0
        self._navigating = False
        self._active = False
        self._nav_handle = None
        self._timer = None

        self._publish_status(
            MissionStatus.IDLE,
            "Mission manager ready",
        )

        self.get_logger().info(
            "Mission manager ready — call /mission/start to begin"
        )

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

        self._cleanup_timer()

        self._waypoints = req.waypoints
        self._idx = 0
        self._navigating = False
        self._active = True

        self._publish_status(
            MissionStatus.RUNNING,
            "Mission started",
        )

        self.get_logger().info(
            f"Mission started: {len(self._waypoints)} waypoints"
        )

        self._timer = self.create_timer(1.0, self._tick)

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
            self._cleanup_timer()

            self._publish_status(
                MissionStatus.SUCCEEDED,
                "Mission completed successfully",
            )
            return

        if not self._nav.wait_for_server(timeout_sec=0.5):
            self.get_logger().info(
                "Waiting for Nav2...",
                throttle_duration_sec=5.0,
            )
            return

        if not self._fromll.wait_for_service(timeout_sec=0.5):
            self.get_logger().info(
                "Waiting for /fromLL...",
                throttle_duration_sec=5.0,
            )
            return

        self._send_next()

    def _send_next(self):
        wp = self._waypoints[self._idx]

        self.get_logger().info(
            f"WP {self._idx + 1}/{len(self._waypoints)}: "
            f"({wp.latitude}, {wp.longitude})"
        )

        self._navigating = True

        req = FromLL.Request()
        req.ll_point = wp
        req.ll_point.altitude = 0.0

        self._fromll.call_async(req).add_done_callback(self._on_ll)

    def _on_ll(self, future):
        result = future.result()

        if result is None:
            self._fail_mission("fromLL service call failed")
            return

        pt = result.map_point

        self.get_logger().info(
            f"fromLL map_point: ({pt.x:.4f}, {pt.y:.4f})"
        )

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position = pt

        self._nav.send_goal_async(goal).add_done_callback(
            self._on_accepted
        )

    def _on_accepted(self, future):
        handle = future.result()

        if not handle.accepted:
            self._fail_mission("Nav2 goal rejected")
            return

        self._nav_handle = handle

        handle.get_result_async().add_done_callback(
            self._on_result
        )

    def _on_result(self, future):
        result = future.result()
        self._nav_handle = None
        self._navigating = False

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self._idx += 1

            self._publish_status(
                MissionStatus.RUNNING,
                f"Waypoint {self._idx}/{len(self._waypoints)} completed",
            )
            return

        if result.status == GoalStatus.STATUS_CANCELED:
            if self._active:
                self._fail_mission("Nav2 goal canceled unexpectedly")
            return

        if result.status == GoalStatus.STATUS_ABORTED:
            self._fail_mission("Nav2 goal aborted")
            return

        self._fail_mission(
            f"Nav2 goal failed with status {result.status}"
        )

    # ── Mission state helpers ─────────────────────────────────────────────────

    def _fail_mission(self, message):
        self.get_logger().error(message)

        self._active = False
        self._navigating = False
        self._nav_handle = None
        self._cleanup_timer()

        self._publish_status(
            MissionStatus.FAILED,
            message,
        )

    def _cancel(self):
        self._active = False
        self._navigating = False

        if self._nav_handle is not None:
            self._nav_handle.cancel_goal_async()
            self._nav_handle = None

        self._cleanup_timer()

        self._publish_status(
            MissionStatus.ABORTED,
            "Mission aborted",
        )

        self.get_logger().info("Mission aborted")

    def _publish_status(self, state, message):
        msg = MissionStatus()
        msg.state = state
        msg.message = message
        msg.current_waypoint = self._idx
        msg.total_waypoints = len(self._waypoints)

        self._status_pub.publish(msg)

    def _cleanup_timer(self):
        """Safely stops and destroys the tick timer if it exists."""
        if self._timer is not None:
            self._timer.cancel()
            self.destroy_timer(self._timer)
            self._timer = None


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


if __name__ == "__main__":
    main()