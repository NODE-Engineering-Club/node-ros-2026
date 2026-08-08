"""Mission manager — idle until /mission/start is called via service."""

import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from njord_msgs.msg import MissionStatus
from njord_msgs.srv import SetBypassTarget, StartMission
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
        self.create_subscription(Odometry, "/odometry/filtered", self._odom_cb, 10)

        self.create_service(
            StartMission,
            "/mission/start",
            self._start_cb,
        )

        self.create_service(
            Trigger,
            "/mission/abort",
            self._abort_cb,
        )

        self.create_service(
            SetBypassTarget,
            "/mission/set_bypass_target",
            self._set_bypass_target_cb,
        )

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
        self._current_pose = None

        # Temporary bypass/replan state.
        self._bypass_target = None
        self._bypass_reason = ""
        self._bypass_pending = False
        self._bypass_active = False

        # True while we are intentionally cancelling the current mission goal
        # in order to insert a temporary bypass target.
        self._cancel_for_bypass = False

        self._publish_status(
            MissionStatus.IDLE,
            "Mission manager ready",
        )

        self.get_logger().info(
            "Mission manager ready — call /mission/start to begin"
        )

    def _odom_cb(self, msg):
        self._current_pose = msg.pose.pose

    # -------------------------------------------------------------------------
    # Service handlers
    # -------------------------------------------------------------------------

    def _start_cb(self, req, res):
        if len(req.waypoints) == 0:
            res.success = False
            res.message = "waypoints must be non-empty"
            return res

        if self._active:
            res.success = False
            res.message = (
                "Mission already running — call /mission/abort first"
            )
            return res

        self._cleanup_timer()

        self._waypoints = req.waypoints
        self._idx = 0

        self._navigating = False
        self._active = True

        self._nav_handle = None

        self._bypass_target = None
        self._bypass_reason = ""
        self._bypass_pending = False
        self._bypass_active = False
        self._cancel_for_bypass = False

        self._publish_status(
            MissionStatus.RUNNING,
            "Mission started",
        )

        self.get_logger().info(
            f"Mission started: {len(self._waypoints)} waypoints"
        )

        self._timer = self.create_timer(
            1.0,
            self._tick,
        )

        res.success = True
        res.message = f"{len(self._waypoints)} waypoints accepted"

        return res

    def _abort_cb(self, req, res):
        del req

        if not self._active:
            res.success = False
            res.message = "No mission running"
            return res

        self._cancel()

        res.success = True
        res.message = "Mission aborted"

        return res

    def _set_bypass_target_cb(self, req, res):
        if not self._active:
            res.success = False
            res.message = "No active mission"
            return res

        self._bypass_target = req.target
        self._bypass_reason = req.reason
        self._bypass_pending = True

        self.get_logger().info(
            "Bypass target requested: "
            f"({req.target.latitude}, {req.target.longitude}), "
            f"reason='{req.reason}'"
        )

        self._publish_status(
            MissionStatus.RUNNING,
            f"Bypass requested: {req.reason}",
        )

        # If Nav2 is currently navigating to a normal waypoint,
        # cancel that goal first. The result callback will recognize
        # that this cancellation is intentional and will not fail the mission.
        if (
            self._navigating
            and self._nav_handle is not None
            and not self._bypass_active
        ):
            self._cancel_for_bypass = True

            self.get_logger().info(
                "Cancelling active mission goal for bypass"
            )

            self._nav_handle.cancel_goal_async()

        res.success = True
        res.message = "Bypass target accepted"

        return res

    # -------------------------------------------------------------------------
    # Mission tick
    # -------------------------------------------------------------------------

    def _tick(self):
        if not self._active:
            return

        if self._navigating:
            return

        # Temporary bypass has priority over the normal mission waypoint.
        if self._bypass_pending:
            self._send_bypass()
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

    # -------------------------------------------------------------------------
    # Normal mission waypoint navigation
    # -------------------------------------------------------------------------

    def _send_next(self):
        wp = self._waypoints[self._idx]

        self.get_logger().info(
            f"WP {self._idx + 1}/{len(self._waypoints)}: "
            f"({wp.latitude}, {wp.longitude})"
        )

        self._navigating = True
        self._bypass_active = False

        req = FromLL.Request()
        req.ll_point = wp
        req.ll_point.altitude = 0.0

        future = self._fromll.call_async(req)

        future.add_done_callback(
            self._on_ll,
        )

    def _on_ll(self, future):
        result = future.result()

        if result is None:
            self._fail_mission(
                "fromLL service call failed"
            )
            return

        pt = result.map_point
        self.get_logger().info(
            f"fromLL map_point: ({pt.x:.4f}, {pt.y:.4f})"
        )

        self._send_nav_goal(
            pt,
            is_bypass=False,
        )

    # -------------------------------------------------------------------------
    # Temporary bypass navigation
    # -------------------------------------------------------------------------

    def _send_bypass(self):
        if self._bypass_target is None:
            self._bypass_pending = False
            return

        if not self._nav.wait_for_server(timeout_sec=0.5):
            self.get_logger().info(
                "Waiting for Nav2 before bypass...",
                throttle_duration_sec=5.0,
            )
            return

        if not self._fromll.wait_for_service(timeout_sec=0.5):
            self.get_logger().info(
                "Waiting for /fromLL before bypass...",
                throttle_duration_sec=5.0,
            )
            return

        target = self._bypass_target

        self.get_logger().info(
            "Sending bypass target: "
            f"({target.latitude}, {target.longitude}), "
            f"reason='{self._bypass_reason}'"
        )

        self._navigating = True
        self._bypass_active = True
        self._bypass_pending = False

        req = FromLL.Request()
        req.ll_point = target
        req.ll_point.altitude = 0.0

        future = self._fromll.call_async(req)

        future.add_done_callback(
            self._on_bypass_ll,
        )

    def _on_bypass_ll(self, future):
        result = future.result()

        if result is None:
            self._fail_mission(
                "Bypass fromLL service call failed"
            )
            return

        pt = result.map_point

        self.get_logger().info(
            "Bypass fromLL map_point: "
            f"({pt.x:.4f}, {pt.y:.4f})"
        )

        self._send_nav_goal(
            pt,
            is_bypass=True,
        )

    # -------------------------------------------------------------------------
    # Shared Nav2 goal handling
    # -------------------------------------------------------------------------

    def _send_nav_goal(self, point, is_bypass):
        goal = NavigateToPose.Goal()

        goal.pose = PoseStamped()

        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        goal.pose.pose.position = point

        future = self._nav.send_goal_async(goal)

        future.add_done_callback(
            lambda goal_future: self._on_accepted(
                goal_future,
                is_bypass,
            )
        )

    def _on_accepted(self, future, is_bypass):
        handle = future.result()

        if not handle.accepted:
            if is_bypass:
                self._fail_mission(
                    "Bypass Nav2 goal rejected"
                )
            else:
                self._fail_mission(
                    "Nav2 goal rejected"
                )
            return

        self._nav_handle = handle

        result_future = handle.get_result_async()

        result_future.add_done_callback(
            lambda nav_future: self._on_result(
                nav_future,
                is_bypass,
            )
        )

    def _on_result(self, future, is_bypass):
        result = future.result()

        self._nav_handle = None
        self._navigating = False

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            if is_bypass:
                reason = self._bypass_reason

                self.get_logger().info(
                    "Bypass completed successfully: "
                    f"{reason}"
                )

                self._bypass_active = False
                self._bypass_target = None
                self._bypass_reason = ""

                self._publish_status(
                    MissionStatus.RUNNING,
                    "Bypass completed; resuming mission",
                )

                # Do not increment the normal mission waypoint index.
                # On the next tick, the original waypoint is sent again.
                return

            self._idx += 1

            self._publish_status(
                MissionStatus.RUNNING,
                (
                    f"Waypoint {self._idx}/"
                    f"{len(self._waypoints)} completed"
                ),
            )
            return

        if result.status == GoalStatus.STATUS_CANCELED:
            # Expected cancellation when BT inserts a bypass.
            if self._cancel_for_bypass and not is_bypass:
                self._cancel_for_bypass = False

                self.get_logger().info(
                    "Mission waypoint canceled for bypass"
                )

                self._publish_status(
                    MissionStatus.RUNNING,
                    "Mission waypoint paused for bypass",
                )

                return

            # Expected cancellation during a full mission abort.
            if not self._active:
                return

            self._fail_mission(
                "Nav2 goal canceled unexpectedly"
            )
            return

        if result.status == GoalStatus.STATUS_ABORTED:
            if is_bypass:
                self._fail_mission(
                    "Bypass Nav2 goal aborted"
                )
            else:
                self._fail_mission(
                    "Nav2 goal aborted"
                )
            return

        self._fail_mission(
            f"Nav2 goal failed with status {result.status}"
        )

    # -------------------------------------------------------------------------
    # Mission state helpers
    # -------------------------------------------------------------------------

    def _fail_mission(self, message):
        self.get_logger().error(message)

        self._active = False
        self._navigating = False

        self._nav_handle = None

        self._bypass_target = None
        self._bypass_reason = ""
        self._bypass_pending = False
        self._bypass_active = False
        self._cancel_for_bypass = False

        self._cleanup_timer()

        self._publish_status(
            MissionStatus.FAILED,
            message,
        )

    def _cancel(self):
        self._active = False
        self._navigating = False

        self._bypass_target = None
        self._bypass_reason = ""
        self._bypass_pending = False
        self._bypass_active = False
        self._cancel_for_bypass = False

        if self._nav_handle is not None:
            self._nav_handle.cancel_goal_async()
            self._nav_handle = None

        self._cleanup_timer()

        self._publish_status(
            MissionStatus.ABORTED,
            "Mission aborted",
        )

        self.get_logger().info(
            "Mission aborted"
        )

    def _publish_status(
        self,
        state,
        message,
    ):
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

            self.destroy_timer(
                self._timer
            )

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

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()