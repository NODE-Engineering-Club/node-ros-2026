"""Mission manager.

Drives the boat through an ordered list of GPS waypoints using Nav2's
``NavigateToPose`` action. Each waypoint is converted from geographic
coordinates to the map frame via ``robot_localization``'s ``/fromLL`` service.

Missions can be started two ways:

* **Service** ``/mission/start`` (``njord_msgs/StartMission``) — the original
  interface, unchanged.
* **Topic** ``/mission/waypoints`` (``geometry_msgs/PoseArray``) — auto-starts a
  mission as soon as a message is received. Poses carry geographic coordinates
  with ``position.x = longitude`` and ``position.y = latitude`` (see the GUI
  ``wgs84`` convention).

The current mission state is published on ``/mission/status``
(``std_msgs/String``: ``idle`` / ``running`` / ``completed`` / ``aborted``) using
a latched (transient-local) QoS so late subscribers immediately receive the
last state.
"""

import rclpy
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PoseArray, PoseStamped
from nav2_msgs.action import NavigateToPose
from njord_msgs.srv import StartMission
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from robot_localization.srv import FromLL
from std_msgs.msg import String
from std_srvs.srv import Trigger

# Mission state strings published on /mission/status.
STATE_IDLE = "idle"
STATE_RUNNING = "running"
STATE_COMPLETED = "completed"
STATE_ABORTED = "aborted"


class MissionManager(Node):
    def __init__(self):
        super().__init__("mission_manager")

        self._nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._fromll = self.create_client(FromLL, "/fromLL")

        # Latched status publisher so a GUI that connects late still receives the
        # current mission state immediately.
        status_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_pub = self.create_publisher(String, "/mission/status", status_qos)

        # Mission control interfaces.
        self.create_service(StartMission, "/mission/start", self._start_cb)
        self.create_service(Trigger, "/mission/abort", self._abort_cb)
        self.create_subscription(PoseArray, "/mission/waypoints", self._waypoints_cb, 10)

        self._waypoints = []
        self._idx = 0
        self._navigating = False
        self._active = False
        self._nav_handle = None
        self._status = STATE_IDLE

        # Single periodic tick — created once here (not per mission) so repeated
        # starts never accumulate duplicate timers. _tick early-returns while no
        # mission is active.
        self.create_timer(1.0, self._tick)

        self._publish_status(STATE_IDLE)
        self.get_logger().info(
            "Mission manager ready — call /mission/start or publish to /mission/waypoints"
        )

    # ── Mission lifecycle ──────────────────────────────────────────────────────

    def _begin(self, waypoints, source):
        """Start a mission from a list of GeoPoint waypoints.

        Returns ``(success, message)``. Shared by the service handler and the
        topic callback so both entry points behave identically.
        """
        if len(waypoints) == 0:
            return False, "waypoints must be non-empty"

        if self._active:
            return False, "Mission already running — call /mission/abort first"

        self._waypoints = waypoints
        self._idx = 0
        self._navigating = False
        self._active = True
        self._publish_status(STATE_RUNNING)
        self.get_logger().info(f"Mission started ({source}): {len(waypoints)} waypoints")
        return True, f"{len(waypoints)} waypoints accepted"

    # ── Service handlers ──────────────────────────────────────────────────────

    def _start_cb(self, req, res):
        res.success, res.message = self._begin(list(req.waypoints), "service")
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

    # ── Topic handler ──────────────────────────────────────────────────────────

    def _waypoints_cb(self, msg):
        """Auto-start a mission from a PoseArray of geographic waypoints.

        Pose convention (matches the GUI ``wgs84`` frame):
            position.x = longitude (deg)
            position.y = latitude  (deg)
        """
        waypoints = []
        for pose in msg.poses:
            gp = GeoPoint()
            gp.longitude = pose.position.x
            gp.latitude = pose.position.y
            gp.altitude = 0.0
            waypoints.append(gp)

        ok, message = self._begin(waypoints, "topic /mission/waypoints")
        if not ok:
            # Do not interrupt a running mission — just report why it was ignored.
            self.get_logger().warn(f"Auto-start ignored: {message}")

    # ── Mission tick ──────────────────────────────────────────────────────────

    def _tick(self):
        if not self._active or self._navigating:
            return
        if self._idx >= len(self._waypoints):
            self.get_logger().info("Mission complete")
            self._active = False
            self._publish_status(STATE_COMPLETED)
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
        self._publish_status(STATE_ABORTED)
        self.get_logger().info("Mission aborted")

    # ── Status ─────────────────────────────────────────────────────────────────

    def _publish_status(self, state):
        """Record and publish the current mission state on /mission/status."""
        self._status = state
        msg = String()
        msg.data = state
        self._status_pub.publish(msg)


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
