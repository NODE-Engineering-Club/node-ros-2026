"""Mission manager — idle until /mission/start is called via service."""

import json
import math
import os

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as NavPath
from njord_msgs.msg import MissionStatus
from njord_msgs.srv import SetBypassTarget, StartMission
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from robot_localization.srv import FromLL
from std_msgs.msg import String
from std_srvs.srv import Trigger


class MissionManager(Node):
    def __init__(self):
        super().__init__("mission_manager")

        # Retry a single failed waypoint goal this many times (in addition to
        # the first attempt) before failing the whole mission. Covers
        # transient Nav2/localization blips on one waypoint without losing
        # everything already completed.
        self.declare_parameter("max_waypoint_retries", 2)
        self._max_waypoint_retries = int(
            self.get_parameter("max_waypoint_retries").value
        )

        # On-disk mission progress checkpoint. If mission_manager crashes and
        # is respawned (or the caller simply retries /mission/start) with the
        # *exact same* waypoint list, the mission resumes at the last
        # incomplete waypoint instead of restarting from 0. A mismatched
        # waypoint list is always treated as a different mission and starts
        # fresh — this file does not track which competition task a
        # checkpoint belongs to, so two tasks that happen to share an
        # identical waypoint list would be indistinguishable. (Today
        # maneuvering.yaml and path_finding.yaml both hold the same
        # placeholder point for exactly this reason — this is safe only
        # because they still need real, distinct course data before use.)
        # checkpoint_path must live somewhere that actually survives however
        # this node gets restarted (container restart vs. process respawn) —
        # verify this default is appropriate for the real deployment.
        self.declare_parameter(
            "checkpoint_path",
            os.path.expanduser("~/.njord_mission_checkpoint.json"),
        )
        self._checkpoint_path = str(
            self.get_parameter("checkpoint_path").value
        )

        # Spec 9.1: "ASV must stop and remain stationary at GPS-point 4".
        # After the last waypoint in any waypoint list is reached, hold here
        # (send no further Nav2 goals) for this long before reporting
        # SUCCEEDED, instead of reporting success the instant the goal
        # completes. 0 still takes one extra ~1s tick (the mission tick
        # timer's period) before reporting success, not truly instant.
        self.declare_parameter("final_hold_duration_sec", 5.0)
        self._final_hold_duration_sec = float(
            self.get_parameter("final_hold_duration_sec").value
        )

        self._nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        # Preview-only: computes a path without executing it, so the GUI can
        # show the upcoming leg before the boat actually starts moving on
        # it. Best-effort — a failure here must never affect the real
        # NavigateToPose goal sent right after it.
        self._compute_path = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
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

        # GUI support: a human-readable trail of what the mission is doing
        # (distinct from /mission/status, which is the terse machine-state
        # used by competition_manager) — point a Foxglove Log/Raw Messages
        # panel at this. And the per-step path preview described above.
        self._log_pub = self.create_publisher(
            String,
            "/mission/log",
            10,
        )
        self._preview_pub = self.create_publisher(
            NavPath,
            "/mission/preview_plan",
            10,
        )

        self._waypoints = []
        self._idx = 0

        # Final-hold state (see final_hold_duration_sec above).
        self._holding = False
        self._hold_start_time = None

        # Retry bookkeeping for the waypoint currently being attempted.
        # Reset whenever _idx changes so a fresh waypoint always starts at
        # attempt 0.
        self._waypoint_attempt_idx = -1
        self._waypoint_attempt_count = 0

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

        resumed_idx = self._resumable_checkpoint_idx(self._waypoints)
        self._idx = resumed_idx if resumed_idx is not None else 0

        self._waypoint_attempt_idx = -1
        self._waypoint_attempt_count = 0

        self._navigating = False
        self._active = True

        self._nav_handle = None

        self._bypass_target = None
        self._bypass_reason = ""
        self._bypass_pending = False
        self._bypass_active = False
        self._cancel_for_bypass = False

        if resumed_idx is not None and resumed_idx >= len(self._waypoints):
            resume_description = "resuming final hold (all waypoints already complete)"
        elif resumed_idx is not None:
            resume_description = f"resuming from waypoint {resumed_idx + 1}"

        if resumed_idx is not None:
            self._publish_status(
                MissionStatus.RUNNING,
                f"Mission resumed from checkpoint — {resume_description}",
            )

            self.get_logger().warning(
                f"Mission started: {len(self._waypoints)} waypoints — "
                f"RESUMING from checkpoint, {resume_description} "
                "(matching in-progress checkpoint found on disk)"
            )
        else:
            self._publish_status(
                MissionStatus.RUNNING,
                "Mission started",
            )

            self.get_logger().info(
                f"Mission started: {len(self._waypoints)} waypoints"
            )

        self._save_checkpoint()

        if resumed_idx is not None:
            self._log_action(f"Mission resumed from checkpoint — {resume_description}")
        else:
            self._log_action(f"Mission started: {len(self._waypoints)} waypoint(s)")

        self._timer = self.create_timer(
            1.0,
            self._tick,
        )

        res.success = True
        res.message = f"{len(self._waypoints)} waypoints accepted"

        if resumed_idx is not None:
            res.message += f"; {resume_description}"

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
        self._log_action(f"Bypass detour requested: {req.reason}")

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
            self._tick_final_hold()
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

    def _tick_final_hold(self):
        """Spec 9.1: "ASV must stop and remain stationary at GPS-point 4".

        Reached once every waypoint in the current list is complete. Sends
        no further Nav2 goals (which, combined with pid_controller's
        stale-setpoint watchdog, brings the boat to an actual physical
        stop) and holds for final_hold_duration_sec before reporting
        SUCCEEDED, instead of reporting success the instant the last
        waypoint's goal completes.
        """
        if not self._holding:
            self._holding = True
            self._hold_start_time = self.get_clock().now()

            # idx already == len(waypoints) here (set by the last successful
            # waypoint completion) — checkpoint it so a crash mid-hold
            # resumes the hold, not the whole mission from scratch.
            self._save_checkpoint()

            self._publish_status(
                MissionStatus.RUNNING,
                f"Holding position for {self._final_hold_duration_sec:.1f}s "
                "at final waypoint",
            )
            self._log_action(
                f"All waypoints complete — holding position for "
                f"{self._final_hold_duration_sec:.1f}s"
            )

            self.get_logger().info(
                f"Mission waypoints complete — holding for "
                f"{self._final_hold_duration_sec:.1f}s before reporting success"
            )
            return

        elapsed = (
            self.get_clock().now() - self._hold_start_time
        ).nanoseconds / 1e9

        if elapsed < self._final_hold_duration_sec:
            return

        self.get_logger().info("Final hold complete — mission succeeded")

        self._active = False
        self._holding = False
        self._hold_start_time = None
        self._cleanup_timer()
        self._clear_checkpoint()

        self._publish_status(
            MissionStatus.SUCCEEDED,
            "Mission completed successfully",
        )
        self._log_action("MISSION SUCCEEDED — holding complete")

    # -------------------------------------------------------------------------
    # Normal mission waypoint navigation
    # -------------------------------------------------------------------------

    def _send_next(self):
        wp = self._waypoints[self._idx]

        self.get_logger().info(
            f"WP {self._idx + 1}/{len(self._waypoints)}: "
            f"({wp.latitude}, {wp.longitude})"
        )
        self._log_action(
            f"Heading to waypoint {self._idx + 1}: "
            f"({wp.latitude:.6f}, {wp.longitude:.6f})"
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

        # A bypass detour requested while holding at the final waypoint
        # means the boat is about to physically move again — the "remain
        # stationary" clock must restart once it returns, not keep counting
        # through the detour.
        if self._holding:
            self._holding = False
            self._hold_start_time = None

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
        # Best-effort preview of this one leg, published to
        # /mission/preview_plan before the real navigate goal below is sent
        # — "the path for each step as it comes up," not the whole mission
        # planned upfront. Bypass detours are skipped (the user asked for
        # per-waypoint preview specifically); a preview failure here must
        # never block or affect the real goal sent immediately after it.
        if not is_bypass:
            self._preview_path(point)

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

    # -------------------------------------------------------------------------
    # Per-step path preview (GUI only — never gates real navigation)
    # -------------------------------------------------------------------------

    def _preview_path(self, point):
        if not self._compute_path.wait_for_server(timeout_sec=0.2):
            return

        try:
            goal = ComputePathToPose.Goal()
            goal.goal = PoseStamped()
            goal.goal.header.frame_id = "map"
            goal.goal.header.stamp = self.get_clock().now().to_msg()
            goal.goal.pose.position = point
            goal.use_start = False
        except Exception as error:  # noqa: BLE001 — preview must fail safe
            self.get_logger().warning(
                f"Could not build path-preview goal, skipping preview: {error}"
            )
            return

        future = self._compute_path.send_goal_async(goal)
        future.add_done_callback(self._on_preview_accepted)

    def _on_preview_accepted(self, future):
        try:
            handle = future.result()
        except Exception as error:  # noqa: BLE001 — preview must fail safe
            self.get_logger().warning(f"Path preview goal failed: {error}")
            return

        if handle is None or not handle.accepted:
            return

        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_preview_result)

    def _on_preview_result(self, future):
        try:
            result = future.result()
        except Exception as error:  # noqa: BLE001 — preview must fail safe
            self.get_logger().warning(f"Path preview result failed: {error}")
            return

        if result is None or result.status != GoalStatus.STATUS_SUCCEEDED:
            return

        self._preview_pub.publish(result.result.path)

    def _on_accepted(self, future, is_bypass):
        handle = future.result()

        if not handle.accepted:
            if is_bypass:
                self._fail_mission(
                    "Bypass Nav2 goal rejected"
                )
            else:
                self._retry_or_fail_waypoint(
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
            self._save_checkpoint()

            self._publish_status(
                MissionStatus.RUNNING,
                (
                    f"Waypoint {self._idx}/"
                    f"{len(self._waypoints)} completed"
                ),
            )
            self._log_action(f"Waypoint {self._idx}/{len(self._waypoints)} reached")
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

            if is_bypass:
                self._fail_mission(
                    "Nav2 goal canceled unexpectedly"
                )
            else:
                self._retry_or_fail_waypoint(
                    "Nav2 goal canceled unexpectedly"
                )
            return

        if result.status == GoalStatus.STATUS_ABORTED:
            if is_bypass:
                self._fail_mission(
                    "Bypass Nav2 goal aborted"
                )
            else:
                self._retry_or_fail_waypoint(
                    "Nav2 goal aborted"
                )
            return

        if is_bypass:
            self._fail_mission(
                f"Bypass Nav2 goal failed with status {result.status}"
            )
        else:
            self._retry_or_fail_waypoint(
                f"Nav2 goal failed with status {result.status}"
            )

    # -------------------------------------------------------------------------
    # Mission state helpers
    # -------------------------------------------------------------------------

    def _retry_or_fail_waypoint(self, message):
        """Retry the current (non-bypass) waypoint up to
        max_waypoint_retries times before failing the whole mission.

        Never call this for a bypass goal — bypass failures always fail the
        mission immediately via _fail_mission, unretried.
        """
        self._navigating = False

        if self._waypoint_attempt_idx != self._idx:
            self._waypoint_attempt_idx = self._idx
            self._waypoint_attempt_count = 0

        self._waypoint_attempt_count += 1

        if self._waypoint_attempt_count <= self._max_waypoint_retries:
            retry_message = (
                f"{message} — retrying waypoint {self._idx + 1}/"
                f"{len(self._waypoints)} "
                f"(retry {self._waypoint_attempt_count}/"
                f"{self._max_waypoint_retries})"
            )
            self.get_logger().warning(retry_message)
            self._log_action(retry_message)
            return

        self._fail_mission(
            f"{message} (waypoint {self._idx + 1} failed after "
            f"{self._waypoint_attempt_count} attempts)"
        )

    def _fail_mission(self, message):
        self.get_logger().error(message)

        self._active = False
        self._navigating = False
        self._holding = False
        self._hold_start_time = None

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
        self._log_action(f"MISSION FAILED: {message}")

    def _cancel(self):
        self._active = False
        self._navigating = False
        self._holding = False
        self._hold_start_time = None

        self._bypass_target = None
        self._bypass_reason = ""
        self._bypass_pending = False
        self._bypass_active = False
        self._cancel_for_bypass = False

        if self._nav_handle is not None:
            self._nav_handle.cancel_goal_async()
            self._nav_handle = None

        self._cleanup_timer()

        # An explicit /mission/abort is an operator decision to stop, not a
        # crash — don't leave a checkpoint behind that would silently resume
        # this same mission on the next /mission/start.
        self._clear_checkpoint()

        self._publish_status(
            MissionStatus.ABORTED,
            "Mission aborted",
        )
        self._log_action("MISSION ABORTED (operator request)")

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

    def _log_action(self, message):
        """Log + publish one line to /mission/log with progress prefixed,
        for a Foxglove Log/Raw Messages panel to show a running narrative of
        mission progress. Purely additive to the existing get_logger() calls
        scattered through this file — never a substitute for the terse
        machine-readable /mission/status."""
        total = len(self._waypoints)
        prefix = f"[{min(self._idx, total)}/{total}]" if total else "[-/-]"

        line = String()
        line.data = f"{prefix} {message}"

        self._log_pub.publish(line)

    # -------------------------------------------------------------------------
    # Checkpoint persistence (resume-from-failure)
    # -------------------------------------------------------------------------

    @staticmethod
    def _waypoints_key(waypoints):
        """Comparable, JSON-serializable form of a waypoint list.

        Used both to write the checkpoint file and to decide whether an
        incoming /mission/start call is a resume of an existing checkpoint
        (exact match) or a genuinely new mission (anything else).
        """
        return [
            {
                "latitude": wp.latitude,
                "longitude": wp.longitude,
                "altitude": wp.altitude,
            }
            for wp in waypoints
        ]

    def _resumable_checkpoint_idx(self, waypoints):
        """Return the checkpoint's saved idx if it matches these waypoints
        exactly, else None (start at 0).

        idx == len(waypoints) is valid and means "every waypoint was already
        completed, only the final hold was interrupted" — _tick() already
        treats idx >= len(waypoints) as "enter/resume the final hold," so
        resuming into that value is correct, not an off-by-one.
        """
        checkpoint = self._load_checkpoint()

        if checkpoint is None:
            return None

        if checkpoint.get("waypoints") != self._waypoints_key(waypoints):
            return None

        idx = checkpoint.get("idx")

        if not isinstance(idx, int) or not (0 <= idx <= len(waypoints)):
            return None

        return idx

    def _load_checkpoint(self):
        try:
            with open(self._checkpoint_path, "r", encoding="utf-8") as f:
                return json.load(f)
        except FileNotFoundError:
            return None
        except (OSError, ValueError) as error:
            self.get_logger().warning(
                f"Ignoring unreadable checkpoint file "
                f"{self._checkpoint_path}: {error}"
            )
            return None

    def _save_checkpoint(self):
        checkpoint = {
            "waypoints": self._waypoints_key(self._waypoints),
            "idx": self._idx,
        }

        try:
            tmp_path = self._checkpoint_path + ".tmp"

            with open(tmp_path, "w", encoding="utf-8") as f:
                json.dump(checkpoint, f)

            os.replace(tmp_path, self._checkpoint_path)
        except OSError as error:
            self.get_logger().error(
                f"Could not write mission checkpoint to "
                f"{self._checkpoint_path}: {error}"
            )

    def _clear_checkpoint(self):
        try:
            os.remove(self._checkpoint_path)
        except FileNotFoundError:
            pass
        except OSError as error:
            self.get_logger().warning(
                f"Could not remove checkpoint file "
                f"{self._checkpoint_path}: {error}"
            )

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