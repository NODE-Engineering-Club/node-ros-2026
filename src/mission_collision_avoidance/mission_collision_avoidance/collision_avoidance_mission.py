"""Mission sequencer for Task 9.2 (Collision Avoidance).

Per the official spec (njord.gitbook.io/2026/9-task-descriptions/
9.2-collision-avoidance, read 2026-08-12): the ASV goes from GPS point 5 to
point 6, facing a marker vessel ("The Otter of Njord") on its path and
passing it safely per COLREG. The attempt begins when the ASV crosses the
first of 2 gates (red/green buoy pairs) and finishes on crossing the second,
after which the boat must proceed to point 6 and hold station. 2 attempts
are allowed. The ASV must immediately accelerate to the task's set speed
(2 knots, ~1.029 m/s) at the start of an attempt.

Runs through the existing competition infrastructure exactly like
mission_maneuvering_pathfinding does -- competition_manager (loads
collision_avoidance.yaml, dispatches to mission_manager for the point 5 -> 6
transit) and mission_manager (Nav2 waypoint sequencing). This node only
decides when to start the task (and enforces the task's speed setpoint);
the gate-crossing and COLREG give-way handling during the transit is
boat_bt's CollisionAvoidanceTask subtree, exactly as for every other
competition task.

Readiness checks before starting: a GPS fix must have been received at
least once, and both competition_manager services must be available.

Failure handling: a task ending in FAILED/ABORTED, or not reaching
SUCCEEDED within task_timeout_s, is retried up to task_retries times (spec:
"2 attempts" -> task_retries=1), same _abort_mission pattern as
mission_maneuvering_pathfinding (a local timeout leaves competition_manager
stuck at STATE_RUNNING, so an explicit /mission/abort is needed before a
retry's set_task can succeed).
"""

import rclpy
from njord_msgs.msg import CompetitionState
from njord_msgs.srv import SetCompetitionTask
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger

DEFAULT_TASK_TIMEOUT_S = 180.0
DEFAULT_GPS_TIMEOUT_S = 30.0
DEFAULT_SERVICE_TIMEOUT_S = 30.0
DEFAULT_TASK_RETRIES = 1
DEFAULT_ENFORCE_TASK_SPEED = True
DEFAULT_TASK_SPEED_MPS = 1.029  # 2 knots
DEFAULT_RESTORE_SPEED_MPS = 1.0  # must track nav2_params.yaml's desired_linear_vel
DEFAULT_CONTROLLER_SERVER_NODE_NAME = "controller_server"
DEFAULT_CONTROLLER_SPEED_PARAM_NAME = "FollowPath.desired_linear_vel"


class CollisionAvoidanceMission(Node):
    def __init__(self):
        super().__init__("collision_avoidance_mission")

        self.declare_parameter("task_timeout_s", DEFAULT_TASK_TIMEOUT_S)
        self.declare_parameter("gps_timeout_s", DEFAULT_GPS_TIMEOUT_S)
        self.declare_parameter("service_timeout_s", DEFAULT_SERVICE_TIMEOUT_S)
        self.declare_parameter("task_retries", DEFAULT_TASK_RETRIES)
        self.declare_parameter("enforce_task_speed", DEFAULT_ENFORCE_TASK_SPEED)
        self.declare_parameter("task_speed_mps", DEFAULT_TASK_SPEED_MPS)
        self.declare_parameter("restore_speed_mps", DEFAULT_RESTORE_SPEED_MPS)
        self.declare_parameter(
            "controller_server_node_name", DEFAULT_CONTROLLER_SERVER_NODE_NAME
        )
        self.declare_parameter(
            "controller_speed_param_name", DEFAULT_CONTROLLER_SPEED_PARAM_NAME
        )

        self._task_timeout_s = float(self.get_parameter("task_timeout_s").value)
        self._gps_timeout_s = float(self.get_parameter("gps_timeout_s").value)
        self._service_timeout_s = float(self.get_parameter("service_timeout_s").value)
        self._task_retries = int(self.get_parameter("task_retries").value)
        self._enforce_task_speed = bool(self.get_parameter("enforce_task_speed").value)
        self._task_speed_mps = float(self.get_parameter("task_speed_mps").value)
        self._restore_speed_mps = float(self.get_parameter("restore_speed_mps").value)
        self._controller_speed_param_name = str(
            self.get_parameter("controller_speed_param_name").value
        )

        controller_server_node_name = str(
            self.get_parameter("controller_server_node_name").value
        )

        self._fix = None
        self._status = None
        self.create_subscription(NavSatFix, "/gps_driver/gps_raw", self._fix_cb, 10)
        self.create_subscription(CompetitionState, "/competition/status", self._status_cb, 10)

        self._set_task_client = self.create_client(SetCompetitionTask, "/competition/set_task")
        self._start_client = self.create_client(Trigger, "/competition/start")
        self._mission_abort_client = self.create_client(Trigger, "/mission/abort")
        self._set_speed_client = self.create_client(
            SetParameters, f"/{controller_server_node_name}/set_parameters"
        )

    def _fix_cb(self, msg):
        self._fix = msg

    def _status_cb(self, msg):
        self._status = msg

    # ── Readiness ────────────────────────────────────────────────────────

    def wait_for_gps_fix(self):
        deadline = self.get_clock().now().nanoseconds + int(self._gps_timeout_s * 1e9)
        while self._fix is None and self.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(self, timeout_sec=0.5)
        return self._fix is not None

    def wait_for_services(self):
        ready = True
        for client, name in (
            (self._set_task_client, "/competition/set_task"),
            (self._start_client, "/competition/start"),
        ):
            if not client.wait_for_service(timeout_sec=self._service_timeout_s):
                self.get_logger().error(f"{name} not available after {self._service_timeout_s:.0f}s")
                ready = False
        return ready

    # ── Sequencing ───────────────────────────────────────────────────────

    def run(self):
        if not self.wait_for_gps_fix():
            self.get_logger().error(f"No GPS fix within {self._gps_timeout_s:.0f}s — aborting mission")
            return False
        if not self.wait_for_services():
            self.get_logger().error("competition_manager services unavailable — aborting mission")
            return False

        if self._enforce_task_speed:
            self._set_task_speed(self._task_speed_mps)

        try:
            self.get_logger().info("Starting Task 9.2: collision_avoidance (GPS point 5 -> 6)")
            succeeded = self._run_task_with_retries(
                "collision_avoidance", CompetitionState.TASK_COLLISION_AVOIDANCE
            )

            if succeeded:
                self.get_logger().info("Task 9.2 (collision_avoidance) succeeded")
            else:
                self.get_logger().error(
                    "Task 9.2 (collision_avoidance) did not succeed after "
                    f"{self._task_retries + 1} attempt(s) — attempt failed"
                )

            return succeeded
        finally:
            if self._enforce_task_speed:
                self._set_task_speed(self._restore_speed_mps)

    def _run_task_with_retries(self, name, task_id):
        """Run the task with up to task_retries retries. Returns whether it
        ultimately succeeded.

        A retry re-issues set_task + start against competition_manager
        exactly as a fresh attempt would; if mission_manager still holds a
        matching on-disk checkpoint from the failed attempt (see
        mission_manager.py), this transparently resumes at the last
        incomplete waypoint instead of re-running the whole task from GPS
        point 5.
        """
        succeeded = False
        attempt = 0
        while attempt <= self._task_retries:
            if attempt > 0:
                self.get_logger().warning(
                    f"Retrying task '{name}' "
                    f"(attempt {attempt + 1}/{self._task_retries + 1})"
                )
            succeeded = self._run_task(name, task_id)
            if succeeded:
                break

            # A timeout in _wait_for_task_outcome (Nav2 itself never calls
            # back) leaves competition_manager stuck at STATE_RUNNING --
            # mission_status_callback only reacts to a real terminal
            # MissionStatus from mission_manager, which never arrives on a
            # local timeout. Without clearing it, the next attempt's
            # set_task is rejected with "Cannot change task while a
            # competition task is running" and the retry silently can't
            # happen at all. Best-effort and unconditional, same pattern as
            # mission_maneuvering_pathfinding's _abort_mission.
            self._abort_mission()

            attempt += 1

        return succeeded

    def _run_task(self, name, task_id):
        set_req = SetCompetitionTask.Request(task=task_id)
        future = self._set_task_client.call_async(set_req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._service_timeout_s)
        res = future.result()
        if res is None or not res.success:
            self.get_logger().error(f"set_task('{name}') rejected: {res.message if res else 'no response'}")
            return False

        future = self._start_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._service_timeout_s)
        res = future.result()
        if res is None or not res.success:
            self.get_logger().error(f"start('{name}') rejected: {res.message if res else 'no response'}")
            return False

        return self._wait_for_task_outcome(task_id)

    def _wait_for_task_outcome(self, task_id):
        deadline = self.get_clock().now().nanoseconds + int(self._task_timeout_s * 1e9)
        while self.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(self, timeout_sec=0.5)
            status = self._status
            if status is None or status.task != task_id:
                continue
            if status.state == CompetitionState.STATE_SUCCEEDED:
                return True
            if status.state in (CompetitionState.STATE_FAILED, CompetitionState.STATE_ABORTED):
                self.get_logger().error(f"Task ended in state {status.state}: {status.message}")
                return False
        self.get_logger().error(f"Task timed out after {self._task_timeout_s:.0f}s")
        return False

    def _abort_mission(self):
        """Best-effort /mission/abort. Never blocks or fails the sequence.

        Rejection is expected and harmless when there's nothing to abort
        (the task already ended in a real FAILED/ABORTED, or never made it
        past set_task/start) -- this doesn't retry or fail the mission on
        rejection, same philosophy as mission_maneuvering_pathfinding's
        _abort_mission.
        """
        if not self._mission_abort_client.wait_for_service(timeout_sec=self._service_timeout_s):
            self.get_logger().warning(
                "/mission/abort service unavailable — cannot clear a stuck task before retrying"
            )
            return

        future = self._mission_abort_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._service_timeout_s)
        res = future.result()

        if res is None or not res.success:
            self.get_logger().info(
                "/mission/abort did not report success (expected if the task "
                f"already ended cleanly): {res.message if res else 'no response'}"
            )

    # ── Task speed setpoint (spec: "must immediately accelerate to set ────
    # ── speed for task" at attempt start) ──────────────────────────────

    def _set_task_speed(self, speed_mps):
        """Best-effort override of Nav2's cruise speed for the duration of
        this task. Never blocks or fails the mission -- a crash mid-task
        would leave the global param changed until the stack restarts; this
        is a known, documented limitation (see README/TODOS), not silently
        hidden.
        """
        if not self._set_speed_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning(
                f"{self._set_speed_client.srv_name} unavailable — cannot set task speed "
                f"to {speed_mps:.3f} m/s"
            )
            return False

        request = SetParameters.Request(
            parameters=[
                Parameter(
                    name=self._controller_speed_param_name,
                    value=ParameterValue(
                        type=ParameterType.PARAMETER_DOUBLE,
                        double_value=speed_mps,
                    ),
                )
            ]
        )

        future = self._set_speed_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        res = future.result()

        if res is None or not res.results or not res.results[0].successful:
            reason = res.results[0].reason if res and res.results else "no response"
            self.get_logger().warning(
                f"Failed to set {self._controller_speed_param_name}={speed_mps:.3f}: {reason}"
            )
            return False

        self.get_logger().info(f"Set {self._controller_speed_param_name}={speed_mps:.3f} m/s")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = CollisionAvoidanceMission()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
