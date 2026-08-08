"""Mission sequencer for Task 9.1 (Maneuvering and Path Finding).

Runs the task's two sub-tasks in order through the existing competition
infrastructure — competition_manager (loads maneuvering.yaml/path_finding.yaml,
dispatches to mission_manager) and mission_manager (Nav2 waypoint sequencing)
— rather than driving Nav2 directly. This node only decides *which task* and
*when to advance*; cardinal-marker avoidance during each task is handled by
boat_bt's ManeuveringTask/PathFindingTask subtrees exactly as for every other
competition task.

Readiness checks before starting: a GPS fix must have been received at least
once, and both competition_manager services must be available.

Failure handling: if a task ends in FAILED/ABORTED, or does not reach
SUCCEEDED within task_timeout_s, the sequence stops before the next task.
This node does not attempt to cancel or recover a stuck task — competition_
manager exposes no cancel endpoint for waypoint tasks (mission_manager's
/mission/abort operates one layer below it and isn't wired up here).
"""

import rclpy
from njord_msgs.msg import CompetitionState
from njord_msgs.srv import SetCompetitionTask
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger

TASK_SEQUENCE = (
    ("maneuvering", CompetitionState.TASK_MANEUVERING),
    ("path_finding", CompetitionState.TASK_PATH_FINDING),
)

DEFAULT_TASK_TIMEOUT_S = 180.0
DEFAULT_GPS_TIMEOUT_S = 30.0
DEFAULT_SERVICE_TIMEOUT_S = 30.0


class ManeuveringPathfindingMission(Node):
    def __init__(self):
        super().__init__("maneuvering_pathfinding_mission")

        self.declare_parameter("task_timeout_s", DEFAULT_TASK_TIMEOUT_S)
        self.declare_parameter("gps_timeout_s", DEFAULT_GPS_TIMEOUT_S)
        self.declare_parameter("service_timeout_s", DEFAULT_SERVICE_TIMEOUT_S)

        self._task_timeout_s = float(self.get_parameter("task_timeout_s").value)
        self._gps_timeout_s = float(self.get_parameter("gps_timeout_s").value)
        self._service_timeout_s = float(self.get_parameter("service_timeout_s").value)

        self._fix = None
        self._status = None
        self.create_subscription(NavSatFix, "/gps_driver/gps_raw", self._fix_cb, 10)
        self.create_subscription(CompetitionState, "/competition/status", self._status_cb, 10)

        self._set_task_client = self.create_client(SetCompetitionTask, "/competition/set_task")
        self._start_client = self.create_client(Trigger, "/competition/start")

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

        for name, task_id in TASK_SEQUENCE:
            self.get_logger().info(f"Starting task '{name}'")
            if not self._run_task(name, task_id):
                self.get_logger().error(f"Task '{name}' did not succeed — stopping sequence")
                return False
            self.get_logger().info(f"Task '{name}' succeeded")

        self.get_logger().info("Maneuvering + Path Finding sequence complete")
        return True

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


def main(args=None):
    rclpy.init(args=args)
    node = ManeuveringPathfindingMission()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
