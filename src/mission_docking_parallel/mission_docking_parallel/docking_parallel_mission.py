"""Mission sequencer for Task 3.2 (Parallel Docking).

Same three-phase pattern as mission_docking (Task 3.1) — see that
package's docstring for the full architectural rationale (why
competition_manager's waypoint-task XOR direct-BT-task model needs a
sequencer to stitch GPS transit around a direct-BT task at all). This is a
deliberately separate package rather than a parameterized variant of
mission_docking: 3.1 and 3.2 are two different missions with two different
close-range controllers, kept independently robust rather than sharing a
coupled implementation.

Phases:
  1. GPS transit from wherever the boat starts to point 11 (staging point,
     ~10 m from the berth per spec), direct through mission_manager.
  2. The close-range parallel docking manoeuvre, via competition_manager's
     TASK_DOCKING_PARALLEL selection.
  3. GPS transit from the berth to point 12 — reaching this stops the
     spec's task timer.

Point 10 (the official start line) is not itself navigated to — the boat
is expected to already be there when this node is launched; see
config/docking_3_2_waypoints.yaml.

IMPORTANT — TASK_DOCKING_PARALLEL's close-range manoeuvre
(ExecuteDockingParallel, in parallel_docking_nodes.cpp) and its
perception (wall_detector_node.py) are both real now, not the
<AlwaysSuccess/> placeholder this used to route to. But UNVERIFIED: the
whole close-range path has never been run against a real wall, on the
bench or in the water. This sequencer's orchestration (transit legs,
retry, task-clearing) is real and tested the same way mission_docking's
was; do not use the close-range phase for a real attempt until it has had
a bench check.

Spec 9.3 allows 2 attempts at the parallel docking manoeuvre;
max_docking_attempts defaults to 2 to match, same retry-and-clear pattern
as mission_docking (see that module for why: TASK_DOCKING_PARALLEL, like
TASK_DOCKING, has no failure path of its own once a real controller
exists there, so a stuck attempt can only be ended by clearing to
TASK_NONE, which competition_manager's set_task_callback allows while
STATE_RUNNING specifically for waypoint-less tasks like this one).

Readiness checks before starting: a GPS fix must have been received at
least once, and both mission_manager's and competition_manager's services
must be available.

Failure handling: a transit leg or docking failure stops the sequence —
this node does not attempt to cancel or recover a stuck leg/task, same
philosophy as mission_maneuvering_pathfinding and mission_docking.
"""

from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geographic_msgs.msg import GeoPoint
from njord_msgs.msg import CompetitionState
from njord_msgs.msg import MissionStatus
from njord_msgs.srv import SetCompetitionTask
from njord_msgs.srv import StartMission
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from std_srvs.srv import Trigger

DEFAULT_WAYPOINTS_FILE = "docking_3_2_waypoints.yaml"

DEFAULT_GPS_TIMEOUT_S = 30.0
DEFAULT_SERVICE_TIMEOUT_S = 30.0
DEFAULT_TRANSIT_TIMEOUT_S = 120.0
DEFAULT_DOCKING_TIMEOUT_S = 180.0
DEFAULT_MAX_DOCKING_ATTEMPTS = 2


class DockingParallelMission(Node):
    def __init__(self):
        super().__init__("docking_parallel_mission")

        self.declare_parameter("waypoints_file", DEFAULT_WAYPOINTS_FILE)
        self.declare_parameter("gps_timeout_s", DEFAULT_GPS_TIMEOUT_S)
        self.declare_parameter("service_timeout_s", DEFAULT_SERVICE_TIMEOUT_S)
        self.declare_parameter("transit_timeout_s", DEFAULT_TRANSIT_TIMEOUT_S)
        self.declare_parameter("docking_timeout_s", DEFAULT_DOCKING_TIMEOUT_S)
        self.declare_parameter("max_docking_attempts", DEFAULT_MAX_DOCKING_ATTEMPTS)

        self._gps_timeout_s = float(self.get_parameter("gps_timeout_s").value)
        self._service_timeout_s = float(self.get_parameter("service_timeout_s").value)
        self._transit_timeout_s = float(self.get_parameter("transit_timeout_s").value)
        self._docking_timeout_s = float(self.get_parameter("docking_timeout_s").value)
        self._max_docking_attempts = int(self.get_parameter("max_docking_attempts").value)

        self._point_10, self._point_11, self._point_12 = self._load_waypoints(
            str(self.get_parameter("waypoints_file").value)
        )

        self._fix = None
        self._mission_status = None
        self._competition_status = None

        self.create_subscription(NavSatFix, "/gps_driver/gps_raw", self._fix_cb, 10)

        # Both mission_manager and competition_manager publish their status
        # topics TRANSIENT_LOCAL — match that so a late-connecting
        # subscription doesn't just see nothing until the next state change.
        transient_local_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            MissionStatus, "/mission/status", self._mission_status_cb, transient_local_qos
        )
        self.create_subscription(
            CompetitionState, "/competition/status", self._competition_status_cb, transient_local_qos
        )

        self._mission_start_client = self.create_client(StartMission, "/mission/start")
        self._set_task_client = self.create_client(SetCompetitionTask, "/competition/set_task")
        self._start_client = self.create_client(Trigger, "/competition/start")

        # GUI support, same idea as mission_docking's
        # /docking_mission/points/<label>: these points don't go through
        # competition_manager's task system (docking_parallel.yaml is
        # deliberately waypoint-less), so they don't get
        # /competition/waypoints/<index> for free.
        self._publish_reference_points(transient_local_qos)

    def _fix_cb(self, msg):
        self._fix = msg

    def _mission_status_cb(self, msg):
        self._mission_status = msg

    def _competition_status_cb(self, msg):
        self._competition_status = msg

    # ── Waypoints ────────────────────────────────────────────────────────

    @staticmethod
    def _load_waypoints(filename):
        share = Path(get_package_share_directory("mission_docking_parallel"))
        path = share / "config" / filename

        with path.open("r", encoding="utf-8") as f:
            data = yaml.safe_load(f)

        points = data["points"]

        def to_geo_point(key):
            p = points[key]
            return GeoPoint(
                latitude=float(p["latitude"]),
                longitude=float(p["longitude"]),
                altitude=float(p.get("altitude", 0.0)),
            )

        return (
            to_geo_point("point_10_start"),
            to_geo_point("point_11_staging"),
            to_geo_point("point_12_finish"),
        )

    def _publish_reference_points(self, qos):
        for label, point in (
            ("point_10_start", self._point_10),
            ("point_11_staging", self._point_11),
            ("point_12_finish", self._point_12),
        ):
            publisher = self.create_publisher(NavSatFix, f"/docking_parallel_mission/points/{label}", qos)

            message = NavSatFix()
            message.header.frame_id = "map"
            message.header.stamp = self.get_clock().now().to_msg()
            message.status.status = NavSatStatus.STATUS_FIX
            message.status.service = NavSatStatus.SERVICE_GPS
            message.latitude = point.latitude
            message.longitude = point.longitude
            message.altitude = point.altitude

            publisher.publish(message)

    # ── Readiness ────────────────────────────────────────────────────────

    def wait_for_gps_fix(self):
        deadline = self.get_clock().now().nanoseconds + int(self._gps_timeout_s * 1e9)
        while self._fix is None and self.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(self, timeout_sec=0.5)
        return self._fix is not None

    def wait_for_services(self):
        ready = True
        for client, name in (
            (self._mission_start_client, "/mission/start"),
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
            self.get_logger().error(f"No GPS fix within {self._gps_timeout_s:.0f}s — aborting docking mission")
            return False
        if not self.wait_for_services():
            self.get_logger().error(
                "mission_manager/competition_manager services unavailable — aborting docking mission"
            )
            return False

        # Defensive: a previous run (or a respawn mid-run) could leave
        # competition_manager's task selection at TASK_DOCKING_PARALLEL,
        # which would incorrectly suppress collision/buoy avoidance (see
        # GlobalSafety in simple_boat.xml) during this leg's transit.
        self._clear_competition_task()

        self.get_logger().info("Transit leg 1/2: heading to point 11 (staging point)")
        if not self._run_transit_leg(self._point_11):
            self.get_logger().error("Transit to point 11 failed — aborting docking mission")
            return False
        self.get_logger().info("Reached point 11")

        if not self._run_docking():
            self.get_logger().error(
                f"Parallel docking did not succeed within {self._max_docking_attempts} attempt(s) "
                "— aborting docking mission"
            )
            return False
        self.get_logger().info("Parallel docking succeeded")

        # Re-enable collision/buoy avoidance for the exit transit — nothing
        # else clears TASK_DOCKING_PARALLEL back off once the dock
        # manoeuvre itself is done.
        self._clear_competition_task()

        self.get_logger().info("Transit leg 2/2: heading to point 12 (finish point)")
        if not self._run_transit_leg(self._point_12):
            self.get_logger().error(
                "Transit to point 12 failed — docking succeeded but the mission did not "
                "complete cleanly"
            )
            return False

        self.get_logger().info("Reached point 12 — Task 3.2 parallel docking mission complete")
        return True

    def _run_transit_leg(self, point):
        req = StartMission.Request(waypoints=[point])

        # Discard any status left over from a previous leg/run before
        # sending the request — /mission/status is TRANSIENT_LOCAL, so a
        # stale message would otherwise look like an instantaneous result
        # for this leg.
        self._mission_status = None

        future = self._mission_start_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._service_timeout_s)
        res = future.result()

        if res is None or not res.success:
            self.get_logger().error(f"/mission/start rejected: {res.message if res else 'no response'}")
            return False

        return self._wait_for_mission_outcome()

    def _wait_for_mission_outcome(self):
        deadline = self.get_clock().now().nanoseconds + int(self._transit_timeout_s * 1e9)

        while self.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(self, timeout_sec=0.5)
            status = self._mission_status
            if status is None:
                continue
            if status.state == MissionStatus.SUCCEEDED:
                return True
            if status.state in (MissionStatus.FAILED, MissionStatus.ABORTED):
                self.get_logger().error(f"Transit leg ended in state {status.state}: {status.message}")
                return False

        self.get_logger().error(f"Transit leg timed out after {self._transit_timeout_s:.0f}s")
        return False

    def _run_docking(self):
        for attempt in range(1, self._max_docking_attempts + 1):
            self.get_logger().info(f"Parallel docking attempt {attempt}/{self._max_docking_attempts}")

            set_req = SetCompetitionTask.Request(task=CompetitionState.TASK_DOCKING_PARALLEL)
            future = self._set_task_client.call_async(set_req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=self._service_timeout_s)
            res = future.result()

            if res is None or not res.success:
                self.get_logger().error(
                    f"set_task(docking_parallel) rejected on attempt {attempt}: "
                    f"{res.message if res else 'no response'}"
                )
                continue

            # Every attempt selects the same task id (TASK_DOCKING_PARALLEL),
            # so a leftover FAILED/SUCCEEDED status from a *previous*
            # attempt would otherwise look like this attempt's result.
            self._competition_status = None

            future = self._start_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=self._service_timeout_s)
            res = future.result()

            if res is None or not res.success:
                self.get_logger().error(
                    f"start(docking_parallel) rejected on attempt {attempt}: "
                    f"{res.message if res else 'no response'}"
                )
                continue

            if self._wait_for_docking_outcome():
                return True

            self.get_logger().warning(
                f"Parallel docking attempt {attempt}/{self._max_docking_attempts} did not succeed"
            )

            # See mission_docking.py's identical comment: the close-range
            # controller (once implemented) is not expected to ever return
            # BT::NodeStatus::FAILURE either, so a stuck attempt only ends
            # via this node's own timeout, leaving competition_manager at
            # STATE_RUNNING until explicitly cleared — required for attempt
            # 2 to even be possible.
            self._clear_competition_task()

        return False

    def _wait_for_docking_outcome(self):
        deadline = self.get_clock().now().nanoseconds + int(self._docking_timeout_s * 1e9)

        while self.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(self, timeout_sec=0.5)
            status = self._competition_status
            if status is None or status.task != CompetitionState.TASK_DOCKING_PARALLEL:
                continue
            if status.state == CompetitionState.STATE_SUCCEEDED:
                return True
            if status.state in (CompetitionState.STATE_FAILED, CompetitionState.STATE_ABORTED):
                self.get_logger().error(f"Docking attempt ended in state {status.state}: {status.message}")
                return False

        self.get_logger().error(f"Parallel docking attempt timed out after {self._docking_timeout_s:.0f}s")
        return False

    def _clear_competition_task(self):
        """Select TASK_NONE. Best-effort — never blocks the sequence."""
        future = self._set_task_client.call_async(
            SetCompetitionTask.Request(task=CompetitionState.TASK_NONE)
        )
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._service_timeout_s)
        res = future.result()

        if res is None or not res.success:
            self.get_logger().warning(
                f"set_task(TASK_NONE) did not succeed: {res.message if res else 'no response'}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = DockingParallelMission()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
