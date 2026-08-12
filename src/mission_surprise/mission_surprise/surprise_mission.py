"""Mission sequencer for Task 9.4 (Surprise).

TEAM WORKING DRAFT — the official Njord 2026 handbook page for 9.4 is still
blank ("revealed during competition"). This sequencer implements the team's
own course-sketch interpretation: a normal docking (as in Task 3.1), an
open-water transit past buoys/cardinal marks under COLREG, and a parallel
docking (as in Task 3.2) that also serves as the finish — no exit manoeuvre
after the final dock. See config/surprise_9_4_waypoints.yaml and TODOS.md
for the open questions (real waypoint IDs/order, whether "S"/"E" are
genuinely cardinal marks, real parallel-berth dimensions). UNVERIFIED — this
whole sequence has never been run, on the bench or in the water; it chains
mission_docking's proven orchestration pattern with mission_docking_parallel's
UNVERIFIED close-range controller (see that package's own caveat).

Five phases, all driven directly against competition_manager/mission_manager
(same reasoning as mission_docking/mission_docking_parallel/
mission_maneuvering_pathfinding: competition_manager's waypoint-task XOR
direct-BT-task model has no built-in way to chain multiple tasks into one
run, so a sequencer node is what stitches them together):

  1. The close-range normal-docking manoeuvre, via competition_manager's
     TASK_DOCKING selection — boat_bt's existing ExecuteDocking state
     machine (docking_nodes.cpp), completely unchanged. Its built-in hold
     (10s) + reverse-out-of-berth is what satisfies "hold, then exit the
     berth" — no separate waypoint is used for that reversal, per spec
     ("no GPS waypoints exist inside either dock/berth").
  2. GPS transit from wherever the reversal leaves the boat, through
     point_14_exit, the five open-water waypoints (point_4_1..point_4_5,
     in navigation order), to point_13_parallel_approach — one single
     mission_manager leg (it already drives a waypoint list sequentially
     and holds at the final point before reporting SUCCEEDED). Driven
     direct through mission_manager, same as mission_docking's transit legs.
  3. Immediately before phase 2, competition_manager's task selection is
     set to TASK_SURPRISE (surprise.yaml is deliberately waypoint-less, so
     this only flips competition_manager to STATE_RUNNING — it does NOT
     itself drive mission_manager, so there's no conflict with phase 2's
     direct call). This activates simple_boat.xml's SurpriseTask subtree
     (cardinal-mark passing + individual buoy COLREG-side passing) for the
     whole transit; GlobalSafety's generic buoy-standoff/collision-risk
     reflex is also active throughout, since "surprise" is not one of the
     tasks GlobalSafety excludes (only "docking"/"docking_parallel" are).
  4. The close-range parallel-docking manoeuvre, via TASK_DOCKING_PARALLEL
     — boat_bt's ExecuteDockingParallel (parallel_docking_nodes.cpp).
     UNVERIFIED against a real wall — see that file's header.
  5. Done. Unlike mission_docking_parallel.py, there is NO transit leg
     after the parallel dock succeeds — spec: "the task ends the moment the
     ASV is stationary inside the final (parallel) berth... no exit
     manoeuvre is required."

Point 12 (the official start point) is not itself navigated to — the boat
is RC-positioned there by the operator before switching to Autonomous Mode,
same as point_7/point_10 in the 3.1/3.2 sequencers.

Spec 9.3 allows 2 attempts per docking manoeuvre; max_docking_attempts
defaults to 2 and applies independently to both the normal-docking and
parallel-docking phases, same retry-and-clear pattern as
mission_docking.py/mission_docking_parallel.py: neither ExecuteDocking nor
ExecuteDockingParallel is expected to ever return BT::NodeStatus::FAILURE,
so a stuck attempt only ends via this node's own timeout, and
competition_manager must be explicitly cleared to TASK_NONE before a retry
(or the next phase) can select a new task.

Competition task reset: simple_boat.xml's GlobalSafety subtree suppresses
the generic collision/buoy reflex only while the current task is "docking"
or "docking_parallel" (dock/wall geometry must not be treated as an
obstacle during a close approach). This node explicitly clears to
TASK_NONE between every phase transition so that reflex — and, for phase
2, the SurpriseTask cardinal/buoy handling — is active exactly when it
should be and nowhere else.

Failure handling: same philosophy as the existing sequencers — a failed
phase aborts the whole run; this node does not attempt to cancel or
recover a stuck leg/task.
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

DEFAULT_WAYPOINTS_FILE = "surprise_9_4_waypoints.yaml"

DEFAULT_GPS_TIMEOUT_S = 30.0
DEFAULT_SERVICE_TIMEOUT_S = 30.0
DEFAULT_TRANSIT_TIMEOUT_S = 180.0
DEFAULT_DOCKING_TIMEOUT_S = 180.0
DEFAULT_MAX_DOCKING_ATTEMPTS = 2

_TASK_NAMES = {
    CompetitionState.TASK_DOCKING: "docking",
    CompetitionState.TASK_DOCKING_PARALLEL: "docking_parallel",
}


class SurpriseMission(Node):
    def __init__(self):
        super().__init__("surprise_mission")

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

        self._waypoints = self._load_waypoints(str(self.get_parameter("waypoints_file").value))

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
        # competition_manager's task system (surprise.yaml is deliberately
        # waypoint-less), so they don't get /competition/waypoints/<index>
        # for free.
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
        share = Path(get_package_share_directory("mission_surprise"))
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

        return {
            "point_12_start": to_geo_point("point_12_start"),
            "point_14_exit": to_geo_point("point_14_exit"),
            "transit": [
                to_geo_point("point_14_exit"),
                to_geo_point("point_4_1"),
                to_geo_point("point_4_2"),
                to_geo_point("point_4_3"),
                to_geo_point("point_4_4"),
                to_geo_point("point_4_5"),
                to_geo_point("point_13_parallel_approach"),
            ],
        }

    def _publish_reference_points(self, qos):
        transit_labels = ("point_14_exit", "point_4_1", "point_4_2", "point_4_3", "point_4_4",
                           "point_4_5", "point_13_parallel_approach")

        labeled_points = [("point_12_start", self._waypoints["point_12_start"])]
        labeled_points += list(zip(transit_labels, self._waypoints["transit"]))

        for label, point in labeled_points:
            publisher = self.create_publisher(NavSatFix, f"/surprise_mission/points/{label}", qos)

            message = NavSatFix()
            message.header.frame_id = "map"
            message.header.stamp = self.get_clock().now().to_msg()
            message.status.status = NavSatStatus.STATUS_FIX
            message.status.service = NavSatStatus.SERVICE_GPS
            message.latitude = point.latitude
            message.longitude = point.longitude
            message.altitude = point.altitude

            publisher.publish(message)
            # Deliberately not stored/destroyed later — static reference
            # points for the lifetime of this node, same as mission_docking.

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
            self.get_logger().error(f"No GPS fix within {self._gps_timeout_s:.0f}s — aborting surprise mission")
            return False
        if not self.wait_for_services():
            self.get_logger().error(
                "mission_manager/competition_manager services unavailable — aborting surprise mission"
            )
            return False

        # Defensive: a previous run (or a respawn mid-run) could leave
        # competition_manager's task selection somewhere that incorrectly
        # suppresses collision/buoy avoidance or blocks the next set_task
        # call.
        self._clear_competition_task()

        self.get_logger().info("Phase 1/3: normal docking")
        if not self._run_docking_phase(CompetitionState.TASK_DOCKING):
            self.get_logger().error(
                f"Normal docking did not succeed within {self._max_docking_attempts} attempt(s) "
                "— aborting surprise mission"
            )
            return False
        self.get_logger().info("Normal docking succeeded")

        # Re-enable collision/buoy avoidance (and, via TASK_SURPRISE below,
        # cardinal/buoy COLREG handling) for the open-water leg — nothing
        # else clears TASK_DOCKING back off once the dock manoeuvre itself
        # is done.
        self._clear_competition_task()

        self.get_logger().info(
            "Phase 2/3: open-water transit (point 14 -> 4.1..4.5 -> point 13)"
        )
        if not self._select_task(CompetitionState.TASK_SURPRISE):
            self.get_logger().error("Could not select TASK_SURPRISE — aborting surprise mission")
            return False
        if not self._run_transit_leg(self._waypoints["transit"]):
            self.get_logger().error("Open-water transit failed — aborting surprise mission")
            return False
        self.get_logger().info("Reached point 13 (parallel-dock approach)")

        self._clear_competition_task()

        self.get_logger().info("Phase 3/3: parallel docking")
        if not self._run_docking_phase(CompetitionState.TASK_DOCKING_PARALLEL):
            self.get_logger().error(
                f"Parallel docking did not succeed within {self._max_docking_attempts} attempt(s) "
                "— aborting surprise mission"
            )
            return False

        self.get_logger().info(
            "Parallel docking succeeded — Task 9.4 surprise mission complete "
            "(remaining stationary in berth, no exit manoeuvre)"
        )
        return True

    def _run_transit_leg(self, waypoints):
        req = StartMission.Request(waypoints=waypoints)

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

    def _run_docking_phase(self, task_id):
        """Run the close-range docking manoeuvre for task_id (TASK_DOCKING
        or TASK_DOCKING_PARALLEL), retrying up to max_docking_attempts
        times. Shared between phases 1 and 3 — the retry-and-clear pattern
        is identical for both, only the selected task id differs.
        """
        task_name = _TASK_NAMES[task_id]

        for attempt in range(1, self._max_docking_attempts + 1):
            self.get_logger().info(f"{task_name} attempt {attempt}/{self._max_docking_attempts}")

            if not self._select_task(task_id):
                continue

            # Staleness guard: every attempt selects the same task id, so
            # filtering /competition/status on status.task alone would not
            # distinguish a leftover FAILED/SUCCEEDED from a *previous*
            # attempt from this attempt's real outcome.
            self._competition_status = None

            future = self._start_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=self._service_timeout_s)
            res = future.result()

            if res is None or not res.success:
                self.get_logger().error(
                    f"start({task_name}) rejected on attempt {attempt}: "
                    f"{res.message if res else 'no response'}"
                )
                continue

            if self._wait_for_docking_outcome(task_id):
                return True

            self.get_logger().warning(f"{task_name} attempt {attempt}/{self._max_docking_attempts} did not succeed")

            # Neither ExecuteDocking nor ExecuteDockingParallel is expected
            # to ever return BT::NodeStatus::FAILURE — a stuck attempt only
            # ends via this node's own timeout below, leaving
            # competition_manager at STATE_RUNNING until explicitly
            # cleared, required for the next attempt to even be possible.
            self._clear_competition_task()

        return False

    def _wait_for_docking_outcome(self, task_id):
        deadline = self.get_clock().now().nanoseconds + int(self._docking_timeout_s * 1e9)

        while self.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(self, timeout_sec=0.5)
            status = self._competition_status
            if status is None or status.task != task_id:
                continue
            if status.state == CompetitionState.STATE_SUCCEEDED:
                return True
            if status.state in (CompetitionState.STATE_FAILED, CompetitionState.STATE_ABORTED):
                self.get_logger().error(f"Docking attempt ended in state {status.state}: {status.message}")
                return False

        self.get_logger().error(f"Docking attempt timed out after {self._docking_timeout_s:.0f}s")
        return False

    def _select_task(self, task_id):
        future = self._set_task_client.call_async(SetCompetitionTask.Request(task=task_id))
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._service_timeout_s)
        res = future.result()

        if res is None or not res.success:
            self.get_logger().error(
                f"set_task({task_id}) rejected: {res.message if res else 'no response'}"
            )
            return False

        return True

    def _clear_competition_task(self):
        """Select TASK_NONE. Best-effort — never blocks the sequence.

        Rejection is expected and harmless if competition_manager is
        already at TASK_NONE or a mission is currently pending; either way
        there's nothing to clear, so this doesn't retry or fail the
        mission on rejection.
        """
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
    node = SurpriseMission()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
