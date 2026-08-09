"""Mission sequencer for Task 3.1 (Normal Docking).

Task 3.1 needs three phases the existing competition infrastructure doesn't
stitch together on its own:

  1. GPS transit from wherever the boat starts to point 8 (the staging
     point, ~10 m from the berth per spec), driven directly through
     mission_manager's /mission/start — plain open-water Nav2 travel, not
     a scored competition task on its own, so this deliberately does not
     go through competition_manager.
  2. The close-range docking manoeuvre itself, driven by boat_bt's existing
     ExecuteDocking state machine (docking_nodes.cpp) via the normal
     competition_manager TASK_DOCKING selection — completely unchanged
     from what already runs today for a direct-BT (waypoint-less)
     competition task.
  3. GPS transit from the berth to point 9, again direct through
     mission_manager — reaching this is what actually stops the spec's
     task timer.

Point 7 (the official start line) is not itself navigated to — the boat is
expected to already be there when this node is launched; see
config/docking_3_1_waypoints.yaml.

Spec 9.3 allows 2 attempts at the docking manoeuvre. max_docking_attempts
defaults to 2 to match. Only the docking manoeuvre itself is retried — the
GPS transit legs are not (matching mission_maneuvering_pathfinding's
philosophy: a transit failure stops the attempt, it is not silently
retried). boat_bt_node already resets its docking state machine on every
new (TASK_DOCKING, RUNNING) transition (see resetDockingController() in
docking_nodes.cpp / boat_bt_node.cpp's competition_status_callback), so
re-selecting TASK_DOCKING for a second attempt is already safe with zero
changes to that code.

Readiness checks before starting: a GPS fix must have been received at
least once, and both mission_manager's and competition_manager's services
must be available.

Failure handling: a transit leg or docking failure stops the sequence —
this node does not attempt to cancel or recover a stuck leg/task, same
philosophy as mission_maneuvering_pathfinding.

Competition task reset: simple_boat.xml's GlobalSafety subtree suppresses
generic collision avoidance ONLY while competition_manager's current task
is "docking" (dock geometry must not be treated as an obstacle during the
close approach). Nothing else clears that back automatically once docking
finishes, so this node explicitly re-selects TASK_NONE: defensively before
starting (in case a previous docking run left TASK_DOCKING selected),
between failed docking attempts (see _run_docking — also required to make
a second attempt possible at all, since docking_nodes.cpp's ExecuteDocking
never returns BT::NodeStatus::FAILURE, so a stuck attempt only ever ends
via this node's own timeout, leaving competition_manager at STATE_RUNNING
until explicitly cleared), and right after a successful dock, before the
transit out to point 9, so collision/buoy avoidance is genuinely active
again for the exit leg rather than staying suppressed by a stale task
selection. competition_manager's set_task_callback allows clearing to
TASK_NONE while STATE_RUNNING specifically for waypoint-less tasks like
docking (see competition_manager.py) — waypoint-based tasks still require
/mission/abort first, unchanged.
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

DEFAULT_WAYPOINTS_FILE = "docking_3_1_waypoints.yaml"

DEFAULT_GPS_TIMEOUT_S = 30.0
DEFAULT_SERVICE_TIMEOUT_S = 30.0
DEFAULT_TRANSIT_TIMEOUT_S = 120.0
DEFAULT_DOCKING_TIMEOUT_S = 180.0
DEFAULT_MAX_DOCKING_ATTEMPTS = 2


class DockingMission(Node):
    def __init__(self):
        super().__init__("docking_mission")

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

        self._point_7, self._point_8, self._point_9 = self._load_waypoints(
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

        # GUI support (same idea as competition_manager's
        # /competition/waypoints/<index>, which this task's points don't go
        # through since they're not part of competition_manager's task
        # system): one latched NavSatFix per point so points 7/8/9 are
        # plottable on a Foxglove Map panel alongside the live ASV position.
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
        share = Path(get_package_share_directory("mission_docking"))
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
            to_geo_point("point_7_start"),
            to_geo_point("point_8_staging"),
            to_geo_point("point_9_finish"),
        )

    def _publish_reference_points(self, qos):
        for label, point in (
            ("point_7_start", self._point_7),
            ("point_8_staging", self._point_8),
            ("point_9_finish", self._point_9),
        ):
            publisher = self.create_publisher(NavSatFix, f"/docking_mission/points/{label}", qos)

            message = NavSatFix()
            message.header.frame_id = "map"
            message.header.stamp = self.get_clock().now().to_msg()
            message.status.status = NavSatStatus.STATUS_FIX
            message.status.service = NavSatStatus.SERVICE_GPS
            message.latitude = point.latitude
            message.longitude = point.longitude
            message.altitude = point.altitude

            publisher.publish(message)
            # Deliberately not stored/destroyed later — these are static
            # reference points for the lifetime of this node, unlike
            # competition_manager's per-task waypoint publishers which are
            # recreated on every task change.

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

        # Defensive: a previous docking run (or a respawn mid-run) could
        # leave competition_manager's task selection at TASK_DOCKING, which
        # would incorrectly suppress collision/buoy avoidance (see
        # GlobalSafety in simple_boat.xml) during this leg's transit.
        self._clear_competition_task()

        self.get_logger().info("Transit leg 1/2: heading to point 8 (staging point)")
        if not self._run_transit_leg(self._point_8):
            self.get_logger().error("Transit to point 8 failed — aborting docking mission")
            return False
        self.get_logger().info("Reached point 8")

        if not self._run_docking():
            self.get_logger().error(
                f"Docking did not succeed within {self._max_docking_attempts} attempt(s) "
                "— aborting docking mission"
            )
            return False
        self.get_logger().info("Docking succeeded")

        # Re-enable collision/buoy avoidance for the exit transit — nothing
        # else clears TASK_DOCKING back off once the dock manoeuvre itself
        # is done, and GlobalSafety's suppression is keyed purely on the
        # currently-selected task, not on whether ExecuteDocking is
        # actually running right now.
        self._clear_competition_task()

        self.get_logger().info("Transit leg 2/2: heading to point 9 (finish point)")
        if not self._run_transit_leg(self._point_9):
            self.get_logger().error(
                "Transit to point 9 failed — docking succeeded but the mission did not "
                "complete cleanly"
            )
            return False

        self.get_logger().info("Reached point 9 — Task 3.1 docking mission complete")
        return True

    def _run_transit_leg(self, point):
        req = StartMission.Request(waypoints=[point])

        # Discard any status left over from a previous leg/run before
        # sending the request — /mission/status is TRANSIENT_LOCAL, so a
        # stale message would otherwise look like an instantaneous result
        # for this leg. See docstring for the general staleness hazard
        # this guards against.
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
            self.get_logger().info(f"Docking attempt {attempt}/{self._max_docking_attempts}")

            set_req = SetCompetitionTask.Request(task=CompetitionState.TASK_DOCKING)
            future = self._set_task_client.call_async(set_req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=self._service_timeout_s)
            res = future.result()

            if res is None or not res.success:
                self.get_logger().error(
                    f"set_task(docking) rejected on attempt {attempt}: "
                    f"{res.message if res else 'no response'}"
                )
                continue

            # Same staleness guard as _run_transit_leg, and doubly
            # important here: every attempt selects the *same* task id
            # (TASK_DOCKING), so filtering /competition/status on
            # status.task alone (as mission_maneuvering_pathfinding does
            # across genuinely different tasks) would not distinguish a
            # leftover FAILED/SUCCEEDED from a *previous* docking attempt
            # from this attempt's real outcome.
            self._competition_status = None

            future = self._start_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=self._service_timeout_s)
            res = future.result()

            if res is None or not res.success:
                self.get_logger().error(
                    f"start(docking) rejected on attempt {attempt}: "
                    f"{res.message if res else 'no response'}"
                )
                continue

            if self._wait_for_docking_outcome():
                return True

            self.get_logger().warning(f"Docking attempt {attempt}/{self._max_docking_attempts} did not succeed")

            # docking_nodes.cpp's ExecuteDocking never returns
            # BT::NodeStatus::FAILURE — a target that's never found just
            # leaves it RUNNING indefinitely, so _wait_for_docking_outcome
            # above almost always ends via its own timeout, not a real
            # FAILED/ABORTED status. competition_manager stays STATE_RUNNING
            # in that case, and would reject the next attempt's set_task —
            # explicitly abort back to TASK_NONE first so attempt 2 can
            # actually start.
            self._clear_competition_task()

        return False

    def _wait_for_docking_outcome(self):
        deadline = self.get_clock().now().nanoseconds + int(self._docking_timeout_s * 1e9)

        while self.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(self, timeout_sec=0.5)
            status = self._competition_status
            if status is None or status.task != CompetitionState.TASK_DOCKING:
                continue
            if status.state == CompetitionState.STATE_SUCCEEDED:
                return True
            if status.state in (CompetitionState.STATE_FAILED, CompetitionState.STATE_ABORTED):
                self.get_logger().error(f"Docking attempt ended in state {status.state}: {status.message}")
                return False

        self.get_logger().error(f"Docking attempt timed out after {self._docking_timeout_s:.0f}s")
        return False

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
    node = DockingMission()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
