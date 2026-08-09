"""High-level competition task and lifecycle coordination."""

import rclpy
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

from competition_manager.task_loader import CompetitionTask
from competition_manager.task_loader import CompetitionTaskError
from competition_manager.task_loader import CompetitionTaskLoader


class CompetitionManager(Node):
    """Maintain and publish the current Njord competition task state."""

    TASK_NAMES = {
        CompetitionState.TASK_NONE: "TASK_NONE",
        CompetitionState.TASK_MANEUVERING: "TASK_MANEUVERING",
        CompetitionState.TASK_PATH_FINDING: "TASK_PATH_FINDING",
        CompetitionState.TASK_COLLISION_AVOIDANCE:
            "TASK_COLLISION_AVOIDANCE",
        CompetitionState.TASK_DOCKING: "TASK_DOCKING",
        CompetitionState.TASK_SURPRISE: "TASK_SURPRISE",
    }

    STATE_NAMES = {
        CompetitionState.STATE_IDLE: "STATE_IDLE",
        CompetitionState.STATE_READY: "STATE_READY",
        CompetitionState.STATE_RUNNING: "STATE_RUNNING",
        CompetitionState.STATE_SUCCEEDED: "STATE_SUCCEEDED",
        CompetitionState.STATE_FAILED: "STATE_FAILED",
        CompetitionState.STATE_ABORTED: "STATE_ABORTED",
    }

    def __init__(self) -> None:
        super().__init__("competition_manager")

        transient_local_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._status_publisher = self.create_publisher(
            CompetitionState,
            "/competition/status",
            transient_local_qos,
        )

        self._mission_status_subscription = self.create_subscription(
            MissionStatus,
            "/mission/status",
            self.mission_status_callback,
            transient_local_qos,
        )

        self._set_task_service = self.create_service(
            SetCompetitionTask,
            "/competition/set_task",
            self.set_task_callback,
        )

        self._start_service = self.create_service(
            Trigger,
            "/competition/start",
            self.start_callback,
        )

        self._complete_service = self.create_service(
            Trigger,
            "/competition/complete",
            self.complete_callback,
        )

        self._mission_start_client = self.create_client(
            StartMission,
            "/mission/start",
        )

        self._task_loader = CompetitionTaskLoader()

        self._current_task = CompetitionState.TASK_NONE
        self._current_task_definition: CompetitionTask | None = None

        # GUI support (spec 9.1: "the GUI must plot the position on the ASV
        # in relation to the given GPS points"): one latched NavSatFix per
        # waypoint of the currently selected task, republished whenever the
        # task changes, so a Foxglove Map panel can plot them as static pins
        # alongside the live ASV position (/gps_driver/gps_raw, which also
        # gives the "route taken" trail for free via the Map panel's
        # built-in position history). Recreated (not just republished) on
        # every task change so a shorter waypoint list doesn't leave stale
        # pins from a longer previous task's list still latched on unused
        # topic indices.
        self._waypoint_publishers: list = []

        self._current_state = CompetitionState.STATE_IDLE
        self._current_message = "Competition Manager initialized"

        self._latest_mission_state = MissionStatus.IDLE
        self._mission_start_pending = False

        self.publish_status()

        self.get_logger().info(
            "Competition Manager started: "
            "task=TASK_NONE, state=STATE_IDLE"
        )

    def set_task_callback(
        self,
        request: SetCompetitionTask.Request,
        response: SetCompetitionTask.Response,
    ) -> SetCompetitionTask.Response:
        """Select, validate, or clear the active competition task."""

        if self._mission_start_pending:
            response.success = False
            response.message = (
                "Cannot change task while a mission start request "
                "is pending"
            )
            self.get_logger().warning(response.message)
            return response

        if request.task == CompetitionState.TASK_NONE:
            # Clearing to TASK_NONE is how a direct-BT task (waypoint-less
            # — docking, collision_avoidance, surprise) gets aborted: those
            # tasks have no failure path of their own (e.g. docking_nodes.cpp
            # never returns BT::NodeStatus::FAILURE, it just keeps trying
            # indefinitely), so this is the only way to end a stuck one.
            # It must NOT be exempted from the STATE_RUNNING guard for a
            # waypoint-based task (maneuvering/path_finding) though — that
            # would silently orphan mission_manager, which is not listening
            # for this and would just keep driving the boat toward its
            # waypoints regardless. Those must still go through
            # /mission/abort first, same as today.
            current_task_has_waypoints = bool(
                self._current_task_definition
                and self._current_task_definition.waypoints
            )

            if (
                self._current_state == CompetitionState.STATE_RUNNING
                and current_task_has_waypoints
            ):
                response.success = False
                response.message = (
                    "Cannot clear task while a waypoint-based mission is "
                    "running — call /mission/abort first"
                )
                self.get_logger().warning(response.message)
                return response

            previous_task = self._current_task

            self._current_task = CompetitionState.TASK_NONE
            self._current_task_definition = None
            self._latest_mission_state = MissionStatus.IDLE

            self._publish_task_waypoints(None)

            self.set_competition_state(
                CompetitionState.STATE_IDLE,
                "Competition task cleared",
                force_publish=(
                    previous_task != CompetitionState.TASK_NONE
                ),
            )

            response.success = True
            response.message = self._current_message

            self.get_logger().info(
                "Competition task updated: "
                "task=TASK_NONE, state=STATE_IDLE"
            )
            return response

        if self._current_state == CompetitionState.STATE_RUNNING:
            response.success = False
            response.message = (
                "Cannot change task while a competition task is running"
            )
            self.get_logger().warning(response.message)
            return response

        if request.task not in self.TASK_NAMES:
            response.success = False
            response.message = (
                f"Invalid competition task value: {request.task}"
            )
            self.get_logger().warning(response.message)
            return response

        try:
            task_definition = self._task_loader.load(request.task)
        except CompetitionTaskError as error:
            response.success = False
            response.message = (
                "Could not select competition task: "
                f"{error}"
            )
            self.get_logger().error(response.message)
            return response

        previous_task = self._current_task

        self._current_task = request.task
        self._current_task_definition = task_definition
        self._latest_mission_state = MissionStatus.IDLE

        self._publish_task_waypoints(task_definition)

        self.set_competition_state(
            CompetitionState.STATE_READY,
            (
                "Competition task selected: "
                f"{self.task_name(request.task)}"
            ),
            force_publish=(previous_task != request.task),
        )

        response.success = True
        response.message = self._current_message

        self.get_logger().info(
            "Competition task definition loaded: "
            f"id={task_definition.task_id}, "
            f"name={task_definition.name}, "
            f"waypoints={len(task_definition.waypoints)}"
        )

        self.get_logger().info(
            "Competition task updated: "
            f"task={self.task_name(self._current_task)}, "
            f"state={self.state_name(self._current_state)}"
        )

        return response

    def start_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Start the selected competition task."""

        del request

        if self._current_task == CompetitionState.TASK_NONE:
            response.success = False
            response.message = (
                "Cannot start competition task: no task is selected"
            )
            self.get_logger().warning(response.message)
            return response

        if self._current_task_definition is None:
            response.success = False
            response.message = (
                "Cannot start competition task: "
                "task definition is not loaded"
            )
            self.get_logger().error(response.message)
            return response

        if self._current_state != CompetitionState.STATE_READY:
            response.success = False
            response.message = (
                "Cannot start competition task while state is "
                f"{self.state_name(self._current_state)}"
            )
            self.get_logger().warning(response.message)
            return response

        if self._mission_start_pending:
            response.success = False
            response.message = (
                "Competition task start request is already pending"
            )
            self.get_logger().warning(response.message)
            return response

        waypoint_required_tasks = (
            CompetitionState.TASK_MANEUVERING,
            CompetitionState.TASK_PATH_FINDING,
        )

        if (
            self._current_task in waypoint_required_tasks
            and not self._current_task_definition.waypoints
        ):
            response.success = False
            response.message = (
                "Cannot start competition task: "
                f"{self._current_task_definition.name} has no configured "
                "waypoints"
            )

            self.get_logger().error(response.message)
            return response

        # Tasks without geographic waypoints are executed directly by the
        # Behavior Tree instead of MissionManager. This applies to behaviors
        # such as collision avoidance and docking, whose completion is
        # reported through /competition/complete.
        if not self._current_task_definition.waypoints:
            self.set_competition_state(
                CompetitionState.STATE_RUNNING,
                (
                    "Competition task started: "
                    f"{self._current_task_definition.name}"
                ),
            )

            response.success = True
            response.message = self._current_message

            self.get_logger().info(
                "Direct Behavior Tree task started without "
                "MissionManager waypoints: "
                f"task={self._current_task_definition.task_id}"
            )
            return response

        if not self._mission_start_client.wait_for_service(
            timeout_sec=1.0
        ):
            response.success = False
            response.message = (
                "Cannot start competition task: "
                "/mission/start service is unavailable"
            )
            self.get_logger().error(response.message)
            return response

        mission_request = StartMission.Request()
        mission_request.waypoints = [
            self.create_geo_point(waypoint)
            for waypoint in self._current_task_definition.waypoints
        ]

        self._mission_start_pending = True

        future = self._mission_start_client.call_async(
            mission_request
        )
        future.add_done_callback(
            self.mission_start_response_callback
        )

        response.success = True
        response.message = (
            "Mission start requested for "
            f"{self._current_task_definition.name}"
        )

        self.get_logger().info(
            "Mission start request sent: "
            f"task={self._current_task_definition.task_id}, "
            f"waypoints={len(mission_request.waypoints)}"
        )

        return response

    def complete_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Mark a directly executed competition task as successful."""

        del request

        if self._current_task == CompetitionState.TASK_NONE:
            response.success = False
            response.message = (
                "Cannot complete competition task: no task is selected"
            )
            self.get_logger().warning(response.message)
            return response

        if self._current_task_definition is None:
            response.success = False
            response.message = (
                "Cannot complete competition task: "
                "task definition is unavailable"
            )
            self.get_logger().error(response.message)
            return response

        if self._current_state == CompetitionState.STATE_SUCCEEDED:
            response.success = True
            response.message = self._current_message
            return response

        if self._current_state != CompetitionState.STATE_RUNNING:
            response.success = False
            response.message = (
                "Cannot complete competition task while state is "
                f"{self.state_name(self._current_state)}"
            )
            self.get_logger().warning(response.message)
            return response

        if self._current_task_definition.waypoints:
            response.success = False
            response.message = (
                "Waypoint-based tasks must complete through "
                "/mission/status"
            )
            self.get_logger().warning(response.message)
            return response

        self.set_competition_state(
            CompetitionState.STATE_SUCCEEDED,
            (
                "Competition task succeeded: "
                f"{self._current_task_definition.name}"
            ),
        )

        response.success = True
        response.message = self._current_message

        self.get_logger().info(
            "Direct Behavior Tree task reported completion: "
            f"task={self._current_task_definition.task_id}"
        )

        return response

    def mission_start_response_callback(self, future) -> None:
        """Handle MissionManager's response to /mission/start."""

        self._mission_start_pending = False

        try:
            result = future.result()
        except Exception as error:
            message = (
                "Mission start service call failed: "
                f"{error}"
            )

            self.set_competition_state(
                CompetitionState.STATE_READY,
                message,
            )
            self.get_logger().error(message)
            return

        if result is None:
            message = "Mission start service returned no response"

            self.set_competition_state(
                CompetitionState.STATE_READY,
                message,
            )
            self.get_logger().error(message)
            return

        if not result.success:
            message = (
                "MissionManager rejected competition task: "
                f"{result.message}"
            )

            self.set_competition_state(
                CompetitionState.STATE_READY,
                message,
            )
            self.get_logger().warning(message)
            return

        if self._current_task_definition is None:
            message = (
                "Mission started, but the competition task "
                "definition is unavailable"
            )

            self.set_competition_state(
                CompetitionState.STATE_FAILED,
                message,
            )
            self.get_logger().error(message)
            return

        self.set_competition_state(
            CompetitionState.STATE_RUNNING,
            (
                "Competition task started: "
                f"{self._current_task_definition.name}"
            ),
        )

        self.get_logger().info(
            "MissionManager accepted competition task: "
            f"{result.message}"
        )

    def mission_status_callback(
        self,
        message: MissionStatus,
    ) -> None:
        """Map Mission Manager terminal states to the active task."""

        previous_mission_state = self._latest_mission_state
        self._latest_mission_state = message.state

        if self._current_task == CompetitionState.TASK_NONE:
            return

        if self._current_state != CompetitionState.STATE_RUNNING:
            return

        if message.state == MissionStatus.SUCCEEDED:
            self.set_competition_state(
                CompetitionState.STATE_SUCCEEDED,
                self.mission_message(
                    "succeeded",
                    message.message,
                ),
            )
            return

        if message.state == MissionStatus.FAILED:
            self.set_competition_state(
                CompetitionState.STATE_FAILED,
                self.mission_message(
                    "failed",
                    message.message,
                ),
            )
            return

        if message.state == MissionStatus.ABORTED:
            self.set_competition_state(
                CompetitionState.STATE_ABORTED,
                self.mission_message(
                    "aborted",
                    message.message,
                ),
            )
            return

        if message.state != previous_mission_state:
            self.get_logger().debug(
                "Mission state change received while competition task "
                f"is running: mission_state={message.state}"
            )

    def set_competition_state(
        self,
        new_state: int,
        message: str,
        force_publish: bool = False,
    ) -> bool:
        """Update and publish a competition lifecycle state."""

        state_changed = new_state != self._current_state
        message_changed = message != self._current_message

        if not state_changed and not message_changed and not force_publish:
            return False

        previous_state = self._current_state

        self._current_state = new_state
        self._current_message = message

        self.publish_status()

        if state_changed:
            self.get_logger().info(
                "Competition lifecycle transition: "
                f"{self.state_name(previous_state)} -> "
                f"{self.state_name(new_state)}; "
                f"task={self.task_name(self._current_task)}"
            )

        return True

    def publish_status(self) -> None:
        """Publish the current competition task and lifecycle state."""

        message = CompetitionState()
        message.task = self._current_task
        message.state = self._current_state
        message.message = self._current_message

        self._status_publisher.publish(message)

    def _publish_task_waypoints(
        self,
        task_definition: "CompetitionTask | None",
    ) -> None:
        """(Re)publish the selected task's waypoints for GUI plotting.

        Each waypoint gets its own latched (TRANSIENT_LOCAL) NavSatFix
        publisher on /competition/waypoints/<index>, so a Foxglove Map panel
        opened after task selection still sees them immediately. Publishers
        from the previous task are destroyed first — a shorter new list must
        not leave old higher-index pins latched on screen.
        """

        for publisher in self._waypoint_publishers:
            self.destroy_publisher(publisher)

        self._waypoint_publishers = []

        if task_definition is None:
            return

        waypoint_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        for index, waypoint in enumerate(task_definition.waypoints):
            publisher = self.create_publisher(
                NavSatFix,
                f"/competition/waypoints/{index}",
                waypoint_qos,
            )

            message = NavSatFix()
            message.header.frame_id = "map"
            message.header.stamp = self.get_clock().now().to_msg()
            message.status.status = NavSatStatus.STATUS_FIX
            message.status.service = NavSatStatus.SERVICE_GPS
            message.latitude = waypoint.latitude
            message.longitude = waypoint.longitude
            message.altitude = waypoint.altitude

            publisher.publish(message)

            self._waypoint_publishers.append(publisher)

        self.get_logger().info(
            "Published "
            f"{len(task_definition.waypoints)} waypoint(s) for GUI "
            f"plotting: {task_definition.name} "
            "(/competition/waypoints/0.."
            f"{len(task_definition.waypoints) - 1})"
        )

    def mission_message(
        self,
        lifecycle: str,
        mission_message: str,
    ) -> str:
        """Create a readable message for a mission transition."""

        if self._current_task_definition is not None:
            task_name = self._current_task_definition.name
        else:
            task_name = self.task_name(self._current_task)

        base_message = f"Competition task {lifecycle}: {task_name}"

        if mission_message:
            return f"{base_message} — {mission_message}"

        return base_message

    @staticmethod
    def create_geo_point(waypoint) -> GeoPoint:
        """Convert a typed competition waypoint into GeoPoint."""

        point = GeoPoint()
        point.latitude = waypoint.latitude
        point.longitude = waypoint.longitude
        point.altitude = waypoint.altitude

        return point

    @classmethod
    def task_name(cls, task: int) -> str:
        """Return a readable CompetitionState task name."""

        return cls.TASK_NAMES.get(
            task,
            f"UNKNOWN_TASK_{task}",
        )

    @classmethod
    def state_name(cls, state: int) -> str:
        """Return a readable CompetitionState lifecycle name."""

        return cls.STATE_NAMES.get(
            state,
            f"UNKNOWN_STATE_{state}",
        )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = CompetitionManager()

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
