import rclpy
from njord_msgs.msg import CompetitionState
from njord_msgs.srv import SetCompetitionTask
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


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

        competition_status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._status_publisher = self.create_publisher(
            CompetitionState,
            "/competition/status",
            competition_status_qos,
        )

        self._set_task_service = self.create_service(
            SetCompetitionTask,
            "/competition/set_task",
            self.set_task_callback,
        )

        self._current_task = CompetitionState.TASK_NONE
        self._current_state = CompetitionState.STATE_IDLE
        self._current_message = "Competition Manager initialized"

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
        """Select the active competition task."""

        if request.task not in self.TASK_NAMES:
            response.success = False
            response.message = (
                f"Invalid competition task value: {request.task}"
            )

            self.get_logger().warning(response.message)
            return response

        self._current_task = request.task

        if request.task == CompetitionState.TASK_NONE:
            self._current_state = CompetitionState.STATE_IDLE
            self._current_message = "Competition task cleared"
        else:
            self._current_state = CompetitionState.STATE_READY
            task_name = self.TASK_NAMES[request.task]
            self._current_message = (
                f"Competition task selected: {task_name}"
            )

        self.publish_status()

        response.success = True
        response.message = self._current_message

        task_name = self.TASK_NAMES[self._current_task]
        state_name = self.state_name(self._current_state)

        self.get_logger().info(
            f"Competition task updated: "
            f"task={task_name}, state={state_name}"
        )

        return response

    def publish_status(self) -> None:
        """Publish the current competition state."""

        message = CompetitionState()
        message.task = self._current_task
        message.state = self._current_state
        message.message = self._current_message

        self._status_publisher.publish(message)

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
