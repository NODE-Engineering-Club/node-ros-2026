"""Load and validate Njord competition task definitions from YAML."""

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory
from njord_msgs.msg import CompetitionState


@dataclass(frozen=True)
class CompetitionWaypoint:
    """One geographic waypoint belonging to a competition task."""

    latitude: float
    longitude: float
    altitude: float = 0.0


@dataclass(frozen=True)
class CompetitionTask:
    """Validated competition task definition."""

    task_value: int
    task_id: str
    name: str
    description: str
    waypoints: tuple[CompetitionWaypoint, ...]


class CompetitionTaskError(Exception):
    """Base exception for competition task loading errors."""


class CompetitionTaskNotFoundError(CompetitionTaskError):
    """Raised when a task definition file cannot be found."""


class CompetitionTaskValidationError(CompetitionTaskError):
    """Raised when a task definition does not match the expected schema."""


class CompetitionTaskLoader:
    """Locate, load, and validate competition task YAML files."""

    TASK_FILES = {
        CompetitionState.TASK_MANEUVERING: "maneuvering.yaml",
        CompetitionState.TASK_PATH_FINDING: "path_finding.yaml",
        CompetitionState.TASK_COLLISION_AVOIDANCE:
            "collision_avoidance.yaml",
        CompetitionState.TASK_DOCKING: "docking.yaml",
        CompetitionState.TASK_SURPRISE: "surprise.yaml",
        CompetitionState.TASK_DOCKING_PARALLEL: "docking_parallel.yaml",
    }

    EXPECTED_TASK_IDS = {
        CompetitionState.TASK_MANEUVERING: "maneuvering",
        CompetitionState.TASK_PATH_FINDING: "path_finding",
        CompetitionState.TASK_COLLISION_AVOIDANCE:
            "collision_avoidance",
        CompetitionState.TASK_DOCKING: "docking",
        CompetitionState.TASK_SURPRISE: "surprise",
        CompetitionState.TASK_DOCKING_PARALLEL: "docking_parallel",
    }

    SUPPORTED_SCHEMA_VERSION = 1

    def __init__(self) -> None:
        package_share = Path(
            get_package_share_directory("competition_manager")
        )

        self._task_directory = package_share / "competition_tasks"

    def load(self, task: int) -> CompetitionTask:
        """Load, validate, and convert one task definition."""

        if task == CompetitionState.TASK_NONE:
            raise CompetitionTaskValidationError(
                "TASK_NONE does not have a competition task definition"
            )

        filename = self.TASK_FILES.get(task)

        if filename is None:
            raise CompetitionTaskValidationError(
                f"Unknown competition task value: {task}"
            )

        task_path = self._task_directory / filename

        if not task_path.is_file():
            raise CompetitionTaskNotFoundError(
                f"Competition task file not found: {task_path}"
            )

        try:
            with task_path.open(
                "r",
                encoding="utf-8",
            ) as task_file:
                data = yaml.safe_load(task_file)
        except yaml.YAMLError as error:
            raise CompetitionTaskValidationError(
                f"Invalid YAML in {task_path}: {error}"
            ) from error
        except OSError as error:
            raise CompetitionTaskError(
                f"Could not read competition task file "
                f"{task_path}: {error}"
            ) from error

        self._validate(
            task=task,
            data=data,
            task_path=task_path,
        )

        return self._convert(
            task=task,
            data=data,
        )

    def _validate(
        self,
        task: int,
        data: Any,
        task_path: Path,
    ) -> None:
        """Validate the Competition Task Schema v1."""

        if not isinstance(data, dict):
            raise CompetitionTaskValidationError(
                f"{task_path} must contain a YAML mapping"
            )

        version = data.get("version")

        if version != self.SUPPORTED_SCHEMA_VERSION:
            raise CompetitionTaskValidationError(
                f"{task_path} uses unsupported schema version "
                f"{version!r}; expected "
                f"{self.SUPPORTED_SCHEMA_VERSION}"
            )

        task_data = data.get("task")

        if not isinstance(task_data, dict):
            raise CompetitionTaskValidationError(
                f"{task_path} must contain a 'task' mapping"
            )

        expected_task_id = self.EXPECTED_TASK_IDS[task]
        actual_task_id = task_data.get("id")

        if actual_task_id != expected_task_id:
            raise CompetitionTaskValidationError(
                f"{task_path} has task.id={actual_task_id!r}; "
                f"expected {expected_task_id!r}"
            )

        task_name = task_data.get("name")

        if not isinstance(task_name, str) or not task_name.strip():
            raise CompetitionTaskValidationError(
                f"{task_path} must contain a non-empty task.name"
            )

        task_description = task_data.get("description")

        if (
            not isinstance(task_description, str)
            or not task_description.strip()
        ):
            raise CompetitionTaskValidationError(
                f"{task_path} must contain a non-empty "
                "task.description"
            )

        mission_data = data.get("mission")

        if not isinstance(mission_data, dict):
            raise CompetitionTaskValidationError(
                f"{task_path} must contain a 'mission' mapping"
            )

        waypoints = mission_data.get("waypoints")

        if not isinstance(waypoints, list):
            raise CompetitionTaskValidationError(
                f"{task_path} mission.waypoints must be a list"
            )

        for index, waypoint in enumerate(waypoints):
            self._validate_waypoint(
                waypoint=waypoint,
                index=index,
                task_path=task_path,
            )

    @staticmethod
    def _validate_waypoint(
        waypoint: Any,
        index: int,
        task_path: Path,
    ) -> None:
        """Validate one geographic waypoint."""

        if not isinstance(waypoint, dict):
            raise CompetitionTaskValidationError(
                f"{task_path} waypoint {index + 1} must be a mapping"
            )

        for field in ("latitude", "longitude"):
            value = waypoint.get(field)

            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
            ):
                raise CompetitionTaskValidationError(
                    f"{task_path} waypoint {index + 1} field "
                    f"{field!r} must be numeric"
                )

        latitude = float(waypoint["latitude"])
        longitude = float(waypoint["longitude"])

        if not -90.0 <= latitude <= 90.0:
            raise CompetitionTaskValidationError(
                f"{task_path} waypoint {index + 1} latitude "
                "must be between -90 and 90"
            )

        if not -180.0 <= longitude <= 180.0:
            raise CompetitionTaskValidationError(
                f"{task_path} waypoint {index + 1} longitude "
                "must be between -180 and 180"
            )

        if "altitude" in waypoint:
            altitude = waypoint["altitude"]

            if (
                isinstance(altitude, bool)
                or not isinstance(altitude, (int, float))
            ):
                raise CompetitionTaskValidationError(
                    f"{task_path} waypoint {index + 1} altitude "
                    "must be numeric"
                )

    @staticmethod
    def _convert(
        task: int,
        data: dict[str, Any],
    ) -> CompetitionTask:
        """Convert validated YAML data into typed immutable objects."""

        task_data = data["task"]
        waypoint_data = data["mission"]["waypoints"]

        waypoints = tuple(
            CompetitionWaypoint(
                latitude=float(waypoint["latitude"]),
                longitude=float(waypoint["longitude"]),
                altitude=float(waypoint.get("altitude", 0.0)),
            )
            for waypoint in waypoint_data
        )

        return CompetitionTask(
            task_value=task,
            task_id=task_data["id"],
            name=task_data["name"].strip(),
            description=task_data["description"].strip(),
            waypoints=waypoints,
        )
