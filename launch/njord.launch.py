import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# Works in both dev (/workspace/launch/) and prod (/launch/)
_CONFIG = os.path.join(os.path.dirname(__file__), "..", "config")


def _cmd(name):
    """Find an entry point on PATH — works in dev (system pip) and prod (venv)."""
    path = shutil.which(name)
    if not path:
        raise RuntimeError(f"Entry point '{name}' not found on PATH")
    return [path]


def generate_launch_description():
    nav2_launch = os.path.join(
        get_package_share_directory("nav2_bringup"), "launch", "navigation_launch.py"
    )

    return LaunchDescription(
        [
            Node(
                package="mavros",
                executable="mavros_node",
                name="mavros",
                parameters=[
                    {
                        "fcu_url": "udp://:14550@localhost:14555",
                        "gcs_url": "udp://@localhost:14556",
                        "tgt_system": 1,
                        "tgt_component": 1,
                    }
                ],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_node",
                parameters=[os.path.join(_CONFIG, "ekf.yaml")],
            ),
            Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform_node",
                parameters=[os.path.join(_CONFIG, "navsat.yaml")],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                launch_arguments={
                    "params_file": os.path.join(_CONFIG, "nav2_params.yaml"),
                    "use_collision_monitor": "False",
                }.items(),
            ),
            ExecuteProcess(cmd=_cmd("sensors-launch-all"), output="screen"),
            ExecuteProcess(cmd=_cmd("vision-detector-node"), output="screen"),
            ExecuteProcess(cmd=_cmd("perception-launch-all"), output="screen"),
            ExecuteProcess(cmd=_cmd("control-launch-all"), output="screen"),
            ExecuteProcess(cmd=_cmd("mission-manager"), output="screen"),
        ]
    )
