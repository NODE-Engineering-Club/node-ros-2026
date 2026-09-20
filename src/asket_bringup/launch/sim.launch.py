"""Bring the whole stack up with no hardware present.

    ros2 launch asket_bringup sim.launch.py

This is shorthand for ``asket.launch.py sim:=true``. Keeping it as a separate,
obvious entry point matters: the first thing anybody does with this repository
is run it with nothing plugged in.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("asket_bringup"), "launch", "asket.launch.py"]
                    )
                ),
                launch_arguments={"sim": "true", "sonar_host": "127.0.0.1"}.items(),
            )
        ]
    )
