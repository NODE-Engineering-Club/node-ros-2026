import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    nav2_launch = os.path.join(
        get_package_share_directory("nav2_bringup"), "launch", "navigation_launch.py"
    )
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={"params_file": "/navigation/config/nav2_params.yaml"}.items(),
        ),
    ])
