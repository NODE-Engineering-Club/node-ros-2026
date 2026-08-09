from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim", default_value="false"),
        Node(
            package="mission_docking_parallel",
            executable="docking_parallel_mission",
            name="docking_parallel_mission",
            output="screen",
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim")}],
        ),
    ])
