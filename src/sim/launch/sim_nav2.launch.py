"""
Launch the 2D Python simulation together with Nav2.

The project-specific navigation launch excludes the docking server until
the docking competition task is implemented and configured.
"""

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Create the simulation and Nav2 launch description."""

    bringup_share = get_package_share_directory("bringup")
    bringup_config = bringup_share + "/config"
    description_share = get_package_share_directory("description")
    sim_share = get_package_share_directory("sim")

    robot_description = xacro.process_file(
        description_share + "/urdf/asket.urdf.xacro"
    ).toxml()

    launch_arguments = [
        DeclareLaunchArgument(
            "enable_nav2",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "enable_mission",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "enable_rviz",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "enable_foxglove",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "enable_perception",
            default_value="true",
        ),
    ]

    sim_time = {
        "use_sim_time": False,
    }

    nodes = [
        Node(
            package="sim",
            executable="simulator",
            name="asket_simulator",
            output="screen",
            parameters=[sim_time],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "robot_description": robot_description,
                },
                sim_time,
            ],
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_node",
            output="screen",
            parameters=[
                bringup_config + "/ekf.yaml",
                sim_time,
            ],
        ),
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform_node",
            output="screen",
            parameters=[
                bringup_config + "/navsat.yaml",
                sim_time,
            ],
            remappings=[
                (
                    "gps/fix",
                    "/gps_driver/gps_raw",
                ),
                (
                    "imu/data",
                    "/imu/data",
                ),
                (
                    "odometry/filtered",
                    "/odometry/filtered",
                ),
            ],
        ),
        Node(
            package="perception",
            executable="lidar_obstacle_node",
            name="lidar_obstacle_node",
            output="screen",
            condition=IfCondition(
                LaunchConfiguration("enable_perception")
            ),
            parameters=[sim_time],
        ),
        TimerAction(
            period=3.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        bringup_share
                        + "/launch/navigation_no_collision.launch.py"
                    ),
                    launch_arguments={
                        "params_file": (
                            bringup_config
                            + "/nav2_params.yaml"
                        ),
                        "autostart": "true",
                        "use_composition": "False",
                        "use_sim_time": "false",
                    }.items(),
                    condition=IfCondition(
                        LaunchConfiguration("enable_nav2")
                    ),
                ),
            ],
        ),
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="mission",
                    executable="mission_manager",
                    name="mission_manager",
                    output="screen",
                    condition=IfCondition(
                        LaunchConfiguration("enable_mission")
                    ),
                    parameters=[sim_time],
                ),
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=[
                "-d",
                sim_share + "/config/rviz2_sim.rviz",
            ],
            condition=IfCondition(
                LaunchConfiguration("enable_rviz")
            ),
            parameters=[sim_time],
        ),
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            output="screen",
            condition=IfCondition(
                LaunchConfiguration("enable_foxglove")
            ),
        ),
    ]

    return LaunchDescription(
        launch_arguments + nodes
    )
