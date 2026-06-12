"""
sim_nav2.launch.py
==================
Lance la simulation 2D Python + Nav2 complet, SANS toucher à njord.launch.py.
Isolé : seuls les packages sim, bringup/config, description sont utilisés.

Lancement :
    ros2 launch sim sim_nav2.launch.py
Options :
    enable_nav2:=true/false   (défaut true)
    enable_mission:=true/false (défaut true)
    enable_rviz:=true/false   (défaut true)
    enable_foxglove:=true/false (défaut false)
"""
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg        = get_package_share_directory("bringup")      + "/config"
    desc_share = get_package_share_directory("description")
    sim_share  = get_package_share_directory("sim")

    # Process URDF for robot_state_publisher
    urdf = xacro.process_file(
        desc_share + "/urdf/asket.urdf.xacro"
    ).toxml()

    # ── Launch arguments ──────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument("enable_nav2",     default_value="true"),
        DeclareLaunchArgument("enable_mission",  default_value="true"),
        DeclareLaunchArgument("enable_rviz",     default_value="true"),
        DeclareLaunchArgument("enable_foxglove", default_value="false"),
        DeclareLaunchArgument("enable_perception", default_value="true"),
    ]

    # use_sim_time = false — simulator uses wall clock (no /clock topic)
    sim_time = {"use_sim_time": "false"}

    nodes = [

        # ── 1. Simulator (LiDAR + odom + GPS + IMU + TF) ─────────────────
        Node(
            package="sim",
            executable="simulator",
            name="asket_simulator",
            output="screen",
            parameters=[sim_time],
        ),

        # ── 2. Robot state publisher (static TF from URDF) ────────────────
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": urdf}, sim_time],
        ),

        # ── 3. EKF — fuses /odom + /imu/data ─────────────────────────────
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_node",
            output="screen",
            parameters=[cfg + "/ekf.yaml", sim_time],
            remappings=[
                # EKF config uses odom0: /odom — already correct
                # Output: /odometry/filtered
            ],
        ),

        # ── 4. NavSat transform — GPS → map frame ─────────────────────────
        #    Needs: /gps_driver/gps_raw + /odometry/filtered + /imu/data
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform_node",
            output="screen",
            parameters=[cfg + "/navsat.yaml", sim_time],
            remappings=[
                ("gps/fix",    "/gps_driver/gps_raw"),
                ("imu/data",   "/imu/data"),
                ("odometry/filtered", "/odometry/filtered"),
            ],
        ),

        # ── 5. LiDAR obstacle node (scan_raw → /obstacles/lidar) ──────────
        Node(
            package="perception",
            executable="lidar_obstacle_node",
            name="lidar_obstacle_node",
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_perception")),
            parameters=[sim_time],
        ),

        # ── 6. Nav2 stack (planner + controller + BT navigator) ───────────
        #    Delayed 3 s to let EKF establish /odometry/filtered first
        TimerAction(
            period=3.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        get_package_share_directory("nav2_bringup")
                        + "/launch/navigation_launch.py"
                    ),
                    launch_arguments={
                        "params_file":             sim_share + "/config/nav2_sim_params.yaml",
                        "use_collision_monitor":   "False",
                        "use_sim_time":            "false",
                    }.items(),
                    condition=IfCondition(LaunchConfiguration("enable_nav2")),
                ),
            ],
        ),

        # ── 7. Mission manager (GPS waypoint sequencer) ───────────────────
        #    Delayed 5 s to let Nav2 lifecycle nodes activate first
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="mission",
                    executable="mission_manager",
                    name="mission_manager",
                    output="screen",
                    condition=IfCondition(LaunchConfiguration("enable_mission")),
                    parameters=[sim_time],
                ),
            ],
        ),

        # ── 8. RViz2 ──────────────────────────────────────────────────────
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", sim_share + "/config/rviz2_sim.rviz"],
            condition=IfCondition(LaunchConfiguration("enable_rviz")),
            parameters=[sim_time],
        ),

        # ── 9. Foxglove bridge (optionnel, port 8765) ─────────────────────
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            condition=IfCondition(LaunchConfiguration("enable_foxglove")),
        ),
    ]

    return LaunchDescription(args + nodes)
