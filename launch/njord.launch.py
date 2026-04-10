import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    nav2_launch = os.path.join(
        get_package_share_directory("nav2_bringup"), "launch", "navigation_launch.py"
    )

    return LaunchDescription([
        # MAVROS2 – bridge between ROS2 and ArduPilot via MAVLink
        Node(
            package="mavros",
            executable="mavros_node",
            name="mavros",
            parameters=[{
                "fcu_url": "udp://:14550@localhost:14555",
                "gcs_url": "udp://@localhost:14556",
                "tgt_system": 1,
                "tgt_component": 1,
            }],
        ),

        # Localization – EKF odometry fusion
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_node",
            parameters=["/config/ekf.yaml"],
        ),

        # Localization – GPS/UTM transform
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform_node",
            parameters=["/config/navsat.yaml"],
        ),

        # Navigation – Nav2 stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={"params_file": "/config/nav2_params.yaml"}.items(),
        ),

        # Sensors – camera, lidar, imu/gps relay (pip-installed entry point)
        ExecuteProcess(
            cmd=["/opt/venv/bin/sensors-launch-all"],
            output="screen",
        ),

        # Vision – ONNX YOLO26n-seg inference node
        ExecuteProcess(
            cmd=["/opt/venv/bin/vision-detector-node"],
            output="screen",
        ),

        # Perception – lidar obstacle extraction + sensor fusion
        ExecuteProcess(
            cmd=["/opt/venv/bin/perception-launch-all"],
            output="screen",
        ),

        # Control – PID + actuator driver
        ExecuteProcess(
            cmd=["/opt/venv/bin/control-launch-all"],
            output="screen",
        ),

        # Mission – waypoint sequencer
        ExecuteProcess(
            cmd=["/opt/venv/bin/mission-manager"],
            output="screen",
        ),
    ])
