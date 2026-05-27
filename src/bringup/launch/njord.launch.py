import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    cfg = get_package_share_directory("bringup") + "/config"
    desc_share = get_package_share_directory("description")
    urdf = xacro.process_file(desc_share + "/urdf/asket.urdf.xacro").toxml()
    # Write URDF next to the meshes/ directory so Gazebo resolves relative mesh paths
    urdf_path = os.path.join(desc_share, "asket.urdf")
    with open(urdf_path, "w") as f:
        f.write(urdf)
    world = os.path.join(desc_share, "worlds", "basicWorld.sdf")

    # fmt: off
    args = [
        DeclareLaunchArgument("enable_mavros",       default_value="true"),
        DeclareLaunchArgument("enable_localization",  default_value="true"),
        DeclareLaunchArgument("enable_nav2",          default_value="true"),
        DeclareLaunchArgument("enable_sensors",       default_value="true"),
        DeclareLaunchArgument("enable_perception",    default_value="true"),
        DeclareLaunchArgument("enable_control",       default_value="true"),
        DeclareLaunchArgument("enable_mission",       default_value="true"),
        DeclareLaunchArgument("enable_vision",        default_value="true"),
        DeclareLaunchArgument("vision_confidence",    default_value="0.5"),
        DeclareLaunchArgument("camera_device",        default_value="/dev/video0"),
        DeclareLaunchArgument("lidar_device",         default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("use_sim",         default_value="false"),
        DeclareLaunchArgument("enable_foxglove",      default_value="true"),
        DeclareLaunchArgument("enable_webbridge",     default_value="true"),
    ]
    # fmt: on

    sim_time = {"use_sim_time": LaunchConfiguration("use_sim")}

    nodes = [
        # Robot description — publishes TF frames from URDF
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": urdf}, sim_time],
        ),
        # MAVROS — FCU bridge
        Node(
            package="mavros",
            executable="mavros_node",
            name="mavros",
            condition=IfCondition(LaunchConfiguration("enable_mavros")),
            parameters=[
                {
                    "fcu_url": "udp://:14550@localhost:14555",
                    "gcs_url": "udp://@localhost:14556",
                    "tgt_system": 1,
                    "tgt_component": 1,
                },
                sim_time,
            ],
        ),
        # Localization — EKF + NavSat transform
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_node",
            condition=IfCondition(LaunchConfiguration("enable_localization")),
            parameters=[cfg + "/ekf.yaml", sim_time],
        ),
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform_node",
            condition=IfCondition(LaunchConfiguration("enable_localization")),
            parameters=[cfg + "/navsat.yaml", sim_time],
        ),
        # Nav2 — collision_monitor disabled (no polygons defined)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory("nav2_bringup")
                + "/launch/navigation_launch.py"
            ),
            launch_arguments={
                "params_file": cfg + "/nav2_params.yaml",
                "use_collision_monitor": "False",
                "use_sim_time": LaunchConfiguration("use_sim"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("enable_nav2")),
        ),
        # Sensors
        Node(
            package="sensors",
            executable="camera_driver",
            name="camera_driver",
            condition=IfCondition(LaunchConfiguration("enable_sensors")),
            parameters=[{"device": LaunchConfiguration("camera_device")}, sim_time],
        ),
        Node(
            package="sensors",
            executable="lidar_driver",
            name="lidar_driver",
            condition=IfCondition(LaunchConfiguration("enable_sensors")),
            parameters=[{"device": LaunchConfiguration("lidar_device")}, sim_time],
        ),
        Node(
            package="sensors",
            executable="imu_gps_driver",
            name="imu_gps_driver",
            condition=IfCondition(LaunchConfiguration("enable_sensors")),
            parameters=[sim_time],
        ),
        # Perception
        Node(
            package="perception",
            executable="lidar_obstacle_node",
            name="lidar_obstacle_node",
            condition=IfCondition(LaunchConfiguration("enable_perception")),
            parameters=[sim_time],
        ),
        Node(
            package="perception",
            executable="fusion_node",
            name="fusion_node",
            condition=IfCondition(LaunchConfiguration("enable_perception")),
            parameters=[sim_time],
        ),
        # Control
        Node(
            package="control",
            executable="nav_to_pid",
            name="nav_to_pid",
            condition=IfCondition(LaunchConfiguration("enable_control")),
            parameters=[sim_time],
        ),
        Node(
            package="control",
            executable="pid_controller",
            name="pid_controller",
            condition=IfCondition(LaunchConfiguration("enable_control")),
            parameters=[sim_time],
        ),
        Node(
            package="control",
            executable="actuator_driver",
            name="actuator_driver",
            condition=IfCondition(LaunchConfiguration("enable_control")),
            parameters=[sim_time],
        ),
        # Mission
        Node(
            package="mission",
            executable="mission_manager",
            name="mission_manager",
            condition=IfCondition(LaunchConfiguration("enable_mission")),
            parameters=[sim_time],
        ),
        # Vision
        Node(
            package="vision",
            executable="vision_node",
            name="vision_node",
            condition=IfCondition(LaunchConfiguration("enable_vision")),
            parameters=[
                {"confidence": LaunchConfiguration("vision_confidence")},
                sim_time,
            ],
        ),
        # Telemetry — rosbridge WebSocket (port 9090) + MJPEG video server (port 8080)
        # Node(
        #     package="rosbridge_server",
        #     executable="rosbridge_websocket",
        #     name="rosbridge_websocket",
        #     parameters=[{"port": 9090}, sim_time],
        # ),
        # Node(
        #     package="web_video_server",
        #     executable="web_video_server",
        #     name="web_video_server",
        #     parameters=[{"port": 8080}, sim_time],
        # ),
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            condition=IfCondition(LaunchConfiguration("enable_foxglove")),
        ),
        # Web dashboard bridge — exposes camera/LiDAR/YOLO/odometry on port 8081
        Node(
            package="webbridge",
            executable="webbridge_node",
            name="webbridge_node",
            condition=IfCondition(LaunchConfiguration("enable_webbridge")),
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="ros_gz_bridge",
            output="screen",
            parameters=[{"config_file": cfg + "/gz_bridge.yaml"}],
            condition=IfCondition(LaunchConfiguration("use_sim")),
        ),
        # Gazebo simulation
        ExecuteProcess(
            cmd=["gz", "sim", world],
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_sim")),
        ),
        # Spawn robot — delayed to allow Gazebo to finish loading the world
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2", "run", "ros_gz_sim", "create",
                        "-world", "default",
                        "-file", urdf_path,
                        "-name", "asket",
                        "-x", "0", "-y", "0", "-z", "0.1",
                    ],
                    output="screen",
                ),
            ],
            condition=IfCondition(LaunchConfiguration("use_sim")),
        ),
    ]

    return LaunchDescription(args + nodes)
