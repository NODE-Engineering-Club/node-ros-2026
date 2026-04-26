from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    cfg  = get_package_share_directory("bringup") + "/config"
    urdf = xacro.process_file(
        get_package_share_directory("asket_description") + "/urdf/asket.urdf.xacro"
    ).toxml()

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
    ]
    # fmt: on

    nodes = [
        # Robot description — publishes TF frames from URDF
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": urdf}],
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
                }
            ],
        ),
        # Localization — EKF + NavSat transform
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_node",
            condition=IfCondition(LaunchConfiguration("enable_localization")),
            parameters=[cfg + "/ekf.yaml"],
        ),
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform_node",
            condition=IfCondition(LaunchConfiguration("enable_localization")),
            parameters=[cfg + "/navsat.yaml"],
        ),
        # Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory("nav2_bringup")
                + "/launch/navigation_launch.py"
            ),
            launch_arguments={
                "params_file": cfg + "/nav2_params.yaml",
                "use_collision_monitor": "True",
            }.items(),
            condition=IfCondition(LaunchConfiguration("enable_nav2")),
        ),
        # Sensors
        Node(
            package="sensors",
            executable="camera_driver",
            name="camera_driver",
            condition=IfCondition(LaunchConfiguration("enable_sensors")),
            parameters=[{"device": LaunchConfiguration("camera_device")}],
        ),
        Node(
            package="sensors",
            executable="lidar_driver",
            name="lidar_driver",
            condition=IfCondition(LaunchConfiguration("enable_sensors")),
            parameters=[{"device": LaunchConfiguration("lidar_device")}],
        ),
        Node(
            package="sensors",
            executable="imu_gps_driver",
            name="imu_gps_driver",
            condition=IfCondition(LaunchConfiguration("enable_sensors")),
        ),
        # Perception
        Node(
            package="perception",
            executable="lidar_obstacle_node",
            name="lidar_obstacle_node",
            condition=IfCondition(LaunchConfiguration("enable_perception")),
        ),
        Node(
            package="perception",
            executable="fusion_node",
            name="fusion_node",
            condition=IfCondition(LaunchConfiguration("enable_perception")),
        ),
        # Control
        Node(
            package="control",
            executable="nav_to_pid",
            name="nav_to_pid",
            condition=IfCondition(LaunchConfiguration("enable_control")),
        ),
        Node(
            package="control",
            executable="pid_controller",
            name="pid_controller",
            condition=IfCondition(LaunchConfiguration("enable_control")),
        ),
        Node(
            package="control",
            executable="actuator_driver",
            name="actuator_driver",
            condition=IfCondition(LaunchConfiguration("enable_control")),
        ),
        # Mission
        Node(
            package="mission",
            executable="mission_manager",
            name="mission_manager",
            condition=IfCondition(LaunchConfiguration("enable_mission")),
        ),
        # Vision
        Node(
            package="vision",
            executable="vision_node",
            name="vision_node",
            condition=IfCondition(LaunchConfiguration("enable_vision")),
            parameters=[{"confidence": LaunchConfiguration("vision_confidence")}],
        ),
        # Telemetry — rosbridge WebSocket (port 9090) + MJPEG video server (port 8080)
        Node(
            package="rosbridge_server",
            executable="rosbridge_websocket",
            name="rosbridge_websocket",
            parameters=[{"port": 9090}],
        ),
        Node(
            package="web_video_server",
            executable="web_video_server",
            name="web_video_server",
            parameters=[{"port": 8080}],
        ),
    ]

    return LaunchDescription(args + nodes)
