import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    cfg = get_package_share_directory("bringup") + "/config"
    nav2_params = ParameterFile(
        RewrittenYaml(source_file=cfg + "/nav2_params.yaml", param_rewrites={}, convert_types=True),
        allow_substs=True,
    )
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
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("enable_mavros"), "' == 'true' and '",
                LaunchConfiguration("use_sim"), "' != 'true'"
            ])),
            parameters=[
                {
                    "fcu_url": "tcp://localhost:5777",
                    "gcs_url": "udp://@localhost:14556",
                    "tgt_system": 1,
                    "tgt_component": 1,
                    "local_position.frame_id": "odom",
                    "local_position.tf.child_frame_id": "base_link",
                    "local_position.rate": 30.0,
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
            remappings=[("gps/fix", "/gps_driver/gps_raw"), ("imu/data", "/imu_driver/imu_raw")],
            parameters=[cfg + "/navsat.yaml", sim_time],
        ),
        # Nav2 — individual nodes so we can omit opennav_docking (unsupported on USV)
        GroupAction(
            condition=IfCondition(LaunchConfiguration("enable_nav2")),
            actions=[
                SetParameter("use_sim_time", LaunchConfiguration("use_sim")),
            ] + [
                Node(package=pkg, executable=exe, name=name, output="screen",
                     parameters=[nav2_params], remappings=[("/tf", "tf"), ("/tf_static", "tf_static")])
                for pkg, exe, name in [
                    ("nav2_controller",      "controller_server",  "controller_server"),
                    ("nav2_smoother",        "smoother_server",    "smoother_server"),
                    ("nav2_planner",         "planner_server",     "planner_server"),
                    ("nav2_route",           "route_server",       "route_server"),
                    ("nav2_behaviors",       "behavior_server",    "behavior_server"),
                    ("nav2_bt_navigator",    "bt_navigator",       "bt_navigator"),
                    ("nav2_waypoint_follower","waypoint_follower",  "waypoint_follower"),
                    ("nav2_velocity_smoother","velocity_smoother", "velocity_smoother"),
                    ("nav2_collision_monitor","collision_monitor",  "collision_monitor"),
                ]
            ] + [
                Node(
                    package="nav2_lifecycle_manager",
                    executable="lifecycle_manager",
                    name="lifecycle_manager_navigation",
                    output="screen",
                    parameters=[{
                        "autostart": True,
                        "node_names": [
                            "controller_server", "smoother_server", "planner_server",
                            "route_server", "behavior_server", "velocity_smoother",
                            "collision_monitor", "bt_navigator", "waypoint_follower",
                        ],
                    }],
                ),
            ],
        ),
        # Sensors
        Node(
            package="sensors",
            executable="camera_driver",
            name="camera_driver",
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("enable_sensors"), "' == 'true' and '",
                LaunchConfiguration("use_sim"), "' != 'true'"
            ])),
            parameters=[{"device": LaunchConfiguration("camera_device"), "frame_id": "front_camera", "topic": "/front_camera_driver/image_raw"}, sim_time],
        ),
        Node(
            package="sensors",
            executable="lidar_driver",
            name="lidar_driver",
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("enable_sensors"), "' == 'true' and '",
                LaunchConfiguration("use_sim"), "' != 'true'"
            ])),
            parameters=[{"device": LaunchConfiguration("lidar_device")}, sim_time],
        ),
        Node(
            package="sensors",
            executable="imu_gps_driver",
            name="imu_gps_driver",
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("enable_sensors"), "' == 'true' and '",
                LaunchConfiguration("use_sim"), "' != 'true'"
            ])),
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
            parameters=[{"lidar_frame": "lidar", "camera_frame": "front_camera"}, sim_time],
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
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("enable_control"), "' == 'true' and '",
                LaunchConfiguration("use_sim"), "' != 'true'"
            ])),
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
        # Static map→odom identity TF.
        # ArduPilot's onboard EKF (or Gazebo in sim) gives us a globally-anchored
        # odom frame — odom origin == GPS home / Gazebo world origin. So map and
        # odom are coincident and we publish identity. /fromLL still works because
        # navsat_transform_node establishes its own datum from the first GPS fix.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_map_odom_tf",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        ),
        # Sim-only: Gazebo names sensor frames with the full scoped model path
        # (e.g. "asket/base_link/Lidar_sensor") while the TF tree only has the URDF link "lidar".
        # This static identity TF bridges the gap so collision_monitor can look up the transform.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_lidar_sensor_tf",
            arguments=["0", "0", "0", "0", "0", "0", "lidar", "asket/base_link/Lidar_sensor"],
            condition=IfCondition(LaunchConfiguration("use_sim")),
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
            cmd=["gz", "sim", "-r", world] if "DISPLAY" in os.environ else ["gz", "sim", "-s", "-r", world],
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
