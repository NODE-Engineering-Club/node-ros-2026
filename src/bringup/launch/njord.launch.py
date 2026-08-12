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
    # robot_state_publisher's robot_description parameter is what actually
    # reaches TF/Foxglove/RViz — those tools don't get the "written next to
    # meshes/" trick above (that only works for Gazebo, which loads the file
    # from disk via -file below). They need package:// mesh URIs to resolve
    # anything, or they render TF frames with no hull mesh (bare relative
    # paths like "meshes/hull_visual.obj" carry no location info once this
    # is just a string on the wire). Only this copy is rewritten — the file
    # written above for Gazebo's spawn is untouched, so its already-working
    # resolution isn't at risk.
    urdf_for_ros = urdf.replace('filename="meshes/', 'filename="package://description/meshes/')
    worlds_dir = os.path.join(desc_share, "worlds")

    # fmt: off
    args = [
        DeclareLaunchArgument("world",                default_value="basicWorld.sdf",
                              description="World file name under description/worlds/ to load in Gazebo "
                                          "(e.g. dockingWorld.sdf for the U-shaped Task 3.1 berth)"),
        DeclareLaunchArgument("enable_mavros",       default_value="true"),
        DeclareLaunchArgument("enable_localization",  default_value="true"),
        DeclareLaunchArgument("enable_nav2",          default_value="true"),
        DeclareLaunchArgument("enable_sensors",       default_value="true"),
        DeclareLaunchArgument("enable_perception",    default_value="true"),
        DeclareLaunchArgument("enable_geo_fusion",    default_value="true"),
        DeclareLaunchArgument("enable_control",       default_value="true"),
        DeclareLaunchArgument("enable_mission",       default_value="true"),
        DeclareLaunchArgument("enable_maneuvering_pathfinding_mission", default_value="false",
                              description="Run the Task 9.1 (Maneuvering + Path Finding) mission "
                                          "sequencer — off by default so bringing up the stack "
                                          "doesn't immediately start the competition run."),
        DeclareLaunchArgument("enable_docking_mission", default_value="false",
                              description="Run the Task 3.1 (Normal Docking) mission sequencer "
                                          "— off by default, same reasoning as "
                                          "enable_maneuvering_pathfinding_mission."),
        DeclareLaunchArgument("enable_docking_parallel_mission", default_value="false",
                              description="Run the Task 3.2 (Parallel Docking) mission "
                                          "sequencer — off by default, same reasoning as "
                                          "enable_maneuvering_pathfinding_mission. Its "
                                          "close-range manoeuvre (ExecuteDockingParallel) "
                                          "and wall_detector_node are both real now but "
                                          "UNVERIFIED — never run against a real wall, on "
                                          "the bench or in the water — see "
                                          "DockingParallelTask in simple_boat.xml. Do a "
                                          "bench check before enabling for a real attempt."),
        DeclareLaunchArgument("enable_collision_avoidance_mission", default_value="false",
                              description="Run the Task 9.2 (Collision Avoidance) mission "
                                          "sequencer — off by default, same reasoning as "
                                          "enable_maneuvering_pathfinding_mission. Its "
                                          "gate-crossing + COLREG give-way handling "
                                          "(collision_nodes.cpp's updateGateState/"
                                          "updateMarkerVesselState) is real now but "
                                          "UNVERIFIED — never run against real gates or a "
                                          "real vessel, on the bench or in the water — see "
                                          "CollisionAvoidanceTask in simple_boat.xml. Do a "
                                          "bench check before enabling for a real attempt."),
        DeclareLaunchArgument("enable_surprise_mission", default_value="false",
                              description="Run the Task 9.4 (Surprise) mission sequencer "
                                          "— off by default, same reasoning as "
                                          "enable_maneuvering_pathfinding_mission. TEAM "
                                          "WORKING DRAFT (the official 9.4 spec is still "
                                          "unreleased) that chains normal docking -> "
                                          "open-water transit -> parallel docking into one "
                                          "run — see mission_surprise/surprise_mission.py's "
                                          "docstring and TODOS.md. UNVERIFIED end to end — "
                                          "never run, on the bench or in the water. Do NOT "
                                          "enable alongside enable_docking_mission/"
                                          "enable_docking_parallel_mission/"
                                          "enable_collision_avoidance_mission — they would "
                                          "race for the same competition_manager task "
                                          "selection."),
        DeclareLaunchArgument("enable_competition",   default_value="true"),
        DeclareLaunchArgument("enable_boat_bt",       default_value="true"),
        DeclareLaunchArgument("enable_vision",        default_value="true"),
        DeclareLaunchArgument("vision_confidence",    default_value="0.5"),
        DeclareLaunchArgument("camera_device",        default_value="/dev/video0"),
        DeclareLaunchArgument("lidar_device",         default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("use_sim",         default_value="false"),
        DeclareLaunchArgument(
            "headless",
            default_value="true",
            description=(
                "Run Gazebo server-only. Set false only when graphical "
                "rendering is known to work."
            ),
        ),
        DeclareLaunchArgument("enable_foxglove",      default_value="true"),
        DeclareLaunchArgument("fcu_url",              default_value="tcp://localhost:5777"),
        DeclareLaunchArgument("gcs_url",              default_value="udp://@localhost:14556"),
        DeclareLaunchArgument("use_pico_bridge",      default_value="false"),
        DeclareLaunchArgument("pico_port",            default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("enable_bms",           default_value="false"),
        DeclareLaunchArgument("bms_port",             default_value="/dev/ttyACM3"),
        DeclareLaunchArgument("lidar_camera_extrinsic", default_value="",
                              description="Path to lidar_camera_extrinsic.yaml; "
                                          "empty = use URDF nominal TF for lidar→front_camera"),
    ]
    # fmt: on

    sim_time = {"use_sim_time": LaunchConfiguration("use_sim")}

    nodes = [
        # Robot description — publishes TF frames from URDF
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": urdf_for_ros}, sim_time],
        ),
        # MAVROS — FCU bridge
        Node(
            package="mavros",
            executable="mavros_node",
            # No `name=` here — deliberately. Setting it makes launch_ros inject
            # a global `-r __node:=mavros` remap, and mavros's own plugin.cpp
            # creates each plugin's topics on a dynamically-constructed
            # sub-node (`rclcpp::Node::make_shared(subnode, ...)`) that is
            # supposed to be immune to that via `use_global_arguments(false)`
            # (see plugin.cpp's own comment on exactly this hazard). In the
            # mavros build bundled for Jazzy that protection doesn't hold: the
            # global remap leaks into every plugin sub-node, renaming all of
            # them to "mavros" and collapsing every plugin's topics onto the
            # single namespace /mavros/mavros/<leaf> instead of
            # /mavros/<plugin>/<leaf>. Most of the time that only causes
            # cosmetic doubling + harmless QoS-mismatch warnings between
            # plugins that happen to share a leaf name (e.g. local_position's
            # and mocap_pose_estimate's "pose"), but rc_io's "in"/"out" (its
            # actual topic names, not namespaced under "rc" at all once
            # collapsed) collide fatally with a differently-typed entity and
            # SIGABRT the whole node — 100% reproducible, not a startup race.
            # mavros_node's own compiled-in default node name is already
            # "mavros", so dropping the redundant name= keeps every topic
            # this repo depends on (/mavros/state, /mavros/imu/data, ...)
            # unchanged while leaving plugin sub-nodes unremapped and correctly
            # namespaced. Confirmed against the real Pixhawk: connects and
            # stays up indefinitely with no crash once this remap is gone.
            respawn=True,
            respawn_delay=2.0,
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("enable_mavros"), "' == 'true' and '",
                LaunchConfiguration("use_sim"), "' != 'true'"
            ])),
            arguments=[
                "--ros-args",
                "-p", ["fcu_url:=", LaunchConfiguration("fcu_url")],
                "-p", ["gcs_url:=", LaunchConfiguration("gcs_url")],
                "-p", "tgt_system:=1",
                "-p", "tgt_component:=1",
                "-p", "local_position.frame_id:=odom",
                "-p", "local_position.tf.child_frame_id:=base_link",
                "-p", "local_position.rate:=30.0",
                "--log-level", "mavros:=warn",
                "--log-level", "rcl.logging_rosout:=error",
            ],
            # No sim_time param here: this node's condition already excludes
            # use_sim, and adding it back (even as a single-key dict) forces
            # launch_ros to emit a second --params-file, which alone is enough
            # to reproduce the race described above.
            parameters=[
                cfg + "/mavros_denylist.yaml",
            ],
        ),
        # This particular FCU/link doesn't auto-stream position or extended-
        # status message groups on connect (RAW_SENSORS group — IMU, raw GPS —
        # comes through fine, but GLOBAL_POSITION_INT/BATTERY_STATUS never do
        # without this) — confirmed against the real Pixhawk: /mavros/battery
        # and /mavros/global_position/raw/fix stay completely silent (no
        # publisher ever calls publish(), not a QoS mismatch) until
        # /mavros/set_stream_rate is called explicitly. Requesting STREAM_ALL
        # once, a few seconds after mavros_node starts (long enough to be past
        # plugin loading and have an FCU connection), fixes it for the rest of
        # the session. This is a workaround for FCU/link stream-rate config,
        # not a bug in this repo's code.
        TimerAction(
            period=10.0,
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("enable_mavros"), "' == 'true' and '",
                LaunchConfiguration("use_sim"), "' != 'true'"
            ])),
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2", "service", "call", "/mavros/set_stream_rate",
                        "mavros_msgs/srv/StreamRate",
                        "{stream_id: 0, message_rate: 10, on_off: true}",
                    ],
                    output="log",
                ),
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
        # Anchors navsat_transform_node's GPS datum to the FC's own local-position
        # origin, so GPS waypoints (/fromLL) and odom-frame position agree on
        # where (0, 0) is — see sensors/datum_sync.py for why this matters.
        Node(
            package="sensors",
            executable="datum_sync",
            name="datum_sync",
            condition=IfCondition(LaunchConfiguration("enable_localization")),
        ),
        # Nav2 — individual nodes so we can omit opennav_docking (unsupported on USV)
        GroupAction(
            condition=IfCondition(LaunchConfiguration("enable_nav2")),
            actions=[
                SetParameter("use_sim_time", LaunchConfiguration("use_sim")),
            ] + [
                Node(package=pkg, executable=exe, name=name, output="screen",
                     parameters=[nav2_params],
                     remappings=[("/tf", "tf"), ("/tf_static", "tf_static")] + extra_remaps)
                for pkg, exe, name, extra_remaps in [
                    # controller_server/behavior_server's raw output must not
                    # land on the shared cmd_vel name: it's pre-smoothing,
                    # pre-collision-check, and (once twist_mux is added below)
                    # would otherwise also collide with the final arbitrated
                    # /cmd_vel. Route it into velocity_smoother instead.
                    ("nav2_controller",      "controller_server",  "controller_server", [("cmd_vel", "cmd_vel_nav")]),
                    ("nav2_smoother",        "smoother_server",    "smoother_server",   []),
                    ("nav2_planner",         "planner_server",     "planner_server",    []),
                    ("nav2_route",           "route_server",       "route_server",      []),
                    ("nav2_behaviors",       "behavior_server",    "behavior_server",   [("cmd_vel", "cmd_vel_nav")]),
                    ("nav2_bt_navigator",    "bt_navigator",       "bt_navigator",       []),
                    ("nav2_waypoint_follower","waypoint_follower",  "waypoint_follower", []),
                    ("nav2_velocity_smoother","velocity_smoother", "velocity_smoother",  [("cmd_vel", "cmd_vel_nav")]),
                    ("nav2_collision_monitor","collision_monitor",  "collision_monitor",  []),
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
            parameters=[{
                "device":          LaunchConfiguration("camera_device"),
                "frame_id":        "front_camera",
                "topic":           "/front_camera_driver/image_raw",
                "camera_info_url": "package://bringup/config/front_camera.yaml",
            }, sim_time],
        ),
        # RPLIDAR S2M1-R2L via Slamtec's official driver (vendored as the
        # src/sllidar_ros2 submodule). The A-series custom driver that used to
        # live here (sensors/lidar_driver) spoke the old 115200-baud/PWM-motor
        # protocol and doesn't apply to the S2's 1 Mbps express-scan protocol.
        Node(
            package="sllidar_ros2",
            executable="sllidar_node",
            name="lidar_driver",
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("enable_sensors"), "' == 'true' and '",
                LaunchConfiguration("use_sim"), "' != 'true'"
            ])),
            remappings=[("scan", "/lidar_driver/scan_raw")],
            parameters=[{
                "channel_type":      "serial",
                "serial_port":       LaunchConfiguration("lidar_device"),
                "serial_baudrate":   1000000,
                "frame_id":          "lidar",
                "inverted":          False,
                "angle_compensate":  True,
                "scan_mode":         "DenseBoost",
            }, sim_time],
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
        # Calibrated LiDAR→camera TF (only when extrinsic YAML is provided)
        Node(
            package="calibration",
            executable="extrinsic_tf_publisher",
            name="lidar_camera_extrinsic_tf",
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("lidar_camera_extrinsic"), "' != ''"
            ])),
            parameters=[{
                "extrinsic_yaml": LaunchConfiguration("lidar_camera_extrinsic"),
                "parent_frame":   "lidar",
                "child_frame":    "front_camera_cal",
            }],
        ),
        Node(
            package="perception",
            executable="fusion_node",
            name="fusion_node",
            condition=IfCondition(LaunchConfiguration("enable_perception")),
            parameters=[{
                "lidar_frame":  "lidar",
                "camera_frame": PythonExpression([
                    "'front_camera_cal' if '",
                    LaunchConfiguration("lidar_camera_extrinsic"),
                    "' != '' else 'front_camera'",
                ]),
            }, sim_time],
        ),
        # U-shaped docking-berth detector (Task 3.1) — DBSCAN + RANSAC over
        # /obstacles/lidar, publishes /perception/dock_target.
        Node(
            package="perception",
            executable="dock_detector_node",
            name="dock_detector_node",
            condition=IfCondition(LaunchConfiguration("enable_perception")),
            parameters=[sim_time],
        ),
        # Straight pier-wall detector (Task 3.2) — DBSCAN + RANSAC over
        # /obstacles/lidar, publishes /perception/wall_target. Same
        # LIDAR-only reasoning as dock_detector_node above; UNVERIFIED
        # against a real berth wall, see wall_detector_node's docstring.
        Node(
            package="perception",
            executable="wall_detector_node",
            name="wall_detector_node",
            condition=IfCondition(LaunchConfiguration("enable_perception")),
            parameters=[sim_time],
        ),
        # Geo-referenced fusion — labelled obstacles in the global GPS frame on
        # /obstacles/global (runs alongside fusion_node for comparison).
        Node(
            package="fusion",
            executable="geo_fusion_node",
            name="geo_fusion_node",
            condition=IfCondition(LaunchConfiguration("enable_geo_fusion")),
            parameters=[{
                "lidar_frame": "lidar",
                "base_frame":  "base_link",
                "map_frame":   "map",
                "camera_frame": PythonExpression([
                    "'front_camera_cal' if '",
                    LaunchConfiguration("lidar_camera_extrinsic"),
                    "' != '' else 'front_camera'",
                ]),
            }, sim_time],
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
                LaunchConfiguration("use_sim"), "' != 'true' and '",
                LaunchConfiguration("use_pico_bridge"), "' != 'true'"
            ])),
            parameters=[sim_time],
        ),
        # Arbitrates between Nav2's own /cmd_vel output (via collision_monitor,
        # remapped to nav2/cmd_vel) and boat_bt's direct docking commands
        # (boat_bt/cmd_vel) — both used to independently publish straight onto
        # the shared /cmd_vel that nav_to_pid/ros_gz_bridge consume, racing
        # each other whenever Nav2's pipeline stayed alive (e.g. its
        # collision_monitor safety-stop heartbeat) during a BT-direct task
        # like docking. See bringup/config/twist_mux.yaml for priorities.
        Node(
            package="twist_mux",
            executable="twist_mux",
            name="twist_mux",
            remappings=[("/cmd_vel_out", "/cmd_vel")],
            parameters=[
                cfg + "/twist_mux.yaml",
                sim_time,
            ],
        ),

        # Pico bridge — alternative actuation path: motor commands over serial
        # to a Raspberry Pi Pico, bypassing MAVROS/the Pixhawk for motor control.
        # mavros is still used for GPS/IMU sensing in this mode.
        #
        # Deliberately allowed to run even when use_sim:=true (unlike
        # actuator_driver above, which stays sim-excluded since it targets
        # mavros/the Pixhawk): this lets a bench test drive REAL motors via
        # the real Pico off of Gazebo-simulated perception/mission logic —
        # the boat physically can't move (elevated/on a stand), but the
        # thrusters WILL spin in response to real /control/effort commands.
        # Never enable this combination with the boat in the water.
        Node(
            package="control",
            executable="pico_bridge",
            name="pico_bridge",
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("enable_control"), "' == 'true' and '",
                LaunchConfiguration("use_pico_bridge"), "' == 'true'"
            ])),
            parameters=[{"port": LaunchConfiguration("pico_port")}, sim_time],
        ),
        # BMS reader — sole owner of the serial link to the battery management
        # system (MCP2221 USB-UART bridge). Publishes sensor_msgs/BatteryState
        # on /battery/state (renders in Foxglove's built-in Battery panel) and
        # the full raw report (per-cell voltages, FET states) on
        # /battery/status_raw.
        #
        # Deliberately allowed to run even when use_sim:=true, same reasoning
        # as pico_bridge above: real battery telemetry has nothing to do with
        # whether the boat's motion is simulated, so a hybrid bench test (real
        # hardware, Gazebo-simulated perception/mission) should still surface
        # the real pack's voltage/cell balance.
        Node(
            package="sensors",
            executable="bms_reader",
            name="bms_reader",
            condition=IfCondition(LaunchConfiguration("enable_bms")),
            parameters=[{"port": LaunchConfiguration("bms_port")}, sim_time],
        ),
        # Mission
        #
        # respawn=True: mission_manager holds no state that survives a crash
        # (waypoint list / progress index are plain in-memory attributes), so
        # a respawn always restarts a *new* mission at waypoint 0 unless the
        # caller resends a /mission/start whose waypoints match an on-disk
        # checkpoint from a previous run (see mission_manager.py) — respawn
        # only guarantees the service comes back, not that progress survives.
        Node(
            package="mission",
            executable="mission_manager",
            name="mission_manager",
            condition=IfCondition(LaunchConfiguration("enable_mission")),
            parameters=[sim_time],
            respawn=True,
            respawn_delay=2.0,
            output="screen",
        ),

        # Competition lifecycle coordination.
        #
        # respawn=True: a crash here loses the selected task/lifecycle state
        # (also in-memory only) — the operator (or the mission sequencer, if
        # it is still alive) must re-issue /competition/set_task +
        # /competition/start after a respawn.
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="competition_manager",
                    executable="competition_manager",
                    name="competition_manager",
                    condition=IfCondition(
                        LaunchConfiguration("enable_competition")
                    ),
                    parameters=[sim_time],
                    respawn=True,
                    respawn_delay=2.0,
                    output="screen",
                ),
            ],
        ),

        # Competition Behavior Tree.
        #
        # respawn=True: a crash mid-docking resets the docking state machine
        # to WAITING_FOR_TARGET on respawn (docking_state_ is in-memory only)
        # — acceptable because dock_detector_node will republish a fresh
        # target and the approach just restarts, rather than the whole
        # process staying dead.
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="boat_bt",
                    executable="boat_bt_node",
                    name="boat_bt",
                    condition=IfCondition(
                        LaunchConfiguration("enable_boat_bt")
                    ),
                    respawn=True,
                    respawn_delay=2.0,
                    parameters=[
                        sim_time,
                        # YOLO class ids for yolo26n-seg-navier.onnx, read from
                        # the model's own embedded metadata (names dict):
                        # {0: 'green', 1: 'red', 2: 'north', 3: 'east',
                        #  4: 'south', 5: 'west'}.
                        {
                            "cardinal_north_class_id": "2",
                            "cardinal_east_class_id": "3",
                            "cardinal_south_class_id": "4",
                            "cardinal_west_class_id": "5",
                            "buoy_green_class_id": "0",
                            "buoy_red_class_id": "1",
                            "buoy_min_standoff_m": 1.0,
                        },
                    ],
                    output="screen",
                ),
            ],
        ),

        # Task 9.1 mission sequencer — starts after competition_manager/boat_bt
        # so /competition/set_task and /competition/start are already up.
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package="mission_maneuvering_pathfinding",
                    executable="maneuvering_pathfinding_mission",
                    name="maneuvering_pathfinding_mission",
                    condition=IfCondition(
                        LaunchConfiguration("enable_maneuvering_pathfinding_mission")
                    ),
                    parameters=[sim_time],
                    output="screen",
                ),
            ],
        ),

        # Task 9.2 (Collision Avoidance) mission sequencer — same startup
        # timing as the Task 9.1 sequencer above.
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package="mission_collision_avoidance",
                    executable="collision_avoidance_mission",
                    name="collision_avoidance_mission",
                    condition=IfCondition(
                        LaunchConfiguration("enable_collision_avoidance_mission")
                    ),
                    parameters=[sim_time],
                    output="screen",
                ),
            ],
        ),

        # Task 3.1 (Normal Docking) mission sequencer — same startup timing
        # as the Task 9.1 sequencer above; the two are mutually exclusive in
        # practice (both default off) but there's no harm in sharing the
        # delay since only one is normally enabled at a time.
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package="mission_docking",
                    executable="docking_mission",
                    name="docking_mission",
                    condition=IfCondition(
                        LaunchConfiguration("enable_docking_mission")
                    ),
                    parameters=[sim_time],
                    output="screen",
                ),
            ],
        ),

        # Task 3.2 (Parallel Docking) mission sequencer — same startup
        # timing as the other sequencers above. Its close-range manoeuvre
        # (DockingParallelTask -> ExecuteDockingParallel in simple_boat.xml)
        # is real now but UNVERIFIED against a real wall; the transit-leg
        # orchestration is real and follows the same pattern as
        # mission_docking's.
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package="mission_docking_parallel",
                    executable="docking_parallel_mission",
                    name="docking_parallel_mission",
                    condition=IfCondition(
                        LaunchConfiguration("enable_docking_parallel_mission")
                    ),
                    parameters=[sim_time],
                    output="screen",
                ),
            ],
        ),

        # Task 9.4 (Surprise) mission sequencer — same startup timing as the
        # other sequencers above. Chains TASK_DOCKING -> open-water transit
        # (TASK_SURPRISE) -> TASK_DOCKING_PARALLEL into one run; see
        # mission_surprise/surprise_mission.py's docstring. TEAM WORKING
        # DRAFT, UNVERIFIED end to end — see enable_surprise_mission above.
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package="mission_surprise",
                    executable="surprise_mission",
                    name="surprise_mission",
                    condition=IfCondition(
                        LaunchConfiguration("enable_surprise_mission")
                    ),
                    parameters=[sim_time],
                    output="screen",
                ),
            ],
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
        # Sim-only: Gazebo publishes NavSatFix with the scoped sensor frame
        # "asket/base_link/GPS_sensor", while robot_state_publisher exposes
        # the URDF GPS link as "GPS". Publish the real URDF GPS offset under
        # the scoped Gazebo sensor name so navsat_transform_node can transform
        # GPS measurements into base_link and initialize /fromLL and /toLL.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_gps_sensor_tf",
            arguments=[
                "-0.18827", "0", "0.174775",
                "0", "0", "0",
                "base_link",
                "asket/base_link/GPS_sensor",
            ],
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
        # Gazebo simulation — server-only by default.
        #
        # Do not infer GUI availability from DISPLAY alone. Dev containers
        # may expose DISPLAY through Xvfb or VS Code while having no usable
        # GPU/rendering device, causing graphical Gazebo to exit immediately.
        ExecuteProcess(
            cmd=[
                "gz",
                "sim",
                "-s",
                "-r",
                PathJoinSubstitution(
                    [worlds_dir, LaunchConfiguration("world")]
                ),
            ],
            output="screen",
            condition=IfCondition(
                PythonExpression([
                    "'", LaunchConfiguration("use_sim"),
                    "' == 'true' and '",
                    LaunchConfiguration("headless"),
                    "' == 'true'",
                ])
            ),
        ),
        ExecuteProcess(
            cmd=[
                "gz",
                "sim",
                "-r",
                PathJoinSubstitution(
                    [worlds_dir, LaunchConfiguration("world")]
                ),
            ],
            output="screen",
            condition=IfCondition(
                PythonExpression([
                    "'", LaunchConfiguration("use_sim"),
                    "' == 'true' and '",
                    LaunchConfiguration("headless"),
                    "' != 'true'",
                ])
            ),
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
