"""The one launch file.

    ros2 launch asket_bringup asket.launch.py            # real hardware
    ros2 launch asket_bringup asket.launch.py sim:=true  # nothing plugged in

``sim:=true`` swaps the *sources* and nothing else. Everything downstream —
the sonar bridge, the recorder, the diagnostics, the backend, the GUI — is
identical in both modes. That is the whole point: the simulated path is not a
parallel implementation that can quietly rot.

The sonar in particular is not special-cased. In sim, ``fake_sonar_node``
serves real Ping Protocol frames on a real UDP socket and ``omniscan_bridge``
connects to ``127.0.0.1`` instead of the sonar's address.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    sim = LaunchConfiguration("sim")
    sonar_host = LaunchConfiguration("sonar_host")
    log_level = LaunchConfiguration("log_level")

    sim_config = PathJoinSubstitution(
        [FindPackageShare("asket_sim"), "config", "sim.yaml"]
    )
    sonar_config = PathJoinSubstitution(
        [FindPackageShare("omniscan_bridge"), "config", "omniscan.yaml"]
    )
    # Its own file, deliberately: it carries the "has anybody measured this"
    # flag that the pre-flight check reads (docs/open_questions.md Q2).
    mounting_config = PathJoinSubstitution(
        [FindPackageShare("omniscan_bridge"), "config", "mounting.yaml"]
    )
    topics_config = PathJoinSubstitution(
        [FindPackageShare("gui_backend"), "config", "topics.yaml"]
    )
    link_config = PathJoinSubstitution(
        [FindPackageShare("gui_backend"), "config", "link_profiles.yaml"]
    )
    gui_static = PathJoinSubstitution(
        [FindPackageShare("gui_backend"), "static"]
    )
    recorder_config = PathJoinSubstitution(
        [FindPackageShare("mission_recorder"), "config", "recorder.yaml"]
    )
    system_test_config = PathJoinSubstitution(
        [FindPackageShare("system_test"), "config", "system_test.yaml"]
    )

    args = [
        DeclareLaunchArgument(
            "sim",
            default_value="false",
            description="Use simulated sources instead of hardware.",
        ),
        DeclareLaunchArgument(
            "sonar_host",
            default_value="192.168.2.92",
            description="Omniscan 3D address.",
        ),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument(
            "gui_port", default_value="8080", description="Mission GUI HTTP port."
        ),
        DeclareLaunchArgument(
            "missions_root",
            default_value="/data/missions",
            description="Where mission directories are written.",
        ),
        DeclareLaunchArgument(
            "tiles_path",
            default_value="/data/maps/survey.mbtiles",
            description=(
                "Pre-downloaded offline map tiles. There is no internet in the "
                "field; without this the map degrades to a coordinate grid."
            ),
        ),
    ]

    simulated_sources = GroupAction(
        condition=IfCondition(sim),
        actions=[
            Node(
                package="asket_sim",
                executable="sim_node",
                name="asket_sim",
                parameters=[sim_config],
                arguments=["--ros-args", "--log-level", log_level],
                output="screen",
            ),
            Node(
                package="asket_sim",
                executable="fake_sonar_node",
                name="fake_sonar",
                parameters=[sim_config],
                arguments=["--ros-args", "--log-level", log_level],
                output="screen",
            ),
        ],
    )

    # Real sources are the existing packages (MAVROS, rplidar, pico_bridge).
    # They are launched by node-ros-2026's own bringup, which this workspace
    # must not modify — so nothing is started here, and this group exists only
    # to make the asymmetry explicit rather than surprising.
    real_sources = GroupAction(condition=UnlessCondition(sim), actions=[])

    # Identical in both modes. In sim it simply connects to localhost, where
    # fake_sonar_node is serving real Ping Protocol frames.
    sonar_bridge = Node(
        package="omniscan_bridge",
        executable="omniscan_bridge_node",
        name="omniscan_bridge",
        parameters=[sonar_config, {"host": sonar_host, "mounting_path": mounting_config}],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
    )

    mission_gui = Node(
        package="gui_backend",
        executable="gui_backend_node",
        name="gui_backend",
        parameters=[
            {
                "port": LaunchConfiguration("gui_port"),
                "topics_config": topics_config,
                "link_profiles_config": link_config,
                "tiles_path": LaunchConfiguration("tiles_path"),
                "static_dir": gui_static,
            }
        ],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
    )

    recorder = Node(
        package="mission_recorder",
        executable="mission_recorder_node",
        name="mission_recorder",
        parameters=[recorder_config, {"missions_root": LaunchConfiguration("missions_root")}],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
    )

    built_in_test = Node(
        package="system_test",
        executable="system_test_node",
        name="system_test",
        parameters=[system_test_config],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
    )

    return LaunchDescription(
        args
        + [simulated_sources, real_sources, sonar_bridge, recorder, built_in_test, mission_gui]
    )
