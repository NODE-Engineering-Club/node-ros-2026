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
        parameters=[sonar_config, {"host": sonar_host}],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
    )

    return LaunchDescription(args + [simulated_sources, real_sources, sonar_bridge])
