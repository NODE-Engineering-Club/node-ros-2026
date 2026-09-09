"""The mission GUI, on its own.

Includable from ``njord.launch.py`` behind ``enable_gui`` (off by default), and
runnable alone:

    ros2 launch gui_backend gui.launch.py

Independent on purpose. The GUI is a **separate process** from ``pico_bridge``
and must stay one: that node owns the serial link and runs a 20 Hz heartbeat.
Nothing the GUI does — a slow client, a large export, a stalled WebSocket — may
starve it.

**The failsafe is 500 ms, and what it does depends on the mode.** Both timeouts
in the firmware are ``500``; the 600 ms figure repeated in this repository was
never right. The two cases are not the same event and must not be described as
one:

* **SBUS lost, outside AUTONOMOUS** — ``trigger_estop()``. The relay opens, ESC
  power is cut, and the latch is set: re-arming is blocked until the operator
  cycles the arm switch or selects ESTOP on channel 8.
* **Serial lost, inside AUTONOMOUS** — both thrusters are held at neutral. Power
  is *not* cut and nothing is latched; the vessel coasts and stays armed, and it
  resumes the moment setpoints return.

So a stalled GUI does not stop the boat, and a lost transmitter does. Anything
that reports the two as the same "link lost" is telling the operator the wrong
thing about what the vessel just did.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import os


def generate_launch_description():
    gui_share = get_package_share_directory("gui_backend")

    topics_config = PathJoinSubstitution([gui_share, "config", "topics.yaml"])
    link_config = PathJoinSubstitution([gui_share, "config", "link_profiles.yaml"])
    static_dir = os.path.join(gui_share, "static")

    args = [
        DeclareLaunchArgument(
            "gui_port",
            default_value="8090",
            description=(
                "HTTP/WebSocket port. Clear of foxglove_bridge (8765), "
                "rosbridge (9090) and web_video_server (8080)."
            ),
        ),
        DeclareLaunchArgument(
            "gui_host",
            default_value="0.0.0.0",
            description=(
                "Bind address. 0.0.0.0 so the laptop ashore can reach it; "
                "127.0.0.1 makes the GUI unreachable, which defeats the point."
            ),
        ),
        DeclareLaunchArgument(
            "tiles_path",
            default_value="/data/maps/survey.mbtiles",
            description=(
                "Offline basemap. Missing is survivable — the map degrades to a "
                "coordinate grid and says so — but you will not see the coast."
            ),
        ),
        DeclareLaunchArgument("use_sim", default_value="false"),
        DeclareLaunchArgument(
            "enable_sonar",
            default_value="false",
            description=(
                "Start omniscan_bridge. Off by default: the Omniscan is survey "
                "hardware and is not on the boat for competition work."
            ),
        ),
        DeclareLaunchArgument("sonar_host", default_value="192.168.2.92"),
    ]

    sim_time = {"use_sim_time": LaunchConfiguration("use_sim")}

    nodes = [
        Node(
            package="gui_backend",
            executable="gui_backend_node",
            name="gui_backend",
            output="screen",
            parameters=[
                {
                    "host": LaunchConfiguration("gui_host"),
                    "port": LaunchConfiguration("gui_port"),
                    "topics_config": topics_config,
                    "link_profiles_config": link_config,
                    "tiles_path": LaunchConfiguration("tiles_path"),
                    "static_dir": static_dir,
                },
                sim_time,
            ],
        ),
        Node(
            package="system_test",
            executable="system_test_node",
            name="system_test",
            output="screen",
            parameters=[
                {
                    "mounting_path": PathJoinSubstitution(
                        [get_package_share_directory("omniscan_bridge"),
                         "config", "mounting.yaml"]
                    ),
                    # The competition stack's own nodes, so a missing one is
                    # reported rather than silently absent from the verdict.
                    "expected_nodes": ["pico_bridge", "gui_backend"],
                },
                sim_time,
            ],
        ),
        Node(
            package="mission_recorder",
            executable="mission_recorder_node",
            name="mission_recorder",
            output="screen",
            parameters=[
                PathJoinSubstitution(
                    [get_package_share_directory("mission_recorder"),
                     "config", "recorder.yaml"]
                ),
                sim_time,
            ],
        ),
        Node(
            package="omniscan_bridge",
            executable="omniscan_bridge_node",
            name="omniscan_bridge",
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_sonar")),
            parameters=[
                PathJoinSubstitution(
                    [get_package_share_directory("omniscan_bridge"),
                     "config", "omniscan.yaml"]
                ),
                {
                    "host": LaunchConfiguration("sonar_host"),
                    "mounting_path": PathJoinSubstitution(
                        [get_package_share_directory("omniscan_bridge"),
                         "config", "mounting.yaml"]
                    ),
                },
                sim_time,
            ],
        ),
    ]

    return LaunchDescription(args + nodes)
