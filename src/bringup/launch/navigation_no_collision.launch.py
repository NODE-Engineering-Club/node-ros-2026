# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch_ros.descriptions import ComposableNode
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """Launch Nav2 without the docking server."""

    bringup_dir = get_package_share_directory("nav2_bringup")

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    use_composition = LaunchConfiguration("use_composition")
    container_name = LaunchConfiguration("container_name")
    container_name_full = (
        namespace,
        "/",
        container_name,
    )
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    # Docking server is intentionally excluded for now.
    lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "route_server",
        "behavior_server",
        "velocity_smoother",
        "collision_monitor",
        "bt_navigator",
        "waypoint_follower",
    ]

    remappings = [
        (
            "/tf",
            "tf",
        ),
        (
            "/tf_static",
            "tf_static",
        ),
    ]

    param_substitutions = {
        "autostart": autostart,
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM",
        "1",
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            bringup_dir,
            "params",
            "nav2_params.yaml",
        ),
        description="Full path to the Nav2 parameters file",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically start the Nav2 lifecycle nodes",
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="False",
        description="Use composed bringup if true",
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        "container_name",
        default_value="nav2_container",
        description="Name of the component container",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Respawn nodes if they crash",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level",
    )

    load_nodes = GroupAction(
        condition=IfCondition(
            PythonExpression(
                [
                    "not ",
                    use_composition,
                ]
            )
        ),
        actions=[
            SetParameter(
                "use_sim_time",
                use_sim_time,
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=[
                    "--ros-args",
                    "--log-level",
                    log_level,
                ],
                remappings=remappings + [
                    (
                        "cmd_vel",
                        "cmd_vel_nav",
                    ),
                ],
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=[
                    "--ros-args",
                    "--log-level",
                    log_level,
                ],
                remappings=remappings,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=[
                    "--ros-args",
                    "--log-level",
                    log_level,
                ],
                remappings=remappings,
            ),
            Node(
                package="nav2_route",
                executable="route_server",
                name="route_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=[
                    "--ros-args",
                    "--log-level",
                    log_level,
                ],
                remappings=remappings,
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=[
                    "--ros-args",
                    "--log-level",
                    log_level,
                ],
                remappings=remappings + [
                    (
                        "cmd_vel",
                        "cmd_vel_nav",
                    ),
                ],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=[
                    "--ros-args",
                    "--log-level",
                    log_level,
                ],
                remappings=remappings,
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=[
                    "--ros-args",
                    "--log-level",
                    log_level,
                ],
                remappings=remappings,
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=[
                    "--ros-args",
                    "--log-level",
                    log_level,
                ],
                remappings=remappings + [
                    (
                        "cmd_vel",
                        "cmd_vel_nav",
                    ),
                ],
            ),
            Node(
                package="nav2_collision_monitor",
                executable="collision_monitor",
                name="collision_monitor",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=[
                    "--ros-args",
                    "--log-level",
                    log_level,
                ],
                remappings=remappings,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                arguments=[
                    "--ros-args",
                    "--log-level",
                    log_level,
                ],
                parameters=[
                    {
                        "autostart": autostart,
                    },
                    {
                        "node_names": lifecycle_nodes,
                    },
                ],
            ),
        ],
    )

    load_composable_nodes = GroupAction(
        condition=IfCondition(
            use_composition
        ),
        actions=[
            SetParameter(
                "use_sim_time",
                use_sim_time,
            ),
            LoadComposableNodes(
                target_container=container_name_full,
                composable_node_descriptions=[
                    ComposableNode(
                        package="nav2_controller",
                        plugin="nav2_controller::ControllerServer",
                        name="controller_server",
                        parameters=[configured_params],
                        remappings=remappings + [
                            (
                                "cmd_vel",
                                "cmd_vel_nav",
                            ),
                        ],
                    ),
                    ComposableNode(
                        package="nav2_smoother",
                        plugin="nav2_smoother::SmootherServer",
                        name="smoother_server",
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package="nav2_planner",
                        plugin="nav2_planner::PlannerServer",
                        name="planner_server",
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package="nav2_route",
                        plugin="nav2_route::RouteServer",
                        name="route_server",
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package="nav2_behaviors",
                        plugin="behavior_server::BehaviorServer",
                        name="behavior_server",
                        parameters=[configured_params],
                        remappings=remappings + [
                            (
                                "cmd_vel",
                                "cmd_vel_nav",
                            ),
                        ],
                    ),
                    ComposableNode(
                        package="nav2_bt_navigator",
                        plugin="nav2_bt_navigator::BtNavigator",
                        name="bt_navigator",
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package="nav2_waypoint_follower",
                        plugin=(
                            "nav2_waypoint_follower::"
                            "WaypointFollower"
                        ),
                        name="waypoint_follower",
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package="nav2_velocity_smoother",
                        plugin=(
                            "nav2_velocity_smoother::"
                            "VelocitySmoother"
                        ),
                        name="velocity_smoother",
                        parameters=[configured_params],
                        remappings=remappings + [
                            (
                                "cmd_vel",
                                "cmd_vel_nav",
                            ),
                        ],
                    ),
                    ComposableNode(
                        package="nav2_collision_monitor",
                        plugin=(
                            "nav2_collision_monitor::"
                            "CollisionMonitor"
                        ),
                        name="collision_monitor",
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package="nav2_lifecycle_manager",
                        plugin=(
                            "nav2_lifecycle_manager::"
                            "LifecycleManager"
                        ),
                        name="lifecycle_manager_navigation",
                        parameters=[
                            {
                                "autostart": autostart,
                                "node_names": lifecycle_nodes,
                            },
                        ],
                    ),
                ],
            ),
        ],
    )

    launch_description = LaunchDescription()

    launch_description.add_action(
        stdout_linebuf_envvar
    )

    launch_description.add_action(
        declare_namespace_cmd
    )
    launch_description.add_action(
        declare_use_sim_time_cmd
    )
    launch_description.add_action(
        declare_params_file_cmd
    )
    launch_description.add_action(
        declare_autostart_cmd
    )
    launch_description.add_action(
        declare_use_composition_cmd
    )
    launch_description.add_action(
        declare_container_name_cmd
    )
    launch_description.add_action(
        declare_use_respawn_cmd
    )
    launch_description.add_action(
        declare_log_level_cmd
    )

    launch_description.add_action(
        load_nodes
    )
    launch_description.add_action(
        load_composable_nodes
    )

    return launch_description
