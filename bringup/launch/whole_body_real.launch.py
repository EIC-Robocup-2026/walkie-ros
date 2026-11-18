#!/usr/bin/env python3
"""Whole-body base chain for the real robot, on top of real_omnibot.launch.py.

Starts what base_mode:=whole_body needs besides move_group and the commander:

- a static identity map -> odom (static_map_odom:=true), for a run without AMCL,
- the collision monitor with the real Nav2 parameters and its lifecycle manager
  (collision_monitor:=true), with state_topic set so that the base follower sees a stop,
- whole_body_base_follower,
- whole_body_marker, the hand target in RViz (use_marker:=true).

    pixi run ros2 launch robot_bringup whole_body_real.launch.py

When localization_Nav2_real.launch.py runs, pass static_map_odom:=false (AMCL publishes
map -> odom) and collision_monitor:=false (Nav2 starts its own monitor).

The real Nav2 parameters use YAML anchors, which --params-file rejects, so the monitor
reads them through RewrittenYaml like every Nav2 launch file.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_navigation = get_package_share_directory("robot_navigation")

    use_sim_time = LaunchConfiguration("use_sim_time")
    sim_time = ParameterValue(use_sim_time, value_type=bool)

    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use /clock. Only for a test against a simulation",
        ),
        DeclareLaunchArgument(
            "static_map_odom",
            default_value="true",
            description="Publish an identity map -> odom. Set to false when AMCL runs",
        ),
        DeclareLaunchArgument(
            "collision_monitor",
            default_value="true",
            description="Start the collision monitor. Set to false when Nav2 runs",
        ),
        DeclareLaunchArgument(
            "use_marker",
            default_value="true",
            description="Start the hand target marker for RViz",
        ),
        DeclareLaunchArgument(
            "nav2_params",
            default_value=os.path.join(
                pkg_navigation, "config", "nav2", "nav2_mppi_walkie_params.yaml"
            ),
            description="Nav2 parameter file with the collision_monitor section",
        ),
    ]

    static_map_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_odom",
        arguments=["--frame-id", "map", "--child-frame-id", "odom"],
        parameters=[{"use_sim_time": sim_time}],
        condition=IfCondition(LaunchConfiguration("static_map_odom")),
    )

    monitor_params = RewrittenYaml(
        source_file=LaunchConfiguration("nav2_params"),
        root_key="",
        param_rewrites={
            "use_sim_time": use_sim_time,
            "collision_monitor.ros__parameters.state_topic": "/collision_monitor_state",
        },
        convert_types=True,
    )
    collision_monitor = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        parameters=[monitor_params],
        condition=IfCondition(LaunchConfiguration("collision_monitor")),
    )
    # A manager of its own, so that it does not clash with lifecycle_manager_nav2.
    monitor_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_whole_body",
        output="screen",
        parameters=[
            {"use_sim_time": sim_time, "autostart": True, "node_names": ["collision_monitor"]}
        ],
        condition=IfCondition(LaunchConfiguration("collision_monitor")),
    )

    follower = Node(
        package="openarm_bimanual_commander_cpp",
        executable="whole_body_base_follower",
        output="screen",
        parameters=[{"use_sim_time": sim_time}],
    )

    hand_marker = Node(
        package="openarm_bimanual_commander_cpp",
        executable="whole_body_marker",
        output="screen",
        parameters=[{"use_sim_time": sim_time}],
        condition=IfCondition(LaunchConfiguration("use_marker")),
    )

    return LaunchDescription(
        declared_arguments
        + [static_map_odom, collision_monitor, monitor_manager, follower, hand_marker]
    )
