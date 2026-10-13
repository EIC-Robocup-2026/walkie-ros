#!/usr/bin/env python3
"""
O3DE Simulation Bringup with Nav2 and MoveIt Integration for Walkie Robot.
Unifies:
  1. O3DE Omnibot Bringup (robot_state_publisher, twist_mux, topic_based_ros2_control)
  2. Nav2 Autonomous Navigation (AMCL / costmaps / MPPI / behavior tree)
  3. MoveIt Manipulation (move_group with dual arms & lift trajectory controllers)
  4. Visualization in RViz2 (Nav2 mode or MoveIt interactive planning mode)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _launch_setup(context, *args, **kwargs):
    pkg_robot_bringup = get_package_share_directory("robot_bringup")
    pkg_robot_navigation = get_package_share_directory("robot_navigation")
    pkg_moveit = get_package_share_directory("openarm_bimanual_moveit_config")

    sim_time = context.perform_substitution(LaunchConfiguration("use_sim_time"))
    nav_enabled = context.perform_substitution(LaunchConfiguration("use_nav")).lower() == "true"
    moveit_enabled = context.perform_substitution(LaunchConfiguration("use_moveit")).lower() == "true"
    rviz_enabled = context.perform_substitution(LaunchConfiguration("use_rviz")).lower() == "true"
    mode = context.perform_substitution(LaunchConfiguration("rviz_mode")).lower()
    j2_fixed = context.perform_substitution(LaunchConfiguration("right_joint2_fixed"))
    map_yaml = context.perform_substitution(LaunchConfiguration("map"))

    launch_nav_rviz = "true" if (rviz_enabled and mode == "nav") else "false"
    launch_moveit_rviz = "true" if (rviz_enabled and mode == "moveit") else "false"

    pointclouds_enabled = context.perform_substitution(LaunchConfiguration("use_pointclouds"))

    actions = []

    # 1. O3DE Omnibot Bringup (RSP, twist_mux, twist_stamper, ros2_control spawners, rosbridge, RViz2)
    o3de_omnibot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_bringup, "launch", "o3de_omnibot.launch.py")
        ),
        launch_arguments={
            "use_sim_time": sim_time,
            "use_rviz": launch_nav_rviz,
            "right_joint2_fixed": j2_fixed,
            "use_pointclouds": pointclouds_enabled,
        }.items(),
    )
    actions.append(o3de_omnibot_launch)

    # 2. Nav2 Navigation Stack (delayed slightly to allow controller spawners and odometry to initialize)
    if nav_enabled:
        localization_nav2_sim_launch = TimerAction(
            period=3.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            pkg_robot_navigation, "launch", "localization_Nav2_sim.launch.py"
                        )
                    ),
                    launch_arguments={
                        "use_sim_time": sim_time,
                        "map": map_yaml,
                    }.items(),
                )
            ],
        )
        actions.append(localization_nav2_sim_launch)

    # 3. MoveIt Manipulation (move_group for dual arms + lift + MoveIt RViz)
    if moveit_enabled:
        moveit_launch = TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            pkg_moveit, "launch", "moveit_only.launch.py"
                        )
                    ),
                    launch_arguments={
                        "hardware_type": "o3de",
                        "use_rviz": launch_moveit_rviz,
                        "right_joint2_fixed": j2_fixed,
                        "lift_at_top": "true",
                    }.items(),
                )
            ],
        )
        actions.append(moveit_launch)

    return actions


def generate_launch_description():
    pkg_robot_navigation = get_package_share_directory("robot_navigation")
    default_aws_map = os.path.join(
        pkg_robot_navigation,
        "map",
        "08_23102025_SimWalkie_awssmallhouse_slam_toolbox",
        "map.yaml",
    )

    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock if true",
        ),
        DeclareLaunchArgument(
            "use_nav",
            default_value="true",
            description="Whether to launch Nav2 stack",
        ),
        DeclareLaunchArgument(
            "use_moveit",
            default_value="true",
            description="Whether to launch MoveIt move_group",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Whether to launch RViz2",
        ),
        DeclareLaunchArgument(
            "rviz_mode",
            default_value="nav",
            description="RViz configuration mode: 'nav' (Nav2 navigation) or 'moveit' (MoveIt manipulation with interactive markers)",
        ),
        DeclareLaunchArgument(
            "right_joint2_fixed",
            default_value="false",
            description="Lock the right arm joint2 as fixed",
        ),
        DeclareLaunchArgument(
            "use_pointclouds",
            default_value="false",
            description="Whether to generate and self-filter 3D point clouds from RGB-D cameras (CPU intensive)",
        ),
        DeclareLaunchArgument(
            "map",
            default_value=default_aws_map,
            description="Full path to the map YAML file for simulation",
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=_launch_setup)])
