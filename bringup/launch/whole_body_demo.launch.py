#!/usr/bin/env python3
"""Whole-body MoveIt demo in MuJoCo, on a planar base model.

Starts the obstacle scene, Nav2 (which stays idle and owns the collision monitor),
move_group with base_mode:=whole_body, the commander and the base follower. Run the
client in a second terminal:

    pixi run ros2 launch robot_bringup whole_body_demo.launch.py
    pixi run ros2 run openarm_bimanual_commander_cpp whole_body_demo

map -> odom comes from ground truth, because the omni controller's odometry reports
commands and not wheel motion. A real robot needs AMCL there.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory("robot_bringup")
    pkg_description = get_package_share_directory("walkie_description")
    pkg_moveit = get_package_share_directory("openarm_bimanual_moveit_config")
    pkg_commander = get_package_share_directory("openarm_bimanual_commander_cpp")

    declared_arguments = [
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Run MuJoCo without the viewer window",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz with the map-frame MoveIt config",
        ),
        DeclareLaunchArgument(
            "start_router",
            default_value="true",
            description="Start rmw_zenohd. Set to false if a router already runs",
        ),
        DeclareLaunchArgument(
            "mujoco_model",
            default_value=os.path.join(pkg_description, "mujoco", "scene_whole_body.xml"),
            description="MJCF scene to load",
        ),
    ]

    router = Node(
        package="rmw_zenoh_cpp",
        executable="rmw_zenohd",
        output="log",
        condition=IfCondition(LaunchConfiguration("start_router")),
    )

    # An include's launch arguments stay set after it, so without the scope of this group
    # the use_rviz:=false below would also reach the MoveIt include.
    simulation = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_bringup, "launch", "nav_mujoco_omnibot.launch.py")
                ),
                launch_arguments={
                    "mujoco_model": LaunchConfiguration("mujoco_model"),
                    "headless": LaunchConfiguration("headless"),
                    "use_rviz": "false",
                    "truth_map_odom": "true",
                }.items(),
            )
        ]
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit, "launch", "moveit_only.launch.py")
        ),
        launch_arguments={
            "hardware_type": "mujoco",
            "base_mode": "whole_body",
            "use_rviz": LaunchConfiguration("use_rviz"),
            # The table starts 1.55 m from the robot and the wall 3.15 m. Anything out
            # of range is unknown space, which the planner treats as free.
            "planar_octomap_max_range": "3.5",
        }.items(),
    )

    follower = Node(
        package="openarm_bimanual_commander_cpp",
        executable="whole_body_base_follower",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # The commander's MoveGroupInterface needs move_group's action server at startup.
    wait_for_move_group = Node(
        package="openarm_bimanual_moveit_config",
        executable="wait_for_graph.py",
        name="wait_for_move_group",
        output="screen",
        arguments=["--action", "/move_action"],
        parameters=[{"use_sim_time": True}],
    )
    commander = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_commander, "launch", "commander.launch.py")),
        launch_arguments={"base_mode": "whole_body", "use_sim_time": "true"}.items(),
    )
    start_commander = RegisterEventHandler(
        OnProcessExit(target_action=wait_for_move_group, on_exit=[commander])
    )

    return LaunchDescription(
        declared_arguments
        + [router, simulation, moveit, follower, wait_for_move_group, start_commander]
    )
