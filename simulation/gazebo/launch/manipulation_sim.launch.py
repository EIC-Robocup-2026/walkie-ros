#!/usr/bin/env python3
"""
Launch the manipulation-test world.

Spawns the robot close to the table so the arm can reach the objects.
  Table centre : x=1.5, y=0  (tabletop surface at z=0.75 m)
  Robot spawn  : x=0,   y=0  (1.5 m in front of the table)

Usage:
  ros2 launch robot_simulation manipulation_sim.launch.py
  ros2 launch robot_simulation manipulation_sim.launch.py use_rviz:=False
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg = get_package_share_directory("robot_simulation")

    manipulation_world = os.path.join(pkg, "worlds", "manipulation_test.world")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "gzsim_omnibot.launch.py")
        ),
        launch_arguments={
            "world": manipulation_world,
            "x_pose": "0.0",
            "y_pose": "0.0",
        }.items(),
    )

    return LaunchDescription([sim])
