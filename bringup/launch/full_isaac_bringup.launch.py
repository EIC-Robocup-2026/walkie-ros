#!/usr/bin/env python3
# ==============================================================================
# Walkie Bot - Full Isaac Sim Bringup Launch File (Alias)
# RoboCup 2027
#
# Provides the 'full_isaac_bringup.launch.py' entrypoint delegating to
# isaac_full_bringup.launch.py.
# ==============================================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_bringup = get_package_share_directory("robot_bringup")
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, "launch", "isaac_full_bringup.launch.py")
            )
        )
    ])
