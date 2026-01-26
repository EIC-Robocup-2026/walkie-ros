#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)


def generate_launch_description():
    pkg_robot_simulation = get_package_share_directory("robot_simulation")
    pkg_robot_navigation = get_package_share_directory("robot_navigation")
    pkg_rosbridge_server = get_package_share_directory("rosbridge_server")

    gzsim_omnibot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_simulation, "launch", "gzsim_omnibot.launch.py")
        ),
    )

    localization_nav2_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_robot_navigation, "launch", "localization_Nav2_sim.launch.py"
            )
        ),
    )

    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                pkg_rosbridge_server, "launch", "rosbridge_websocket_launch.xml"
            )
        ),
        launch_arguments={
            "delay_between_messages": "0.0",
        }.items(),
    )

    ld = LaunchDescription()

    ld.add_action(localization_nav2_sim_launch)
    ld.add_action(gzsim_omnibot_launch)
    ld.add_action(rosbridge_launch)

    return ld
