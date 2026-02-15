#!/usr/bin/env python3
import os
from struct import pack
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    AnyLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "robot_simulation"

    gzsim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                "launch",
                "gzsim_omnibot.launch.py",
            )
        ),
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_navigation"),
                "launch",
                "localization_Nav2_sim.launch.py",
            )
        ),
    )

    delayed_nav_launch = TimerAction(
        period=5.0,  # Adjust this delay (seconds) based on your PC speed
        actions=[nav_launch],
    )

    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rosbridge_server"),
                "launch",
                "rosbridge_websocket_launch.xml",
            )
        ),
        launch_arguments={"delay_between_messages": "0.0"}.items(),
    )
    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzsim_launch)
    ld.add_action(delayed_nav_launch)
    ld.add_action(rosbridge_launch)

    return ld
