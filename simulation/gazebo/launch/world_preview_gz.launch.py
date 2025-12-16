#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_name = "robot_simulation"

    description_package_name = "walkie_description"

    default_world = os.path.join(
        get_package_share_directory(package_name),
        "worlds",
        "aws_small_house.world",
    )

    # Launch configuration variables
    world_file = LaunchConfiguration("world", default=default_world)

    gzmodel_cmd = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.path.join(get_package_share_directory(package_name), "models")
        + ":"
        + os.path.join(get_package_share_directory(description_package_name), ".."),
    )

    gz_system_plugin_cmd = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH", value="/opt/ros/jazzy/lib/"
    )

    # Include the Gazebo launch file, provided by the ros_gz_sim package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                )
            ]
        ),
        launch_arguments={
            "gz_args": ["-r -v4 ", world_file],
            "on_exit_shutdown": "true",
        }.items(),
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gz_system_plugin_cmd)
    ld.add_action(gzmodel_cmd)
    ld.add_action(gazebo)

    return ld
