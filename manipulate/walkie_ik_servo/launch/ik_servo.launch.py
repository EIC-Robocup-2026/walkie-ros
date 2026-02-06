"""Launch file for the walkie_ik_servo IK direct control node.

Loads parameters from config/ik_params.yaml and starts the IK servo node
that bridges end-effector Pose commands to ros2_control JointTrajectory commands.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("walkie_ik_servo")

    # Default config file path
    default_params_file = os.path.join(pkg_share, "config", "ik_params.yaml")

    # Declare launch arguments
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Path to the YAML parameter file for the IK servo node",
    )

    # IK Servo node
    ik_servo_node = Node(
        package="walkie_ik_servo",
        executable="ik_servo_node.py",
        name="ik_servo_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription(
        [
            params_file_arg,
            ik_servo_node,
        ]
    )
