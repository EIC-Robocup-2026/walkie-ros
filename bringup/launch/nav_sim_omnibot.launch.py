#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_robot_simulation = get_package_share_directory("robot_simulation")
    pkg_robot_navigation = get_package_share_directory("robot_navigation")
    pkg_rosbridge_server = get_package_share_directory("rosbridge_server")
    pkg_walkie_description = get_package_share_directory("walkie_description")

    default_robot = os.path.join(
        pkg_walkie_description,
        "robots",
        "gz_walkie.urdf.xacro",
    )

    robot_model = LaunchConfiguration("robot_model")
    ros2_control = LaunchConfiguration("ros2_control")
    dual_lidar = LaunchConfiguration("dual_lidar")

    declare_robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value=default_robot,
        description="Path to robot model file",
    )

    declare_ros2_control_arg = DeclareLaunchArgument(
        "ros2_control",
        default_value="gazebo",
        description="ROS 2 control hardware type",
    )

    declare_dual_lidar_arg = DeclareLaunchArgument(
        "dual_lidar",
        default_value="False",
        description="Use dual lidar",
    )

    gzsim_omnibot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_simulation, "launch", "gzsim_omnibot.launch.py")
        ),
        launch_arguments={
            "robot_model": robot_model,
            "ros2_control": ros2_control,
            "dual_lidar": dual_lidar,
        }.items(),
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

    ld.add_action(declare_robot_model_arg)
    ld.add_action(declare_ros2_control_arg)
    ld.add_action(declare_dual_lidar_arg)

    ld.add_action(localization_nav2_sim_launch)
    ld.add_action(gzsim_omnibot_launch)
    ld.add_action(rosbridge_launch)

    return ld
