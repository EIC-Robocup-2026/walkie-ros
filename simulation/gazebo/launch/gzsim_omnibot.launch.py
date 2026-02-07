#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "robot_simulation"
    package_dir = os.path.join(get_package_share_directory(package_name))

    description_package_name = "walkie_description"

    default_world = os.path.join(
        get_package_share_directory(package_name), "worlds", "aws_small_house.world"
    )

    default_robot = os.path.join(
        get_package_share_directory(description_package_name),
        "robots",
        "gz_walkie.urdf.xacro",
    )

    # Launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    x_pose = LaunchConfiguration("x_pose", default="-2.0")
    y_pose = LaunchConfiguration("y_pose", default="1.0")
    world_file = LaunchConfiguration("world", default=default_world)
    robot_model = LaunchConfiguration("robot_model", default=default_robot)
    ros2_control = LaunchConfiguration("ros2_control", default="gazebo")
    dual_lidar = LaunchConfiguration("dual_lidar", default="False")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_rviz = LaunchConfiguration("use_rviz")

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(package_dir, "rviz", "turtlebot3_rviz.rviz"),
        description="Full path to the RVIZ config file to use",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="Whether to start RVIZ"
    )

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

    foxgloveBridge_cmd = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"send_buffer_limit": 50000000},  # 50 MB
        ],
    )

    twist_mux_params = os.path.join(
        get_package_share_directory("robot_bringup"),
        "config",
        "twist_mux",
        "twist_mux.yml",
    )
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {"use_sim_time": True}],
        remappings=[("/cmd_vel_out", "/cmd_vel")],
    )

    twist_stamped_frame_id = "base_footprint"
    twist_stamper_node = Node(
        package="twist_stamper",
        executable="twist_stamper",
        name="twist_stamper",
        output="screen",
        remappings=[
            ("/cmd_vel_in", "/cmd_vel"),
            ("/cmd_vel_out", "/omni_wheel_drive_controller/cmd_vel"),
        ],
        parameters=[
            {"frame_id": twist_stamped_frame_id},
        ],
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                "launch",
                "robot_state_publisher.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "robot_model": robot_model,
            "ros2_control": ros2_control,
            "dual_lidar": dual_lidar,
        }.items(),
    )

    # Run the spawner node from the ros_gz_sim package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "walkie_bot",
            "-x",
            x_pose,
            "-y",
            y_pose,
            "-z",
            "0.1",
        ],
        output="screen",
    )

    omni_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["omni_wheel_drive_controller", "--switch-timeout", "30.0"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--switch-timeout", "30.0"],
    )

    delayed_spawners = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[
                # forward_velocity_spawner,
                omni_controller_spawner,
                joint_broad_spawner,
            ],
        )
    )

    bridge_params = os.path.join(
        get_package_share_directory(package_name), "config", "gazebo", "gz_bridge.yaml"
    )
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}",
        ],
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, "launch", "rviz.launch.py")
        ),
        condition=IfCondition(use_rviz),
        launch_arguments={"rviz_config": rviz_config_file}.items(),
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gz_system_plugin_cmd)
    ld.add_action(gzmodel_cmd)
    ld.add_action(gazebo)
    ld.add_action(ros_gz_bridge)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_entity_node)

    ld.add_action(twist_mux)
    ld.add_action(twist_stamper_node)
    ld.add_action(delayed_spawners)
    # ld.add_action(forward_velocity_spawner)
    # ld.add_action(omni_controller_spawner)
    # ld.add_action(joint_broad_spawner)

    ld.add_action(foxgloveBridge_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(rviz_cmd)

    return ld
