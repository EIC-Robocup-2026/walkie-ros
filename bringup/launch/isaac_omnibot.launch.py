#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "robot_bringup"
    package_dir = os.path.join(get_package_share_directory(package_name))
    pkg_rosbridge_server = get_package_share_directory("rosbridge_server")
    description_package_name = "walkie_description"

    default_robot = os.path.join(
        get_package_share_directory(description_package_name),
        "robots",
        "gz_walkie.urdf.xacro",
    )

    # Launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    robot_model = LaunchConfiguration("robot_model", default=default_robot)
    ros2_control = LaunchConfiguration("ros2_control", default="topic_base")
    use_zed = LaunchConfiguration("use_zed", default="true")

    declare_use_zed = DeclareLaunchArgument(
        "use_zed", default_value="true", description="Whether to use ZED camera"
    )

    robot_description_content = Command(
        [
            "xacro ",
            default_robot,
            " ros2_control:=",
            ros2_control,
            " use_zed:=",
            use_zed,
        ]
    )

    twist_mux_params = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "twist_mux",
        "twist_mux.yml",
    )
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[("/cmd_vel_out", "/cmd_vel")],
    )

    # twist_stamped_frame_id = 'base_footprint'
    # twist_stamper_node = Node(
    #     package='robot_navigation',
    #     executable='accel_stamp_node.py',
    #     name='Accel_Stamp',
    #     output='screen',
    #     remappings=[
    #             ('/cmd_vel_in', '/cmd_vel'),
    #             ('/cmd_vel_out', '/omni_wheel_drive_controller/cmd_vel'),
    #     ],
    #     parameters=[
    #         {'frame_id': twist_stamped_frame_id},
    #     ]
    # )

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
            "use_zed": use_zed,
        }.items(),
    )

    controllers_config = os.path.join(
        get_package_share_directory(description_package_name),
        "config",
        "ros2_controller",
        "isaac_controllers.yaml",
    )
    controller_manager_spawner = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description", robot_description_content},
            {"use_sim_time": use_sim_time},
            controllers_config,
        ],
    )

    OFFSET_DELAY = 0.0
    JOINT_BROAD_DELAY = 2.0 + OFFSET_DELAY
    OMNI_DELAY = 4.0 + OFFSET_DELAY
    LEFT_ARM_DELAY = 6.0 + OFFSET_DELAY
    RIGHT_ARM_DELAY = 8.0 + OFFSET_DELAY

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--switch-timeout", "30.0"],
    )

    omni_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["omni_wheel_drive_controller", "--switch-timeout", "30.0"],
    )

    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_controller", "--switch-timeout", "30.0"],
    )

    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_arm_controller", "--switch-timeout", "30.0"],
    )

    left_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_gripper_controller", "--switch-timeout", "30.0"],
    )

    right_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_gripper_controller", "--switch-timeout", "30.0"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--switch-timeout", "30.0"],
    )

    delayed_joint_broad_spawner = TimerAction(
        period=JOINT_BROAD_DELAY,
        actions=[joint_broad_spawner],
    )

    delayed_omni_controller_spawner = TimerAction(
        period=OMNI_DELAY,
        actions=[omni_controller_spawner],
    )

    delayed_left_arm_controller_spawner = TimerAction(
        period=LEFT_ARM_DELAY,
        actions=[left_arm_controller_spawner],
    )

    delayed_right_arm_controller_spawner = TimerAction(
        period=RIGHT_ARM_DELAY,
        actions=[right_arm_controller_spawner],
    )

    delayed_left_gripper_controller_spawner = TimerAction(
        period=LEFT_ARM_DELAY,
        actions=[left_gripper_controller_spawner],
    )

    delayed_right_gripper_controller_spawner = TimerAction(
        period=RIGHT_ARM_DELAY,
        actions=[right_gripper_controller_spawner],
    )

    delayed_arm_controller_spawner = TimerAction(
        period=RIGHT_ARM_DELAY,
        actions=[arm_controller_spawner],
    )

    current_pose_publisher = Node(
        package="robot_navigation",
        executable="current_pose_publisher.py",
        name="current_pose_publisher",
        output="screen",
        parameters=[
            {"source_frame": "map"},
            {"target_frame": "base_link"},
            {"publish_rate": 10.0},
            {"topic_name": "current_pose"},
        ],
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
    # Add launch arguments
    ld.add_action(declare_use_zed)
    ld.add_action(robot_state_publisher_cmd)

    ld.add_action(twist_mux)
    ld.add_action(twist_stamper_node)

    ld.add_action(controller_manager_spawner)
    ld.add_action(delayed_joint_broad_spawner)
    ld.add_action(delayed_omni_controller_spawner)
    # ld.add_action(delayed_arm_controller_spawner)
    ld.add_action(delayed_left_arm_controller_spawner)
    ld.add_action(delayed_left_gripper_controller_spawner)
    ld.add_action(delayed_right_arm_controller_spawner)
    ld.add_action(delayed_right_gripper_controller_spawner)

    ld.add_action(current_pose_publisher)
    ld.add_action(rosbridge_launch)

    return ld
