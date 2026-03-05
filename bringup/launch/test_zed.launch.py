#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessStart
from launch.actions import RegisterEventHandler


def generate_launch_description():
    package_name = "robot_bringup"
    package_dir = os.path.join(get_package_share_directory(package_name))
    pkg_rosbridge_server = get_package_share_directory("rosbridge_server")
    description_package_name = 'walkie_description'

    default_robot = os.path.join(get_package_share_directory(description_package_name),
                                 'robots',
                                 'gz_walkie_1arm.urdf.xacro'
                                 )

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    robot_model = LaunchConfiguration('robot_model', default=default_robot)
    ros2_control = LaunchConfiguration('ros2_control', default='mock')
    use_zed = LaunchConfiguration('use_zed', default='true')

    declare_use_zed = DeclareLaunchArgument(
        'use_zed',
        default_value='true',
        description='Whether to use ZED camera'
    )

    robot_description_content = Command(
        ['xacro ', default_robot, ' ros2_control:=', ros2_control, ' use_zed:=', use_zed])

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                "launch",
                "robot_state_publisher.launch.py",
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'robot_model': robot_model,
            'ros2_control': ros2_control,
            'use_zed': use_zed,
        }.items()
    )

    controllers_config = os.path.join(
        get_package_share_directory(description_package_name),
        "config",
        "ros2_controller",
        "real_controllers.yaml",
    )

    controller_manager_spawner = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description", robot_description_content},
            controllers_config,
        ],
    )

    omni_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["omni_wheel_drive_controller", "--switch-timeout", "30.0"],
    )

    delayed_omni_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_spawner,
            on_start=[omni_controller_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--switch-timeout", "30.0"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_spawner,
            on_start=[joint_broad_spawner],
        )
    )

    servo_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["head_servo_controller", "--switch-timeout", "30.0"],
    )

    delayed_servo_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_spawner,
            on_start=[servo_controller_spawner],
        )
    )


    # ZED Camera launch (ZED2i)
    # zed_camera_node = Node(
    #     package='zed_wrapper',
    #     executable='zed_wrapper_node',
    #     output='screen',
    #     parameters=[{
    #         'camera_model': 'zed2i',
    #         'base_frame': 'head_zed_camera_link',
    #         'publish_urdf': False,
    #     }],
    # )

    # ZED Camera Launch
    zed_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            ])
        ),
        launch_arguments={
            'camera_model': 'zed2i',
            'camera_name': 'zed_head',
            'base_frame': 'zed_head_camera_link',
            'publish_urdf': 'false',
            'publish_tf': 'false',
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(use_zed)
    )

    ld = LaunchDescription()
    # Add launch arguments
    ld.add_action(declare_use_zed)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(controller_manager_spawner)
    ld.add_action(delayed_omni_controller_spawner)
    ld.add_action(delayed_joint_broad_spawner)
    ld.add_action(delayed_servo_controller_spawner)
    ld.add_action(zed_camera_launch)

    return ld
