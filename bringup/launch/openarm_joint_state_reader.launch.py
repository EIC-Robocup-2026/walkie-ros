#!/usr/bin/env python3
"""
Read the OpenArm joint states in free-drive (motors disabled, movable by
hand) and visualize the pose in RViz — without running the full bringup.

Do NOT run while real_omnibot.launch.py / openarm.bimanual.launch.py is
active: this disables the arm motors on startup.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_dir = get_package_share_directory('robot_bringup')
    robot_model = os.path.join(
        get_package_share_directory('walkie_description'),
        'robots', 'gz_walkie.urdf.xacro')

    arms = LaunchConfiguration('arms')
    left_can_interface = LaunchConfiguration('left_can_interface')
    right_can_interface = LaunchConfiguration('right_can_interface')
    hand = LaunchConfiguration('hand')
    rate_hz = LaunchConfiguration('rate_hz')
    disable_on_start = LaunchConfiguration('disable_on_start')
    print_throttle_s = LaunchConfiguration('print_throttle_s')
    use_rsp = LaunchConfiguration('use_rsp')
    rviz = LaunchConfiguration('rviz')
    lift_position = LaunchConfiguration('lift_position')

    declared_arguments = [
        DeclareLaunchArgument(
            'arms', default_value='both', choices=['both', 'left', 'right'],
            description='Which arm(s) to read'),
        DeclareLaunchArgument(
            'left_can_interface', default_value='can1',
            description='CAN interface for the left arm'),
        DeclareLaunchArgument(
            'right_can_interface', default_value='can0',
            description='CAN interface for the right arm'),
        DeclareLaunchArgument(
            'hand', default_value='true',
            description='Also read the gripper motors'),
        DeclareLaunchArgument(
            'rate_hz', default_value='50.0',
            description='Joint state polling/publishing rate'),
        DeclareLaunchArgument(
            'disable_on_start', default_value='true',
            description='Send disable_all on startup so the arms are limp'),
        DeclareLaunchArgument(
            'print_throttle_s', default_value='2.0',
            description='Log a rad/deg joint table to the console at this period '
                        '(s); 0 disables'),
        DeclareLaunchArgument(
            'use_rsp', default_value='true',
            description='Start robot_state_publisher + joint_state_publisher for TF'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz'),
        DeclareLaunchArgument(
            'lift_position', default_value='0.7435',
            description='Displayed lift_joint position (m); the real lift homes '
                        'against the upper hard stop, so default is the top'),
    ]

    reader_node = Node(
        package='robot_bringup',
        executable='openarm_joint_state_reader.py',
        name='openarm_joint_state_reader',
        output='screen',
        parameters=[{
            'arms': arms,
            'left_can_interface': left_can_interface,
            'right_can_interface': right_can_interface,
            'hand': hand,
            'rate_hz': rate_hz,
            'disable_on_start': disable_on_start,
            'print_throttle_s': print_throttle_s,
            'joint_states_topic': 'openarm/joint_states',
        }],
    )

    # Kinematics-only robot description: ros2_control:=mock keeps every real
    # hardware plugin out of the URDF (no controller_manager runs here).
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={
            'robot_model': robot_model,
            'ros2_control': 'mock',
            'use_fake_arm_hardware': 'true',
            'use_arm': 'true',
            'use_zed': 'false',
        }.items(),
        condition=IfCondition(use_rsp),
    )

    # Merges the arm states with zero-defaults for every other walkie joint
    # (wheels, lift, head) so TF is complete and RViz can render the robot.
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'source_list': ['openarm/joint_states'],
            'rate': 50,
            'zeros.lift_joint': ParameterValue(lift_position, value_type=float),
        }],
        condition=IfCondition(use_rsp),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', os.path.join(
            bringup_dir, 'config', 'rviz', 'openarm_read.rviz')],
        condition=IfCondition(rviz),
    )

    return LaunchDescription(declared_arguments + [
        reader_node,
        robot_state_publisher,
        joint_state_publisher,
        rviz_node,
    ])
