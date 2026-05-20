#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Resolve the UV venv site-packages path relative to this launch file.
# os.path.realpath resolves the symlink that colcon --symlink-install creates
# (install/share/.../lift_homing.launch.py -> src/.../bringup/launch/lift_homing.launch.py)
# so _pkg_dir correctly points to the bringup/ source directory where .venv lives.
# Without --symlink-install, set ROBOT_BRINGUP_VENV_SITE to the site-packages path.
_pkg_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
_venv_site = os.path.join(_pkg_dir, '.venv', 'lib', 'python3.12', 'site-packages')
_venv_site = os.environ.get('ROBOT_BRINGUP_VENV_SITE', _venv_site)
_existing_pythonpath = os.environ.get('PYTHONPATH', '')
_pythonpath = os.pathsep.join(
    path for path in (_venv_site, _existing_pythonpath) if path
)


def generate_launch_description():
    motor_id = LaunchConfiguration("motor_id")
    motor_type = LaunchConfiguration("motor_type")
    homing_erpm = LaunchConfiguration("homing_erpm")
    current_diff_limit = LaunchConfiguration("current_diff_limit")
    abs_current_cap = LaunchConfiguration("abs_current_cap")
    control_frequency = LaunchConfiguration("control_frequency")
    max_velocity_cm_s = LaunchConfiguration("max_velocity_cm_s")
    max_acceleration_cm_s2 = LaunchConfiguration("max_acceleration_cm_s2")
    homing_backoff_time_s = LaunchConfiguration("homing_backoff_time_s")
    hardware_max_cm_s = LaunchConfiguration("hardware_max_cm_s")
    lift_m_per_rad = LaunchConfiguration("lift_m_per_rad")
    lift_min_cm = LaunchConfiguration("lift_min_cm")
    lift_max_cm = LaunchConfiguration("lift_max_cm")
    mock = LaunchConfiguration("mock")
    joint_name = LaunchConfiguration("joint_name")
    topic_joint_states = LaunchConfiguration("topic_joint_states")
    topic_lift_controller_commands = LaunchConfiguration("topic_lift_controller_commands")
    topic_position_commands = LaunchConfiguration("topic_position_commands")

    return LaunchDescription([
        DeclareLaunchArgument("mock", default_value="false",
                              description="Run mock node (no CAN bus). Set to 'true' for sim/dev without hardware."),
        DeclareLaunchArgument("motor_id", default_value="16"),
        DeclareLaunchArgument("motor_type", default_value="AK45-10"),
        DeclareLaunchArgument("homing_erpm", default_value="-8500"),
        DeclareLaunchArgument("current_diff_limit", default_value="0.85"),
        DeclareLaunchArgument("abs_current_cap", default_value="4.0"),
        DeclareLaunchArgument("control_frequency", default_value="100.0"),
        DeclareLaunchArgument("max_velocity_cm_s", default_value="5.0"),
        DeclareLaunchArgument("max_acceleration_cm_s2", default_value="20.0"),
        DeclareLaunchArgument("homing_backoff_time_s", default_value="3.0"),
        DeclareLaunchArgument("hardware_max_cm_s", default_value="3.0",
                              description="Hard ceiling for sustainable lift speed (cm/s). Caps the SW trajectory so the RViz visualization (which mirrors traj_current_pos_m) cannot outrun what the motor can physically follow."),
        DeclareLaunchArgument("lift_m_per_rad", default_value="-0.00012406",
                              description="Signed m/rad (motor-side). Calibrated: commanded 5 cm, got 4 cm → 0.8× of 1 rev = 1 cm."),
        DeclareLaunchArgument("lift_min_cm", default_value="0.0",
                              description="URDF prismatic lower limit (cm). Bottom of travel."),
        DeclareLaunchArgument("lift_max_cm", default_value="74.35",
                              description="URDF prismatic upper limit (cm). Hard-stop home is here."),
        DeclareLaunchArgument("joint_name", default_value="lift_joint"),
        DeclareLaunchArgument("topic_joint_states", default_value="lift/joint_states"),
        DeclareLaunchArgument("topic_lift_controller_commands", default_value="lift_controller/commands"),
        DeclareLaunchArgument("topic_position_commands", default_value="lift/cmd"),
        # Real hardware node
        Node(
            condition=UnlessCondition(mock),
            package="robot_bringup",
            executable="lift_homing_node.py",
            name="tmotor_ros2_bridge",
            output="screen",
            additional_env={'PYTHONPATH': _pythonpath},
            parameters=[{
                "motor_id": motor_id,
                "motor_type": motor_type,
                "homing_erpm": homing_erpm,
                "current_diff_limit": current_diff_limit,
                "abs_current_cap": abs_current_cap,
                "control_frequency": control_frequency,
                "max_velocity_cm_s": max_velocity_cm_s,
                "max_acceleration_cm_s2": max_acceleration_cm_s2,
                "homing_backoff_time_s": homing_backoff_time_s,
                "hardware_max_cm_s": hardware_max_cm_s,
                "lift_m_per_rad": lift_m_per_rad,
                "lift_min_cm": lift_min_cm,
                "lift_max_cm": lift_max_cm,
                "joint_name": joint_name,
                "topic_joint_states": topic_joint_states,
                "topic_lift_controller_commands": topic_lift_controller_commands,
                "topic_position_commands": topic_position_commands,
            }],
        ),
        # Mock node (no CAN bus — for simulation / dev without hardware)
        Node(
            condition=IfCondition(mock),
            package="robot_bringup",
            executable="lift_homing_mock_node.py",
            name="tmotor_ros2_bridge",
            output="screen",
            parameters=[{
                "control_frequency": control_frequency,
                "max_velocity_cm_s": max_velocity_cm_s,
                "max_acceleration_cm_s2": max_acceleration_cm_s2,
                "hardware_max_cm_s": hardware_max_cm_s,
                "lift_min_cm": lift_min_cm,
                "lift_max_cm": lift_max_cm,
                "joint_name": joint_name,
                "topic_joint_states": topic_joint_states,
                "topic_lift_controller_commands": topic_lift_controller_commands,
                "topic_position_commands": topic_position_commands,
            }],
        ),
    ])
