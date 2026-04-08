# Launches pick_and_place_isaac alongside the bimanual commander.
# The commander action servers must be up before the task node starts,
# so the task node is delayed by 5 s.
#
# Usage (after the full MoveIt stack is already running):
#   ros2 launch openarm_bimanual_commander_cpp pick_and_place_isaac.launch.py

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("openarm", package_name="openarm_bimanual_moveit_config")
        .to_moveit_configs()
    )

    common_params = [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
        {"use_sim_time": True},
    ]

    commander = Node(
        package="openarm_bimanual_commander_cpp",
        executable="commander",
        name="bimanual_commander",
        output="screen",
        parameters=common_params,
    )

    # Delayed so commander action servers are ready before the task starts
    pick_and_place = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="openarm_bimanual_commander_cpp",
                executable="pick_and_place_isaac",
                name="pick_and_place_isaac",
                output="screen",
                parameters=[{"use_sim_time": True}],
            )
        ],
    )

    return LaunchDescription([commander, pick_and_place])
