from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("walkie_bot", package_name="openarm_bimanual_moveit_config")
        .to_moveit_configs()
    )

    commander_node = Node(
        package="openarm_bimanual_commander_cpp",
        executable="commander",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([commander_node])
