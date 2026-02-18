from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="keyboard_teleop",
                executable="walkie_keyboard_input",
                name="walkie_keyboard_input",
                output="screen",
            )
        ]
    )
