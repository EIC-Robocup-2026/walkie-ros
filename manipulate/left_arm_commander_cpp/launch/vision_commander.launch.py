"""
vision_commander.launch.py

Launches the full camera-based pick and place pipeline:
  1. vision_pick_server  — YOLO + depth camera -> get_target_position service
  2. commander_vision    — arm motion using the vision service (no Isaac Sim TF)

Both nodes work identically in simulation and on the real robot.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("left_walkie_arm", package_name="left_arm_moveit_config")
        .to_moveit_configs()
    )

    # 1. Vision server: camera -> object position in base_link
    vision_server = Node(
        package='perception',
        executable='vision_pick_server.py',
        name='vision_pick_server',
        output='screen',
        parameters=[{
            # Change target_class_id to pick a different object:
            #   39 = bottle,  41 = cup,  45 = bowl
            'target_class_id':       39,
            'confidence_threshold':  0.50,
            'target_frame':          'base_link',
            'depth_search_radius':   7,
            'min_depth_m':           0.15,
            'max_depth_m':           3.0,
        }],
    )

    # 2. Arm commander: uses vision service, no Isaac Sim dependency
    arm_commander = Node(
        package='left_arm_commander_cpp',
        executable='commander_vision',
        name='commander_vision',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([
        vision_server,
        arm_commander,
    ])
