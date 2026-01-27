from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # CHANGED: "dual_walkie_arm" -> "left_walkie_arm"
    # This must match the name used in your SRDF and other config files
    moveit_config = MoveItConfigsBuilder("left_walkie_arm", package_name="moveit_config").to_moveit_configs()

    commander_node = Node(
        package="left_arm_commander_cpp",
        executable="commander",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            # Add this if you want to use simulation time (good practice for Gazebo)
            # {'use_sim_time': True} 
        ],
    )

    return LaunchDescription([
        commander_node
    ])