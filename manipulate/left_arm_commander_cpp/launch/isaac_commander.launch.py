from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("left_walkie_arm", package_name="left_arm_moveit_config") \
        .to_moveit_configs()

    return LaunchDescription([
       Node(
            package='left_arm_commander_cpp',
            executable='commander_isaac',
            name='commander_isaac',
            output='screen',
            parameters=[
                moveit_config.to_dict(),
                {'use_sim_time': True}
            ],
            # ADD THIS TO CONNECT ISAAC TO ROS
            remappings=[
                ('/tf', '/isaac_tf') 
            ]
        )
    ])
