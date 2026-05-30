from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("walkie_bot", package_name="openarm_bimanual_moveit_config")
        .to_moveit_configs()
    )

    # Software gripper speed cap (command-position units per second). Lower =
    # gentler; <= 0 disables the ramp. Override with: gripper_speed:=0.4
    gripper_speed = LaunchConfiguration("gripper_speed")
    declare_gripper_speed = DeclareLaunchArgument(
        "gripper_speed",
        default_value="0.1",
        description="Software gripper speed cap (command units/s); <=0 disables ramp",
    )

    # Resting/open gripper command at startup so the first move also ramps.
    gripper_init_pos = LaunchConfiguration("gripper_init_pos")
    declare_gripper_init_pos = DeclareLaunchArgument(
        "gripper_init_pos",
        default_value="0.0",
        description="Assumed gripper command position at startup (ramp seed)",
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
            {"gripper_speed": gripper_speed},
            {"gripper_init_pos": gripper_init_pos},
        ],
    )

    return LaunchDescription(
        [declare_gripper_speed, declare_gripper_init_pos, commander_node]
    )
