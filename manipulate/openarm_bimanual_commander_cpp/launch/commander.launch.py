from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def _make_commander(context: LaunchContext, base_mode_lc):
    # The commander builds its own robot model, so base_mode has to reach its SRDF
    # expansion too. A model that differs from move_group's is refused per request.
    base_mode = context.perform_substitution(base_mode_lc)
    moveit_config = (
        MoveItConfigsBuilder("walkie_bot", package_name="openarm_bimanual_moveit_config")
        .robot_description_semantic(
            file_path="config/openarm_bimanual.srdf",
            mappings={"base_mode": base_mode},
        )
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
            {"gripper_speed": LaunchConfiguration("gripper_speed")},
            {"gripper_init_pos": LaunchConfiguration("gripper_init_pos")},
            {"base_mode": base_mode},
            {"plan_only": ParameterValue(LaunchConfiguration("plan_only"), value_type=bool)},
            {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)},
        ],
    )
    return [commander_node]


def generate_launch_description():
    # Software gripper speed cap (command-position units per second). Lower =
    # gentler; <= 0 disables the ramp. Override with: gripper_speed:=0.4
    declare_gripper_speed = DeclareLaunchArgument(
        "gripper_speed",
        default_value="0.1",
        description="Software gripper speed cap (command units/s); <=0 disables ramp",
    )

    # Resting/open gripper command at startup so the first move also ramps.
    declare_gripper_init_pos = DeclareLaunchArgument(
        "gripper_init_pos",
        default_value="0.0",
        description="Assumed gripper command position at startup (ramp seed)",
    )

    declare_base_mode = DeclareLaunchArgument(
        "base_mode",
        default_value="fixed",
        description="Base model: fixed | map | whole_body. Must match move_group.",
    )

    declare_plan_only = DeclareLaunchArgument(
        "plan_only",
        default_value="false",
        description="Plan and store without executing. Read once at startup.",
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use the simulation clock. Set to true under Gazebo and MuJoCo.",
    )

    return LaunchDescription(
        [
            declare_gripper_speed,
            declare_gripper_init_pos,
            declare_base_mode,
            declare_plan_only,
            declare_use_sim_time,
            OpaqueFunction(function=_make_commander, args=[LaunchConfiguration("base_mode")]),
        ]
    )
