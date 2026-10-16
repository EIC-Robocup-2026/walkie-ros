import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def _make_nodes(context: LaunchContext, output_json_lc, collision_checks_lc, planning_trials_lc):
    output_json = context.perform_substitution(output_json_lc)
    collision_checks = context.perform_substitution(collision_checks_lc)
    planning_trials = context.perform_substitution(planning_trials_lc)

    mappings = {
        "ros2_control": "mock",
        "add_world_link": "true",
        "left_use_fake_arm_hardware": "true",
        "right_use_fake_arm_hardware": "true",
        "right_joint2_fixed": "false",
    }

    moveit_config = (
        MoveItConfigsBuilder("openarm", package_name="openarm_bimanual_moveit_config")
        .robot_description(
            file_path="config/openarm_bimanual.urdf.xacro",
            mappings=mappings,
        )
        .robot_description_semantic(
            file_path="config/openarm_bimanual.srdf",
            mappings={"right_joint2_fixed": "false"},
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    moveit_params = moveit_config.to_dict()

    bench_node = Node(
        package="openarm_bimanual_commander_cpp",
        executable="benchmark_collision",
        output="screen",
        parameters=[moveit_params],
        arguments=[
            "--output", output_json,
            "--collision-checks", collision_checks,
            "--planning-trials", planning_trials,
        ],
    )

    return [bench_node]

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("output_json", default_value="benchmark_results.json"),
        DeclareLaunchArgument("collision_checks", default_value="5000"),
        DeclareLaunchArgument("planning_trials", default_value="20"),
    ]

    output_json = LaunchConfiguration("output_json")
    collision_checks = LaunchConfiguration("collision_checks")
    planning_trials = LaunchConfiguration("planning_trials")

    nodes_func = OpaqueFunction(
        function=_make_nodes,
        args=[output_json, collision_checks, planning_trials],
    )

    return LaunchDescription(declared_arguments + [nodes_func])
