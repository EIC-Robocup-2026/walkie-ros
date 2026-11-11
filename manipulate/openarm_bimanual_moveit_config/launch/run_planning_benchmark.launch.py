"""Planning benchmark of the whole-body setup near a table, in one process.

Loads the robot model, IK plugins and OMPL pipeline the way moveit_only.launch.py
does for hardware_type:=mujoco base_mode:=whole_body, and runs benchmark_planning
from openarm_bimanual_commander_cpp. Needs no robot, simulator, router or move_group.

The IK and OMPL settings can be overridden per run to compare them:
    ros2 launch openarm_bimanual_moveit_config run_planning_benchmark.launch.py \
        ik_solve_type:=Speed segment_fraction:=0.01 output_json:=/tmp/speed.json
"""

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

_IK_GROUPS = ("left_arm", "right_arm", "left_arm_lift", "right_arm_lift")


def _make_nodes(context: LaunchContext):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    moveit_config = (
        MoveItConfigsBuilder("openarm", package_name="openarm_bimanual_moveit_config")
        .robot_description(
            file_path="config/openarm_bimanual.urdf.xacro",
            mappings={
                "ros2_control": "mujoco",
                "add_world_link": "false",
                "left_use_fake_arm_hardware": "true",
                "right_use_fake_arm_hardware": "true",
                "right_joint2_fixed": "false",
            },
        )
        .robot_description_semantic(
            file_path="config/openarm_bimanual.srdf",
            mappings={"right_joint2_fixed": "false", "base_mode": "whole_body"},
        )
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )
    # Same response adapter order as move_group in whole_body mode.
    moveit_config.planning_pipelines["ompl"]["response_adapters"].insert(
        0, "openarm_bimanual_commander_cpp/UnwrapPlanarHeading"
    )
    params = moveit_config.to_dict()

    kinematics = params["robot_description_kinematics"]
    for group in _IK_GROUPS:
        if arg("ik_solve_type"):
            kinematics[group]["solve_type"] = arg("ik_solve_type")
        if arg("ik_timeout"):
            kinematics[group]["kinematics_solver_timeout"] = float(arg("ik_timeout"))
    if arg("segment_fraction"):
        for name, entry in params["ompl"].items():
            if isinstance(entry, dict) and "longest_valid_segment_fraction" in entry:
                entry["longest_valid_segment_fraction"] = float(arg("segment_fraction"))
    if arg("totg_path_tolerance"):
        # Read by AddTimeOptimalParameterization as ompl.totg.path_tolerance.
        params["ompl"].setdefault("totg", {})["path_tolerance"] = float(arg("totg_path_tolerance"))

    command = [
        "--output", arg("output_json"),
        "--scenes", arg("scenes"),
        "--sections", arg("sections"),
        "--ik-calls", arg("ik_calls"),
        "--plan-trials", arg("plan_trials"),
        "--wb-reseeds", arg("wb_reseeds"),
        "--planning-time", arg("planning_time"),
        "--attempt-goals", arg("attempt_goals"),
        "--attempt-counts", arg("attempt_counts"),
        "--pose-times", arg("pose_times"),
        "--pose-goals", arg("pose_goals"),
        "--pose-repeats", arg("pose_repeats"),
        "--finger-padding", arg("finger_padding"),
    ]
    if arg("quick").lower() == "true":
        command.append("--quick")
    if arg("target_xs"):
        command += ["--target-xs", arg("target_xs")]

    return [
        Node(
            package="openarm_bimanual_commander_cpp",
            executable="benchmark_planning",
            output="screen",
            parameters=[params],
            arguments=command,
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("output_json", default_value="planning_benchmark.json"),
            DeclareLaunchArgument(
                "scenes",
                default_value="empty,boxes,octomap:0.03,octomap:0.02",
                description="Comma-separated: empty, boxes, octomap:<resolution>.",
            ),
            DeclareLaunchArgument(
                "sections",
                default_value="geometry,collision,ik,arm_plan,attempts,pose_plan,whole_body",
                description="Comma-separated parts to run.",
            ),
            DeclareLaunchArgument("ik_calls", default_value="16"),
            DeclareLaunchArgument("plan_trials", default_value="3"),
            DeclareLaunchArgument(
                "wb_reseeds",
                default_value="0",
                description="If above 0, a second whole-body pass retries each failed IK call "
                "this many times with random seeds.",
            ),
            DeclareLaunchArgument("planning_time", default_value="5.0"),
            DeclareLaunchArgument(
                "attempt_goals",
                default_value="4",
                description="Arm goals with a collision-free start that the attempts section plans to.",
            ),
            DeclareLaunchArgument(
                "attempt_counts",
                default_value="1,4,10000",
                description="Comma-separated planning attempt counts for the attempts section.",
            ),
            DeclareLaunchArgument(
                "pose_times",
                default_value="1.5,5",
                description="Comma-separated planning times in seconds for the pose_plan section.",
            ),
            DeclareLaunchArgument(
                "pose_goals",
                default_value="6",
                description="Hand poses from the ik section that pose_plan plans to, besides the "
                "launch tests' near arm target.",
            ),
            DeclareLaunchArgument("pose_repeats", default_value="4"),
            DeclareLaunchArgument(
                "finger_padding",
                default_value="0.01",
                description="Collision padding of the finger links in m, the commander's finger_padding default.",
            ),
            DeclareLaunchArgument("quick", default_value="false"),
            DeclareLaunchArgument(
                "target_xs",
                default_value="",
                description="Comma-separated target x positions in map (the table spans 1.55 to 2.25). "
                "Empty: the default rows.",
            ),
            DeclareLaunchArgument(
                "ik_solve_type",
                default_value="",
                description="Override TRAC-IK solve_type (Speed, Distance, Manip1, Manip2).",
            ),
            DeclareLaunchArgument("ik_timeout", default_value=""),
            DeclareLaunchArgument(
                "segment_fraction",
                default_value="",
                description="Override longest_valid_segment_fraction of every OMPL group.",
            ),
            DeclareLaunchArgument(
                "totg_path_tolerance",
                default_value="",
                description="Override the corner blend tolerance of time parameterization (MoveIt default 0.1).",
            ),
            OpaqueFunction(function=_make_nodes),
        ]
    )
