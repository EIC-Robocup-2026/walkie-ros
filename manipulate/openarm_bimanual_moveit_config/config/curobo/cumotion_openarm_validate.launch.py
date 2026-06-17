# Headless validation of the cuMotion MoveIt pipeline on the OpenArm bimanual
# robot (no ros2_control / hardware). joint_state_publisher supplies
# /joint_states, which is all PLANNING needs.
#
# move_group params are assembled by hand from the staged config files because
# the openarm_bimanual_moveit_config package is not installed in the container.
# cuMotion is registered as the (only) planning pipeline and made default.
import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

BASE = "/workspaces/isaac_ros-dev/openarm_cumotion"
# move_group / RViz use the FULL (movable) robot.
URDF = f"{BASE}/openarm_bimanual.urdf"
# Planner URDF (env-selectable). NOTE: the arm-fixed-URDF idea did NOT fix RViz
# (the cuMotion 4.4 plugin sends the FULL 24-joint RobotState regardless), so the
# default stays the full URDF which the CLI path (exact cspace joints) needs.
CUMOTION_URDF = f"{BASE}/{os.environ.get('CUMOTION_URDF', 'openarm_bimanual.urdf')}"
XRDF = f"{BASE}/{os.environ.get('CUMOTION_XRDF', 'openarm_bimanual.xrdf')}"
MC = f"{BASE}/moveit_config"


def load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def generate_launch_description():
    urdf_str = Path(URDF).read_text()
    srdf_str = Path(f"{MC}/openarm_bimanual.srdf").read_text()
    kinematics = load_yaml(f"{MC}/kinematics.yaml")
    joint_limits = load_yaml(f"{MC}/joint_limits.yaml")

    cumotion_cfg = load_yaml(os.path.join(
        get_package_share_directory("isaac_ros_cumotion_moveit"),
        "config", "isaac_ros_cumotion_planning.yaml"))
    # CheckStartStateCollision stays ENABLED: the SRDF already disables the safe
    # base<->arm pairs (link0/1/2) and correctly keeps distal pairs checked, so a
    # valid arms-raised start passes while genuine base collisions are caught.
    # The stock cuMotion pipeline config has NO response_adapters, so move_group
    # never publishes /display_planned_path for cuMotion plans -> nothing shows in
    # RViz. Add DisplayMotionPath (publishes the trajectory) for RViz animation.
    #
    # ValidateSolution is deliberately NOT included: the planned trajectory only
    # carries the active group's cspace joints, so move_group's FCL validator
    # places every OTHER joint (e.g. the idle arm) at its URDF zero pose -> folded
    # into base_link -> a FALSE collision that rejects an otherwise valid plan.
    # cuMotion can't take a full 24-joint start_state to fix this (it requires
    # EXACTLY the cspace dimension), and its own world already locks the idle arm
    # at the raised home and plans collision-free, so re-validation is redundant.
    cumotion_cfg["response_adapters"] = [
        "default_planning_response_adapters/DisplayMotionPath",
    ]

    # OMPL pipeline (RRTConnect) registered alongside cuMotion for benchmarking.
    # Both live in the same move_group; pick per request via pipeline_id.
    ompl_cfg = load_yaml(f"{MC}/ompl_planning.yaml")

    planning_pipelines = {
        "planning_pipelines": ["isaac_ros_cumotion", "ompl"],
        "default_planning_pipeline": "isaac_ros_cumotion",
        "isaac_ros_cumotion": cumotion_cfg,
        "ompl": ompl_cfg,
    }

    move_group_params = {
        "robot_description": urdf_str,
        "robot_description_semantic": srdf_str,
        "robot_description_kinematics": kinematics,
        "robot_description_planning": joint_limits,
        **planning_pipelines,
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": False,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "use_sim_time": False,
    }

    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        output="screen", parameters=[{"robot_description": urdf_str}],
    )
    # NOTE: no joint_state_publisher here. /joint_states is provided solely by
    # pub_js.py (arms-raised pose). Running joint_state_publisher too creates a
    # SECOND publisher (zeros/defaults) that fights pub_js, flickering the
    # current state between the raised pose and the bad q=0 folded pose and
    # making plans fail intermittently.
    move_group = Node(
        package="moveit_ros_move_group", executable="move_group",
        output="screen", parameters=[move_group_params],
    )
    cumotion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("isaac_ros_cumotion"),
            "launch", "isaac_ros_cumotion.launch.py")),
        launch_arguments={
            "cumotion_action_server.urdf_file_path": CUMOTION_URDF,
            "cumotion_action_server.xrdf_file_path": XRDF,
            "cumotion_action_server.read_esdf_world": "false",
            "cumotion_action_server.add_ground_plane": "false",
        }.items(),
    )

    # Sole /joint_states source: the arms-raised home pose. Managed by the
    # launch (auto-restarted) so it can't silently die like a detached exec.
    # No respawn: as a launch-managed process it stays alive with the stack;
    # respawn caused stale pub_js to accumulate across repeated launches.
    pub_js = ExecuteProcess(
        cmd=["python3", f"{BASE}/pub_js.py"],
        output="screen",
    )

    return LaunchDescription([rsp, move_group, cumotion, pub_js])
