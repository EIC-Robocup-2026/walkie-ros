# cuMotion analog of moveit_only.launch.py: standalone move_group + RViz with
# cuMotion added as a SECOND planning pipeline (ompl default) + the
# cumotion_planner_node. OVERLAY only — assumes a bringup is already running
# (robot_state_publisher, ros2_control_node, controllers, /joint_states), e.g.
# the real-robot bringup, Gazebo, or moveit_demo_cumotion.launch.py.
#
# For a SELF-CONTAINED stack where the robot renders by itself, use
# moveit_demo_cumotion.launch.py (mock) instead.
#
# Run inside the Isaac container (walkie packages + patched
# isaac_ros_cumotion_moveit overlay built/sourced).
import os

import yaml
from ament_index_python.packages import (
    PackageNotFoundError, get_package_share_directory)
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


# hardware_type -> xacro mappings (must match moveit_only.launch.py)
_HW_MAPPINGS = {
    "mock_components": {
        "ros2_control": "mock",
        "openarm_ros2_control": "true",
        "openarm_use_fake_hardware": "true",
        "add_world_link": "true",
    },
    "gazebo": {
        "ros2_control": "gazebo",
        "openarm_ros2_control": "false",
        "add_world_link": "false",
    },
    "isaac": {
        "ros2_control": "isaac",
        "openarm_ros2_control": "false",
        "add_world_link": "true",
    },
    "real_robot": {
        "ros2_control": "real_robot",
        "openarm_ros2_control": "true",
        "openarm_use_fake_hardware": "false",
        "add_world_link": "false",
    },
}


def _make_nodes(context: LaunchContext, hardware_type_lc, point_cloud_topic_lc,
                octomap_resolution_lc, cumotion_xrdf_lc, cumotion_urdf_lc):
    hardware_type = context.perform_substitution(hardware_type_lc)
    point_cloud_topic = context.perform_substitution(point_cloud_topic_lc)
    octomap_resolution = float(context.perform_substitution(octomap_resolution_lc))
    cumotion_xrdf = context.perform_substitution(cumotion_xrdf_lc)
    cumotion_urdf = context.perform_substitution(cumotion_urdf_lc)
    mappings = _HW_MAPPINGS.get(hardware_type, _HW_MAPPINGS["mock_components"])

    pkg_moveit = get_package_share_directory("openarm_bimanual_moveit_config")
    use_sim_time = hardware_type in ("gazebo", "isaac")

    moveit_config = (
        MoveItConfigsBuilder("openarm", package_name="openarm_bimanual_moveit_config")
        .robot_description(file_path="config/openarm_bimanual.urdf.xacro", mappings=mappings)
        # ompl stays default; cumotion is the additional GPU pipeline.
        .planning_pipelines(pipelines=["ompl", "cumotion"], default_planning_pipeline="ompl")
        .trajectory_execution(moveit_manage_controllers=False)
        .to_moveit_configs()
    )
    moveit_params = moveit_config.to_dict()
    moveit_params["use_sim_time"] = use_sim_time
    if hardware_type not in ("gazebo", "real_robot"):
        moveit_params.pop("sensors_3d", None)
        moveit_params["octomap_resolution"] = 0.0
    else:
        moveit_params["octomap_frame"] = "base_link"
        moveit_params["octomap_resolution"] = octomap_resolution
        for _name in moveit_params.get("sensors", []) or []:
            entry = moveit_params.get(_name)
            if isinstance(entry, dict) and "point_cloud_topic" in entry:
                entry["point_cloud_topic"] = point_cloud_topic

    # MoveItConfigsBuilder does not auto-load move_group.yaml, so merge it here.
    with open(os.path.join(pkg_moveit, "config", "move_group.yaml")) as _f:
        for _k, _v in (yaml.safe_load(_f) or {}).items():
            if isinstance(_v, dict) and isinstance(moveit_params.get(_k), dict):
                moveit_params[_k].update(_v)
            else:
                moveit_params[_k] = _v

    move_group = Node(
        package="moveit_ros_move_group", executable="move_group",
        output="screen", parameters=[moveit_params])

    rviz = Node(
        package="rviz2", executable="rviz2",
        arguments=["-d", os.path.join(pkg_moveit, "config", "moveit_cumotion.rviz")],
        parameters=[moveit_params], output="log")

    # cumotion_planner_node: the GPU planner the cuMotion plugin forwards to.
    # Guarded so the launch still parses where isaac_ros_cumotion isn't installed.
    cumotion_actions = []
    try:
        cumotion_launch = os.path.join(
            get_package_share_directory("isaac_ros_cumotion"),
            "launch", "isaac_ros_cumotion.launch.py")
        cumotion_actions = [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cumotion_launch),
            launch_arguments={
                "cumotion_action_server.urdf_file_path": cumotion_urdf,
                "cumotion_action_server.xrdf_file_path": cumotion_xrdf,
                "cumotion_action_server.read_esdf_world": "false",
                "cumotion_action_server.add_ground_plane": "false",
            }.items())]
    except PackageNotFoundError:
        pass

    return [move_group, rviz, *cumotion_actions]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "hardware_type", default_value="mock_components",
            description="Hardware backend: mock_components | gazebo | isaac | real_robot"),
        DeclareLaunchArgument(
            "point_cloud_topic",
            default_value="/zed_head/zed_node/point_cloud/cloud_registered/filtered",
            description="PointCloud2 topic fed to the MoveIt octomap updater."),
        DeclareLaunchArgument(
            "octomap_resolution", default_value="0.008",
            description="Octomap voxel size in metres."),
        DeclareLaunchArgument(
            "cumotion_xrdf",
            default_value=os.path.join(
                get_package_share_directory("openarm_bimanual_moveit_config"),
                "config", "curobo", "openarm_bimanual.xrdf"),
            description="XRDF for the cuMotion planner (selects the group; default both_arms)."),
        DeclareLaunchArgument(
            "cumotion_urdf",
            default_value="/workspaces/isaac_ros-dev/openarm_cumotion/openarm_bimanual.urdf",
            description="Flat URDF for the cuMotion planner node (head_servo velocity > 0)."),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(
        function=_make_nodes,
        args=[LaunchConfiguration("hardware_type"),
              LaunchConfiguration("point_cloud_topic"),
              LaunchConfiguration("octomap_resolution"),
              LaunchConfiguration("cumotion_xrdf"),
              LaunchConfiguration("cumotion_urdf")])])
