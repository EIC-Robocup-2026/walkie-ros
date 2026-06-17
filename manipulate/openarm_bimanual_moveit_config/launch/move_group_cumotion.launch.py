# move_group with BOTH planning pipelines: ompl (default) + cumotion (GPU).
#
# This is the cuMotion-enabled variant of move_group.launch.py. The plain
# move_group.launch.py stays ompl-only so the normal host bringup is unchanged
# (the cuMotion MoveIt plugin + cuRobo only exist inside the Isaac ROS
# container, so loading them on the host would fail). Run THIS launch where
# cuMotion is installed (the Isaac ROS container).
#
# Pipeline ids come from the *_planning.yaml filenames in config/:
#   ompl_planning.yaml     -> "ompl"      (default, proven fallback)
#   cumotion_planning.yaml -> "cumotion"  (GPU)
#
# The commander already supports per-request pipeline selection. Switch live:
#   ros2 param set <commander_node> planning_pipeline cumotion   # GPU
#   ros2 param set <commander_node> planning_pipeline ""         # back to ompl default
#
# This launch also starts the cumotion_planner_node (the GPU process the plugin
# forwards to). Point it at the XRDF for the group you plan most; override with:
#   ros2 launch ... move_group_cumotion.launch.py \
#       cumotion_xrdf:=/path/to/both_arms.xrdf cumotion_urdf:=/path/to/robot.urdf
import os

from ament_index_python.packages import (
    PackageNotFoundError, get_package_share_directory)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "openarm", package_name="openarm_bimanual_moveit_config")
        # ompl stays default; cumotion is the additional GPU pipeline. Both are
        # loaded into the same move_group; the request's pipeline_id selects one.
        .planning_pipelines(
            pipelines=["ompl", "cumotion"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    ld = generate_move_group_launch(moveit_config)

    # cumotion_planner_node: the GPU planner the plugin forwards requests to.
    # Only added when isaac_ros_cumotion is installed (i.e. inside the container);
    # absent on the host, where this whole launch isn't meant to run anyway.
    default_xrdf = os.path.join(
        get_package_share_directory("openarm_bimanual_moveit_config"),
        "config", "curobo", "both_arms_lift.xrdf")
    default_urdf = os.path.join(
        get_package_share_directory("openarm_bimanual_moveit_config"),
        "config", "openarm_bimanual.urdf")

    ld.add_action(DeclareLaunchArgument("cumotion_xrdf", default_value=default_xrdf))
    ld.add_action(DeclareLaunchArgument("cumotion_urdf", default_value=default_urdf))

    try:
        cumotion_launch = os.path.join(
            get_package_share_directory("isaac_ros_cumotion"),
            "launch", "isaac_ros_cumotion.launch.py")
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cumotion_launch),
            launch_arguments={
                "cumotion_action_server.urdf_file_path":
                    LaunchConfiguration("cumotion_urdf"),
                "cumotion_action_server.xrdf_file_path":
                    LaunchConfiguration("cumotion_xrdf"),
                "cumotion_action_server.read_esdf_world": "false",
                "cumotion_action_server.add_ground_plane": "false",
            }.items(),
        ))
    except PackageNotFoundError:
        # isaac_ros_cumotion not installed here -> move_group still comes up with
        # the cumotion pipeline registered, but no planner backend to forward to.
        # (Expected on the host; run this launch in the Isaac ROS container.)
        pass

    return ld
