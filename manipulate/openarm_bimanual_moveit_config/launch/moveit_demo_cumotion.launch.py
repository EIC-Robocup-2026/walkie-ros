# cuMotion analog of moveit_demo.launch.py: SELF-CONTAINED mock_components stack
# (RSP + mock ros2_control + controllers + move_group + RViz) with cuMotion added
# as a SECOND planning pipeline (ompl stays default) + the cumotion_planner_node.
# No commander (that's moveit_demo_with_commander_cumotion.launch.py).
#
# The robot RENDERS in RViz with no other bringup. Use this for RViz interactive
# cuMotion planning. Run inside the Isaac container (walkie packages + patched
# isaac_ros_cumotion_moveit overlay built/sourced).
#
#   ros2 launch openarm_bimanual_moveit_config moveit_demo_cumotion.launch.py
#   # RViz MotionPlanning -> Context -> Planning Pipeline = "cumotion" (or "ompl"),
#   # Planning Group = both_arms (match cumotion_xrdf), drag marker, Plan.
import os

import xacro
import yaml
from ament_index_python.packages import (
    PackageNotFoundError, get_package_share_directory)
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

_MOCK = {
    "ros2_control": "mock",
    "openarm_ros2_control": "true",
    "openarm_use_fake_hardware": "true",
    "add_world_link": "true",
}


def _make_nodes(context: LaunchContext, controllers_file_lc,
                cumotion_xrdf_lc, cumotion_urdf_lc):
    controllers_file = context.perform_substitution(controllers_file_lc)
    cumotion_xrdf = context.perform_substitution(cumotion_xrdf_lc)
    cumotion_urdf = context.perform_substitution(cumotion_urdf_lc)

    pkg_walkie = get_package_share_directory("walkie_description")
    pkg_moveit = get_package_share_directory("openarm_bimanual_moveit_config")

    xacro_path = os.path.join(pkg_walkie, "robots", "gz_walkie.urdf.xacro")
    robot_description_xml = xacro.process_file(
        xacro_path, mappings=_MOCK).toprettyxml(indent="  ")

    moveit_config = (
        MoveItConfigsBuilder("openarm", package_name="openarm_bimanual_moveit_config")
        .robot_description(file_path="config/openarm_bimanual.urdf.xacro", mappings=_MOCK)
        .planning_pipelines(pipelines=["ompl", "cumotion"], default_planning_pipeline="ompl")
        .trajectory_execution(moveit_manage_controllers=False)
        .to_moveit_configs()
    )
    moveit_params = moveit_config.to_dict()
    moveit_params["use_sim_time"] = False
    moveit_params.pop("sensors_3d", None)
    moveit_params["octomap_resolution"] = 0.0

    with open(os.path.join(pkg_moveit, "config", "move_group.yaml")) as f:
        for k, v in (yaml.safe_load(f) or {}).items():
            if isinstance(v, dict) and isinstance(moveit_params.get(k), dict):
                moveit_params[k].update(v)
            else:
                moveit_params[k] = v

    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_xml}, {"use_sim_time": False}])
    static_world_tf = Node(
        package="tf2_ros", executable="static_transform_publisher",
        arguments=["--frame-id", "world", "--child-frame-id", "base_footprint"],
        output="screen")
    control_node = Node(
        package="controller_manager", executable="ros2_control_node",
        output="screen", parameters=[controllers_file])
    jsb = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager",
                   "--controller-manager-timeout", "30"])
    arm_ctrl = Node(
        package="controller_manager", executable="spawner",
        arguments=["left_joint_trajectory_controller", "right_joint_trajectory_controller",
                   "-c", "/controller_manager", "--controller-manager-timeout", "30"],
        output="screen")
    gripper_ctrl = Node(
        package="controller_manager", executable="spawner",
        arguments=["left_gripper_controller", "right_gripper_controller",
                   "-c", "/controller_manager", "--controller-manager-timeout", "30"],
        output="screen")
    lift_ctrl = Node(
        package="controller_manager", executable="spawner",
        arguments=["lift_controller", "-c", "/controller_manager",
                   "--controller-manager-timeout", "30"],
        output="screen")
    move_group = Node(
        package="moveit_ros_move_group", executable="move_group",
        output="screen", parameters=[moveit_params])
    rviz = Node(
        package="rviz2", executable="rviz2",
        arguments=["-d", os.path.join(pkg_moveit, "config", "moveit_cumotion.rviz")],
        parameters=[moveit_params], output="log")

    cumotion_actions = []
    try:
        cumotion_launch = os.path.join(
            get_package_share_directory("isaac_ros_cumotion"),
            "launch", "isaac_ros_cumotion.launch.py")
        cumotion_actions = [TimerAction(period=7.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cumotion_launch),
            launch_arguments={
                "cumotion_action_server.urdf_file_path": cumotion_urdf,
                "cumotion_action_server.xrdf_file_path": cumotion_xrdf,
                "cumotion_action_server.read_esdf_world": "false",
                "cumotion_action_server.add_ground_plane": "false",
            }.items())])]
    except PackageNotFoundError:
        pass

    return [
        rsp,
        static_world_tf,
        TimerAction(period=2.0, actions=[control_node]),
        TimerAction(period=4.0, actions=[jsb]),
        TimerAction(period=5.0, actions=[arm_ctrl]),
        TimerAction(period=5.0, actions=[gripper_ctrl]),
        TimerAction(period=5.0, actions=[lift_ctrl]),
        TimerAction(period=7.0, actions=[move_group]),
        rviz,
        *cumotion_actions,
    ]


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "controllers_file",
            default_value=os.path.join(
                get_package_share_directory("openarm_bimanual_moveit_config"),
                "config", "ros2_controllers.yaml")),
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
    return LaunchDescription(args + [OpaqueFunction(
        function=_make_nodes,
        args=[LaunchConfiguration("controllers_file"),
              LaunchConfiguration("cumotion_xrdf"),
              LaunchConfiguration("cumotion_urdf")])])
