# cuMotion + mock_components test stack.
#
# Same full MoveIt stack as moveit_demo_with_commander.launch.py with
# hardware_type:=mock_components (mock ros2_control hardware + controllers +
# move_group + the bimanual commander), PLUS:
#   - cuMotion registered as a SECOND planning pipeline (ompl stays default)
#   - the cumotion_planner_node (GPU) the plugin forwards to
# so you can flip the commander between OMPL (CPU) and cuMotion (GPU) live and
# actually EXECUTE the plan on mock hardware (no real robot / CAN needed).
#
# WHERE TO RUN: cuMotion (cuRobo + the cuMotion MoveIt plugin) only exists in
# the Isaac ROS container, so this launch must run THERE, with the walkie
# packages (walkie_description, this package, the commander, my_robot_interfaces)
# built/sourced in the container too. On the host (no cuMotion) use the plain
# moveit_demo_with_commander.launch.py instead.
#
# USAGE (inside the container, after building + sourcing the walkie overlay AND
#        the patched isaac_ros_cumotion_moveit overlay):
#   ros2 launch openarm_bimanual_moveit_config \
#       moveit_demo_with_commander_cumotion.launch.py \
#       cumotion_xrdf:=<abs path to the group's xrdf> \
#       cumotion_urdf:=<abs path to a flat URDF (head_servo velocity patched)>
#
#   # then select the GPU pipeline on the commander (empty string = back to ompl):
#   ros2 param set /bimanual_commander planning_pipeline cumotion
#
#   # drive it with the normal action servers, e.g. set_joint_position / go_to_pose
#   # -> plans on the GPU, executes on the mock controllers.
#
# ONE-GROUP CONSTRAINT: the cumotion_planner_node loads ONE XRDF = ONE planning
# group's cspace. cuMotion requests for OTHER groups fail the cspace-dim check,
# so cuMotion-plan only the group matching cumotion_xrdf (default: both_arms);
# leave other groups on OMPL (planning_pipeline "").
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

# mock_components mappings for openarm_bimanual.urdf.xacro / gz_walkie.urdf.xacro.
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
        .robot_description(
            file_path="config/openarm_bimanual.urdf.xacro", mappings=_MOCK)
        # ompl stays default; cumotion is the additional GPU pipeline. Pipeline
        # ids come from the *_planning.yaml filenames: ompl_planning.yaml -> "ompl",
        # cumotion_planning.yaml -> "cumotion".
        .planning_pipelines(
            pipelines=["ompl", "cumotion"], default_planning_pipeline="ompl")
        .trajectory_execution(moveit_manage_controllers=False)
        .to_moveit_configs()
    )
    moveit_params = moveit_config.to_dict()
    moveit_params["use_sim_time"] = False
    # No 3D sensors on mock hardware -> no octomap.
    moveit_params.pop("sensors_3d", None)
    moveit_params["octomap_resolution"] = 0.0

    # MoveItConfigsBuilder does not auto-load move_group.yaml; merge it for
    # start_state_max_bounds_error + trajectory_execution tolerances.
    with open(os.path.join(pkg_moveit, "config", "move_group.yaml")) as f:
        for k, v in (yaml.safe_load(f) or {}).items():
            if isinstance(v, dict) and isinstance(moveit_params.get(k), dict):
                moveit_params[k].update(v)
            else:
                moveit_params[k] = v

    commander_params = [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
        {"use_sim_time": False},
    ]

    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_xml},
                    {"use_sim_time": False}])
    # world->base_footprint so MoveIt's fixed virtual joint resolves.
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
        arguments=["left_joint_trajectory_controller",
                   "right_joint_trajectory_controller",
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

    commander = Node(
        package="openarm_bimanual_commander_cpp", executable="commander",
        name="bimanual_commander", output="screen", parameters=commander_params)

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
        TimerAction(period=2.0,  actions=[control_node]),
        TimerAction(period=4.0,  actions=[jsb]),
        TimerAction(period=5.0,  actions=[arm_ctrl]),
        TimerAction(period=5.0,  actions=[gripper_ctrl]),
        TimerAction(period=5.0,  actions=[lift_ctrl]),
        TimerAction(period=7.0,  actions=[move_group]),
        *cumotion_actions,
        TimerAction(period=10.0, actions=[commander]),
        rviz,
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
            description="XRDF for the cuMotion planner (selects the planning "
                        "group's cspace; default = both_arms 14-DOF)."),
        DeclareLaunchArgument(
            "cumotion_urdf",
            default_value="/workspaces/isaac_ros-dev/openarm_cumotion/"
                          "openarm_bimanual.urdf",
            description="Flat URDF for the cuMotion planner node (head_servo "
                        "velocity limit must be positive)."),
    ]
    return LaunchDescription(args + [OpaqueFunction(
        function=_make_nodes,
        args=[LaunchConfiguration("controllers_file"),
              LaunchConfiguration("cumotion_xrdf"),
              LaunchConfiguration("cumotion_urdf")])])
