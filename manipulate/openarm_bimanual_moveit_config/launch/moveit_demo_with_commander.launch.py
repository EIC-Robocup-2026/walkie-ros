# Extends demo2.launch.py: same full MoveIt stack + bimanual commander action server.
# Usage:
#   ros2 launch openarm_bimanual_moveit_config demo2_with_commander.launch.py
#   ros2 launch openarm_bimanual_moveit_config demo2_with_commander.launch.py hardware_type:=isaac

import os
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

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


def _make_nodes(context: LaunchContext, hardware_type_lc, controllers_file_lc):
    hardware_type  = context.perform_substitution(hardware_type_lc)
    controllers_file = context.perform_substitution(controllers_file_lc)

    mappings = _HW_MAPPINGS.get(hardware_type, _HW_MAPPINGS["mock_components"])

    pkg_walkie = get_package_share_directory("walkie_description")
    pkg_moveit = get_package_share_directory("openarm_bimanual_moveit_config")
    pkg_ros_gz = get_package_share_directory("ros_gz_sim")

    xacro_path = os.path.join(pkg_walkie, "robots", "gz_walkie.urdf.xacro")
    robot_description_xml = xacro.process_file(
        xacro_path, mappings=mappings
    ).toprettyxml(indent="  ")

    use_sim_time = hardware_type in ("gazebo", "isaac")

    moveit_config = (
        MoveItConfigsBuilder("openarm", package_name="openarm_bimanual_moveit_config")
        .robot_description(
            file_path="config/openarm_bimanual.urdf.xacro",
            mappings=mappings,
        )
        .trajectory_execution(moveit_manage_controllers=False)
        .to_moveit_configs()
    )
    moveit_params = moveit_config.to_dict()
    moveit_params["use_sim_time"] = use_sim_time

    # MoveItConfigsBuilder does not auto-load move_group.yaml, so merge it here.
    # Provides start_state_max_bounds_error and trajectory_execution tolerances.
    with open(os.path.join(pkg_moveit, "config", "move_group.yaml")) as _f:
        _mg = yaml.safe_load(_f)
    for _k, _v in _mg.items():
        if isinstance(_v, dict) and isinstance(moveit_params.get(_k), dict):
            moveit_params[_k].update(_v)
        else:
            moveit_params[_k] = _v

    # Commander node params (subset — no full moveit_params to avoid overhead)
    commander_params = [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
        {"use_sim_time": use_sim_time},
    ]

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_xml},
            {"use_sim_time": use_sim_time},
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(pkg_moveit, "config", "moveit.rviz")],
        parameters=[moveit_params],
        output="log",
    )

    commander = Node(
        package="openarm_bimanual_commander_cpp",
        executable="commander",
        name="bimanual_commander",
        output="screen",
        parameters=commander_params,
    )

    # ── Gazebo path ───────────────────────────────────────────────────────
    if hardware_type == "gazebo":
        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz, "launch", "gz_sim.launch.py")
            ),
            launch_arguments={
                "gz_args": "-r --physics-engine gz-physics-bullet-featherstone-plugin"
            }.items(),
        )
        bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            output="screen",
        )
        spawn = Node(
            package="ros_gz_sim",
            executable="create",
            arguments=["-topic", "robot_description",
                       "-name", "walkie", "-x", "0.0", "-y", "0.0", "-z", "0.05"],
            output="screen",
        )
        move_group = Node(
            package="moveit_ros_move_group", executable="move_group",
            output="screen",
            parameters=[moveit_params],
        )
        gz_spawners = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn,
                on_exit=[
                    Node(package="controller_manager", executable="spawner",
                         arguments=["joint_state_broadcaster"]),
                    Node(package="controller_manager", executable="spawner",
                         arguments=["left_joint_trajectory_controller",
                                    "right_joint_trajectory_controller"]),
                    Node(package="controller_manager", executable="spawner",
                         arguments=["left_gripper_controller",
                                    "right_gripper_controller"]),
                    Node(package="controller_manager", executable="spawner",
                         arguments=["lift_controller"]),
                ],
            )
        )
        return [rsp, gazebo, bridge, spawn, gz_spawners, move_group,
                TimerAction(period=15.0, actions=[commander]), rviz]

    # ── mock_components: fake object TFs + pick_and_place node ───────────
    mock_nodes = []
    if hardware_type == "mock_components":
        # Static object poses (relative to base_footprint) for testing without Isaac.
        # Bottle on a table 0.6 m ahead; tables at left/right of the robot.
        mock_nodes = [
            Node(
                package="tf2_ros", executable="static_transform_publisher",
                name="fake_bottle_tf",
                arguments=["0.60", "0.00", "0.92",   # x y z
                           "0", "0", "0", "1",        # qx qy qz qw
                           "base_footprint", "SM_BottleA"],
            ),
            Node(
                package="tf2_ros", executable="static_transform_publisher",
                name="fake_cube_tf",
                arguments=["0.60", "0.00", "0.35",
                           "0", "0", "0", "1",
                           "base_footprint", "Cube"],
            ),
            Node(
                package="tf2_ros", executable="static_transform_publisher",
                name="fake_cube02_tf",
                arguments=["0.55", "0.55", "0.30",
                           "0", "0", "0", "1",
                           "base_footprint", "Cube_02"],
            ),
            Node(
                package="tf2_ros", executable="static_transform_publisher",
                name="fake_cube03_tf",
                arguments=["0.55", "-0.55", "0.30",
                           "0", "0", "0", "1",
                           "base_footprint", "Cube_03"],
            ),
        ]

    # ── non-Gazebo: mock_components / isaac / real_robot ──────────────────
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[controllers_file],
    )
    jsb = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "30"],
    )
    arm_ctrl = Node(
        package="controller_manager", executable="spawner",
        arguments=["left_joint_trajectory_controller",
                   "right_joint_trajectory_controller",
                   "-c", "/controller_manager",
                   "--controller-manager-timeout", "30"],
        output="screen",
    )
    gripper_ctrl = Node(
        package="controller_manager", executable="spawner",
        arguments=["left_gripper_controller", "right_gripper_controller",
                   "-c", "/controller_manager",
                   "--controller-manager-timeout", "30"],
        output="screen",
    )
    lift_ctrl = Node(
        package="controller_manager", executable="spawner",
        arguments=["lift_controller",
                   "-c", "/controller_manager",
                   "--controller-manager-timeout", "30"],
        output="screen",
    )
    move_group = Node(
        package="moveit_ros_move_group", executable="move_group",
        output="screen",
        parameters=[moveit_params],
    )

    pick_and_place = Node(
        package="openarm_bimanual_commander_cpp",
        executable="pick_and_place_isaac2",
        name="pick_and_place_isaac2",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": use_sim_time},
        ],
    )

    # Commander + pick_and_place start after move_group is up
    return [
        rsp,
        *mock_nodes,
        TimerAction(period=2.0,  actions=[control_node]),
        TimerAction(period=4.0,  actions=[jsb]),
        TimerAction(period=5.0,  actions=[arm_ctrl]),
        TimerAction(period=5.0,  actions=[gripper_ctrl]),
        TimerAction(period=5.0,  actions=[lift_ctrl]),
        TimerAction(period=7.0,  actions=[move_group]),
        TimerAction(period=10.0, actions=[commander]),
        TimerAction(period=12.0, actions=[pick_and_place]),
        rviz,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "hardware_type",
            default_value="mock_components",
            description="Hardware backend: mock_components | gazebo | isaac | real_robot",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value=os.path.join(
                get_package_share_directory("openarm_bimanual_moveit_config"),
                "config",
                "ros2_controllers.yaml",
            ),
        ),
    ]

    hardware_type    = LaunchConfiguration("hardware_type")
    controllers_file = LaunchConfiguration("controllers_file")

    nodes_func = OpaqueFunction(
        function=_make_nodes,
        args=[hardware_type, controllers_file],
    )

    return LaunchDescription(declared_arguments + [nodes_func])
