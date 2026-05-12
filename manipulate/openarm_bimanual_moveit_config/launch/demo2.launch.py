# Copyright 2025 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
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


# hardware_type -> xacro mappings for gz_walkie.urdf.xacro
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
    hardware_type = context.perform_substitution(hardware_type_lc)
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
    # Disable octomap when no real 3D sensors are configured (mock/isaac)
    if hardware_type not in ("gazebo", "real_robot"):
        moveit_params.pop("sensors_3d", None)
        moveit_params["octomap_resolution"] = 0.0

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_xml},
            {"use_sim_time": use_sim_time},
        ],
    )

    static_world_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--frame-id", "world", "--child-frame-id", "base_footprint"],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(pkg_moveit, "config", "moveit.rviz")],
        parameters=[moveit_params],
        output="log",
    )

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
                ],
            )
        )
        move_group = Node(
            package="moveit_ros_move_group", executable="move_group",
            output="screen", parameters=[moveit_params],
        )
        return [rsp, gazebo, bridge, spawn, gz_spawners, move_group, rviz]

    # non-Gazebo: mock_components / isaac / real_robot
    # Jazzy ros2_control_node reads robot_description from /robot_description topic.
    # RSP is returned first so it starts and publishes before control_node subscribes.
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
        output="screen", parameters=[moveit_params],
    )
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

    hardware_type = LaunchConfiguration("hardware_type")
    controllers_file = LaunchConfiguration("controllers_file")

    nodes_func = OpaqueFunction(
        function=_make_nodes,
        args=[hardware_type, controllers_file],
    )

    return LaunchDescription(declared_arguments + [nodes_func])
