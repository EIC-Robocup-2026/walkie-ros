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
from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    OpaqueFunction,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

# Xacro mappings passed to gz_walkie for MoveIt fake-hardware demo
_WALKIE_MOVEIT_MAPPINGS = {
    "ros2_control": "mock",
    "openarm_ros2_control": "true",
    "openarm_use_fake_hardware": "true",
    "add_world_link": "true",
}

# MoveItConfigsBuilder uses openarm_bimanual.urdf.xacro (which now wraps gz_walkie)
# evaluated once at launch-file load time
_MOVEIT_CONFIG = (
    MoveItConfigsBuilder("openarm", package_name="openarm_bimanual_moveit_config")
    .robot_description(
        file_path="config/openarm_bimanual.urdf.xacro",
        mappings=_WALKIE_MOVEIT_MAPPINGS,
    )
    .trajectory_execution(moveit_manage_controllers=False)
    .to_moveit_configs()
)


def generate_robot_description(
    context: LaunchContext,
    description_package,
    description_file,
    use_fake_hardware,
    right_can_interface,
    left_can_interface,
):
    """Render gz_walkie xacro and return XML string."""
    description_package_str = context.perform_substitution(description_package)
    description_file_str = context.perform_substitution(description_file)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)

    xacro_path = os.path.join(
        get_package_share_directory(description_package_str),
        "robots",
        description_file_str,
    )

    mappings = dict(_WALKIE_MOVEIT_MAPPINGS)
    mappings["openarm_use_fake_hardware"] = use_fake_hardware_str

    robot_description = xacro.process_file(
        xacro_path,
        mappings=mappings,
    ).toprettyxml(indent="  ")

    return robot_description


def rsp_spawner(
    context: LaunchContext,
    description_package,
    description_file,
    use_fake_hardware,
    right_can_interface,
    left_can_interface,
):
    """Start robot_state_publisher. In Jazzy, ros2_control_node gets robot_description
    from the /robot_description topic (transient_local), so RSP must start first."""
    robot_description = generate_robot_description(
        context,
        description_package,
        description_file,
        use_fake_hardware,
        right_can_interface,
        left_can_interface,
    )
    robot_description_param = {"robot_description": robot_description}
    return [Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description_param],
    )]




def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "description_package",
            default_value="walkie_description",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="gz_walkie.urdf.xacro",
        ),
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),
        DeclareLaunchArgument(
            "runtime_config_package", default_value="openarm_bimanual_moveit_config"
        ),
        DeclareLaunchArgument("right_can_interface", default_value="can0"),
        DeclareLaunchArgument("left_can_interface", default_value="can1"),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ros2_controllers.yaml",
        ),
    ]

    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    right_can_interface = LaunchConfiguration("right_can_interface")
    left_can_interface = LaunchConfiguration("left_can_interface")

    controllers_file = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )

    # robot_state_publisher starts immediately — publishes /robot_description (transient_local)
    rsp_func = OpaqueFunction(
        function=rsp_spawner,
        args=[description_package, description_file, use_fake_hardware,
              right_can_interface, left_can_interface],
    )

    # ros2_control_node starts after 2s — by then RSP has published /robot_description
    # In Jazzy, ros2_control_node gets robot_description from the /robot_description topic,
    # not from the parameter. It uses controllers_file for controller configuration.
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[controllers_file],
    )
    delayed_control_node = TimerAction(period=2.0, actions=[control_node])

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "30"],
    )

    arm_ctrl_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_joint_trajectory_controller",
            "right_joint_trajectory_controller",
            "-c", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
        output="screen",
    )

    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_gripper_controller",
                   "right_gripper_controller",
                   "-c", "/controller_manager",
                   "--controller-manager-timeout", "30"],
        output="screen",
    )

    # Spawners run after ros2_control_node has started (t=2s) and hardware is initialized
    delayed_jsb = TimerAction(period=4.0, actions=[jsb_spawner])
    delayed_arm_ctrl = TimerAction(period=5.0, actions=[arm_ctrl_spawner])
    delayed_gripper = TimerAction(period=5.0, actions=[gripper_spawner])

    moveit_params = _MOVEIT_CONFIG.to_dict()

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_params],
    )

    delayed_move_group = TimerAction(period=7.0, actions=[run_move_group_node])

    rviz_cfg = os.path.join(
        get_package_share_directory(
            "openarm_bimanual_moveit_config"), "config", "moveit.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_cfg],
        parameters=[moveit_params],
    )

    return LaunchDescription(
        declared_arguments
        + [
            rsp_func,
            delayed_control_node,
            delayed_jsb,
            delayed_arm_ctrl,
            delayed_gripper,
            delayed_move_group,
            rviz_node,
        ]
    )
