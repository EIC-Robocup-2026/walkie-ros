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
#
# Standalone MoveIt (move_group + RViz). Assumes bringup is already running
# (robot_state_publisher, ros2_control_node, controllers).

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


# hardware_type -> xacro mappings (must match demo2.launch.py)
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


def _make_nodes(context: LaunchContext, hardware_type_lc):
    hardware_type = context.perform_substitution(hardware_type_lc)
    mappings = _HW_MAPPINGS.get(hardware_type, _HW_MAPPINGS["mock_components"])

    pkg_moveit = get_package_share_directory("openarm_bimanual_moveit_config")

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
    if hardware_type not in ("gazebo", "real_robot"):
        moveit_params.pop("sensors_3d", None)
        moveit_params["octomap_resolution"] = 0.0

    # MoveItConfigsBuilder does not auto-load move_group.yaml, so merge it here.
    # Provides start_state_max_bounds_error and trajectory_execution tolerances.
    with open(os.path.join(pkg_moveit, "config", "move_group.yaml")) as _f:
        _mg = yaml.safe_load(_f)
    for _k, _v in _mg.items():
        if isinstance(_v, dict) and isinstance(moveit_params.get(_k), dict):
            moveit_params[_k].update(_v)
        else:
            moveit_params[_k] = _v

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_params],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(pkg_moveit, "config", "moveit.rviz")],
        parameters=[moveit_params],
        output="log",
    )

    return [move_group, rviz]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "hardware_type",
            default_value="mock_components",
            description="Hardware backend: mock_components | gazebo | isaac | real_robot",
        ),
    ]

    hardware_type = LaunchConfiguration("hardware_type")

    nodes_func = OpaqueFunction(function=_make_nodes, args=[hardware_type])

    return LaunchDescription(declared_arguments + [nodes_func])
