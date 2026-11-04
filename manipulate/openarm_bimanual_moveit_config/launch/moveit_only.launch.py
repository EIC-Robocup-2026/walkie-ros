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
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


# hardware_type -> xacro mappings (must match moveit_demo.launch.py)
_HW_MAPPINGS = {
    "mock_components": {
        "ros2_control": "mock",
        "add_world_link": "true",
    },
    "gazebo": {
        "ros2_control": "gazebo",
        "add_world_link": "false",
    },
    "isaac": {
        "ros2_control": "isaac",
        "add_world_link": "true",
    },
    "real_robot": {
        "ros2_control": "real_robot",
        "add_world_link": "false",
    },
    # MuJoCo owns ros2_control through mujoco_ros2_control, as Gazebo does
    # through gz_ros2_control.
    "mujoco": {
        "ros2_control": "mujoco",
        "add_world_link": "false",
    },
}


# The cloud the octomap updater reads under a planar base_mode, after the floor gate.
_GATED_CLOUD_TOPIC = "/move_group/gated_cloud"


def _planar_scene_nodes(hardware_type, point_cloud_topic, use_sim_time):
    """Nodes that feed and guard the map-frame octomap under a planar base_mode."""
    nodes = []
    if hardware_type == "mujoco":
        # MuJoCo publishes depth images and no cloud, so build the cloud here.
        point_cloud_topic = "/zed_head/whole_body/cloud"
        nodes.append(
            Node(
                package="depth_image_proc",
                executable="point_cloud_xyz_node",
                name="head_depth_to_cloud",
                output="log",
                parameters=[{"use_sim_time": use_sim_time}],
                remappings=[
                    ("image_rect", "/zed_head/zed_node/depth/depth_registered"),
                    ("camera_info", "/zed_head/zed_node/depth/camera_info"),
                    ("points", point_cloud_topic),
                ],
            )
        )
    nodes.append(
        Node(
            package="openarm_bimanual_moveit_config",
            executable="cloud_floor_gate.py",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            remappings=[("cloud_in", point_cloud_topic), ("cloud_out", _GATED_CLOUD_TOPIC)],
        )
    )
    nodes.append(
        Node(
            package="openarm_bimanual_moveit_config",
            executable="octomap_jump_guard.py",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        )
    )
    return nodes


def _make_nodes(context: LaunchContext, hardware_type_lc,
                point_cloud_topic_lc, octomap_resolution_lc,
                left_use_fake_arm_hardware_lc, right_use_fake_arm_hardware_lc,
                right_joint2_fixed_lc, use_rviz_lc, base_mode_lc,
                planar_octomap_resolution_lc, planar_octomap_max_range_lc):
    base_mode = context.perform_substitution(base_mode_lc)
    if base_mode not in ("fixed", "map", "whole_body"):
        raise ValueError(f"base_mode must be fixed, map or whole_body, not '{base_mode}'")
    planar = base_mode != "fixed"
    hardware_type = context.perform_substitution(hardware_type_lc)
    point_cloud_topic = context.perform_substitution(point_cloud_topic_lc)
    octomap_resolution = float(context.perform_substitution(octomap_resolution_lc))
    left_use_fake_arm_hardware = context.perform_substitution(left_use_fake_arm_hardware_lc)
    right_use_fake_arm_hardware = context.perform_substitution(right_use_fake_arm_hardware_lc)
    right_joint2_fixed = context.perform_substitution(right_joint2_fixed_lc)
    use_rviz = context.perform_substitution(use_rviz_lc)
    mappings = dict(_HW_MAPPINGS.get(hardware_type, _HW_MAPPINGS["mock_components"]))
    # Arm hardware (mock vs real OpenArm CAN) is selectable per side, independent
    # of hardware_type, which only controls the base/lift/wheels backend.
    mappings["left_use_fake_arm_hardware"] = left_use_fake_arm_hardware
    mappings["right_use_fake_arm_hardware"] = right_use_fake_arm_hardware
    mappings["right_joint2_fixed"] = right_joint2_fixed

    pkg_moveit = get_package_share_directory("openarm_bimanual_moveit_config")

    use_sim_time = hardware_type in ("gazebo", "isaac", "mujoco")

    builder = (
        MoveItConfigsBuilder("openarm", package_name="openarm_bimanual_moveit_config")
        .robot_description(
            file_path="config/openarm_bimanual.urdf.xacro",
            mappings=mappings,
        )
        # SRDF group_state defaults reference openarm_right_joint2; that line is
        # gated by the same flag in the SRDF so it's skipped once the joint is fixed.
        .robot_description_semantic(
            file_path="config/openarm_bimanual.srdf",
            mappings={"right_joint2_fixed": right_joint2_fixed, "base_mode": base_mode},
        )
        .trajectory_execution(moveit_manage_controllers=False)
    )
    if planar:
        # The commander's "PTP" fallback calls plain plan() and never uses Pilz, and
        # the other pipelines have no configuration for the planar groups.
        builder = builder.planning_pipelines(
            pipelines=["ompl"], default_planning_pipeline="ompl"
        )
    moveit_config = builder.to_moveit_configs()
    if base_mode == "whole_body":
        # Comes before time parameterization, so that a base turning across +-pi does not
        # take the long way round. Only this mode has a group with the planar joint. The
        # plugin lives in openarm_bimanual_commander_cpp, next to the base follower.
        moveit_config.planning_pipelines["ompl"]["response_adapters"].insert(
            0, "openarm_bimanual_commander_cpp/UnwrapPlanarHeading"
        )
    moveit_params = moveit_config.to_dict()
    moveit_params["use_sim_time"] = use_sim_time
    # Octomap needs the ZED point cloud, which only Gazebo and the real robot
    # publish (no point cloud source is wired for mock/isaac/mujoco). Under a planar
    # base_mode MuJoCo joins them through depth_image_proc.
    if planar and hardware_type in ("gazebo", "real_robot", "mujoco"):
        # No octomap_frame: the planning frame, map, is the octree frame. The octree
        # keeps the whole room in this frame, so it is coarser, and scene updates,
        # which carry all of it, go out less often.
        moveit_params["octomap_resolution"] = float(
            context.perform_substitution(planar_octomap_resolution_lc)
        )
        moveit_params["publish_planning_scene_hz"] = 1.0
        for _name in moveit_params.get("sensors", []) or []:
            entry = moveit_params.get(_name)
            if isinstance(entry, dict) and "point_cloud_topic" in entry:
                entry["point_cloud_topic"] = _GATED_CLOUD_TOPIC
                # The camera sees the visual meshes, and the mask is built from the
                # collision geometry. A return outside the mask becomes a voxel on the
                # robot that the robot's own surface then shields from clearing.
                entry["padding_offset"] = 0.10
                entry["max_range"] = float(
                    context.perform_substitution(planar_octomap_max_range_lc)
                )
    elif hardware_type not in ("gazebo", "real_robot"):
        moveit_params.pop("sensors_3d", None)
        moveit_params["octomap_resolution"] = 0.0
    else:
        # Octomap is anchored to base_link so voxels move with the mobile base.
        moveit_params["octomap_frame"] = "base_link"
        moveit_params["octomap_resolution"] = octomap_resolution
        # Override the cloud topic at runtime; sensors_3d.yaml uses the named-sensor
        # layout: moveit_params["<sensor_name>"]["point_cloud_topic"].
        for _name in moveit_params.get("sensors", []) or []:
            entry = moveit_params.get(_name)
            if isinstance(entry, dict) and "point_cloud_topic" in entry:
                entry["point_cloud_topic"] = point_cloud_topic

    # MoveItConfigsBuilder does not auto-load move_group.yaml, so merge it here.
    # Provides start_state_max_bounds_error and trajectory_execution tolerances.
    with open(os.path.join(pkg_moveit, "config", "move_group.yaml")) as _f:
        _mg = yaml.safe_load(_f)
    for _k, _v in _mg.items():
        if isinstance(_v, dict) and isinstance(moveit_params.get(_k), dict):
            moveit_params[_k].update(_v)
        else:
            moveit_params[_k] = _v

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

    rviz_config = "moveit_map.rviz" if planar else "moveit.rviz"
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(pkg_moveit, "config", rviz_config)],
        parameters=[moveit_params],
        output="log",
        condition=IfCondition(use_rviz),
    )

    if not planar:
        return [move_group, rviz]

    # The planar model takes its base pose from map -> base_footprint. Started before
    # that exists, move_group runs against a fake origin, so it waits for the transform.
    wait_for_map = Node(
        package="openarm_bimanual_moveit_config",
        executable="wait_for_graph.py",
        name="wait_for_map",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )
    started_after_map = [move_group, rviz]
    if hardware_type in ("gazebo", "real_robot", "mujoco"):
        started_after_map += _planar_scene_nodes(hardware_type, point_cloud_topic, use_sim_time)
    return [
        wait_for_map,
        RegisterEventHandler(OnProcessExit(target_action=wait_for_map, on_exit=started_after_map)),
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "hardware_type",
            default_value="mock_components",
            description="Hardware backend: mock_components | gazebo | isaac | "
                        "real_robot | mujoco",
        ),
        DeclareLaunchArgument(
            "point_cloud_topic",
            default_value="/zed_head/zed_node/point_cloud/cloud_registered/filtered",
            description="PointCloud2 topic fed to the MoveIt octomap updater "
                        "(self-filtered ZED cloud; same name in sim via gz_bridge "
                        "and on real ZED).",
        ),
        DeclareLaunchArgument(
            "octomap_resolution",
            default_value="0.008",
            description="Octomap voxel size in metres.",
        ),
        DeclareLaunchArgument(
            "left_use_fake_arm_hardware",
            default_value="true",
            description="Mock (true) or real OpenArm CAN hardware (false) for the "
                        "left arm. Only takes effect when hardware_type=real_robot.",
        ),
        DeclareLaunchArgument(
            "right_use_fake_arm_hardware",
            default_value="true",
            description="Mock (true) or real OpenArm CAN hardware (false) for the "
                        "right arm. Only takes effect when hardware_type=real_robot.",
        ),
        DeclareLaunchArgument(
            "right_joint2_fixed",
            default_value="false",
            description="Lock the right arm's joint2 as a fixed URDF joint at its "
                        "nominal (zero) position, dropping it from MoveIt's IK/"
                        "planning DOF for the right arm groups.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz2 alongside move_group.",
        ),
        DeclareLaunchArgument(
            "base_mode",
            default_value="fixed",
            description="Base model: fixed (today's arm-only model, world to "
                        "base_footprint), map (planar base, planning frame map) or "
                        "whole_body (map plus the whole_body_* commander option). "
                        "The commander needs the same value.",
        ),
        DeclareLaunchArgument(
            "planar_octomap_resolution",
            default_value="0.03",
            description="Octomap voxel size in metres under base_mode map and "
                        "whole_body. fixed uses octomap_resolution.",
        ),
        DeclareLaunchArgument(
            "planar_octomap_max_range",
            default_value="1.5",
            description="Octomap updater range in metres under base_mode map and "
                        "whole_body. Anything farther is unknown, which plans as free.",
        ),
    ]

    hardware_type = LaunchConfiguration("hardware_type")
    point_cloud_topic = LaunchConfiguration("point_cloud_topic")
    octomap_resolution = LaunchConfiguration("octomap_resolution")
    left_use_fake_arm_hardware = LaunchConfiguration("left_use_fake_arm_hardware")
    right_use_fake_arm_hardware = LaunchConfiguration("right_use_fake_arm_hardware")
    right_joint2_fixed = LaunchConfiguration("right_joint2_fixed")
    use_rviz = LaunchConfiguration("use_rviz")
    base_mode = LaunchConfiguration("base_mode")
    planar_octomap_resolution = LaunchConfiguration("planar_octomap_resolution")
    planar_octomap_max_range = LaunchConfiguration("planar_octomap_max_range")

    nodes_func = OpaqueFunction(
        function=_make_nodes,
        args=[hardware_type, point_cloud_topic, octomap_resolution,
              left_use_fake_arm_hardware, right_use_fake_arm_hardware,
              right_joint2_fixed, use_rviz, base_mode,
              planar_octomap_resolution, planar_octomap_max_range],
    )

    return LaunchDescription(declared_arguments + [nodes_func])
