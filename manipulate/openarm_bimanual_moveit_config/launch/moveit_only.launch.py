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
# Standalone MoveIt (move_group + RViz, and hybrid planning with hybrid_planning:=true).
# Assumes bringup is already running (robot_state_publisher, ros2_control_node,
# controllers; for map_frame:=map also the map -> base_footprint TF, e.g. Nav2).

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
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


def _without_sensors(params):
    """Copy of MoveIt parameters without the 3D sensors (no octomap updaters)."""
    params = dict(params)
    for name in params.pop("sensors", None) or []:
        params.pop(name, None)
    return params


def _make_nodes(context: LaunchContext, hardware_type_lc,
                point_cloud_topic_lc, octomap_resolution_lc,
                left_use_fake_arm_hardware_lc, right_use_fake_arm_hardware_lc,
                right_joint2_fixed_lc, use_rviz_lc, map_frame_lc, hybrid_planning_lc):
    hardware_type = context.perform_substitution(hardware_type_lc)
    point_cloud_topic = context.perform_substitution(point_cloud_topic_lc)
    octomap_resolution = context.perform_substitution(octomap_resolution_lc)
    left_use_fake_arm_hardware = context.perform_substitution(left_use_fake_arm_hardware_lc)
    right_use_fake_arm_hardware = context.perform_substitution(right_use_fake_arm_hardware_lc)
    right_joint2_fixed = context.perform_substitution(right_joint2_fixed_lc)
    use_rviz = context.perform_substitution(use_rviz_lc)
    # "auto": map frame and hybrid planning in MuJoCo (Nav2 provides map -> base_footprint),
    # the robot frame and move_group only elsewhere.
    map_frame = context.perform_substitution(map_frame_lc)
    if map_frame == "auto":
        map_frame = "map" if hardware_type == "mujoco" else ""
    elif map_frame == "none":
        map_frame = ""
    hybrid_planning = context.perform_substitution(hybrid_planning_lc).lower()
    if hybrid_planning == "auto":
        hybrid_planning = "true" if hardware_type == "mujoco" else "false"
    if octomap_resolution == "auto":
        # A map-frame octomap keeps the whole room. Every scene update carries all of it to
        # RViz and the hybrid planners (several MB at 2 cm, and RViz falls behind), so it
        # is coarser than the robot-frame one.
        octomap_resolution = 0.03 if map_frame else 0.008
    octomap_resolution = float(octomap_resolution)
    mappings = dict(_HW_MAPPINGS.get(hardware_type, _HW_MAPPINGS["mock_components"]))
    # Arm hardware (mock vs real OpenArm CAN) is selectable per side, independent
    # of hardware_type, which only controls the base/lift/wheels backend.
    mappings["left_use_fake_arm_hardware"] = left_use_fake_arm_hardware
    mappings["right_use_fake_arm_hardware"] = right_use_fake_arm_hardware
    mappings["right_joint2_fixed"] = right_joint2_fixed

    pkg_moveit = get_package_share_directory("openarm_bimanual_moveit_config")

    use_sim_time = hardware_type in ("gazebo", "isaac", "mujoco")

    moveit_config = (
        MoveItConfigsBuilder("openarm", package_name="openarm_bimanual_moveit_config")
        .robot_description(
            file_path="config/openarm_bimanual.urdf.xacro",
            mappings=mappings,
        )
        # SRDF group_state defaults reference openarm_right_joint2; that line is
        # gated by the same flag in the SRDF so it's skipped once the joint is fixed.
        # map_frame turns the virtual joint into a planar joint from that frame.
        .robot_description_semantic(
            file_path="config/openarm_bimanual.srdf",
            mappings={"right_joint2_fixed": right_joint2_fixed, "map_frame": map_frame},
        )
        .trajectory_execution(moveit_manage_controllers=False)
        .to_moveit_configs()
    )
    moveit_params = moveit_config.to_dict()
    moveit_params["use_sim_time"] = use_sim_time
    if hardware_type == "mujoco":
        # Head camera cloud only (from the depth image, no robot_self_filter).
        moveit_params = _without_sensors(moveit_params)
        with open(os.path.join(pkg_moveit, "config", "sensors_3d_mujoco.yaml")) as _f:
            moveit_params.update(yaml.safe_load(_f))
    # Octomap needs a 3D sensor, which only Gazebo, the real robot and MuJoCo
    # publish (no source is wired for mock/isaac).
    if hardware_type not in ("gazebo", "real_robot", "mujoco"):
        moveit_params = _without_sensors(moveit_params)
        moveit_params["octomap_resolution"] = 0.0
    else:
        # In the map frame the voxels stay where the obstacles are while the base
        # drives; otherwise the octomap is anchored to base_link and moves with it.
        moveit_params["octomap_frame"] = map_frame or "base_link"
        moveit_params["octomap_resolution"] = octomap_resolution
        if map_frame:
            # scene updates with the octomap at 2 Hz instead of 4 Hz, for the same reason
            moveit_params["planning_scene_monitor"] = {"publish_planning_scene_hz": 2.0}
        # Override the head camera cloud topic at runtime; the sensors files use the
        # named-sensor layout: moveit_params["<sensor_name>"]["point_cloud_topic"].
        if point_cloud_topic and isinstance(moveit_params.get("head_cloud"), dict):
            moveit_params["head_cloud"]["point_cloud_topic"] = point_cloud_topic

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

    rviz_arguments = ["-d", os.path.join(pkg_moveit, "config", "moveit.rviz")]
    if map_frame:
        # show the scene in the planning frame, so the octomap stays put while the base drives
        rviz_arguments += ["-f", map_frame]
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=rviz_arguments,
        parameters=[moveit_params],
        output="log",
        condition=IfCondition(use_rviz),
    )

    nodes = [move_group, rviz]
    if hybrid_planning == "true":
        nodes += _hybrid_planning_nodes(pkg_moveit, moveit_params, use_sim_time, map_frame)
    return nodes


def _hybrid_planning_nodes(pkg_moveit, moveit_params, use_sim_time, map_frame):
    """Hybrid planning components (config/hybrid_planner.yaml) and hybrid_planning_relay."""
    with open(os.path.join(pkg_moveit, "config", "hybrid_planner.yaml")) as _f:
        hybrid = yaml.safe_load(_f)
    # Robot model, kinematics and planning pipelines as for move_group. No 3D sensors:
    # the octomap comes from move_group's /monitored_planning_scene.
    common = _without_sensors(moveit_params)
    sim_time = {"use_sim_time": use_sim_time}
    # MoveItPlanningPipeline 2.12.4 ignores the topics of planning_scene_monitor_options:
    # its scene monitor (node global_planner_private, which component remappings do not
    # reach) listens to /planning_scene and publishes monitored_planning_scene. Process-wide
    # remaps in a container of its own make it read move_group's scene, with the octomap,
    # and keep it off move_group's topic, which the local planner reads.
    global_container = _respawning_container(
        "hybrid_global_planner_container",
        [
            ComposableNode(
                package="moveit_hybrid_planning",
                plugin="moveit::hybrid_planning::GlobalPlannerComponent",
                name="global_planner",
                parameters=[hybrid["global_planner"], common, sim_time],
            ),
        ],
        sim_time,
        ros_arguments=[
            "-r", "/planning_scene:=/monitored_planning_scene",
            "-r", "monitored_planning_scene:=/global_planner/monitored_planning_scene",
        ],
    )
    container = _respawning_container(
        "hybrid_planning_container",
        [
            ComposableNode(
                package="moveit_hybrid_planning",
                plugin="moveit::hybrid_planning::LocalPlannerComponent",
                name="local_planner",
                parameters=[hybrid["local_planner"], common, sim_time],
            ),
            ComposableNode(
                package="moveit_hybrid_planning",
                plugin="moveit::hybrid_planning::HybridPlanningManager",
                name="hybrid_planning_manager",
                parameters=[hybrid["hybrid_planning_manager"], sim_time],
            ),
        ],
        sim_time,
        # The manager starts the local planner when a global trajectory arrives (fixed
        # topic name): it must see the relayed one, which the local planner follows.
        ros_arguments=[
            "-r", "/global_trajectory:=" + hybrid["local_planner"]["global_solution_topic"],
        ],
    )
    relay = Node(
        package="openarm_bimanual_moveit_config",
        executable="hybrid_planning_relay.py",
        parameters=[{
            "global_output_topic": hybrid["local_planner"]["global_solution_topic"],
            "local_input_topic": hybrid["local_planner"]["local_solution_topic"],
            "map_frame": map_frame,
        }, sim_time],
        output="screen",
    )
    return global_container + container + [relay]


def _respawning_container(name, components, sim_time, ros_arguments):
    """
    A component container that is restarted when it exits, with its components loaded at
    every start. The hybrid planning manager 2.12.4 aborts its process when a goal it has
    already aborted is aborted again (a failed replan while the local planner still holds),
    so the container comes back within seconds instead of leaving hybrid planning down.
    """
    container = Node(
        package="rclcpp_components",
        executable="component_container_mt",
        name=name,
        namespace="/",
        parameters=[sim_time],
        ros_arguments=ros_arguments,
        respawn=True,
        respawn_delay=1.0,
        output="screen",
    )
    load = RegisterEventHandler(OnProcessStart(
        target_action=container,
        on_start=[LoadComposableNodes(target_container="/" + name,
                                      composable_node_descriptions=components)],
    ))
    return [container, load]


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
            default_value="",
            description="PointCloud2 topic of the head camera fed to the MoveIt octomap "
                        "updater. Empty: the sensors file's topic (sensors_3d.yaml: the "
                        "self-filtered ZED cloud, same name via gz_bridge and on the real "
                        "ZED; sensors_3d_mujoco.yaml: the cloud from the MuJoCo depth image).",
        ),
        DeclareLaunchArgument(
            "octomap_resolution",
            default_value="auto",
            description="Octomap voxel size in metres. auto: 0.03 in the map frame, "
                        "0.008 otherwise.",
        ),
        DeclareLaunchArgument(
            "map_frame",
            default_value="auto",
            description="Plan in this fixed frame: a planar virtual joint from it to "
                        "base_footprint, read from TF, and the octomap in it. none: "
                        "plan in the robot frame, octomap in base_link. auto: map for "
                        "hardware_type:=mujoco, none otherwise.",
        ),
        DeclareLaunchArgument(
            "hybrid_planning",
            default_value="auto",
            description="Start MoveIt hybrid planning (global + local planner, action "
                        "run_hybrid_planning) next to move_group. auto: true for "
                        "hardware_type:=mujoco, false otherwise.",
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
    ]

    hardware_type = LaunchConfiguration("hardware_type")
    point_cloud_topic = LaunchConfiguration("point_cloud_topic")
    octomap_resolution = LaunchConfiguration("octomap_resolution")
    left_use_fake_arm_hardware = LaunchConfiguration("left_use_fake_arm_hardware")
    right_use_fake_arm_hardware = LaunchConfiguration("right_use_fake_arm_hardware")
    right_joint2_fixed = LaunchConfiguration("right_joint2_fixed")
    use_rviz = LaunchConfiguration("use_rviz")
    map_frame = LaunchConfiguration("map_frame")
    hybrid_planning = LaunchConfiguration("hybrid_planning")

    nodes_func = OpaqueFunction(
        function=_make_nodes,
        args=[hardware_type, point_cloud_topic, octomap_resolution,
              left_use_fake_arm_hardware, right_use_fake_arm_hardware,
              right_joint2_fixed, use_rviz, map_frame, hybrid_planning],
    )

    return LaunchDescription(declared_arguments + [nodes_func])
