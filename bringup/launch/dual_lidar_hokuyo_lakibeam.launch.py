#!/usr/bin/env python3
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch_ros.actions import ComposableNodeContainer, LifecycleNode, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    dual_lidar_config_file_path = os.path.join(
        get_package_share_directory("robot_bringup"),
        "config",
        "lidar",
        "dual_lidar_config.yml",
    )

    # Load configuration file
    with open(dual_lidar_config_file_path, "r") as file:
        lidar_config = yaml.safe_load(file)

    # Extract Hokuyo parameters
    hokuyo_config_params = lidar_config["urg_node2"]["ros__parameters"]
    hokuyo_launch_config = lidar_config["urg_node2"]["launch_config"]

    # Extract Lakibeam parameters
    lakibeam_config_params = lidar_config["lakibeam_lidar"]["ros__parameters"]

    # Hokuyo URG lifecycle node
    hokuyo_lifecycle_node = LifecycleNode(
        package="urg_node2",
        executable="urg_node2_node",
        name=hokuyo_launch_config["node_name"],
        remappings=[("scan", hokuyo_launch_config["scan_topic"])],
        parameters=[hokuyo_config_params],
        namespace="",
        output="screen",
    )

    # Hokuyo: Unconfigure -> Inactive transition
    hokuyo_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=hokuyo_lifecycle_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(hokuyo_lifecycle_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
    )

    # Hokuyo: Inactive -> Active transition
    hokuyo_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=hokuyo_lifecycle_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(hokuyo_lifecycle_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
    )

    # ========== Lakibeam Configuration ==========
    # Lakibeam node
    lakibeam_node = Node(
        package="lakibeam1",
        name="lakibeam_lidar_node",
        executable="lakibeam1_scan_node",
        parameters=[lakibeam_config_params],
        output="screen",
    )

    # ========== Lidar Merger Configuration ==========
    dual_laser_merger_node = ComposableNodeContainer(
        name="demo_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="dual_laser_merger",
                plugin="merger_node::MergerNode",
                name="dual_laser_merger",
                parameters=[
                    {"laser_1_topic": "/front_lidar"},
                    {"laser_2_topic": "/back_lidar"},
                    {"merged_scan_topic": "/merged_lidar"},
                    {"target_frame": "base_link"},
                    {"laser_1_x_offset": 0.0},
                    {"laser_1_y_offset": 0.0},
                    {"laser_1_yaw_offset": 0.0},
                    {"laser_2_x_offset": -0.0},
                    {"laser_2_y_offset": 0.0},
                    {"laser_2_yaw_offset": 0.0},
                    {"tolerance": 0.01},
                    {"queue_size": 5},
                    {"angle_increment": 0.001},
                    {"scan_time": 0.067},
                    {"range_min": 0.3},
                    {"range_max": 25.0},
                    {"min_height": -1.0},
                    {"max_height": 1.0},
                    {"angle_min": -3.141592654},
                    {"angle_max": 3.141592654},
                    {"inf_epsilon": 1.0},
                    {"use_inf": True},
                    {"allowed_radius": 0.45},
                    {"enable_shadow_filter": False},
                    {"enable_average_filter": False},
                ],
                remappings=[("/merged_lidar", "/scan")],
            )
        ],
        output="screen",
    )

    # ========== Launch Description ==========
    ld = LaunchDescription()

    # Nodes
    ld.add_action(hokuyo_lifecycle_node)
    ld.add_action(lakibeam_node)
    ld.add_action(dual_laser_merger_node)

    # Event handlers for Hokuyo lifecycle (only if auto_start is true)
    if hokuyo_launch_config["auto_start"]:
        ld.add_action(hokuyo_configure_event_handler)
        ld.add_action(hokuyo_activate_event_handler)

    return ld
