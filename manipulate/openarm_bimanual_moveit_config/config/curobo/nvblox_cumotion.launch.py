#!/usr/bin/env python3
"""Perceived-environment avoidance for cuMotion via an nvBlox ESDF world.

Brings up the perception -> ESDF -> cuMotion additions on top of a robot that is
already publishing depth + TF + /joint_states (the Gazebo sim OR the real robot --
both expose the ZED head on the SAME topics, see simulation gz_bridge.yaml):

    depth (ZED head) -> robot_segmenter (mask the robot out) -> /cumotion/world_depth
                     -> nvblox_node (fuse -> 3D ESDF; serve get_esdf_and_gradient)
                     -> move_group_cumotion (read_esdf_world:=true)

The robot_segmenter is REQUIRED: without it nvblox fuses the arms themselves into
the map and cuMotion deadlocks avoiding its own body. It masks using the SAME
cuMotion URDF + XRDF spheres the planner uses (cumotion_urdf / cumotion_xrdf).

This launch does NOT start the sim or the robot driver -- run those separately so
they provide the depth image, camera TF, and /joint_states this stack consumes.

  # sim: terminal 1 -> ros2 launch robot_simulation gzsim_omnibot.launch.py
  #      terminal 2 -> ros2 launch openarm_bimanual_moveit_config \
  #                        config/curobo/nvblox_cumotion.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

PKG = "openarm_bimanual_moveit_config"


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory(PKG), "config", "curobo")
    seg_params = os.path.join(cfg, "nvblox", "robot_segmenter.yaml")
    nvblox_params = os.path.join(cfg, "nvblox", "nvblox.yaml")

    default_xrdf = os.path.join(cfg, "both_arms_lift.xrdf")
    default_urdf = os.path.join(
        get_package_share_directory(PKG), "config", "openarm_bimanual.urdf")

    args = [
        # Robot description for BOTH the planner and the segmenter (the single-arm
        # xrdfs still carry both arms' spheres, so masking stays correct for any group).
        DeclareLaunchArgument("cumotion_xrdf", default_value=default_xrdf),
        DeclareLaunchArgument("cumotion_urdf", default_value=default_urdf),
        # ZED head depth -- same topics in sim and on the real robot.
        DeclareLaunchArgument(
            "depth_image_topic",
            default_value="/zed_head/zed_node/depth/depth_registered"),
        DeclareLaunchArgument(
            "depth_camera_info_topic",
            default_value="/zed_head/zed_node/depth/depth_registered/camera_info"),
        # Robot-masked depth that nvblox actually fuses.
        DeclareLaunchArgument("world_depth_topic", default_value="/cumotion/world_depth"),
        # nvblox map frame. odom works for sim (gazebo) and the real robot (nav2);
        # use base_link for a static-base bench with no odom.
        DeclareLaunchArgument("global_frame", default_value="odom"),
    ]
    xrdf = LaunchConfiguration("cumotion_xrdf")
    urdf = LaunchConfiguration("cumotion_urdf")
    depth_img = LaunchConfiguration("depth_image_topic")
    depth_info = LaunchConfiguration("depth_camera_info_topic")
    world_depth = LaunchConfiguration("world_depth_topic")
    global_frame = LaunchConfiguration("global_frame")

    # robot_segmenter is component-only -> load it into a container. Topics + the
    # robot description come from robot_segmenter.yaml; override urdf/xrdf and the
    # depth topics so they track the launch args.
    segmenter_container = ComposableNodeContainer(
        name="cumotion_perception_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                name="robot_segmenter",
                package="isaac_ros_cumotion_robot_segmenter",
                plugin="nvidia::isaac_ros::manipulator::RobotSegmenter",
                parameters=[seg_params, {
                    "urdf_path": urdf,
                    "xrdf_path": xrdf,
                    "depth_image_topics": [depth_img],
                    "depth_camera_infos": [depth_info],
                    "world_depth_publish_topics": [world_depth],
                }],
            ),
        ],
        output="screen",
    )

    # nvblox fuses the masked depth into a 3D ESDF and serves get_esdf_and_gradient.
    # Its depth INPUT (camera_0/depth/{image,camera_info}) is remapped to the
    # segmented world_depth + the camera's original camera_info.
    nvblox_node = Node(
        package="nvblox_ros", executable="nvblox_node", name="nvblox_node",
        parameters=[nvblox_params, {"global_frame": global_frame}],
        remappings=[
            ("camera_0/depth/image", world_depth),
            ("camera_0/depth/camera_info", depth_info),
        ],
        output="screen",
    )

    # move_group + the cuMotion planner with the ESDF world turned ON.
    move_group_cumotion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(PKG), "launch", "move_group_cumotion.launch.py")),
        launch_arguments={
            "cumotion_xrdf": xrdf,
            "cumotion_urdf": urdf,
            "cumotion_read_esdf_world": "true",
        }.items(),
    )

    return LaunchDescription(args + [segmenter_container, nvblox_node, move_group_cumotion])
