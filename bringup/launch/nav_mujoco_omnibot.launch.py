#!/usr/bin/env python3
"""Run Walkie in MuJoCo with Nav2.

Includes mujoco_omnibot.launch.py and starts Nav2 (nav2_bringup) with the
simulation parameters from robot_navigation, map_server, and no AMCL. The
map -> odom transform is a static identity, so the robot pose in the map is the
wheel odometry, and the robot must start at the map origin. A goal from the
RViz "2D Goal Pose" tool (/goal_pose) or the Nav2 panel drives the robot.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_robot_bringup = get_package_share_directory("robot_bringup")
    pkg_robot_navigation = get_package_share_directory("robot_navigation")
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")
    pkg_walkie_description = get_package_share_directory("walkie_description")

    default_map_dir = os.path.join(
        pkg_robot_bringup, "config", "map", "empty_floor"
    )
    default_nav2_config = os.path.join(
        pkg_robot_navigation,
        "config",
        "nav2",
        "nav2_mppi_walkie_params.yaml",
    )
    bt_xml_path = os.path.join(
        pkg_robot_navigation,
        "config",
        "nav2",
        "behavior_tree",
        "nav_to_pose_real_recovery_bt.xml",
    )
    walkie_nav_config = os.path.join(
        pkg_robot_navigation,
        "config",
        "walkie_nav.yaml",
    )

    map_dir = LaunchConfiguration("map")
    map_yaml_file = PathJoinSubstitution([map_dir, "map.yaml"])
    nav2_config = LaunchConfiguration("nav2_config")
    keepout_mask_file = LaunchConfiguration("keepout_mask")
    use_keepout_zones = LaunchConfiguration("use_keepout_zones")
    mujoco_model = LaunchConfiguration("mujoco_model")
    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    nav_debug = LaunchConfiguration("nav_debug")

    declare_map_arg = DeclareLaunchArgument(
        "map",
        default_value=default_map_dir,
        description="Full path to the map directory containing map.yaml and keepout.yaml",
    )
    declare_nav2_config_arg = DeclareLaunchArgument(
        "nav2_config",
        default_value=default_nav2_config,
        description="Full path to the nav2 config YAML file",
    )
    declare_use_keepout_zones_arg = DeclareLaunchArgument(
        "use_keepout_zones",
        default_value="false",
        description="Whether to enable keepout (no-go) zone filters",
    )
    declare_keepout_mask_arg = DeclareLaunchArgument(
        "keepout_mask",
        default_value=PathJoinSubstitution([map_dir, "keepout.yaml"]),
        description="Full path to keepout mask YAML file to load. Defaults to keepout.yaml in map_dir.",
    )
    declare_nav_debug_arg = DeclareLaunchArgument(
        "nav_debug",
        default_value="false",
        description="Publish nav_commander debug markers (ROI cells, PCA, approach angle)",
    )
    declare_mujoco_model_arg = DeclareLaunchArgument(
        "mujoco_model",
        default_value=os.path.join(pkg_walkie_description, "mujoco", "scene.xml"),
        description="MJCF scene to load. It must include walkie.xml and a plane geom named floor",
    )
    declare_headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run MuJoCo without the viewer window",
    )
    declare_use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Start RViz",
    )
    declare_rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(pkg_robot_bringup, "config", "rviz", "mujoco_nav.rviz"),
        description="RViz config file",
    )

    mujoco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_bringup, "launch", "mujoco_omnibot.launch.py")
        ),
        launch_arguments={
            "mujoco_model": mujoco_model,
            "headless": headless,
            "use_rviz": use_rviz,
            "rviz_config": rviz_config,
        }.items(),
    )

    # Odometry-only localization: AMCL would have nothing to match in a world
    # without walls. omni_wheel_drive_controller publishes odom -> base_footprint.
    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom",
        output="screen",
        arguments=["--frame-id", "map", "--child-frame-id", "odom"],
        parameters=[{"use_sim_time": True}],
    )

    # Simulation parameter substitutions via RewrittenYaml
    nav2_rewritten_params = RewrittenYaml(
        source_file=nav2_config,
        root_key="",
        param_rewrites={
            "use_sim_time": "true",
            "local_costmap.local_costmap.ros__parameters.stvl_layer.enabled": "false",
            "collision_monitor.ros__parameters.pointcloud.enabled": "false",
        },
        convert_types=True,
    )

    # Nav2 nodes publish "cmd_vel"; remap it to cmd_vel_controller so commands
    # pass velocity_smoother and collision_monitor before reaching twist_mux on
    # cmd_vel_nav. SetRemap rules come before nav2_bringup's own
    # cmd_vel -> cmd_vel_nav remaps, and the first matching rule wins.
    # serve_static_map defaults to use_localization, so it is set explicitly.
    # use_composition is off because nav2_bringup starts nav2_container with
    # "component_container --isolated", a flag the Jazzy component_container
    # ignores. All servers would then share one executor thread, and if
    # lifecycle_manager_nav2 is loaded before the other servers, its blocking
    # startup stops the container from loading them.
    nav2_launch = GroupAction(
        actions=[
            SetRemap(src="cmd_vel", dst="cmd_vel_controller"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2_bringup, "launch", "bringup_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": "true",
                    "map": map_yaml_file,
                    "params_file": nav2_rewritten_params,
                    "default_nav_to_pose_bt_xml": bt_xml_path,
                    "use_localization": "false",
                    "serve_static_map": "true",
                    "use_keepout_zones": use_keepout_zones,
                    "keepout_mask": keepout_mask_file,
                    "use_speed_zones": "false",
                    "autostart": "true",
                    "use_composition": "false",
                }.items(),
            ),
        ]
    )

    # Object approach-pose action server (/navigate_to_object -> Nav2)
    nav_commander = Node(
        package="robot_navigation",
        executable="nav_commander.py",
        name="nav_commander",
        output="screen",
        parameters=[
            walkie_nav_config,
            {
                "use_sim_time": True,
                "debug": nav_debug,
            },
        ],
    )

    ld = LaunchDescription()

    ld.add_action(declare_map_arg)
    ld.add_action(declare_nav2_config_arg)
    ld.add_action(declare_use_keepout_zones_arg)
    ld.add_action(declare_keepout_mask_arg)
    ld.add_action(declare_nav_debug_arg)
    ld.add_action(declare_mujoco_model_arg)
    ld.add_action(declare_headless_arg)
    ld.add_action(declare_use_rviz_arg)
    ld.add_action(declare_rviz_config_arg)

    ld.add_action(mujoco_launch)
    ld.add_action(map_to_odom)
    ld.add_action(nav2_launch)
    ld.add_action(nav_commander)

    return ld
