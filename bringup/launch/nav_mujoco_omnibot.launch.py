#!/usr/bin/env python3
"""Run Walkie in MuJoCo with Nav2.

Includes mujoco_omnibot.launch.py and starts Nav2 (nav2_bringup) with the
simulation rewrites of the robot_navigation parameters, map_server and
nav_commander. A goal from the RViz "2D Goal Pose" tool (/goal_pose) or the Nav2
panel drives the robot.

Without world, the scene is the empty floor and there is no AMCL: the
map -> odom transform is a static identity, so the robot pose in the map is the
wheel odometry, and the robot must start at the map origin.

With world:=NAME, the exported world (see robot_bringup.mujoco_launch) supplies
the MuJoCo model and the map, AMCL localizes against that map, and AMCL's
initial pose in a generated copy of nav2_config matches the world spawn.

truth_map_odom:=true replaces both the static transform and AMCL with
ground_truth_map_odom.py, which feeds map -> odom from MuJoCo's ground truth.
The world loader writes the spawn's map X and Y into the MuJoCo world frame, so
the two frames coincide in a world as well.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from nav2_common.launch import RewrittenYaml
from robot_bringup.mujoco_launch import declare_world_arguments, prepare_world_from_context


def _flag(context, name):
    """Return a boolean launch argument the way IfCondition reads it."""
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value not in ("true", "1", "false", "0"):
        raise ValueError(f"{name} must be true or false, not {value!r}")
    return value in ("true", "1")


def _resolve_world(context, *, description_dir, default_model, default_map_dir):
    """Select the model, map, Nav2 parameters and the source of map -> odom."""
    prepared = prepare_world_from_context(
        context, description_dir=description_dir,
        replaced_defaults={"mujoco_model": default_model, "map": default_map_dir},
        nav2_params_source=LaunchConfiguration("nav2_config").perform(context),
    )
    truth_map_odom = _flag(context, "truth_map_odom")
    if prepared is None:
        map_dir = LaunchConfiguration("map").perform(context)
        return [
            SetLaunchConfiguration("map_yaml", os.path.join(map_dir, "map.yaml")),
            SetLaunchConfiguration("use_localization", "False"),
            SetLaunchConfiguration("static_map_odom", str(not truth_map_odom)),
        ]

    # A world names its map file; its directory stands in for the map argument,
    # so a keepout mask left at its default is looked up next to the world's map.
    world_map_dir = os.path.dirname(prepared["map_path"])
    actions = [
        SetLaunchConfiguration("mujoco_model", prepared["model_path"]),
        SetLaunchConfiguration("map", world_map_dir),
        SetLaunchConfiguration("map_yaml", prepared["map_path"]),
        SetLaunchConfiguration("nav2_config", prepared["params_path"]),
        SetLaunchConfiguration("use_localization", str(not truth_map_odom)),
        SetLaunchConfiguration("static_map_odom", "False"),
        LogInfo(msg=f"Prepared world variant {prepared['variant']}: "
                    f"{prepared['model_path']}; map: {prepared['map_path']}"),
    ]
    keepout_mask = LaunchConfiguration("keepout_mask").perform(context)
    if keepout_mask == os.path.join(default_map_dir, "keepout.yaml"):
        actions.append(SetLaunchConfiguration(
            "keepout_mask", os.path.join(world_map_dir, "keepout.yaml")
        ))
    return actions


def generate_launch_description():
    pkg_robot_bringup = get_package_share_directory("robot_bringup")
    pkg_robot_navigation = get_package_share_directory("robot_navigation")
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")
    pkg_walkie_description = get_package_share_directory("walkie_description")

    default_map_dir = os.path.join(
        pkg_robot_bringup, "config", "map", "empty_floor"
    )
    default_model = os.path.join(pkg_walkie_description, "mujoco", "scene.xml")
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
    # Set by _resolve_world: map.yaml in the map directory, or the world's map file.
    map_yaml_file = LaunchConfiguration("map_yaml")
    nav2_config = LaunchConfiguration("nav2_config")
    keepout_mask_file = LaunchConfiguration("keepout_mask")
    use_keepout_zones = LaunchConfiguration("use_keepout_zones")
    mujoco_model = LaunchConfiguration("mujoco_model")
    use_localization = LaunchConfiguration("use_localization")
    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    nav_debug = LaunchConfiguration("nav_debug")

    declare_map_arg = DeclareLaunchArgument(
        "map",
        default_value=default_map_dir,
        description="Full path to the map directory containing map.yaml and keepout.yaml, "
                    "without world. Its origin must match the MuJoCo world origin",
    )
    declare_nav2_config_arg = DeclareLaunchArgument(
        "nav2_config",
        default_value=default_nav2_config,
        description="Full path to the nav2 config YAML file. With world, a copy with the "
                    "AMCL initial pose at the world spawn is used",
    )
    declare_use_keepout_zones_arg = DeclareLaunchArgument(
        "use_keepout_zones",
        default_value="false",
        description="Whether to enable keepout (no-go) zone filters",
    )
    declare_keepout_mask_arg = DeclareLaunchArgument(
        "keepout_mask",
        default_value=PathJoinSubstitution([map_dir, "keepout.yaml"]),
        description="Full path to keepout mask YAML file to load. Defaults to keepout.yaml "
                    "in the map directory, or next to the world's map with world",
    )
    declare_nav_debug_arg = DeclareLaunchArgument(
        "nav_debug",
        default_value="false",
        description="Publish nav_commander debug markers (ROI cells, PCA, approach angle)",
    )
    declare_mujoco_model_arg = DeclareLaunchArgument(
        "mujoco_model",
        default_value=default_model,
        description="MJCF scene to load without world. It must include walkie.xml "
                    "and a plane geom named floor",
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
    truth_map_odom = LaunchConfiguration("truth_map_odom")
    declare_truth_map_odom_arg = DeclareLaunchArgument(
        "truth_map_odom",
        default_value="false",
        description="Feed map -> odom from /mujoco_ground_truth/odom instead of a static "
                    "identity or AMCL, so the map pose of the base is the true one",
    )

    # The world, if any, is prepared once here; the included simulator launch
    # receives the generated model. world and the spawn arguments are cleared
    # because the group passes this launch's configurations to the include.
    mujoco_launch = GroupAction(actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_robot_bringup, "launch", "mujoco_omnibot.launch.py")
            ),
            launch_arguments={
                "world": "",
                "spawn": "",
                "spawn_pose": "",
                "mujoco_model": mujoco_model,
                "headless": headless,
                "use_rviz": use_rviz,
                "rviz_config": rviz_config,
            }.items(),
        ),
    ])

    # Odometry-only localization on the empty floor: AMCL would have nothing to
    # match in a world without walls. omni_wheel_drive_controller publishes
    # odom -> base_footprint. With a world, AMCL publishes map -> odom, and with
    # truth_map_odom ground_truth_map_odom.py does.
    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom",
        output="screen",
        arguments=["--frame-id", "map", "--child-frame-id", "odom"],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("static_map_odom")),
    )
    ground_truth_map_odom = Node(
        package="robot_bringup",
        executable="ground_truth_map_odom.py",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(truth_map_odom),
    )

    # Simulation parameter substitutions via RewrittenYaml. state_topic lets the
    # whole-body base follower see a monitor stop as soon as it happens.
    nav2_rewritten_params = RewrittenYaml(
        source_file=nav2_config,
        root_key="",
        param_rewrites={
            "use_sim_time": "true",
            "local_costmap.local_costmap.ros__parameters.stvl_layer.enabled": "false",
            "collision_monitor.ros__parameters.pointcloud.enabled": "false",
            "collision_monitor.ros__parameters.state_topic": "/collision_monitor_state",
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
                    "use_localization": use_localization,
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

    for declare_world_arg in declare_world_arguments(nav2=True):
        ld.add_action(declare_world_arg)
    ld.add_action(declare_map_arg)
    ld.add_action(declare_nav2_config_arg)
    ld.add_action(declare_use_keepout_zones_arg)
    ld.add_action(declare_keepout_mask_arg)
    ld.add_action(declare_nav_debug_arg)
    ld.add_action(declare_mujoco_model_arg)
    ld.add_action(declare_headless_arg)
    ld.add_action(declare_use_rviz_arg)
    ld.add_action(declare_rviz_config_arg)
    ld.add_action(declare_truth_map_odom_arg)
    ld.add_action(OpaqueFunction(function=_resolve_world, kwargs={
        "description_dir": pkg_walkie_description,
        "default_model": default_model,
        "default_map_dir": default_map_dir,
    }))

    ld.add_action(mujoco_launch)
    ld.add_action(map_to_odom)
    ld.add_action(ground_truth_map_odom)
    ld.add_action(nav2_launch)
    ld.add_action(nav_commander)

    return ld
