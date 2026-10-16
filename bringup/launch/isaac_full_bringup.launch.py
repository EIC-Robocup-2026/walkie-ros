#!/usr/bin/env python3
# ==============================================================================
# Walkie Bot - Isaac Sim Full System Bringup Launch File
# RoboCup 2027
#
# Launches the complete ROS 2 stack corresponding to real_omnibot.launch.py,
# tailored for Isaac Sim:
#   1. robot_state_publisher (URDF via Xacro, ros2_control:=topic_base, use_sim_time:=true)
#   2. twist_mux (priority arbiter for cmd_vel, nav, joy, key -> /omni_wheel_drive_controller/cmd_vel)
#   3. ros2_control_node (controller manager with topic_based_ros2_control bridge):
#      - joint_broad (JointStateBroadcaster -> /joint_states)
#      - omni_wheel_drive_controller (OmniWheelDriveController -> /odom, odom->base_footprint TF)
#      - head_servo_controller & auto-init to 0.25 rad pitch
#      - left_joint_trajectory_controller & right_joint_trajectory_controller & home pose auto-init
#      - left_gripper_controller & right_gripper_controller
#      - lift_controller (JointTrajectoryController)
#   4. robot_self_filter (point cloud collision self-filtering for 3D LiDAR & ZED camera)
#   5. current_pose_publisher (map -> base_footprint at 10 Hz)
#   6. walkie_tf_server (TF transform lookup services)
#   7. cloud_to_map & ob_pose_service_cpp (depth camera perception pipeline)
#   8. rosbridge_websocket & optional foxglove_bridge
#   9. RViz2 (pre-configured with full sensor suite and robot model)
# ==============================================================================

import os
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "robot_bringup"
    description_package_name = "walkie_description"
    pkg_rosbridge_server = get_package_share_directory("rosbridge_server")

    default_robot = os.path.join(
        get_package_share_directory(description_package_name),
        "robots",
        "gz_walkie.urdf.xacro",
    )

    # --------------------------------------------------------------------------
    # Launch Configuration Variables
    # --------------------------------------------------------------------------
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    robot_model = LaunchConfiguration("robot_model", default=default_robot)
    ros2_control = LaunchConfiguration("ros2_control", default="topic_base")
    use_zed = LaunchConfiguration("use_zed", default="true")
    use_unitree_lidar = LaunchConfiguration("use_unitree_lidar", default="true")
    use_arm = LaunchConfiguration("use_arm", default="true")
    use_rviz = LaunchConfiguration("use_rviz", default="true")
    right_joint2_fixed = LaunchConfiguration("right_joint2_fixed", default="false")
    unitree_cloud_frame = LaunchConfiguration("unitree_cloud_frame", default="unitree4d_l2_imu_initial")
    unitree_imu_frame = LaunchConfiguration("unitree_imu_frame", default="unitree4d_l2_imu")
    use_rosbridge = LaunchConfiguration("use_rosbridge", default="true")
    use_foxglove = LaunchConfiguration("use_foxglove", default="false")
    use_perception = LaunchConfiguration("use_perception", default="true")
    use_tf_server = LaunchConfiguration("use_tf_server", default="true")
    use_moveit = LaunchConfiguration("use_moveit", default="false")

    # --------------------------------------------------------------------------
    # Launch Arguments
    # --------------------------------------------------------------------------
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock from Isaac Sim (/clock)",
    )
    declare_robot_model = DeclareLaunchArgument(
        "robot_model",
        default_value=default_robot,
        description="Path to robot URDF/Xacro description",
    )
    declare_ros2_control = DeclareLaunchArgument(
        "ros2_control",
        default_value="topic_base",
        description="ros2_control hardware plugin mode (default: topic_base for Isaac Sim)",
    )
    declare_use_zed = DeclareLaunchArgument(
        "use_zed",
        default_value="true",
        description="Whether to enable ZED camera topics and self-filtering",
    )
    declare_use_unitree_lidar = DeclareLaunchArgument(
        "use_unitree_lidar",
        default_value="true",
        description="Whether to enable Unitree 3D LiDAR point cloud self-filtering",
    )
    declare_use_arm = DeclareLaunchArgument(
        "use_arm",
        default_value="true",
        description="Whether to spawn OpenArm trajectory and gripper controllers",
    )
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz2 pre-configured for Walkie Bot",
    )
    declare_right_joint2_fixed = DeclareLaunchArgument(
        "right_joint2_fixed",
        default_value="false",
        description="Lock the right arm joint2 as a fixed URDF joint at its nominal "
                    "(zero) position, dropping it from MoveIt's IK/planning DOF",
    )
    declare_unitree_cloud_frame = DeclareLaunchArgument(
        "unitree_cloud_frame",
        default_value="unitree4d_l2_imu_initial",
        description="Frame ID for the Unitree LiDAR point cloud",
    )
    declare_unitree_imu_frame = DeclareLaunchArgument(
        "unitree_imu_frame",
        default_value="unitree4d_l2_imu",
        description="IMU frame ID used by the Unitree LiDAR node",
    )
    declare_use_rosbridge = DeclareLaunchArgument(
        "use_rosbridge",
        default_value="true",
        description="Launch rosbridge WebSocket server on port 9090",
    )
    declare_use_foxglove = DeclareLaunchArgument(
        "use_foxglove",
        default_value="false",
        description="Launch Foxglove bridge server on port 8765",
    )
    declare_use_perception = DeclareLaunchArgument(
        "use_perception",
        default_value="true",
        description="Launch depth-camera perception nodes (cloud_to_map, ob_pose_service)",
    )
    declare_use_tf_server = DeclareLaunchArgument(
        "use_tf_server",
        default_value="true",
        description="Launch walkie_tf_server for TF transform lookups",
    )
    declare_use_moveit = DeclareLaunchArgument(
        "use_moveit",
        default_value="false",
        description="Whether to launch MoveIt 2 (move_group + MoveIt RViz Motion Planning UI)",
    )

    # --------------------------------------------------------------------------
    # 1. Robot Description (URDF / Xacro)
    # --------------------------------------------------------------------------
    robot_description_content = Command(
        [
            "xacro ",
            robot_model,
            " ros2_control:=",
            ros2_control,
            " use_zed:=",
            use_zed,
            " use_arm:=",
            use_arm,
            " use_fake_arm_hardware:=true",
            " right_joint2_fixed:=",
            right_joint2_fixed,
        ]
    )

    # --------------------------------------------------------------------------
    # 2. Robot State Publisher (RSP)
    # --------------------------------------------------------------------------
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(description_package_name),
                "launch",
                "robot_state_publisher.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "robot_model": robot_model,
            "ros2_control": ros2_control,
            "use_zed": use_zed,
            "use_arm": use_arm,
            "use_fake_arm_hardware": "true",
            "right_joint2_fixed": right_joint2_fixed,
        }.items(),
    )

    # --------------------------------------------------------------------------
    # 3. Twist Mux (Velocity Input Priority Multiplexer)
    # --------------------------------------------------------------------------
    twist_mux_params = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "twist_mux",
        "twist_mux.yml",
    )
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[
            twist_mux_params,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[("/cmd_vel_out", "/omni_wheel_drive_controller/cmd_vel")],
    )

    # --------------------------------------------------------------------------
    # 4. Controller Manager (ros2_control_node with topic_based_ros2_control)
    # --------------------------------------------------------------------------
    controllers_config = os.path.join(
        get_package_share_directory(description_package_name),
        "config",
        "ros2_controller",
        "isaac_controllers.yaml",
    )

    # Pre-generate a copy with openarm_right_joint2 stripped when right_joint2_fixed is true
    with open(controllers_config) as _cf:
        _isaac_controllers_no_joint2 = yaml.safe_load(_cf)
    for _ctrl_name in (
        "arm_controller",
        "right_arm_controller",
        "right_forward_position_controller",
        "right_forward_velocity_controller",
        "right_joint_trajectory_controller",
    ):
        _joints = _isaac_controllers_no_joint2.get(_ctrl_name, {}).get(
            "ros__parameters", {}
        ).get("joints", [])
        if "openarm_right_joint2" in _joints:
            _joints.remove("openarm_right_joint2")
    _fd, controllers_config_no_joint2 = tempfile.mkstemp(suffix=".yaml")
    with os.fdopen(_fd, "w") as _cf:
        yaml.safe_dump(_isaac_controllers_no_joint2, _cf)
    controllers_config_selected = PythonExpression([
        "'", controllers_config_no_joint2, "' if '", right_joint2_fixed,
        "' == 'true' else '", controllers_config, "'",
    ])

    controller_manager_spawner = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {"robot_description": ParameterValue(robot_description_content, value_type=str)},
            {"use_sim_time": use_sim_time},
            controllers_config_selected,
        ],
    )

    # Spawners
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--switch-timeout", "30.0"],
    )
    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_spawner,
            on_start=[joint_broad_spawner],
        )
    )

    omni_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["omni_wheel_drive_controller", "--switch-timeout", "30.0"],
    )
    delayed_omni_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_spawner,
            on_start=[omni_controller_spawner],
        )
    )

    servo_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["head_servo_controller", "--switch-timeout", "30.0"],
    )
    delayed_servo_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_spawner,
            on_start=[servo_controller_spawner],
        )
    )

    head_servo_init_pub = ExecuteProcess(
        cmd=[
            "ros2", "topic", "pub", "--times", "1",
            "/head_servo_controller/commands",
            "std_msgs/msg/Float64MultiArray",
            '{"layout": {"dim": [{"label": "", "size": 1, "stride": 1}], "data_offset": 0}, "data": [0.25]}',
        ],
        output="screen",
    )
    delayed_head_servo_init = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=servo_controller_spawner,
            on_exit=[head_servo_init_pub],
        )
    )

    arm_trajectory_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_joint_trajectory_controller",
            "right_joint_trajectory_controller",
            "--switch-timeout", "30.0",
        ],
        condition=IfCondition(use_arm),
    )
    delayed_arm_trajectory_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_spawner,
            on_start=[arm_trajectory_spawner],
        )
    )

    arm_gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_gripper_controller",
            "right_gripper_controller",
            "--switch-timeout", "30.0",
        ],
        condition=IfCondition(use_arm),
    )
    delayed_arm_gripper_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_spawner,
            on_start=[arm_gripper_spawner],
        )
    )

    arm_home_left = ExecuteProcess(
        cmd=[
            "ros2", "topic", "pub", "--times", "1",
            "/left_joint_trajectory_controller/joint_trajectory",
            "trajectory_msgs/msg/JointTrajectory",
            "{joint_names: [openarm_left_joint1, openarm_left_joint2, openarm_left_joint3, "
            "openarm_left_joint4, openarm_left_joint5, openarm_left_joint6, openarm_left_joint7], "
            "points: [{positions: [0.2618, 0.0, 0.0, 0.2618, 0.0, 0.0, 0.0], "
            "time_from_start: {sec: 5, nanosec: 0}}]}",
        ],
        output="screen",
        condition=IfCondition(use_arm),
    )
    arm_home_right_condition = IfCondition(PythonExpression(
        ["'", use_arm, "' == 'true' and '", right_joint2_fixed, "' != 'true'"]
    ))
    arm_home_right = ExecuteProcess(
        cmd=[
            "ros2", "topic", "pub", "--times", "1",
            "/right_joint_trajectory_controller/joint_trajectory",
            "trajectory_msgs/msg/JointTrajectory",
            "{joint_names: [openarm_right_joint1, openarm_right_joint2, openarm_right_joint3, "
            "openarm_right_joint4, openarm_right_joint5, openarm_right_joint6, openarm_right_joint7], "
            "points: [{positions: [-0.2618, 0.0, 0.0, 0.2618, 0.0, 0.0, 0.0], "
            "time_from_start: {sec: 5, nanosec: 0}}]}",
        ],
        output="screen",
        condition=arm_home_right_condition,
    )
    arm_home_right_joint2_fixed_condition = IfCondition(PythonExpression(
        ["'", use_arm, "' == 'true' and '", right_joint2_fixed, "' == 'true'"]
    ))
    arm_home_right_joint2_fixed = ExecuteProcess(
        cmd=[
            "ros2", "topic", "pub", "--times", "1",
            "/right_joint_trajectory_controller/joint_trajectory",
            "trajectory_msgs/msg/JointTrajectory",
            "{joint_names: [openarm_right_joint1, openarm_right_joint3, "
            "openarm_right_joint4, openarm_right_joint5, openarm_right_joint6, openarm_right_joint7], "
            "points: [{positions: [-0.2618, 0.0, 0.2618, 0.0, 0.0, 0.0], "
            "time_from_start: {sec: 5, nanosec: 0}}]}",
        ],
        output="screen",
        condition=arm_home_right_joint2_fixed_condition,
    )

    delayed_arm_home = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_trajectory_spawner,
            on_exit=[arm_home_left, arm_home_right, arm_home_right_joint2_fixed],
        )
    )

    lift_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["lift_controller", "--switch-timeout", "30.0"],
    )
    delayed_lift_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_spawner,
            on_start=[lift_controller_spawner],
        )
    )

    # --------------------------------------------------------------------------
    # 5. Point Cloud Self-Filters (Collision Geometry Filtering)
    # --------------------------------------------------------------------------
    unitree_self_filter_config = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "lidar",
        "unitree_self_filter.yaml",
    )
    unitree_self_filter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_self_filter"),
                "launch",
                "self_filter.launch.py",
            )
        ),
        launch_arguments={
            "robot_description": robot_description_content,
            "filter_config": unitree_self_filter_config,
            "in_pointcloud_topic": "/unilidar/cloud",
            "out_pointcloud_topic": "/unilidar/cloud/filtered",
            "lidar_sensor_type": "0",
            "use_sim_time": use_sim_time,
        }.items(),
        condition=IfCondition(use_unitree_lidar),
    )

    zed_self_filter_config = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "camera",
        "zed2i_self_filter.yaml",
    )
    zed_self_filter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_self_filter"),
                "launch",
                "self_filter.launch.py",
            )
        ),
        launch_arguments={
            "robot_description": robot_description_content,
            "filter_config": zed_self_filter_config,
            "in_pointcloud_topic": "/zed_head/zed_node/point_cloud/cloud_registered",
            "out_pointcloud_topic": "/zed_head/zed_node/point_cloud/cloud_registered/filtered",
            "lidar_sensor_type": "1",
            "use_sim_time": use_sim_time,
        }.items(),
        condition=IfCondition(use_zed),
    )

    # --------------------------------------------------------------------------
    # 6. Navigation Support Nodes
    # --------------------------------------------------------------------------
    current_pose_publisher = Node(
        package=package_name,
        executable="current_pose_publisher.py",
        name="current_pose_publisher",
        output="screen",
        parameters=[
            {"source_frame": "map"},
            {"target_frame": "base_footprint"},
            {"publish_rate": 10.0},
            {"topic_name": "current_pose"},
            {"use_sim_time": use_sim_time},
        ],
    )

    walkie_tf_server = Node(
        package="walkie_tf",
        executable="tf_server",
        name="walkie_tf_server",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_tf_server),
    )

    # --------------------------------------------------------------------------
    # 7. Perception Services
    # --------------------------------------------------------------------------
    cloud_to_map = Node(
        package="walkie_perception",
        executable="cloud_to_map",
        name="cloud_to_map",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(
            PythonExpression(["'", use_zed, "' == 'true' and '", use_perception, "' == 'true'"])
        ),
    )

    ob_pose_service = Node(
        package="walkie_perception",
        executable="ob_pose_service_cpp",
        name="ob_pose_service_cpp",
        output="screen",
        parameters=[{
            "depth_topic": "/zed_head/zed_node/depth/depth_registered",
            "info_topic": "/zed_head/zed_node/depth/camera_info",
            "target_frame": "map",
            "use_sim_time": use_sim_time,
        }],
        condition=IfCondition(
            PythonExpression(["'", use_zed, "' == 'true' and '", use_perception, "' == 'true'"])
        ),
    )

    # --------------------------------------------------------------------------
    # 8. Web Bridges & Visualization
    # --------------------------------------------------------------------------
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(pkg_rosbridge_server, "launch", "rosbridge_websocket_launch.xml")
        ),
        launch_arguments={"delay_between_messages": "0.0"}.items(),
        condition=IfCondition(use_rosbridge),
    )

    foxglove_bridge_cmd = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"send_buffer_limit": 50000000},
            {"address": "0.0.0.0"},
        ],
        condition=IfCondition(use_foxglove),
    )

    # --------------------------------------------------------------------------
    # 9. MoveIt 2 (move_group + MoveIt RViz Motion Planning UI)
    # --------------------------------------------------------------------------
    pkg_moveit = get_package_share_directory("openarm_bimanual_moveit_config")
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit, "launch", "moveit_only.launch.py")
        ),
        launch_arguments={
            "hardware_type": "isaac",
            "use_sim_time": use_sim_time,
            "right_joint2_fixed": right_joint2_fixed,
            "use_rviz": use_rviz,
        }.items(),
        condition=IfCondition(use_moveit),
    )

    # When use_moveit is true, moveit_only launches RViz configured with moveit.rviz.
    # Otherwise, bringup launches default.rviz.
    rviz_condition = IfCondition(
        PythonExpression(["'", use_rviz, "' == 'true' and '", use_moveit, "' != 'true'"])
    )
    rviz_config_path = PathJoinSubstitution([
        get_package_share_directory(package_name), "config", "rviz", "default.rviz"
    ])
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=rviz_condition,
    )

    # --------------------------------------------------------------------------
    # Build Launch Description
    # --------------------------------------------------------------------------
    ld = LaunchDescription()

    # Arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_robot_model)
    ld.add_action(declare_ros2_control)
    ld.add_action(declare_use_zed)
    ld.add_action(declare_use_unitree_lidar)
    ld.add_action(declare_use_arm)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_right_joint2_fixed)
    ld.add_action(declare_unitree_cloud_frame)
    ld.add_action(declare_unitree_imu_frame)
    ld.add_action(declare_use_rosbridge)
    ld.add_action(declare_use_foxglove)
    ld.add_action(declare_use_perception)
    ld.add_action(declare_use_tf_server)
    ld.add_action(declare_use_moveit)

    # RSP & Twist Mux
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(twist_mux)

    # Controller Manager & Spawners
    ld.add_action(controller_manager_spawner)
    ld.add_action(delayed_joint_broad_spawner)
    ld.add_action(delayed_omni_controller_spawner)
    ld.add_action(delayed_servo_controller_spawner)
    ld.add_action(delayed_head_servo_init)
    ld.add_action(delayed_arm_trajectory_spawner)
    ld.add_action(delayed_arm_gripper_spawner)
    ld.add_action(delayed_arm_home)
    ld.add_action(delayed_lift_controller_spawner)

    # Self-filters
    ld.add_action(unitree_self_filter)
    ld.add_action(zed_self_filter)

    # Navigation & Perception
    ld.add_action(current_pose_publisher)
    ld.add_action(walkie_tf_server)
    ld.add_action(cloud_to_map)
    ld.add_action(ob_pose_service)

    # MoveIt
    ld.add_action(moveit_launch)

    # Bridges & RViz
    ld.add_action(rosbridge_launch)
    ld.add_action(foxglove_bridge_cmd)
    ld.add_action(rviz2)

    return ld
