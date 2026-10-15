#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_name = "robot_bringup"
    description_package_name = "walkie_description"

    default_robot = os.path.join(
        get_package_share_directory(description_package_name),
        "robots",
        "gz_walkie.urdf.xacro",
    )

    # Launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    robot_model = LaunchConfiguration("robot_model", default=default_robot)
    ros2_control = LaunchConfiguration("ros2_control", default="real_robot")
    use_rviz = LaunchConfiguration("use_rviz", default="false")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_robot_model = DeclareLaunchArgument(
        "robot_model",
        default_value=default_robot,
        description="Path of robot urdf/xacro file to use",
    )
    declare_ros2_control = DeclareLaunchArgument(
        "ros2_control",
        default_value="real_robot",
        description="ros2_control configuration type (e.g., real_robot, mock)",
    )
    use_rviz = LaunchConfiguration("use_rviz", default="false")
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Launch RViz2 as part of drivebase bringup",
    )

    use_foxglove = LaunchConfiguration("use_foxglove", default="true")
    declare_use_foxglove = DeclareLaunchArgument(
        "use_foxglove",
        default_value="true",
        description="Launch Foxglove WebSocket bridge (port 8765)",
    )

    use_head_beep = LaunchConfiguration("use_head_beep", default="true")
    declare_use_head_beep = DeclareLaunchArgument(
        "use_head_beep",
        default_value="true",
        description="Chirp like R2D2 when the head servo is touched/pressed",
    )

    head_pos_err_threshold = LaunchConfiguration("head_pos_err_threshold", default="0.08")
    declare_head_pos_err_threshold = DeclareLaunchArgument(
        "head_pos_err_threshold",
        default_value="0.03",
        description="Position deviation threshold (rad) to trigger head touch beep (default: 0.08 rad / ~4.6 deg)",
    )

    position_only = LaunchConfiguration("position_only", default="true")
    declare_position_only = DeclareLaunchArgument(
        "position_only",
        default_value="true",
        description="Detect head touch using position error only (ignore effort/load)",
    )

    head_effort_threshold = LaunchConfiguration("head_effort_threshold", default="60.0")
    declare_head_effort_threshold = DeclareLaunchArgument(
        "head_effort_threshold",
        default_value="60.0",
        description="Load change threshold (counts) to trigger head touch beep (default: 60.0)",
    )

    head_trigger_duration = LaunchConfiguration("head_trigger_duration", default="0.12")
    declare_head_trigger_duration = DeclareLaunchArgument(
        "head_trigger_duration",
        default_value="0.12",
        description="Duration (seconds) deflection must persist to prevent shake false-positives (default: 0.12s)",
    )

    # Robot Description (URDF): arms mock components, cameras disabled
    robot_description_content = Command(
        [
            "xacro ",
            robot_model,
            " ros2_control:=",
            ros2_control,
            " use_arm:=false",
            " use_fake_arm_hardware:=true",
            " use_zed:=false",
            " dual_lidar:=false",
        ]
    )

    # Twist Mux: priority-based velocity command multiplexer
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

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": ParameterValue(
                    robot_description_content, value_type=str
                ),
            }
        ],
    )

    # Controller Manager
    controllers_config = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "ros2_controller",
        "drivebase_controllers.yaml",
    )
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(
                    robot_description_content, value_type=str
                )
            },
            controllers_config,
        ],
    )

    # Omni Wheel Drive Controller Spawner
    omni_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["omni_wheel_drive_controller", "--switch-timeout", "30.0"],
    )

    delayed_omni_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[omni_controller_spawner],
        )
    )

    # Joint State Broadcaster Spawner (Wheel & Head Joints)
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--switch-timeout", "30.0"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    # Head Servo Controller Spawner
    servo_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["head_servo_controller", "--switch-timeout", "30.0"],
    )

    delayed_servo_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[servo_controller_spawner],
        )
    )

    # Initialize head position to 0.25 rad once head servo controller activates
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

    # TF Server
    walkie_tf_server = Node(
        package="walkie_tf",
        executable="tf_server",
        name="walkie_tf_server",
        output="screen",
    )

    # Optional RViz2
    rviz_config_path = PathJoinSubstitution(
        [get_package_share_directory(package_name), "config", "rviz", "default.rviz"]
    )
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    # Foxglove WebSocket bridge for browser/desktop visualization (port 8765)
    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"send_buffer_limit": 50000000},  # 50 MB
            {"address": "0.0.0.0"},
        ],
        condition=IfCondition(use_foxglove),
    )

    # R2D2-style beeps when head camera / servo is pressed
    head_touch_beep_node = Node(
        package="robot_bringup",
        executable="head_touch_beep_node.py",
        name="head_touch_beep",
        output="screen",
        parameters=[
            {
                "position_only": ParameterValue(position_only, value_type=bool),
                "position_error_threshold": ParameterValue(head_pos_err_threshold, value_type=float),
                "effort_delta_threshold": ParameterValue(head_effort_threshold, value_type=float),
                "trigger_duration": ParameterValue(head_trigger_duration, value_type=float),
            }
        ],
        condition=IfCondition(use_head_beep),
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_robot_model)
    ld.add_action(declare_ros2_control)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_use_foxglove)
    ld.add_action(declare_use_head_beep)
    ld.add_action(declare_position_only)
    ld.add_action(declare_head_pos_err_threshold)
    ld.add_action(declare_head_effort_threshold)
    ld.add_action(declare_head_trigger_duration)

    ld.add_action(robot_state_publisher)
    ld.add_action(twist_mux)
    ld.add_action(controller_manager)
    ld.add_action(delayed_omni_controller_spawner)
    ld.add_action(delayed_joint_broad_spawner)
    ld.add_action(delayed_servo_controller_spawner)
    ld.add_action(delayed_head_servo_init)
    ld.add_action(head_touch_beep_node)
    ld.add_action(walkie_tf_server)
    ld.add_action(foxglove_bridge)
    ld.add_action(rviz2)

    return ld
