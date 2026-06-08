#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "robot_bringup"
    package_dir = os.path.join(get_package_share_directory(package_name))
    pkg_rosbridge_server = get_package_share_directory("rosbridge_server")
    description_package_name = "walkie_description"

    default_robot = os.path.join(get_package_share_directory(description_package_name),
                                 'robots',
                                 'gz_walkie.urdf.xacro'
                                 )

    # Launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    robot_model = LaunchConfiguration("robot_model", default=default_robot)
    ros2_control = LaunchConfiguration("ros2_control", default="real_robot")
    use_zed = LaunchConfiguration("use_zed", default="true")
    use_unitree_lidar = LaunchConfiguration("use_unitree_lidar", default="true")
    use_arm = LaunchConfiguration("use_arm", default="true")
    use_fake_arm_hardware = LaunchConfiguration("use_fake_arm_hardware", default="false")
    left_can_interface = LaunchConfiguration("left_can_interface", default="can1")
    right_can_interface = LaunchConfiguration("right_can_interface", default="can0")
    unitree_cloud_frame = LaunchConfiguration("unitree_cloud_frame", default="unitree4d_l2_imu_initial")
    unitree_imu_frame = LaunchConfiguration("unitree_imu_frame", default="unitree4d_l2_imu")

    custom_zed_params_path = os.path.join(
        get_package_share_directory(package_name), "config", "camera", "zed2i.yaml"
    )

    declare_use_zed = DeclareLaunchArgument(
        "use_zed", default_value="true", description="Whether to use ZED camera"
    )
    declare_use_arm = DeclareLaunchArgument(
        "use_arm", default_value="true", description="Whether to bring up the OpenArm"
    )
    declare_use_fake_arm_hardware = DeclareLaunchArgument(
        "use_fake_arm_hardware",
        default_value="false",
        description="Use mock hardware for the arm (false = real CAN hardware)",
    )
    declare_left_can_interface = DeclareLaunchArgument(
        "left_can_interface", default_value="can1", description="CAN interface for the left arm"
    )
    declare_right_can_interface = DeclareLaunchArgument(
        "right_can_interface", default_value="can0", description="CAN interface for the right arm"
    )
    declare_use_unitree_lidar = DeclareLaunchArgument(
        "use_unitree_lidar",
        default_value="true",
        description="Launch the Unitree lidar node as part of the real omnibot bringup",
    )
    declare_unitree_cloud_frame = DeclareLaunchArgument(
        "unitree_cloud_frame",
        default_value="unitree4d_l2_imu_initial",
        description="Frame ID for the Unitree lidar point cloud",
    )
    declare_unitree_imu_frame = DeclareLaunchArgument(
        "unitree_imu_frame",
        default_value="unitree4d_l2_imu",
        description="IMU frame ID used by the Unitree lidar node",
    )

    robot_description_content = Command(
        [
            "xacro ",
            default_robot,
            " ros2_control:=",
            ros2_control,
            " use_zed:=",
            use_zed,
            " use_arm:=",
            use_arm,
            " use_fake_arm_hardware:=",
            use_fake_arm_hardware,
            " left_can_interface:=",
            left_can_interface,
            " right_can_interface:=",
            right_can_interface,
        ]
    )

    twist_mux_params = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "twist_mux",
        "twist_mux.yml",
    )
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[
            twist_mux_params,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[("/cmd_vel_out", "/omni_wheel_drive_controller/cmd_vel")],
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                "launch",
                "robot_state_publisher.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "robot_model": robot_model,
            "ros2_control": ros2_control,
            "use_zed": use_zed,
        }.items(),
    )

    controllers_config = os.path.join(
        get_package_share_directory(description_package_name),
        "config",
        "ros2_controller",
        "real_controllers.yaml",
    )
    controller_manager_spawner = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": ParameterValue(robot_description_content, value_type=str)},
            controllers_config,
        ],
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

    # Dual lidar launch (Hokuyo + Lakibeam)
    dual_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                "launch",
                "dual_lidar_hokuyo_lakibeam.launch.py",
            )
        ),
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

    # Send head to 0.0 once the servo controller is active (publishes once then exits)
    head_servo_init_pub = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--times', '1',
            '/head_servo_controller/commands',
            'std_msgs/msg/Float64MultiArray',
            '{"layout": {"dim": [{"label": "", "size": 1, "stride": 1}], "data_offset": 0}, "data": [0.25]}',
        ],
        output='screen',
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

    # Startup pose: drive each arm to a home configuration instead of all-zeros.
    # Left:  joint1 = +15 deg, joint4 = +15 deg, rest 0   (0.2618 rad)
    # Right: joint1 = -15 deg, joint4 = +15 deg, rest 0
    # Sent once the trajectory controllers are active; time_from_start ramps the
    # motion (use a longer time for the first power-on if joints start far away).
    arm_home_left = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--times', '1',
            '/left_joint_trajectory_controller/joint_trajectory',
            'trajectory_msgs/msg/JointTrajectory',
            '{joint_names: [openarm_left_joint1, openarm_left_joint2, openarm_left_joint3, '
            'openarm_left_joint4, openarm_left_joint5, openarm_left_joint6, openarm_left_joint7], '
            'points: [{positions: [0.2618, 0.0, 0.0, 0.2618, 0.0, 0.0, 0.0], '
            'time_from_start: {sec: 5, nanosec: 0}}]}',
        ],
        output='screen',
        condition=IfCondition(use_arm),
    )
    arm_home_right = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--times', '1',
            '/right_joint_trajectory_controller/joint_trajectory',
            'trajectory_msgs/msg/JointTrajectory',
            '{joint_names: [openarm_right_joint1, openarm_right_joint2, openarm_right_joint3, '
            'openarm_right_joint4, openarm_right_joint5, openarm_right_joint6, openarm_right_joint7], '
            'points: [{positions: [-0.2618, 0.0, 0.0, 0.2618, 0.0, 0.0, 0.0], '
            'time_from_start: {sec: 5, nanosec: 0}}]}',
        ],
        output='screen',
        condition=IfCondition(use_arm),
    )

    delayed_arm_home = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_trajectory_spawner,
            on_exit=[arm_home_left, arm_home_right],
        )
    )

    # lift_controller (JointTrajectoryController) is intentionally NOT spawned.
    # The lift_homing_node owns the FollowJointTrajectory action server at
    # lift_controller/follow_joint_trajectory directly, so MoveIt connects to
    # the real motor without the JTC mock-components bridge.

    current_pose_publisher = Node(
        package="robot_navigation",
        executable="current_pose_publisher.py",
        name="current_pose_publisher",
        output="screen",
        parameters=[
            {"source_frame": "map"},
            {"target_frame": "base_footprint"},
            {"publish_rate": 10.0},
            {"topic_name": "current_pose"},
        ],
    )

    unitree_lidar_node = Node(
        package="unitree_lidar_ros2",
        executable="unitree_lidar_ros2_node",
        name="unitree_lidar_ros2_node",
        output="screen",
        parameters=[
            {"initialize_type": 2},
            {"work_mode": 5},
            {"use_system_timestamp": True},
            {"range_min": 0.0},
            {"range_max": 100.0},
            {"cloud_scan_num": 18},
            {"serial_port": '/dev/ttyACM0'},
            {"baudrate": 4000000},
            {"lidar_port": 6101},
            {"lidar_ip": '10.0.0.103'},
            {"local_port": 6201},
            {"local_ip": '10.0.0.201'},
            {"cloud_frame": unitree_cloud_frame},
            {"cloud_topic": "unilidar/cloud"},
            {"imu_frame": unitree_imu_frame},
            {"imu_topic": "unilidar/imu"},
        ],
        condition=IfCondition(use_unitree_lidar),
        remappings=[
            ('/tf', '/tf_unitree_ignore'),
            ('/tf_static', '/tf_static_unitree_ignore')
        ]
    )

    # Self-filter the Unitree lidar cloud using the robot's URDF collision geometry.
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

    # Depth camera launch (RealsenseD415)
    realsense_camera_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        output="screen",
        # Set the parameter pointcloud.enable to true
        parameters=[
            {"pointcloud.enable": True},
            {"camera_name": "bottom"},
        ],
    )

    # ZED Camera Launch
    zed_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("zed_wrapper"), "launch", "zed_camera.launch.py"]
            )
        ),
        launch_arguments={
            "camera_model": "zed2i",
            "camera_name": "zed_head",
            "base_frame": "zed_head_camera_link",
            "publish_urdf": "false",
            "publish_tf": "false",
            "use_sim_time": use_sim_time,
            "ros_params_override_path": custom_zed_params_path,
        }.items(),
        condition=IfCondition(use_zed),
    )

    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                pkg_rosbridge_server, "launch", "rosbridge_websocket_launch.xml"
            )
        ),
        launch_arguments={
            "delay_between_messages": "0.0",
        }.items(),
    )

    foxgloveBridge_cmd = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"send_buffer_limit": 50000000},  # 50 MB
            {"address": "0.0.0.0"},
        ],
    )

    rviz_config_path = PathJoinSubstitution([
                get_package_share_directory(package_name), 'config', 'rviz', 'default.rviz'
            ])
    rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': use_sim_time}]
        )

    # Relay lift_homing_node's SW-trajectory joint state into /joint_states so
    # robot_state_publisher (and RViz) reflect real lift motion.
    # joint_broad is configured to exclude lift_joint (real_controllers.yaml)
    # so there is no duplicate/conflicting publisher on that joint.
    lift_joint_state_relay = Node(
        package='topic_tools',
        executable='relay',
        name='lift_joint_state_relay',
        arguments=['lift/joint_states', '/joint_states'],
        output='screen',
    )

    walkie_tf_server = Node(
        package='walkie_tf',
        executable='tf_server',
        name='walkie_tf_server',
        output='screen',
    )

    ld = LaunchDescription()
    # Add launch arguments
    ld.add_action(declare_use_zed)
    ld.add_action(declare_use_arm)
    ld.add_action(declare_use_fake_arm_hardware)
    ld.add_action(declare_left_can_interface)
    ld.add_action(declare_right_can_interface)
    ld.add_action(declare_use_unitree_lidar)
    ld.add_action(declare_unitree_cloud_frame)
    ld.add_action(declare_unitree_imu_frame)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(twist_mux)
    ld.add_action(controller_manager_spawner)
    ld.add_action(delayed_omni_controller_spawner)
    ld.add_action(delayed_joint_broad_spawner)
    ld.add_action(delayed_arm_trajectory_spawner)
    ld.add_action(delayed_arm_gripper_spawner)
    ld.add_action(delayed_arm_home)
    # delayed_lift_controller_spawner removed — see comment above
    ld.add_action(dual_lidar_launch)
    ld.add_action(delayed_servo_controller_spawner)
    ld.add_action(delayed_head_servo_init)
    ld.add_action(current_pose_publisher)
    ld.add_action(unitree_lidar_node)
    # ld.add_action(unitree_pointcloud_filter)
    ld.add_action(unitree_self_filter)
    # ld.add_action(realsense_camera_node)
    ld.add_action(zed_camera_launch)
    ld.add_action(rosbridge_launch)
    ld.add_action(foxgloveBridge_cmd)
    ld.add_action(rviz2)
    ld.add_action(lift_joint_state_relay)
    ld.add_action(walkie_tf_server)

    return ld
