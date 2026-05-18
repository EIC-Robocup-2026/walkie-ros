#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
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
    unitree_cloud_frame = LaunchConfiguration("unitree_cloud_frame", default="unitree4d_l2_imu_initial")
    unitree_imu_frame = LaunchConfiguration("unitree_imu_frame", default="unitree4d_l2_imu")

    custom_zed_params_path = os.path.join(
        get_package_share_directory(package_name), "config", "camera", "zed2i.yaml"
    )

    declare_use_zed = DeclareLaunchArgument(
        "use_zed", default_value="true", description="Whether to use ZED camera"
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
            {"robot_description", robot_description_content},
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

    current_pose_publisher = Node(
        package="robot_navigation",
        executable="current_pose_publisher.py",
        name="current_pose_publisher",
        output="screen",
        parameters=[
            {"source_frame": "map"},
            {"target_frame": "base_link"},
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
            {"work_mode": 0},
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
            "camera_name": "zed",
            "base_frame": "zed_camera_link",
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

    ld = LaunchDescription()
    # Add launch arguments
    ld.add_action(declare_use_zed)
    ld.add_action(declare_use_unitree_lidar)
    ld.add_action(declare_unitree_cloud_frame)
    ld.add_action(declare_unitree_imu_frame)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(twist_mux)
    ld.add_action(controller_manager_spawner)
    ld.add_action(delayed_omni_controller_spawner)
    ld.add_action(delayed_joint_broad_spawner)
    # ld.add_action(dual_lidar_launch)
    ld.add_action(delayed_servo_controller_spawner)
    ld.add_action(current_pose_publisher)
    ld.add_action(unitree_lidar_node)
    # ld.add_action(realsense_camera_node)
    # ld.add_action(zed_camera_launch)
    ld.add_action(rosbridge_launch)
    ld.add_action(foxgloveBridge_cmd)
    ld.add_action(rviz2)

    return ld
