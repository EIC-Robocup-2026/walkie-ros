#!/usr/bin/env python3
"""Run Walkie in MuJoCo with ros2_control (mujoco_ros2_control).

Starts robot_state_publisher with the URDF expanded for ros2_control:=mujoco,
mujoco_ros2_control's ros2_control_node (simulator and controller manager in one
process), the controller spawners, twist_mux, dual_laser_merger for /scan, the
walkie_tf server, and RViz. Every node runs on simulated time from /clock.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_robot_bringup = get_package_share_directory("robot_bringup")
    pkg_walkie_description = get_package_share_directory("walkie_description")

    robot_xacro = os.path.join(pkg_walkie_description, "robots", "gz_walkie.urdf.xacro")
    controllers_yaml = os.path.join(
        pkg_walkie_description, "config", "ros2_controller", "mujoco_controllers.yaml"
    )
    plugins_yaml = os.path.join(pkg_robot_bringup, "config", "mujoco", "mujoco_plugins.yaml")
    twist_mux_yaml = os.path.join(pkg_robot_bringup, "config", "twist_mux", "twist_mux.yml")

    mujoco_model = LaunchConfiguration("mujoco_model")
    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

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
        default_value=os.path.join(pkg_robot_bringup, "config", "rviz", "mujoco.rviz"),
        description="RViz config file",
    )

    # robot_state_publisher.launch.py does not forward mujoco_model or
    # mujoco_headless to xacro, so the URDF is expanded here.
    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                robot_xacro,
                " ros2_control:=mujoco",
                " mujoco_model:=",
                mujoco_model,
                " mujoco_headless:=",
                headless,
            ]
        ),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    # Simulator and controller manager. It reads the URDF from /robot_description
    # and the MJCF path from the URDF hardware block, and publishes /clock.
    # Closing the MuJoCo viewer ends the whole launch.
    mujoco_ros2_control = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        output="screen",
        emulate_tty=True,
        parameters=[{"use_sim_time": True}, controllers_yaml, plugins_yaml],
        remappings=[("/imu_sensor_broadcaster/imu", "/imu")],
        on_exit=Shutdown(),
    )

    # Spawners wait for the controller manager services, so no start delay is needed.
    base_controllers_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_broad",
            "omni_wheel_drive_controller",
            "imu_sensor_broadcaster",
            "--param-file",
            controllers_yaml,
        ],
    )
    upper_body_controllers_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "lift_controller",
            "head_servo_controller",
            "left_joint_trajectory_controller",
            "right_joint_trajectory_controller",
            "left_gripper_controller",
            "right_gripper_controller",
            "--param-file",
            controllers_yaml,
        ],
    )

    # Same velocity mux as Gazebo and the real robot: cmd_vel_nav (Nav2),
    # cmd_vel_joy, cmd_vel_key and cmd_vel in, TwistStamped out to the omni drive.
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        output="screen",
        parameters=[twist_mux_yaml, {"use_sim_time": True}],
        remappings=[("/cmd_vel_out", "/omni_wheel_drive_controller/cmd_vel")],
    )

    # Same merger parameters as the real robot (dual_lidar_hokuyo_lakibeam.launch.py),
    # plus publish_on_empty_cloud: in the empty default scene neither lidar has a
    # return in range, and without /scan Nav2's collision_monitor stops the robot.
    # The real robot keeps the default (no scan when a lidar has no valid return).
    dual_laser_merger = Node(
        package="dual_laser_merger",
        executable="dual_laser_merger_node",
        name="dual_laser_merger",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "laser_1_topic": "/front_lidar",
                "laser_2_topic": "/back_lidar",
                "merged_scan_topic": "/merged_lidar",
                "target_frame": "merged_lidar_link",
                "laser_1_x_offset": 0.0,
                "laser_1_y_offset": 0.0,
                "laser_1_yaw_offset": 0.0,
                "laser_2_x_offset": 0.0,
                "laser_2_y_offset": 0.0,
                "laser_2_yaw_offset": 0.0,
                "tolerance": 0.01,
                "queue_size": 5,
                "angle_increment": 0.001,
                "scan_time": 0.067,
                "range_min": 0.3,
                "range_max": 25.0,
                "min_height": -1.0,
                "max_height": 1.0,
                "angle_min": -3.141592654,
                "angle_max": 3.141592654,
                "inf_epsilon": 1.0,
                "use_inf": True,
                "allowed_radius": 0.45,
                "enable_shadow_filter": False,
                "enable_average_filter": False,
                "publish_on_empty_cloud": True,
            }
        ],
        remappings=[("/merged_lidar", "/scan")],
    )

    walkie_tf_server = Node(
        package="walkie_tf",
        executable="tf_server",
        name="walkie_tf_server",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
    )

    ld = LaunchDescription()

    ld.add_action(declare_mujoco_model_arg)
    ld.add_action(declare_headless_arg)
    ld.add_action(declare_use_rviz_arg)
    ld.add_action(declare_rviz_config_arg)

    ld.add_action(robot_state_publisher)
    ld.add_action(mujoco_ros2_control)
    ld.add_action(base_controllers_spawner)
    ld.add_action(upper_body_controllers_spawner)
    ld.add_action(twist_mux)
    ld.add_action(dual_laser_merger)
    ld.add_action(walkie_tf_server)
    ld.add_action(rviz)

    return ld
