import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # ========================================================================
    # 1. SETUP PATHS & ARGS
    # ========================================================================
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_walkie_description = get_package_share_directory('walkie_description')
    pkg_moveit_config = get_package_share_directory('moveit_config')

    # CHANGED: Point to the Left Arm URDF
    xacro_path = os.path.join(pkg_walkie_description, 'robots', 'left-walkie-arm.urdf.xacro')
    
    rviz_config_path = os.path.join(pkg_moveit_config, 'config', 'moveit.rviz')
  
    # CHANGED: Use the standard controller config (ensure this file exists and has left_arm_controller)
    controller_config = os.path.join(pkg_walkie_description, 'config', 'ros2_controller', 'ros2_controllers.yaml')

    # Launch Argument: hardware_type
    hardware_type_arg = DeclareLaunchArgument(
        'hardware_type',
        default_value='mock_components',
        description='Hardware configuration: gazebo, mock_components, real_robot, cubemars, hybrid'
    )
    hardware_type = LaunchConfiguration('hardware_type')

    hardware_config_file_arg = DeclareLaunchArgument(
        'hardware_config_file',
        default_value=os.path.join(get_package_share_directory('walkie_description'), 'config', 'hardware_layout.yaml'),
        description='Path to hardware layout config yaml'
    )
    hardware_config_file = LaunchConfiguration('hardware_config_file')

    # Condition: Are we running Gazebo?
    should_launch_gazebo = PythonExpression(["'", hardware_type, "' == 'gazebo'"])
    use_sim_time = should_launch_gazebo

    # ========================================================================
    # 2. GENERATE ROBOT DESCRIPTION
    # ========================================================================
    robot_description_content = Command([
        'xacro ', xacro_path, 
        ' ros2_control_hardware_type:=', hardware_type,
        ' hardware_config_file:=', hardware_config_file
    ])
    
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    # ========================================================================
    # 3. DEFINE NODES
    # ========================================================================

    # A. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # B. ROS 2 Control Node (Only for Real/Mock Robot)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config],
        output="both",
        condition=UnlessCondition(should_launch_gazebo)
    )

    # C. Gazebo Simulation
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r --physics-engine gz-physics-bullet-featherstone-plugin'
        }.items(),
        condition=IfCondition(should_launch_gazebo)
    )

    # D. ROS-GZ Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/rgb_camera@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        output='screen',
        condition=IfCondition(should_launch_gazebo)
    )

    # E. Spawn Robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'left_walkie_arm', # CHANGED: Naming it appropriately
                   '-x', '0.0', '-y', '0.0', '-z', '0.05'],
        output='screen',
        condition=IfCondition(should_launch_gazebo)
    )

    # G. MoveIt Group
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit_config, 'launch', 'move_group.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # ========================================================================
    # H. RViz
    # ========================================================================
    # CHANGED: Ensure this matches the name in your SRDF file
    moveit_config = MoveItConfigsBuilder("left_walkie_arm", package_name="moveit_config").to_moveit_configs()
    
    rviz_parameters = [
        robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.planning_pipelines,
        moveit_config.joint_limits,
        {'use_sim_time': use_sim_time}
    ]
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=rviz_parameters,
        output='screen'
    )

    # ========================================================================
    # 4. CONTROLLER SPAWNERS (LEFT ONLY)
    # ========================================================================
    
    # 1. Joint States
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        condition=UnlessCondition(should_launch_gazebo) 
    )

    # 2. Left Arm
    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_controller"],
        condition=UnlessCondition(should_launch_gazebo)
    )

    # 3. Left Gripper
    # Ensure this name matches what is in your ros2_controllers.yaml
    left_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_hand_controller"], 
        condition=UnlessCondition(should_launch_gazebo)
    )

    # 4. Gazebo Event Handler (For Sim Mode)
    gazebo_spawner_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"]),
                Node(package="controller_manager", executable="spawner", arguments=["left_arm_controller"]),
                Node(package="controller_manager", executable="spawner", arguments=["left_hand_controller"]),
            ],
        ),
        condition=IfCondition(should_launch_gazebo)
    )

    # ========================================================================
    # 5. BUILD LAUNCH DESCRIPTION
    # ========================================================================
    ld = LaunchDescription()

    # Args
    ld.add_action(hardware_type_arg)
    ld.add_action(hardware_config_file_arg)

    # Core Nodes
    ld.add_action(robot_state_publisher)
    ld.add_action(control_node)
    
    # Spawners (Real/Mock)
    ld.add_action(joint_state_spawner)
    ld.add_action(left_arm_controller_spawner)
    ld.add_action(left_gripper_controller_spawner)

    # Gazebo
    ld.add_action(gazebo)
    ld.add_action(bridge)
    ld.add_action(spawn_entity)
    ld.add_action(gazebo_spawner_callback)

    # Visualization
    ld.add_action(move_group)
    ld.add_action(rviz)

    return ld