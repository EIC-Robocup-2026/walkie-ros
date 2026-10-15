from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launches the full joystick-to-cmd_vel stack for an omnidirectional robot:
    1. joy_linux_node (Driver) -> Publishes /joy
    2. joy_teleop (Translator) -> Subscribes to /joy, Publishes /cmd_vel_joy (Omni-capable, smoothed)
    """
    dev = LaunchConfiguration('dev', default='/dev/input/js1')
    max_linear_vel = LaunchConfiguration('max_linear_vel', default='0.85')
    max_angular_vel = LaunchConfiguration('max_angular_vel', default='1.0')
    turbo_button = LaunchConfiguration('turbo_button', default='5')
    turbo_linear_multiplier = LaunchConfiguration('turbo_linear_multiplier', default='2.0')
    turbo_angular_multiplier = LaunchConfiguration('turbo_angular_multiplier', default='1.5')

    declare_dev = DeclareLaunchArgument(
        'dev',
        default_value='/dev/input/js2',
        description='Path to joystick device (e.g. /dev/input/js2'
    )
    declare_max_linear_vel = DeclareLaunchArgument(
        'max_linear_vel',
        default_value='2.0',
        description='Max linear velocity (m/s)'
    )
    declare_max_angular_vel = DeclareLaunchArgument(
        'max_angular_vel',
        default_value='1.0',
        description='Max angular velocity (rad/s)'
    )
    declare_turbo_button = DeclareLaunchArgument(
        'turbo_button',
        default_value='5',
        description='Button index for Turbo mode (5 = R1 on DualSense / RB on Xbox)'
    )
    declare_turbo_linear_multiplier = DeclareLaunchArgument(
        'turbo_linear_multiplier',
        default_value='5.0',
        description='Multiplier for linear velocity when turbo is active'
    )
    declare_turbo_angular_multiplier = DeclareLaunchArgument(
        'turbo_angular_multiplier',
        default_value='1.5',
        description='Multiplier for angular velocity when turbo is active'
    )

    # 1. joy_linux_node: Reads raw input from the specified /dev/input/jsX device
    joy_driver_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joystick_driver',
        output='screen',
        parameters=[
            {'dev': dev},
        ]
    )

    # 2. Custom Translator Node: Converts /joy to /cmd_vel_joy with acceleration/deceleration limiting
    custom_teleop_node = Node(
        package='joy_interface',
        executable='joy_teleop_smooth',
        name='joy_teleop_translator',
        output='screen',
        parameters=[
            {'max_linear_vel': max_linear_vel},
            {'max_angular_vel': max_angular_vel},
            {'linear_axis': 1},          # Left Stick Vertical    -> linear.x (FWD/BWD)
            {'angular_axis': 3},         # Right Stick Horizontal  -> angular.z (YAW)
            {'strafe_axis': 0},          # Left Stick Horizontal   -> linear.y (Strafe)
            {'cancel_nav_button': 0},    # A button (Xbox) / X button (PS) -> Cancel Nav2 goals
            {'turbo_button': turbo_button},  # R1 button (PS) / RB (Xbox) -> Turbo boost
            {'turbo_linear_multiplier': turbo_linear_multiplier},
            {'turbo_angular_multiplier': turbo_angular_multiplier},
        ]
    )

    return LaunchDescription([
        declare_dev,
        declare_max_linear_vel,
        declare_max_angular_vel,
        declare_turbo_button,
        declare_turbo_linear_multiplier,
        declare_turbo_angular_multiplier,
        joy_driver_node,
        custom_teleop_node,
    ])
