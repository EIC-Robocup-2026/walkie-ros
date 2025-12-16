from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launches the full joystick-to-cmd_vel stack for an omnidirectional robot:
    1. joy_linux_node (Driver) -> Publishes /joy
    2. joy_teleop (Translator) -> Subscribes to /joy, Publishes /cmd_vel (Omni-capable)
    """
    # 1. joy_linux_node: Reads raw input from the /dev/input/jsX device
    joy_driver_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joystick_driver',
        output='screen',
        parameters=[
            # Optional driver parameters (like deadzone or device path) can be added here
        ]
    )

    # 2. Custom Translator Node: Converts /joy to /cmd_vel with max velocity
    # The executable 'joy_teleop' is defined in your package's setup.py.
    custom_teleop_node = Node(
        package='joy_interface',
        executable='joy_teleop_smooth',
        name='joy_teleop_translator',
        output='screen',
        parameters=[
            {'max_linear_vel': 0.45},   # Max linear speed (for both X and Y)
            {'max_angular_vel': 1.0},  # Max angular speed
            {'linear_axis': 1},        # Left Stick Vertical (Y-axis) -> linear.x (FWD/BWD)
            {'angular_axis': 3},       # Right Stick Horizontal (X-axis) -> angular.z (YAW)
            {'strafe_axis': 0},        # Left Stick Horizontal (X-axis) -> linear.y (Strafe) <-- UPDATED
        ]
    )
    
    return LaunchDescription([
        joy_driver_node,
        custom_teleop_node,
    ])