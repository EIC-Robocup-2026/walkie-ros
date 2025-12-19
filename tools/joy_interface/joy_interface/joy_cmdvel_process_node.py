#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class JoyToTwist(Node):
    """
    Subscribes to sensor_msgs/Joy and publishes a Twist message to /cmd_vel.
    This version supports omnidirectional control and implements **direct proportional
    control** with **NO acceleration or deceleration limits (Instant response).**
    
    Control Mapping:
    - Left Stick Up/Down (Axis 1): linear.x (Forward/Backward)
    - Left Stick Left/Right (Axis 0): linear.y (Strafe Left/Right)
    - Right Stick Left/Right (Axis 3): angular.z (Yaw)
    """
    def __init__(self):
        super().__init__('joy_teleop_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber_ = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        self.get_logger().info('JoyToTwist Omni Node started with DIRECT proportional control. Publishing to /cmd_vel.')

        # --- Configuration Parameters ---
        self.declare_parameter('max_linear_vel', 5.0)
        self.declare_parameter('max_angular_vel', 5.0)
        self.declare_parameter('linear_axis', 1)
        self.declare_parameter('angular_axis', 3)
        self.declare_parameter('strafe_axis', 0)
        
        self.max_linear_vel = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        self.max_angular_vel = self.get_parameter('max_angular_vel').get_parameter_value().double_value
        self.linear_axis = self.get_parameter('linear_axis').get_parameter_value().integer_value
        self.angular_axis = self.get_parameter('angular_axis').get_parameter_value().integer_value
        self.strafe_axis = self.get_parameter('strafe_axis').get_parameter_value().integer_value
        
        # REMOVED: All acceleration/deceleration rate parameters.
        # REMOVED: self.last_twist and self.last_time state variables.

    def joy_callback(self, msg):
        """
        Processes incoming Joy messages and converts them to Twist messages
        using direct proportional scaling (no acceleration limiting).
        """
        
        # 0. Time Tracking is REMOVED as it is no longer needed.

        # 1. Calculate and Publish Velocities (Direct Mapping)
        cmd_twist = Twist()
        
        raw_linear_x = msg.axes[self.linear_axis] 
        raw_linear_y = msg.axes[self.strafe_axis]
        raw_angular_z = msg.axes[self.angular_axis]

        # Apply Max Velocity Constraints to get the final commanded speed
        # The output speed is instantly proportional to the joystick axis position.
        cmd_twist.linear.x = raw_linear_x * self.max_linear_vel
        cmd_twist.linear.y = raw_linear_y * self.max_linear_vel
        cmd_twist.angular.z = raw_angular_z * self.max_angular_vel
        
        # 2. Publish
        self.publisher_.publish(cmd_twist)
        
        # REMOVED: Update self.last_twist = new_twist
    
    # REMOVED: The entire _rate_limit helper function.

def main(args=None):
    rclpy.init(args=args)
    joy_to_twist = JoyToTwist()
    rclpy.spin(joy_to_twist)
    joy_to_twist.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()