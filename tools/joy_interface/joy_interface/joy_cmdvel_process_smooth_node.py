#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class JoyToTwist(Node):
    """
    Subscribes to sensor_msgs/Joy and publishes a Twist message to /cmd_vel.
    This version supports omnidirectional control (linear.x, linear.y, angular.z)
    and includes separate acceleration and deceleration limits for smooth, safe motion.
    
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
        self.get_logger().info('JoyToTwist Omni Node started with separate Accel/Decel limits. Publishing to /cmd_vel.')

        # --- Configuration Parameters ---
        self.declare_parameter('max_linear_vel', 5.0)
        self.declare_parameter('max_angular_vel', 5.0)
        self.declare_parameter('linear_axis', 1)
        self.declare_parameter('angular_axis', 3)
        self.declare_parameter('strafe_axis', 0)
        
        # New: Separate Acceleration and Deceleration limits (max change per second)
        # Linear motion (m/s^2)
        self.declare_parameter('accel_linear_rate', 0.2) 
        self.declare_parameter('decel_linear_rate', 0.8) 
        # Angular motion (rad/s^2)
        self.declare_parameter('accel_angular_rate', 1.0) 
        self.declare_parameter('decel_angular_rate', 4.0) 

        self.max_linear_vel = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        self.max_angular_vel = self.get_parameter('max_angular_vel').get_parameter_value().double_value
        self.linear_axis = self.get_parameter('linear_axis').get_parameter_value().integer_value
        self.angular_axis = self.get_parameter('angular_axis').get_parameter_value().integer_value
        self.strafe_axis = self.get_parameter('strafe_axis').get_parameter_value().integer_value
        
        self.accel_linear_rate = self.get_parameter('accel_linear_rate').get_parameter_value().double_value
        self.decel_linear_rate = self.get_parameter('decel_linear_rate').get_parameter_value().double_value
        self.accel_angular_rate = self.get_parameter('accel_angular_rate').get_parameter_value().double_value
        self.decel_angular_rate = self.get_parameter('decel_angular_rate').get_parameter_value().double_value
        
        # --- State Variables for Acceleration Profile ---
        # Stores the velocity published in the last cycle
        self.last_twist = Twist() 
        # Stores the timestamp of the last update
        self.last_time = self.get_clock().now() 

    def joy_callback(self, msg):
        """
        Processes incoming Joy messages and converts them to Twist messages 
        with acceleration limiting applied.
        """
        
        # 0. Calculate Time Elapsed (dt)
        current_time = self.get_clock().now()
        # Time in seconds (T_current - T_last)
        dt = (current_time - self.last_time).nanoseconds / 1e9 
        self.last_time = current_time 

        # 1. Calculate TARGET Velocities (based on Joystick Input)
        target_twist = Twist()
        
        raw_linear_x = msg.axes[self.linear_axis] 
        raw_linear_y = msg.axes[self.strafe_axis]
        raw_angular_z = msg.axes[self.angular_axis]

        # Apply Max Velocity Constraints to get the full-speed target
        target_twist.linear.x = raw_linear_x * self.max_linear_vel
        target_twist.linear.y = raw_linear_y * self.max_linear_vel
        target_twist.angular.z = raw_angular_z * self.max_angular_vel

        # 2. Apply Acceleration/Deceleration Limiting
        new_twist = Twist()
        
        # --- Linear X (Forward/Backward) ---
        new_twist.linear.x = self._rate_limit(
            current_vel=self.last_twist.linear.x,
            target_vel=target_twist.linear.x,
            dt=dt,
            accel_rate=self.accel_linear_rate,
            decel_rate=self.decel_linear_rate
        )

        # --- Linear Y (Strafe) ---
        new_twist.linear.y = self._rate_limit(
            current_vel=self.last_twist.linear.y,
            target_vel=target_twist.linear.y,
            dt=dt,
            accel_rate=self.accel_linear_rate,
            decel_rate=self.decel_linear_rate
        )

        # --- Angular Z (Yaw) ---
        new_twist.angular.z = self._rate_limit(
            current_vel=self.last_twist.angular.z,
            target_vel=target_twist.angular.z,
            dt=dt,
            accel_rate=self.accel_angular_rate,
            decel_rate=self.decel_angular_rate
        )

        # 3. Publish and Update State
        self.publisher_.publish(new_twist)
        self.last_twist = new_twist
    
    def _rate_limit(self, current_vel, target_vel, dt, accel_rate, decel_rate):
        """
        Limits the change in velocity based on dt. Uses the Decel rate only when 
        the robot is actively braking or slowing down (magnitude decrease).
        """
        delta_vel = target_vel - current_vel
        
        # Determine the rate based on magnitude change
        if abs(target_vel) > abs(current_vel):
            # Magnitude is INCREASING (e.g., 0.0 -> 1.0 or -0.5 -> -1.0). Use the slower ACCEL rate.
            rate = accel_rate
        else:
            # Magnitude is DECREASING (e.g., 1.0 -> 0.0 or -1.0 -> -0.5). Use the faster DECEL rate.
            rate = decel_rate

        # If the target is the same as current, return immediately to avoid division by zero or unnecessary calculations
        if delta_vel == 0:
            return target_vel

        # Apply the Rate Limit
        max_delta = rate * dt
        
        if abs(delta_vel) > max_delta:
            # Limit the change to max_delta
            rate_limited_vel = current_vel + (max_delta if delta_vel > 0 else -max_delta)
        else:
            # Required change is small, use the target velocity immediately
            rate_limited_vel = target_vel
            
        return rate_limited_vel

def main(args=None):
    rclpy.init(args=args)
    joy_to_twist = JoyToTwist()
    rclpy.spin(joy_to_twist)
    joy_to_twist.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()