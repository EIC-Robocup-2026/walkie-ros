#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import Joy

class JoyToTwist(Node):
    def __init__(self):
        super().__init__('joy_teleop_node')

        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.subscriber_ = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.get_logger().info('JoyToTwist DIRECT control (stamped, no smoothing) started.')

        # Parameters
        self.declare_parameter('max_linear_vel', 5.0)
        self.declare_parameter('max_angular_vel', 5.0)
        self.declare_parameter('linear_axis', 1)
        self.declare_parameter('angular_axis', 3)
        self.declare_parameter('strafe_axis', 0)
        self.declare_parameter('deadzone', 0.05)
        self.declare_parameter('turbo_button', 5)
        self.declare_parameter('turbo_linear_multiplier', 2.0)
        self.declare_parameter('turbo_angular_multiplier', 1.5)

        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.linear_axis = self.get_parameter('linear_axis').value
        self.angular_axis = self.get_parameter('angular_axis').value
        self.strafe_axis = self.get_parameter('strafe_axis').value
        self.deadzone = self.get_parameter('deadzone').value
        self.turbo_button = self.get_parameter('turbo_button').value
        self.turbo_linear_multiplier = self.get_parameter('turbo_linear_multiplier').value
        self.turbo_angular_multiplier = self.get_parameter('turbo_angular_multiplier').value

        self.last_twist = Twist()

    def _apply_deadzone(self, value):
        return value if abs(value) > self.deadzone else 0.0

    def _get_axis(self, axes, axis_idx):
        if axis_idx < len(axes):
            return self._apply_deadzone(axes[axis_idx])
        return 0.0

    def joy_callback(self, msg):
        current_time = self.get_clock().now()

        is_turbo = self.turbo_button < len(msg.buttons) and bool(msg.buttons[self.turbo_button] == 1)
        eff_max_linear_vel = self.max_linear_vel * (self.turbo_linear_multiplier if is_turbo else 1.0)
        eff_max_angular_vel = self.max_angular_vel * (self.turbo_angular_multiplier if is_turbo else 1.0)

        cmd_twist = Twist()

        # Apply deadzone
        raw_linear_x = self._get_axis(msg.axes, self.linear_axis)
        raw_linear_y = self._get_axis(msg.axes, self.strafe_axis)
        raw_angular_z = self._get_axis(msg.axes, self.angular_axis)

        # Direct proportional mapping (NO smoothing)
        cmd_twist.linear.x = raw_linear_x * eff_max_linear_vel
        cmd_twist.linear.y = raw_linear_y * eff_max_linear_vel
        cmd_twist.angular.z = raw_angular_z * eff_max_angular_vel

        is_input_zero = (
            raw_linear_x == 0.0 and
            raw_linear_y == 0.0 and
            raw_angular_z == 0.0
        )

        is_stopped = (
            cmd_twist.linear.x == 0.0 and
            cmd_twist.linear.y == 0.0 and
            cmd_twist.angular.z == 0.0
        )

        # Idle → publish ONE final zero
        if is_input_zero and is_stopped:
            if (
                self.last_twist.linear.x != 0.0 or
                self.last_twist.linear.y != 0.0 or
                self.last_twist.angular.z != 0.0
            ):
                twist_stamped = TwistStamped()
                twist_stamped.header.stamp = current_time.to_msg()
                twist_stamped.header.frame_id = "base_link"
                twist_stamped.twist = Twist()

                self.publisher_.publish(twist_stamped)
                self.last_twist = Twist()
            return

        # Normal publish
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = current_time.to_msg()
        twist_stamped.header.frame_id = "base_link"
        twist_stamped.twist = cmd_twist

        self.publisher_.publish(twist_stamped)
        self.last_twist = cmd_twist


def main(args=None):
    rclpy.init(args=args)
    node = JoyToTwist()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()