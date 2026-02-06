#!/usr/bin/env python3
"""Test node that publishes target_pose continuously along a circle on the YZ plane.

Publishes geometry_msgs/Pose on /target_pose at a configurable rate (default 50Hz),
tracing a circle in the YZ plane (constant X) around the arm's zero-config EE position.

Usage:
    ros2 run walkie_ik_servo test_circle_pose.py
    ros2 run walkie_ik_servo test_circle_pose.py --ros-args \
        -p radius:=0.08 -p publish_rate:=50.0 -p period:=4.0
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose


class CirclePosePublisher(Node):
    """Publishes Pose continuously tracing a circle on the YZ plane."""

    def __init__(self):
        super().__init__("test_circle_pose")

        # Parameters
        self.declare_parameter("radius", 0.08)  # Circle radius in meters
        self.declare_parameter("publish_rate", 50.0)  # Hz
        self.declare_parameter("period", 4.0)  # Seconds per full revolution
        self.declare_parameter("center_x", 0.34)  # Circle center X (forward)
        self.declare_parameter("center_y", 0.25)  # Circle center Y (left)
        self.declare_parameter("center_z", 0.50)  # Circle center Z (up)
        self.declare_parameter("topic", "/target_pose")

        self.radius = self.get_parameter("radius").value
        self.publish_rate = self.get_parameter("publish_rate").value
        self.period = self.get_parameter("period").value
        self.cx = self.get_parameter("center_x").value
        self.cy = self.get_parameter("center_y").value
        self.cz = self.get_parameter("center_z").value
        topic = self.get_parameter("topic").value

        # Publisher
        self.pub = self.create_publisher(Pose, topic, 10)

        # State: continuously advancing angle
        self._elapsed = 0.0
        self._dt = 1.0 / self.publish_rate

        # Timer fires at publish_rate Hz
        self.timer = self.create_timer(self._dt, self._timer_cb)

        self.get_logger().info(
            f"Circle pose publisher started:\n"
            f"  Topic:    {topic}\n"
            f"  Center:   ({self.cx:.3f}, {self.cy:.3f}, {self.cz:.3f})\n"
            f"  Radius:   {self.radius:.3f} m\n"
            f"  Rate:     {self.publish_rate:.0f} Hz\n"
            f"  Period:   {self.period:.1f} s per revolution"
        )

    def _timer_cb(self):
        """Compute current angle and publish pose."""
        # Continuous angle based on elapsed time
        angle = 2.0 * math.pi * (self._elapsed / self.period)
        self._elapsed += self._dt

        # Wrap elapsed to avoid float overflow (not strictly necessary but clean)
        if self._elapsed >= self.period:
            self._elapsed -= self.period

        msg = Pose()

        # Circle on YZ plane (X stays constant)
        msg.position.x = self.cx
        msg.position.y = self.cy + self.radius * math.cos(angle)
        msg.position.z = self.cz + self.radius * math.sin(angle)

        # Identity quaternion - let IK solver handle orientation
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0

        self.pub.publish(msg)

        self.get_logger().debug(
            f"angle={math.degrees(angle):.0f}deg  "
            f"pos=({msg.position.x:.3f}, {msg.position.y:.3f}, {msg.position.z:.3f})",
            throttle_duration_sec=1.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = CirclePosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
