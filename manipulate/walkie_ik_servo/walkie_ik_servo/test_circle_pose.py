#!/usr/bin/env python3
"""Test node that publishes target_pose continuously along a circle on the YZ plane.

Publishes geometry_msgs/PoseStamped on /target_pose at a configurable rate (default 50Hz),
tracing a circle in the YZ plane (constant X) around the arm's zero-config EE position.

Usage:
    ros2 run walkie_ik_servo test_circle_pose.py
    ros2 run walkie_ik_servo test_circle_pose.py --ros-args \
        -p radius:=0.08 -p publish_rate:=50.0 -p period:=4.0
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class CirclePosePublisher(Node):
    """Publishes PoseStamped continuously tracing a circle on the YZ plane."""

    def __init__(self):
        super().__init__("test_circle_pose")

        # Parameters
        self.declare_parameter("radius", 0.08)  # Circle radius in meters
        self.declare_parameter("publish_rate", 50.0)  # Hz
        self.declare_parameter("period", 4.0)  # Seconds per full revolution
        # Circle center — defaults match the zero-config EE position in base_footprint
        self.declare_parameter("center_x", 0.383)  # Circle center X (forward)
        self.declare_parameter("center_y", 0.194)  # Circle center Y (left)
        self.declare_parameter("center_z", 0.579)  # Circle center Z (up)
        self.declare_parameter("topic", "/target_pose")
        self.declare_parameter("frame_id", "base_footprint")
        # Orientation defaults match the zero-config EE orientation in base_footprint:
        #   R ≈ [[ 0, 0, -1],    i.e. gripper points downward with 90-deg twist
        #        [ 1, 0,  0],    RPY = (-pi/2, 0, pi/2)
        #        [ 0,-1,  0]]
        self.declare_parameter("roll", -1.5707963)  # -pi/2 radians
        self.declare_parameter("pitch", 0.0)  # Radians
        self.declare_parameter("yaw", 1.5707963)  # pi/2 radians

        self.radius = float(self.get_parameter("radius").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.period = float(self.get_parameter("period").value)
        self.cx = float(self.get_parameter("center_x").value)
        self.cy = float(self.get_parameter("center_y").value)
        self.cz = float(self.get_parameter("center_z").value)
        topic = str(self.get_parameter("topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.roll = float(self.get_parameter("roll").value)
        self.pitch = float(self.get_parameter("pitch").value)
        self.yaw = float(self.get_parameter("yaw").value)

        # Compute fixed orientation quaternion from RPY
        self.quat = self.euler_to_quaternion(self.roll, self.pitch, self.yaw)

        # Publisher
        self.pub = self.create_publisher(PoseStamped, topic, 10)

        # State: continuously advancing angle
        self._elapsed = 0.0
        self._dt = 1.0 / self.publish_rate

        # Timer fires at publish_rate Hz
        self.timer = self.create_timer(self._dt, self._timer_cb)

        self.get_logger().info(
            f"Circle pose publisher started:\n"
            f"  Topic:    {topic}\n"
            f"  Center:   ({self.cx:.3f}, {self.cy:.3f}, {self.cz:.3f})\n"
            f"  Orientation (RPY): ({self.roll:.3f}, {self.pitch:.3f}, {self.yaw:.3f})\n"
            f"  Radius:   {self.radius:.3f} m\n"
            f"  Rate:     {self.publish_rate:.0f} Hz\n"
            f"  Period:   {self.period:.1f} s per revolution"
        )

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert RPY (radians) to Quaternion [x, y, z, w]."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy
        return [qx, qy, qz, qw]

    def _timer_cb(self):
        """Compute current angle and publish pose."""
        # Continuous angle based on elapsed time
        angle = 2.0 * math.pi * (self._elapsed / self.period)
        self._elapsed += self._dt

        # Wrap elapsed to avoid float overflow (not strictly necessary but clean)
        if self._elapsed >= self.period:
            self._elapsed -= self.period

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Circle on YZ plane (X stays constant)
        msg.pose.position.x = self.cx
        msg.pose.position.y = self.cy + self.radius * math.cos(angle)
        msg.pose.position.z = self.cz + self.radius * math.sin(angle)

        # Use the computed orientation
        msg.pose.orientation.x = self.quat[0]
        msg.pose.orientation.y = self.quat[1]
        msg.pose.orientation.z = self.quat[2]
        msg.pose.orientation.w = self.quat[3]

        self.pub.publish(msg)

        self.get_logger().debug(
            f"angle={math.degrees(angle):.0f}deg  "
            f"pos=({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, {msg.pose.position.z:.3f})",
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
