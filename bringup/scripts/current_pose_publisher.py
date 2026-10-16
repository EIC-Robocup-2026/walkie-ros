#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
import tf2_ros


def yaw_from_quaternion(q):
    """Extract yaw angle from a quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class CurrentPosePublisher(Node):
    def __init__(self):
        super().__init__("current_pose_publisher")

        # Parameters
        self.declare_parameter("source_frame", "map")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("topic_name", "current_pose")

        self.source_frame = self.get_parameter("source_frame").value
        self.target_frame = self.get_parameter("target_frame").value
        publish_rate = self.get_parameter("publish_rate").value
        topic_name = self.get_parameter("topic_name").value

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, topic_name, 10)

        # Previous state for velocity computation
        self.prev_x = None
        self.prev_y = None
        self.prev_yaw = None
        self.prev_time = None

        # Timer
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

        self.get_logger().info(
            f"Publishing odom: {self.source_frame} -> {self.target_frame} "
            f"on '{topic_name}' at {publish_rate} Hz"
        )

    def timer_callback(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.source_frame,
                self.target_frame,
                rclpy.time.Time(),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return

        cur_x = transform.transform.translation.x
        cur_y = transform.transform.translation.y
        cur_yaw = yaw_from_quaternion(transform.transform.rotation)
        cur_time = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9

        # Compute velocity in body frame
        vx_body = 0.0
        vy_body = 0.0
        vyaw = 0.0

        if self.prev_time is not None:
            dt = cur_time - self.prev_time
            if dt > 0.0:
                # World-frame deltas
                dx = cur_x - self.prev_x
                dy = cur_y - self.prev_y

                # Rotate world-frame velocity into body frame
                cos_yaw = math.cos(cur_yaw)
                sin_yaw = math.sin(cur_yaw)
                vx_body = (dx * cos_yaw + dy * sin_yaw) / dt
                vy_body = (-dx * sin_yaw + dy * cos_yaw) / dt

                # Angular velocity
                dyaw = cur_yaw - self.prev_yaw
                # Normalize to [-pi, pi]
                dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))
                vyaw = dyaw / dt

        self.prev_x = cur_x
        self.prev_y = cur_y
        self.prev_yaw = cur_yaw
        self.prev_time = cur_time

        # Build Odometry message
        msg = Odometry()
        msg.header = transform.header
        msg.child_frame_id = self.target_frame

        # Pose (in source_frame)
        msg.pose.pose.position.x = cur_x
        msg.pose.pose.position.y = cur_y
        msg.pose.pose.position.z = transform.transform.translation.z
        msg.pose.pose.orientation = transform.transform.rotation

        # Twist (in child/body frame)
        msg.twist.twist.linear.x = vx_body
        msg.twist.twist.linear.y = vy_body
        msg.twist.twist.angular.z = vyaw

        self.odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CurrentPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
