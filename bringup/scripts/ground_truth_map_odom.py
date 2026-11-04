#!/usr/bin/env python3
"""Publish map -> odom from MuJoCo's ground truth.

The omni controller runs open loop, so odom -> base_footprint reports commands and
not wheel motion. With a static identity map -> odom, the base pose in map then reads
as the commanded pose, and tracking looks perfect while the base slips. This node
makes map -> base_footprint equal the ground truth pose, in the plane, by closing the
difference through map -> odom. It stands in for localization in simulation only.
"""
import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def yaw_of(q):
    """Return the yaw of a geometry_msgs Quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class GroundTruthMapOdom(Node):
    """Turns ground truth odometry into a map -> odom transform."""

    def __init__(self):
        super().__init__('ground_truth_map_odom')
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        self.rate = self.declare_parameter('rate', 50.0).value
        # Future-dated like AMCL's transform, so lookups at the newest sensor stamp
        # do not extrapolate. map -> odom only drifts slowly, so this adds no error.
        self.future_date = Duration(seconds=self.declare_parameter('future_date', 0.1).value)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.last_publish = None
        self.subscription = self.create_subscription(
            Odometry,
            self.declare_parameter('odom_topic', '/mujoco_ground_truth/odom').value,
            self.on_truth,
            10)

    def on_truth(self, msg):
        now = self.get_clock().now()
        if self.last_publish is not None:
            if (now - self.last_publish).nanoseconds < 1e9 / self.rate:
                return
        try:
            odom_base = self.buffer.lookup_transform(
                self.odom_frame, self.base_frame, Time()).transform
        except TransformException as error:
            self.get_logger().info(f'Waiting: {error}', throttle_duration_sec=5.0)
            return

        truth_x = msg.pose.pose.position.x
        truth_y = msg.pose.pose.position.y
        truth_yaw = yaw_of(msg.pose.pose.orientation)
        odom_x = odom_base.translation.x
        odom_y = odom_base.translation.y
        odom_yaw = yaw_of(odom_base.rotation)

        # T_map_odom = T_map_base * inverse(T_odom_base), in the plane.
        yaw = truth_yaw - odom_yaw
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        transform = TransformStamped()
        transform.header.stamp = (now + self.future_date).to_msg()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.translation.x = truth_x - (cos_yaw * odom_x - sin_yaw * odom_y)
        transform.transform.translation.y = truth_y - (sin_yaw * odom_x + cos_yaw * odom_y)
        transform.transform.rotation.z = math.sin(yaw / 2.0)
        transform.transform.rotation.w = math.cos(yaw / 2.0)
        self.broadcaster.sendTransform(transform)
        self.last_publish = now


def main():
    rclpy.init()
    node = GroundTruthMapOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
