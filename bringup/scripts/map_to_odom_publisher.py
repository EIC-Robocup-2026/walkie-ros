#!/usr/bin/env python3
"""
Publish an identity map -> odom transform on /tf_static at a fixed period.

static_transform_publisher sends the transform once. A node that starts later
depends on the middleware to deliver that one stored message. When the
delivery is missed, the frame can stay unknown to that node: bt_navigator then
rejects goals with "Initial robot pose is not available" (error 9002).
Publishing the same transform again every period gives every subscriber,
including one that missed it, a new copy within one period.
"""

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


class MapToOdomPublisher(Node):

    def __init__(self):
        super().__init__('map_to_odom')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('child_frame_id', 'odom')
        self.declare_parameter('publish_period', 1.0)
        frame_id = self.get_parameter('frame_id').value
        child_frame_id = self.get_parameter('child_frame_id').value
        publish_period = self.get_parameter('publish_period').value
        if publish_period <= 0.0:
            raise ValueError(f'publish_period must be positive, got {publish_period}')

        self.transform = TransformStamped()
        self.transform.header.frame_id = frame_id
        self.transform.child_frame_id = child_frame_id
        self.transform.transform.rotation.w = 1.0

        self.broadcaster = StaticTransformBroadcaster(self)
        self.publish()

        # A steady clock keeps the period independent of /clock, which stops
        # while the simulation is paused or not yet running.
        self.timer = self.create_timer(
            publish_period, self.publish, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )
        self.get_logger().info(
            f'Publishing {frame_id} -> {child_frame_id} on /tf_static every {publish_period} s'
        )

    def publish(self):
        self.transform.header.stamp = self.get_clock().now().to_msg()
        self.broadcaster.sendTransform(self.transform)


def main(args=None):
    rclpy.init(args=args)
    node = MapToOdomPublisher()
    # No rclpy.try_shutdown() here: on SIGINT, rclpy's signal handler is already
    # shutting the context down in another thread, and under rmw_zenoh_cpp a
    # second shutdown raises RCLError.
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
