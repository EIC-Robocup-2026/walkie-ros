#!/usr/bin/env python3
"""Exit once a transform and an optional action server are available.

Launch files use the exit of this process to order startup. Under a planar
base_mode move_group needs map -> base_footprint before it starts, and the commander
needs move_group's action server.
"""
import argparse

from action_msgs.srv import CancelGoal
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target-frame', default='map')
    parser.add_argument('--source-frame', default='base_footprint')
    parser.add_argument('--action', default='', help='action name to wait for, optional')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = Node('wait_for_graph')
    buffer = Buffer()
    listener = TransformListener(buffer, node)  # noqa: F841 (kept alive for its subscriptions)
    # An action server is up when its cancel service is, whatever its action type.
    cancel = node.create_client(CancelGoal, f'{args.action}/_action/cancel_goal') \
        if args.action else None

    waited = 0
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.5)
        waited += 1
        have_transform = buffer.can_transform(args.target_frame, args.source_frame, Time())
        have_action = cancel is None or cancel.service_is_ready()
        if have_transform and have_action:
            node.get_logger().info('Ready')
            break
        if waited % 20 == 0:
            node.get_logger().info(
                f'Waiting: {args.target_frame} -> {args.source_frame}: {have_transform}, '
                f'action {args.action or "none"}: {have_action}')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
