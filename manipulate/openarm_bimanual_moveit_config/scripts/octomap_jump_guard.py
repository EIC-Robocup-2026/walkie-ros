#!/usr/bin/env python3
"""Clear the octomap when map -> odom jumps.

Under a planar base_mode voxels stay where they were written in map. A localization
jump shifts everything observed afterwards relative to them, which leaves ghost
obstacles. This node watches map -> odom and calls /clear_octomap above a threshold.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_of(q):
    """Return the yaw of a geometry_msgs Quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class OctomapJumpGuard(Node):
    """Compares consecutive map -> odom samples and clears on a jump."""

    def __init__(self):
        super().__init__('octomap_jump_guard')
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.jump_distance = self.declare_parameter('jump_distance', 0.1).value
        self.jump_yaw = self.declare_parameter('jump_yaw', 0.1).value
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.clear = self.create_client(Empty, '/clear_octomap')
        self.previous = None
        self.timer = self.create_timer(0.2, self.check)

    def check(self):
        try:
            current = self.buffer.lookup_transform(
                self.map_frame, self.odom_frame, Time()).transform
        except TransformException:
            return
        previous, self.previous = self.previous, current
        if previous is None:
            return
        moved = math.hypot(
            current.translation.x - previous.translation.x,
            current.translation.y - previous.translation.y)
        turned = yaw_of(current.rotation) - yaw_of(previous.rotation)
        turned = abs(math.atan2(math.sin(turned), math.cos(turned)))
        if moved > self.jump_distance or turned > self.jump_yaw:
            self.get_logger().warning(
                f'{self.map_frame} -> {self.odom_frame} jumped {moved:.3f} m and '
                f'{turned:.3f} rad, clearing the octomap')
            self.clear.call_async(Empty.Request())


def main():
    rclpy.init()
    node = OctomapJumpGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
