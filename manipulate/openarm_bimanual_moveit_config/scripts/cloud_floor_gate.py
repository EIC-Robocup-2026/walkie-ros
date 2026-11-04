#!/usr/bin/env python3
"""Gate and decimate the point cloud in front of MoveIt's octomap updater.

Under a planar base_mode the octree lives in map and keeps the whole room, so it
needs a coarse, light input. A floor return that lands in a voxel also puts the
whole-body planner's base in collision, so returns near the floor must not become
voxels. Their rays still have to reach the updater: they are the only rays that
cross the space in front of the robot, and a stale voxel there clears only when a
ray passes through it. So a floor return moves out along its ray, past the
updater's max_range. The updater then clears along the ray and marks nothing.
The output is an unorganized XYZ cloud in the frame of the input.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener


def quaternion_to_matrix(q):
    """Return the 3x3 rotation matrix of a geometry_msgs Quaternion."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def floor_to_free_rays(xyz, rotation, camera_height, floor_height, free_ray_range):
    """Move each return at or below floor_height out along its ray to free_ray_range.

    xyz is an N x 3 array in the camera frame, and rotation takes the camera frame to the
    floor frame. The camera origin stays where it is, because it has no ray direction.
    """
    distance = np.linalg.norm(xyz, axis=1)
    floor = (xyz @ rotation[2] + camera_height <= floor_height) & (distance > 0.0)
    moved = xyz.copy()
    moved[floor] *= (free_ray_range / distance[floor])[:, np.newaxis]
    return moved


class CloudFloorGate(Node):
    """Republishes a cloud with the floor as free rays and with fewer points."""

    def __init__(self):
        super().__init__('cloud_floor_gate')
        self.floor_frame = self.declare_parameter('floor_frame', 'base_footprint').value
        self.floor_height = self.declare_parameter('floor_height', 0.05).value
        self.stride = int(self.declare_parameter('stride', 4).value)
        # Has to exceed the octomap updater's max_range.
        self.free_ray_range = self.declare_parameter('free_ray_range', 100.0).value
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(PointCloud2, 'cloud_out', 1)
        self.subscription = self.create_subscription(
            PointCloud2, 'cloud_in', self.on_cloud, qos_profile_sensor_data)

    def on_cloud(self, msg):
        offsets = {field.name: field.offset for field in msg.fields}
        if not {'x', 'y', 'z'} <= offsets.keys():
            self.get_logger().error('The input cloud has no x, y, z fields', once=True)
            return
        try:
            # The latest transform, not the one at the stamp: this only needs the camera's
            # height over the floor, and the lift moves it by millimetres in a cloud's age.
            transform = self.buffer.lookup_transform(self.floor_frame, msg.header.frame_id, Time())
        except TransformException as error:
            self.get_logger().warning(f'Dropped a cloud: {error}', throttle_duration_sec=5.0)
            return

        dtype = np.dtype({
            'names': ['x', 'y', 'z'],
            'formats': ['<f4', '<f4', '<f4'],
            'offsets': [offsets['x'], offsets['y'], offsets['z']],
            'itemsize': msg.point_step,
        })
        points = np.frombuffer(msg.data, dtype=dtype, count=msg.width * msg.height)
        if msg.height > 1:
            points = points.reshape(msg.height, msg.width)[::self.stride, ::self.stride].ravel()
        else:
            points = points[::self.stride]
        xyz = np.stack([points['x'], points['y'], points['z']], axis=-1)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]

        kept = floor_to_free_rays(
            xyz,
            quaternion_to_matrix(transform.transform.rotation),
            transform.transform.translation.z,
            self.floor_height,
            self.free_ray_range,
        ).astype('<f4')

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = kept.shape[0]
        out.fields = [
            PointField(name=name, offset=4 * index, datatype=PointField.FLOAT32, count=1)
            for index, name in enumerate(('x', 'y', 'z'))
        ]
        out.is_bigendian = False
        out.point_step = 12
        out.row_step = 12 * out.width
        out.is_dense = True
        out.data = kept.tobytes()
        self.publisher.publish(out)


def main():
    rclpy.init()
    node = CloudFloorGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
