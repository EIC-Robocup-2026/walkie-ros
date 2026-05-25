#!/bin/sh
"exec" "${PERCEPTION_VENV:-$HOME/perception-venv/bin/python3}" "$0" "$@"
"""
test_grasp_viz.py — Request a grasp and visualise the result with Open3D.

Usage
─────
  ros2 run walkie_perception test_grasp_viz bottle
  ros2 run walkie_perception test_grasp_viz cell phone
  ros2 run walkie_perception test_grasp_viz --id 42
  ros2 run walkie_perception test_grasp_viz           # first detected object
  ros2 run walkie_perception test_grasp_viz bottle --score 0.3 --grasps 5

  # Override ZED topics (e.g. when namespace is 'zed' not 'zed_head')
  ros2 run walkie_perception test_grasp_viz bottle \\
    --depth  /zed/zed_node/depth/depth_registered \\
    --info   /zed/zed_node/depth/camera_info \\
    --masks  /yolo/masks \\
    --dets   /yolo/tracked_detections_2d

The script:
  1. Waits for CameraInfo, one depth frame, one mask frame, and (if filtering
     by class) a matching detection.
  2. Calls /grasp/from_mask with the target tracker ID and bbox.
  3. Unprojects the masked depth region locally at full resolution.
  4. Reconstructs the GraspGroup from the service response.
  5. Opens an Open3D window showing:
       • coloured point cloud (blue = close, red = far)
       • world frame axis
       • gripper geometries from GraspGroup.to_open3d_geometry_list()
"""

import argparse
import sys
import time
import threading

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, BoundingBox2D
from vision_msgs.msg import Pose2D, Point2D

try:
    import open3d as o3d
except ImportError:
    sys.exit("open3d not installed.  Run: uv pip install open3d")

try:
    from graspnetAPI import GraspGroup
except ImportError:
    sys.exit("graspnetAPI not found.  See README §6 for setup.")

try:
    from walkie_perception.srv import GraspFromMask
except ImportError:
    sys.exit("grasp_interfaces not found.  Build the package and source setup.bash.")


# ── helpers ───────────────────────────────────────────────────────────────────

def _depth_to_xyz(depth: np.ndarray, mask: np.ndarray,
                  fx: float, fy: float, cx: float, cy: float,
                  z_min: float = 0.1, z_max: float = 3.0) -> np.ndarray:
    """Pinhole unproject. Returns (N,3) float32 in camera frame."""
    if mask.shape != depth.shape:
        mask = cv2.resize(mask.astype(np.uint8), (depth.shape[1], depth.shape[0]),
                          interpolation=cv2.INTER_NEAREST).astype(bool)
    vs, us = np.where(mask)
    z = depth[vs, us]
    valid = np.isfinite(z) & (z > z_min) & (z < z_max)
    us, vs, z = us[valid].astype(np.float32), vs[valid].astype(np.float32), z[valid]
    if len(z) == 0:
        return np.empty((0, 3), dtype=np.float32)
    x = (us - cx) * z / fx
    y = (vs - cy) * z / fy
    return np.stack([x, y, z], axis=1).astype(np.float32)


def _xyz_to_pcd(pts: np.ndarray) -> o3d.geometry.PointCloud:
    """Colour by depth: blue (near) → red (far)."""
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    if len(pts):
        z = pts[:, 2]
        t = np.clip((z - z.min()) / (z.ptp() + 1e-6), 0, 1)
        colours = np.stack([t, np.zeros_like(t), 1 - t], axis=1)
        pcd.colors = o3d.utility.Vector3dVector(colours)
    return pcd


def _rebuild_grasp_group(poses, scores, widths) -> GraspGroup:
    """Reconstruct a GraspGroup (17-col array) from service response fields."""
    n = len(scores)
    arr = np.zeros((n, 17), dtype=np.float32)
    for i, (pose, score, width) in enumerate(zip(poses.poses, scores, widths)):
        p = pose.position
        q = pose.orientation
        # quaternion → rotation matrix
        x2, y2, z2 = 2*q.x*q.x, 2*q.y*q.y, 2*q.z*q.z
        xy, xz, yz = 2*q.x*q.y, 2*q.x*q.z, 2*q.y*q.z
        wx, wy, wz = 2*q.w*q.x, 2*q.w*q.y, 2*q.w*q.z
        rot = np.array([
            [1-y2-z2, xy-wz,   xz+wy],
            [xy+wz,   1-x2-z2, yz-wx],
            [xz-wy,   yz+wx,   1-x2-y2],
        ], dtype=np.float32)
        arr[i, 0]    = score
        arr[i, 1]    = width
        arr[i, 2]    = 0.02   # height (fixed)
        arr[i, 3]    = 0.02   # depth
        arr[i, 4:13] = rot.flatten()
        arr[i, 13:16] = [p.x, p.y, p.z]
        arr[i, 16]   = -1
    return GraspGroup(arr)


# ── ROS node ──────────────────────────────────────────────────────────────────

class GraspVizNode(Node):
    def __init__(self, args):
        super().__init__('test_grasp_viz')
        self._args = args

        self._info_lock   = threading.Lock()
        self._depth_lock  = threading.Lock()
        self._mask_lock   = threading.Lock()
        self._det_lock    = threading.Lock()

        self._fx = self._fy = self._cx = self._cy = None
        self._depth_msg  = None
        self._mask_msg   = None
        self._target_tid = args.id       # set when filtering by id
        self._target_cls = ' '.join(args.class_name) if args.class_name else None
        self._bbox       = None          # BoundingBox2D
        self._found_tid  = None          # resolved tracker id

        be = ReliabilityPolicy.BEST_EFFORT
        kl = HistoryPolicy.KEEP_LAST
        qos = QoSProfile(reliability=be, history=kl, depth=5)

        self.create_subscription(CameraInfo, args.info,  self._info_cb,  10)
        self.create_subscription(Image,      args.depth, self._depth_cb, qos)
        self.create_subscription(Image,      args.masks, self._mask_cb,  qos)
        self.create_subscription(Detection2DArray, args.dets, self._det_cb, qos)

        self._cli = self.create_client(GraspFromMask, '/grasp/from_mask')

    # ── subscribers ───────────────────────────────────────────────────────────

    def _info_cb(self, msg: CameraInfo):
        with self._info_lock:
            if self._fx is None:
                self._fx, self._fy = msg.k[0], msg.k[4]
                self._cx, self._cy = msg.k[2], msg.k[5]
                self.get_logger().info(
                    f'CameraInfo: fx={self._fx:.1f} fy={self._fy:.1f} '
                    f'cx={self._cx:.1f} cy={self._cy:.1f}')

    def _depth_cb(self, msg: Image):
        with self._depth_lock:
            self._depth_msg = msg

    def _mask_cb(self, msg: Image):
        with self._mask_lock:
            self._mask_msg = msg

    def _det_cb(self, msg: Detection2DArray):
        if self._target_cls is None and self._target_tid is None:
            # grab the first detection
            if msg.detections:
                d = msg.detections[0]
                tid = int(d.id) if d.id.lstrip('-').isdigit() else -1
                if tid > 0:
                    with self._det_lock:
                        self._found_tid = tid
                        self._bbox = d.bbox
            return

        for d in msg.detections:
            cls = d.results[0].hypothesis.class_id if d.results else ''
            tid = int(d.id) if d.id.lstrip('-').isdigit() else -1
            if tid <= 0:
                continue

            if self._target_tid is not None and tid == self._target_tid:
                with self._det_lock:
                    self._found_tid = tid
                    self._bbox = d.bbox
                return

            if self._target_cls is not None and cls == self._target_cls:
                with self._det_lock:
                    self._found_tid = tid
                    self._bbox = d.bbox
                return

    # ── wait helpers ──────────────────────────────────────────────────────────

    def _wait(self, description: str, check_fn, timeout: float = 15.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if check_fn():
                return True
        self.get_logger().error(f'Timeout waiting for {description}')
        return False

    # ── main pipeline ─────────────────────────────────────────────────────────

    def run(self):
        args = self._args

        print('Waiting for CameraInfo …')
        if not self._wait('CameraInfo', lambda: self._fx is not None):
            return

        print('Waiting for depth frame …')
        if not self._wait('depth frame', lambda: self._depth_msg is not None):
            return

        print('Waiting for mask frame …')
        if not self._wait('mask frame', lambda: self._mask_msg is not None):
            return

        target_str = (f'class={self._target_cls}' if self._target_cls
                      else f'id={self._target_tid}' if self._target_tid is not None
                      else 'first object')
        print(f'Waiting for detection ({target_str}) …')
        if not self._wait(f'detection ({target_str})',
                          lambda: self._found_tid is not None, timeout=20.0):
            return

        with self._det_lock:
            tid  = self._found_tid
            bbox = self._bbox

        print(f'Found tracker_id={tid}  bbox=({bbox.center.position.x:.0f},'
              f'{bbox.center.position.y:.0f})  {bbox.size_x:.0f}×{bbox.size_y:.0f}')

        # ── call service ──────────────────────────────────────────────────────
        if not self._cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/grasp/from_mask service not available')
            return

        req = GraspFromMask.Request()
        with self._mask_lock:
            req.mask = self._mask_msg
        req.tracker_id      = tid
        req.bbox            = bbox
        req.num_frames      = 0
        req.score_threshold = args.score
        req.max_grasps      = args.grasps

        print(f'Calling /grasp/from_mask (score≥{args.score}, max={args.grasps}) …')
        future = self._cli.call_async(req)
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

        resp = future.result()
        if not resp.success:
            print(f'Grasp failed: {resp.message}')
            return

        print(f'\n{resp.message}')
        print(f'  inference {resp.inference_ms:.0f} ms  '
              f'total {resp.total_ms:.0f} ms')
        print(f'  pts extracted={resp.points_extracted}  '
              f'fed={resp.points_fed}  '
              f'frames={resp.frames_used}')
        print(f'  raw grasps={resp.grasps_raw}  returned={resp.grasps_returned}')
        for i, (s, w) in enumerate(zip(resp.scores, resp.widths)):
            print(f'  [{i}] score={s:.3f}  width={w*100:.1f} cm')

        # ── unproject point cloud ─────────────────────────────────────────────
        with self._depth_lock:
            depth_msg = self._depth_msg
        with self._mask_lock:
            mask_msg = self._mask_msg

        depth = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(
            depth_msg.height, depth_msg.width)
        label = np.frombuffer(mask_msg.data, dtype=np.uint16).reshape(
            mask_msg.height, mask_msg.width)
        mask = (label == tid)

        with self._info_lock:
            fx, fy, cx, cy = self._fx, self._fy, self._cx, self._cy

        pts = _depth_to_xyz(depth, mask, fx, fy, cx, cy)
        print(f'\nVisualising {len(pts)} points + {len(resp.scores)} grippers …')

        pcd   = _xyz_to_pcd(pts)
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)

        gg = _rebuild_grasp_group(resp.poses, resp.scores, resp.widths)
        grippers = gg.to_open3d_geometry_list()

        o3d.visualization.draw_geometries(
            [pcd, frame] + grippers,
            window_name=f'GraspNet — {target_str}',
            width=1280, height=720,
        )


# ── entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Visualise GraspNet results for a detected object.')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--id', type=int, metavar='TRACKER_ID',
                       help='Use a specific ByteTrack tracker ID')
    group.add_argument('class_name', nargs='*', default=[],
                       help='COCO class name (e.g. bottle, cell phone)')
    parser.add_argument('--score', type=float, default=0.1,
                        help='Minimum grasp score (default 0.1)')
    parser.add_argument('--grasps', type=int, default=10,
                        help='Max grasps to return (default 10)')
    parser.add_argument('--depth', default='/zed_head/zed_node/depth/depth_registered',
                        help='Depth image topic')
    parser.add_argument('--info',  default='/zed_head/zed_node/depth/camera_info',
                        help='CameraInfo topic')
    parser.add_argument('--masks', default='/yolo/masks',
                        help='YOLO mask label image topic')
    parser.add_argument('--dets',  default='/yolo/tracked_detections_2d',
                        help='YOLO Detection2DArray topic')

    # argparse + ros2 run: strip ROS args before parsing
    argv = [a for a in sys.argv[1:] if not a.startswith('__')]
    args = parser.parse_args(argv)

    rclpy.init()
    node = GraspVizNode(args)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
