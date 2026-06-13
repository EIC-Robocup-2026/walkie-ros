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
    --depth  /zed_head/zed_node/depth/depth_registered \\
    --info   /zed_head/zed_node/depth/camera_info \\
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

import tf2_ros
import tf2_geometry_msgs  # noqa: F401 — registers PoseStamped transform support

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from vision_msgs.msg import Detection2DArray, BoundingBox2D
from vision_msgs.msg import Pose2D, Point2D

try:
    import open3d as o3d
except ImportError:
    sys.exit("open3d not installed.  Run: uv pip install open3d")

try:
    from graspnetAPI.grasp import GraspGroup
except ImportError:
    sys.exit("graspnetAPI not found.  See README §6 for setup.")

try:
    from walkie_perception.srv import GraspFromMask
except ImportError:
    sys.exit("grasp_interfaces not found.  Build the package and source setup.bash.")


# ── helpers ───────────────────────────────────────────────────────────────────

def _edge_agreement(depth: np.ndarray, valid: np.ndarray, tol_m: float = 0.03,
                    radius: int = 1, min_ratio: float = 0.6) -> np.ndarray:
    """Neighbour-disagreement / flying-pixel filter (mirrors the node)."""
    r = max(1, int(radius))
    d = np.where(valid, depth, -1000.0).astype(np.float32)
    agree = np.zeros(depth.shape, dtype=np.int16)
    n = 0
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if dx == 0 and dy == 0:
                continue
            n += 1
            sd = np.roll(np.roll(d, dy, axis=0), dx, axis=1)
            sv = np.roll(np.roll(valid, dy, axis=0), dx, axis=1)
            agree += (sv & (np.abs(sd - d) < tol_m)).astype(np.int16)
    return valid & (agree >= int(np.ceil(n * min_ratio)))


def _cloud_validity(cloud_msg, h_out: int, w_out: int) -> np.ndarray:
    """Boolean (h_out, w_out) mask of pixels where the ZED cloud has a finite
    point, upsampled from the organized cloud (mirrors the node)."""
    H, W = cloud_msg.height, cloud_msg.width
    buf = np.frombuffer(cloud_msg.data, dtype=np.uint8).reshape(
        H, W, cloud_msg.point_step)
    xyz = buf[:, :, 0:12].copy().view(np.float32).reshape(H, W, 3)
    valid = np.isfinite(xyz).all(axis=2)
    if (H, W) != (h_out, w_out):
        valid = cv2.resize(valid.astype(np.uint8), (w_out, h_out),
                           interpolation=cv2.INTER_NEAREST).astype(bool)
    return valid


def _depth_to_xyz(depth: np.ndarray, mask: np.ndarray,
                  fx: float, fy: float, cx: float, cy: float,
                  z_min: float = 0.1, z_max: float = 3.0,
                  edge_filter: bool = True,
                  cloud_valid: np.ndarray = None) -> np.ndarray:
    """Pinhole unproject. Returns (N,3) float32 in camera frame."""
    if mask.shape != depth.shape:
        mask = cv2.resize(mask.astype(np.uint8), (depth.shape[1], depth.shape[0]),
                          interpolation=cv2.INTER_NEAREST).astype(bool)
    valid = np.isfinite(depth) & (depth > z_min) & (depth < z_max)
    if edge_filter:
        valid = _edge_agreement(depth, valid)
    if cloud_valid is not None:
        valid &= cloud_valid
    sel = mask & valid
    vs, us = np.where(sel)
    z = depth[vs, us]
    if len(z) == 0:
        return np.empty((0, 3), dtype=np.float32)
    us, vs = us.astype(np.float32), vs.astype(np.float32)
    x = (us - cx) * z / fx
    y = (vs - cy) * z / fy
    return np.stack([x, y, z], axis=1).astype(np.float32)


def _remove_outliers(pts: np.ndarray, nb_neighbors: int = 20,
                     std_ratio: float = 2.0) -> np.ndarray:
    """Depth-weighted statistical outlier removal (mirrors the node)."""
    if len(pts) <= nb_neighbors:
        return pts
    z = pts[:, 2]
    z_min = float(z.min())
    if z_min <= 0:
        z_min = 0.01
    weights = (z_min / np.clip(z, z_min, None)).reshape(-1, 1)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts * weights)
    _, inlier_idx = pcd.remove_statistical_outlier(
        nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    return pts[inlier_idx]


def _keep_dominant_cluster(pts: np.ndarray, eps: float = 0.02,
                           min_samples: int = 10) -> np.ndarray:
    """DBSCAN — keep the densest×nearest cluster (mirrors the node)."""
    if len(pts) < min_samples:
        return pts
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    labels = np.array(pcd.cluster_dbscan(
        eps=eps, min_points=min_samples, print_progress=False))
    unique_labels = set(labels) - {-1}
    if not unique_labels:
        return pts
    best_label, best_score = -1, -1.0
    for lbl in unique_labels:
        mask = labels == lbl
        cluster_pts = pts[mask]
        score = mask.sum() * float(
            np.mean(1.0 / np.clip(cluster_pts[:, 2], 0.01, None)))
        if score > best_score:
            best_score, best_label = score, lbl
    return pts[labels == best_label]


def _cloud_to_xyz(cloud_msg, mask: np.ndarray) -> np.ndarray:
    """Pull masked XYZ from ZED's organized cloud (body frame) → optical
    convention (X-right, Y-down, Z-fwd), matching the node's extraction."""
    H, W = cloud_msg.height, cloud_msg.width
    buf = np.frombuffer(cloud_msg.data, dtype=np.uint8).reshape(
        H, W, cloud_msg.point_step)
    xyz = buf[:, :, 0:12].copy().view(np.float32).reshape(H, W, 3)
    if mask.shape != (H, W):
        mask = cv2.resize(mask.astype(np.uint8), (W, H),
                          interpolation=cv2.INTER_NEAREST).astype(bool)
    pts = xyz[mask]
    pts = pts[np.isfinite(pts).all(axis=1)]
    if len(pts) == 0:
        return np.empty((0, 3), dtype=np.float32)
    fwd = pts[:, 0]
    pts = pts[(fwd > 0.1) & (fwd < 3.0)]
    if len(pts) == 0:
        return np.empty((0, 3), dtype=np.float32)
    # body (X-fwd, Y-left, Z-up) → optical (X-right, Y-down, Z-fwd)
    return np.stack([-pts[:, 1], -pts[:, 2], pts[:, 0]], axis=1).astype(np.float32)


def _filter_cloud(pts: np.ndarray) -> np.ndarray:
    """Same cleanup the node applies before GraspNet: outlier + DBSCAN."""
    if len(pts) == 0:
        return pts
    pts = _remove_outliers(pts)
    pts = _keep_dominant_cluster(pts)
    return pts


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
        self._cloud_lock  = threading.Lock()
        self._mask_lock   = threading.Lock()
        self._det_lock    = threading.Lock()

        self._fx = self._fy = self._cx = self._cy = None
        self._depth_msg  = None
        self._cloud_msg  = None
        self._mask_msg   = None
        self._use_cloud  = not args.use_depth
        self._target_tid = args.id       # set when filtering by id
        self._target_cls = ' '.join(args.class_name) if args.class_name else None
        self._bbox       = None          # BoundingBox2D
        self._found_tid  = None          # resolved tracker id

        be = ReliabilityPolicy.BEST_EFFORT
        kl = HistoryPolicy.KEEP_LAST
        qos = QoSProfile(reliability=be, history=kl, depth=5)

        self.create_subscription(CameraInfo, args.info,  self._info_cb,  10)
        # Always subscribe to the cloud: cloud mode uses it as the source, depth
        # mode uses it to gate out pixels the cloud doesn't have.
        self.create_subscription(
            PointCloud2, args.cloud_topic, self._cloud_cb, qos)
        if not self._use_cloud:
            self.create_subscription(Image, args.depth, self._depth_cb, qos)
        self.create_subscription(Image,      args.masks, self._mask_cb,  qos)
        self.create_subscription(Detection2DArray, args.dets, self._det_cb, qos)

        self._cli = self.create_client(GraspFromMask, '/grasp/from_mask')

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

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

    def _cloud_cb(self, msg: PointCloud2):
        with self._cloud_lock:
            self._cloud_msg = msg

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

        if self._use_cloud:
            print('Waiting for point cloud …')
            if not self._wait('point cloud', lambda: self._cloud_msg is not None):
                return
        else:
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

        # /yolo/masks is a multi-object label image; pass it straight through
        # with the target tracker_id and let the node select that object.
        with self._mask_lock:
            mask_msg = self._mask_msg
        label = np.frombuffer(mask_msg.data, dtype=np.uint16).reshape(
            mask_msg.height, mask_msg.width)

        req = GraspFromMask.Request()
        req.mask            = mask_msg
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
        # Timing / point counts now live in the node terminal log, not the srv.
        from scipy.spatial.transform import Rotation as R

        def _print_poses(poses, scores, widths, frame_id,
                         height_below=None, height_above=None):
            print(f'\n  frame: {frame_id}')
            has_height = height_below is not None and height_above is not None
            print(f'  {"#":>3}  {"score":>6}  {"width":>7}  '
                  f'{"pos_x":>7} {"pos_y":>7} {"pos_z":>7}  '
                  f'{"qx":>7} {"qy":>7} {"qz":>7} {"qw":>7}  '
                  f'{"roll":>7} {"pitch":>7} {"yaw":>7}  (rad)'
                  + ('  {"below":>7} {"above":>7}  (m)' if has_height else ''))
            for i, (pose, s, w) in enumerate(zip(poses, scores, widths)):
                p = pose.position
                q = pose.orientation
                rpy = R.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz', degrees=False)
                height_str = ''
                if has_height and i < len(height_below):
                    height_str = (f'  {height_below[i]:>7.3f} {height_above[i]:>7.3f}')
                print(f'  [{i:>2}]  {s:>6.3f}  {w*100:>6.1f}cm  '
                      f'{p.x:>7.3f} {p.y:>7.3f} {p.z:>7.3f}  '
                      f'{q.x:>7.4f} {q.y:>7.4f} {q.z:>7.4f} {q.w:>7.4f}  '
                      f'{rpy[0]:>7.4f} {rpy[1]:>7.4f} {rpy[2]:>7.4f}'
                      + height_str)

        # Grasp poses come straight from the service in the planning frame
        # (position + EE-aligned orientation). The service no longer returns
        # camera-frame poses, and it already applies the TF + EE alignment.
        planning_frame = resp.planning_frame or self._args.planning_frame

        if resp.poses_base.poses:
            print(f'\n── Grasp poses ({planning_frame}) ───────────────────────────')
            _print_poses(resp.poses_base.poses, resp.scores, resp.widths,
                         planning_frame,
                         resp.height_below_grasp, resp.height_above_grasp)
        else:
            print(f'\n── Grasp poses ({planning_frame}): empty — the service '
                  f'could not resolve TF {planning_frame} ← camera at request time')

        # Approach poses come directly from the service response.
        if resp.approach_poses_base.poses:
            print(f'\n── Approach poses ({planning_frame}) — move here FIRST ──────')
            _print_poses(resp.approach_poses_base.poses,
                         resp.scores, resp.widths, planning_frame)

        # ── Object bounding box + height decomposition ────────────────────────
        if resp.object_size.x > 0:
            s = resp.object_size
            c = resp.object_bbox_pose.position
            print(f'\n── Object AABB ({resp.planning_frame}) ──────────────────────────')
            print(f'  size   : {s.x*100:.1f} × {s.y*100:.1f} × {s.z*100:.1f} cm  '
                  f'(x × y × z)')
            print(f'  centre : ({c.x:.3f}, {c.y:.3f}, {c.z:.3f}) m')
            if resp.height_below_grasp:
                print(f'\n── Grasp height decomposition (top grasp) ───────────────────')
                print(f'  below gripper : {resp.height_below_grasp[0]*100:6.1f} cm  '
                      f'← object bottom to grasp point')
                print(f'  above gripper : {resp.height_above_grasp[0]*100:6.1f} cm  '
                      f'← grasp point to object top')
                print(f'  total height  : {s.z*100:6.1f} cm')

        # ── build point cloud (ZED cloud or depth unprojection) ───────────────
        # Reuse the label image read above; the viewer mask matches the object
        # the node selects from the multi-object mask via tracker_id.
        mask = (label == tid)

        if self._use_cloud:
            with self._cloud_lock:
                cloud_msg = self._cloud_msg
            pts_raw = _cloud_to_xyz(cloud_msg, mask)
        else:
            with self._depth_lock:
                depth_msg = self._depth_msg
            if depth_msg.encoding == "16UC1":
                depth = np.frombuffer(depth_msg.data, dtype=np.uint16).reshape(
                    depth_msg.height, depth_msg.width).astype(np.float32) / 1000.0
            else:
                depth = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(
                    depth_msg.height, depth_msg.width)
            with self._info_lock:
                fx, fy, cx, cy = self._fx, self._fy, self._cx, self._cy
            # Gate against the cloud's validity — drop pixels the cloud lacks.
            cloud_valid = None
            if not args.no_cloud_gate:
                with self._cloud_lock:
                    cmsg = self._cloud_msg
                if cmsg is not None:
                    cloud_valid = _cloud_validity(cmsg, depth.shape[0], depth.shape[1])
                else:
                    print('  (no cloud yet — skipping cloud-validity gate)')
            pts_raw = _depth_to_xyz(depth, mask, fx, fy, cx, cy,
                                    edge_filter=not args.no_edge_filter,
                                    cloud_valid=cloud_valid)
        # Same cleanup the node applies before GraspNet (outlier + DBSCAN),
        # so the viewer cloud + AABB match what the grasps were computed on.
        pts = _filter_cloud(pts_raw)
        print(f'\nVisualising {len(pts)} points (raw {len(pts_raw)}) …')
        print('  (gripper geometry: the service no longer returns camera-frame\n'
              '   poses — view the gripper markers in RViz on '
              '/grasp/debug/grasp_markers)')

        pcd   = _xyz_to_pcd(pts)
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)

        # AABB in camera frame — computed from the local point cloud.
        geoms = [pcd, frame]
        if len(pts) > 0:
            aabb = o3d.geometry.AxisAlignedBoundingBox.create_from_points(
                o3d.utility.Vector3dVector(pts))
            aabb.color = (1.0, 0.5, 0.0)   # orange wireframe
            geoms.append(aabb)

        o3d.visualization.draw_geometries(
            geoms,
            window_name=f'GraspNet cloud — {target_str}',
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
    parser.add_argument('--cloud-topic',
                        default='/zed_head/zed_node/point_cloud/cloud_registered',
                        help='ZED organized point cloud topic')
    parser.add_argument('--use-depth', action='store_true',
                        help='Build the viewer cloud by unprojecting depth '
                             'instead of using the ZED cloud (default: cloud)')
    parser.add_argument('--no-edge-filter', action='store_true',
                        help='Disable the depth-path flying-pixel filter '
                             '(only relevant with --use-depth)')
    parser.add_argument('--no-cloud-gate', action='store_true',
                        help='Disable gating depth pixels against the cloud '
                             "validity (keeps points the cloud doesn't have)")
    parser.add_argument('--masks', default='/yolo/masks',
                        help='YOLO mask label image topic')
    parser.add_argument('--dets',  default='/yolo/tracked_detections_2d',
                        help='YOLO Detection2DArray topic')
    parser.add_argument('--planning-frame', default='base_footprint',
                        help='MoveIt planning frame to transform poses into (default: base_footprint)')

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
