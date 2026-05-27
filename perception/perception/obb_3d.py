#!/usr/bin/env python3

import json
import threading
import time
from collections import OrderedDict, deque
from typing import Dict, Optional, NamedTuple

import numpy as np
import open3d as o3d
import rclpy
import rclpy.time
import tf2_ros
import tf2_ros.buffer
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_srvs.srv import Trigger
from walkie_perception.srv import GetObjectCloud
from vision_msgs.msg import (
    BoundingBox3D,
    Detection2DArray,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)
from visualization_msgs.msg import Marker, MarkerArray
import message_filters


class OBB(NamedTuple):
    center: np.ndarray
    size:   np.ndarray
    quat:   np.ndarray


class ClusterResult(NamedTuple):
    obb:          OBB
    pts_map:      np.ndarray
    pts_cam:      np.ndarray
    n_candidates: int
    centroid_cam: np.ndarray


def _make_text_marker(frame_id, stamp, ns, mid, pos, text, scale=0.12, lifetime=None):
    m                  = Marker()
    m.header.frame_id  = frame_id
    m.header.stamp     = stamp
    m.ns               = ns
    m.id               = mid
    m.type             = Marker.TEXT_VIEW_FACING
    m.action           = Marker.ADD
    m.pose.position    = Point(x=float(pos[0]), y=float(pos[1]),
                               z=float(pos[2]) + 0.15)
    m.pose.orientation = Quaternion(w=1.0)
    m.scale.z          = scale
    m.color.r = m.color.g = m.color.b = 1.0
    m.color.a          = 0.9
    m.text             = text
    m.lifetime         = lifetime if lifetime is not None else Duration(seconds=0.5).to_msg()
    return m


_BOX_EDGES = [
    (0,1),(0,2),(1,3),(2,3),
    (4,5),(4,6),(5,7),(6,7),
    (0,4),(1,5),(2,6),(3,7),
]


def _make_wireframe_marker(frame_id, stamp, ns, mid, obb, lifetime,
                            r, g, b, line_width=0.008):
    h     = obb.size / 2.0
    R_mat = R.from_quat(obb.quat).as_matrix()
    offsets = np.array([
        [ h[0],  h[1],  h[2]], [-h[0],  h[1],  h[2]],
        [ h[0], -h[1],  h[2]], [-h[0], -h[1],  h[2]],
        [ h[0],  h[1], -h[2]], [-h[0],  h[1], -h[2]],
        [ h[0], -h[1], -h[2]], [-h[0], -h[1], -h[2]],
    ])
    corners = (R_mat @ offsets.T).T + obb.center
    m             = Marker()
    m.header.frame_id = frame_id
    m.header.stamp    = stamp
    m.ns          = ns
    m.id          = mid
    m.type        = Marker.LINE_LIST
    m.action      = Marker.ADD
    m.lifetime    = lifetime
    m.scale.x     = line_width
    m.color.r     = float(r)
    m.color.g     = float(g)
    m.color.b     = float(b)
    m.color.a     = 1.0
    m.pose.orientation.w = 1.0
    for i, j in _BOX_EDGES:
        m.points.append(Point(x=float(corners[i,0]),
                              y=float(corners[i,1]),
                              z=float(corners[i,2])))
        m.points.append(Point(x=float(corners[j,0]),
                              y=float(corners[j,1]),
                              z=float(corners[j,2])))
    return m


def _iou_2d(a, b) -> float:
    ax1 = a.bbox.center.position.x - a.bbox.size_x / 2
    ay1 = a.bbox.center.position.y - a.bbox.size_y / 2
    ax2 = ax1 + a.bbox.size_x
    ay2 = ay1 + a.bbox.size_y
    bx1 = b.bbox.center.position.x - b.bbox.size_x / 2
    by1 = b.bbox.center.position.y - b.bbox.size_y / 2
    bx2 = bx1 + b.bbox.size_x
    by2 = by1 + b.bbox.size_y
    ix = max(0.0, min(ax2, bx2) - max(ax1, bx1))
    iy = max(0.0, min(ay2, by2) - max(ay1, by1))
    inter = ix * iy
    if inter == 0.0:
        return 0.0
    union = a.bbox.size_x * a.bbox.size_y + b.bbox.size_x * b.bbox.size_y - inter
    return inter / union if union > 0 else 0.0


class OBB3D(Node):

    _DEPTH_ENCODINGS = {'32FC1': np.float32, '16UC1': np.uint16}

    def __init__(self) -> None:
        super().__init__('obb3d')

        self.declare_parameter('target_frame',             'map')
        self.declare_parameter('promotion_views',          5)
        self.declare_parameter('min_detection_score',      0.3)
        self.declare_parameter('max_depth_m',              8.0)
        self.declare_parameter('min_depth_m',              0.1)
        self.declare_parameter('voxel_size_m',             0.01)
        self.declare_parameter('dbscan_eps_m',             0.05)
        self.declare_parameter('dbscan_min_points',        10)
        self.declare_parameter('max_cluster_extent_m',     2.0)
        self.declare_parameter('pca_debounce_views',       20)
        self.declare_parameter('sliding_window_frames',    30)
        self.declare_parameter('max_database_size',        200)
        self.declare_parameter('marker_lifetime_s',        0.5)
        self.declare_parameter('max_frame_age_s',          0.30)
        self.declare_parameter('confidence_topic',
            '/zed_head/zed_nodeconfidence/confidence_map')
        self.declare_parameter('min_confidence',           70)
        self.declare_parameter('decay_miss_frames',        5)
        self.declare_parameter('ransac_enabled',           True)
        self.declare_parameter('ransac_distance_thr',      0.02)
        self.declare_parameter('ransac_n_iterations',      100)
        self.declare_parameter('ransac_min_plane_pts',     50)
        self.declare_parameter('iou_skip_threshold',       0.3)
        self.declare_parameter('max_dbscan_points',        2000)
        self.declare_parameter('centroid_drift_thr',       0.3)
        self.declare_parameter('id_reuse_distance_thr',    1.0)
        self.declare_parameter('eigenvalue_flat_thr',      1e-4)
        self.declare_parameter('eigenvalue_cyl_ratio',     0.2)
        self.declare_parameter('per_frame_budget_ms',      30.0)
        self.declare_parameter('initial_box_size_m',       0.05)
        self.declare_parameter('min_cluster_points',       10)

        p = self.get_parameter
        self._target_frame        = p('target_frame').value
        self.promotion_views      = p('promotion_views').value
        self.min_det_score        = p('min_detection_score').value
        self.max_depth_m          = p('max_depth_m').value
        self.min_depth_m          = p('min_depth_m').value
        self.voxel_size           = p('voxel_size_m').value
        self.dbscan_eps           = p('dbscan_eps_m').value
        self.dbscan_min_pts       = p('dbscan_min_points').value
        self.max_cluster_extent   = p('max_cluster_extent_m').value
        self.pca_debounce         = p('pca_debounce_views').value
        self.sliding_window       = p('sliding_window_frames').value
        self.max_db_size          = p('max_database_size').value
        self.marker_lifetime_s    = p('marker_lifetime_s').value
        self.max_frame_age_s      = p('max_frame_age_s').value
        self.confidence_topic     = p('confidence_topic').value
        self.min_confidence       = p('min_confidence').value
        self.decay_miss_frames    = p('decay_miss_frames').value
        self.ransac_enabled       = p('ransac_enabled').value
        self.ransac_dist_thr      = p('ransac_distance_thr').value
        self.ransac_n_iter        = p('ransac_n_iterations').value
        self.ransac_min_pts       = p('ransac_min_plane_pts').value
        self.iou_skip_thr         = p('iou_skip_threshold').value
        self.max_dbscan_pts       = p('max_dbscan_points').value
        self.centroid_drift_thr   = p('centroid_drift_thr').value
        self.id_reuse_thr         = p('id_reuse_distance_thr').value
        self.eig_flat_thr         = p('eigenvalue_flat_thr').value
        self.eig_cyl_ratio        = p('eigenvalue_cyl_ratio').value
        self.per_frame_budget_ms  = p('per_frame_budget_ms').value
        self.initial_box_size_m   = p('initial_box_size_m').value
        self.min_cluster_points   = p('min_cluster_points').value

        self.fx = self.fy = self.cx = self.cy = None
        self._depth_encoding_logged = False

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._tf_consecutive_failures = 0
        self._TF_FAILURE_WARN_LIMIT   = 30

        self._latest_confidence: Optional[np.ndarray] = None
        self._conf_lock = threading.Lock()

        self._latest_mask: Optional[np.ndarray] = None
        self._mask_lock = threading.Lock()

        self._db: OrderedDict = OrderedDict()
        self._db_lock = threading.Lock()

        self._marker_id_map: Dict[str, int] = {}
        self._next_marker_id = 0
        self._prev_published_ids: set = set()

        self._scratch_pcd = o3d.geometry.PointCloud()

        self._t_decode = self._t_tf = 0.0
        self._t_cluster = self._t_pca = self._t_publish = 0.0
        self._frame_count = 0
        self._frame_drop_age = self._frame_drop_tf = 0
        self._budget_skipped_dets = 0
        self._diag_lock = threading.Lock()

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=5)

        self.create_subscription(CameraInfo,
            '/zed_head/zed_nodedepth/camera_info', self._info_cb, qos)

        if self.confidence_topic:
            self.create_subscription(Image, self.confidence_topic,
                                     self._confidence_cb, qos)
            self.get_logger().info(
                f'ZED confidence filter enabled on {self.confidence_topic} '
                f'(min_confidence={self.min_confidence})')

        self.create_subscription(Image, '/yolo/masks', self._mask_cb, 10)

        depth_sub = message_filters.Subscriber(
            self, Image, '/zed_head/zed_nodedepth/depth_registered',
            qos_profile=qos)
        yolo_sub  = message_filters.Subscriber(
            self, Detection2DArray, '/yolo/tracked_detections_2d',
            qos_profile=10)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [depth_sub, yolo_sub], queue_size=10, slop=0.12)
        self.ts.registerCallback(self._sync_cb)

        self.det3d_pub  = self.create_publisher(Detection3DArray,
                                                'apex/detections_3d', 10)
        self.marker_pub = self.create_publisher(MarkerArray,
                                                'apex/markers', 10)
        self.dbg_cluster_pub = self.create_publisher(
            PointCloud2, 'apex/debug/raw_cluster', 10)
        self.dbg_accum_pub   = self.create_publisher(
            PointCloud2, 'apex/debug/accumulated_cloud', 10)
        self.dbg_label_pub   = self.create_publisher(
            MarkerArray, 'apex/debug/labels', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray,
                                              '/diagnostics', 10)

        self.create_timer(1.0, self._publish_diagnostics)
        self.create_service(Trigger, 'apex/dump_database',
                            self._dump_database_cb)
        self.create_service(GetObjectCloud, 'apex/get_object_cloud',
                            self._get_object_cloud_cb)

        self.get_logger().info(
            f'OBB3D ready — target_frame={self._target_frame} '
            f'decay={self.decay_miss_frames} frames')

    def _info_cb(self, msg: CameraInfo) -> None:
        if self.fx is None:
            self.fx, self.fy = msg.k[0], msg.k[4]
            self.cx, self.cy = msg.k[2], msg.k[5]
            self.get_logger().info(
                f'Intrinsics: fx={self.fx:.1f} fy={self.fy:.1f} '
                f'cx={self.cx:.1f} cy={self.cy:.1f}')

    def _confidence_cb(self, msg: Image) -> None:
        if msg.encoding == '8UC1':
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                (msg.height, msg.width)).astype(np.float32)
        elif msg.encoding == '32FC1':
            arr = np.frombuffer(msg.data, dtype=np.float32).reshape(
                (msg.height, msg.width))
        else:
            self.get_logger().warn(
                f'Unknown confidence encoding: {msg.encoding}',
                throttle_duration_sec=5.0)
            return
        with self._conf_lock:
            self._latest_confidence = arr

    def _mask_cb(self, msg: Image) -> None:
        if msg.encoding != '16UC1':
            self.get_logger().warn(
                f'Expected 16UC1 mask, got {msg.encoding}',
                throttle_duration_sec=5.0)
            return
        arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(
            (msg.height, msg.width))
        with self._mask_lock:
            self._latest_mask = arr

    def _get_tf_matrix(self, target: str, source: str, stamp) -> Optional[np.ndarray]:
        try:
            t = self.tf_buffer.lookup_transform(target, source, stamp,
                                                Duration(seconds=0.05))
            self._tf_consecutive_failures = 0
        except tf2_ros.ExtrapolationException:
            try:
                t = self.tf_buffer.lookup_transform(target, source,
                    rclpy.time.Time(), Duration(seconds=0.05))
                self._tf_consecutive_failures = 0
            except Exception as exc:
                self._tf_consecutive_failures += 1
                if self._tf_consecutive_failures == self._TF_FAILURE_WARN_LIMIT:
                    self.get_logger().warn(
                        f'TF {source}→{target} failed '
                        f'{self._TF_FAILURE_WARN_LIMIT} consecutive times. '
                        f'Is SLAM running? Last error: {exc}')
                return None
        except Exception as exc:
            self._tf_consecutive_failures += 1
            self.get_logger().warn(f'TF {source}→{target}: {exc}',
                                   throttle_duration_sec=2.0)
            return None

        mat = np.eye(4)
        mat[:3, :3] = R.from_quat([t.transform.rotation.x,
                                    t.transform.rotation.y,
                                    t.transform.rotation.z,
                                    t.transform.rotation.w]).as_matrix()
        mat[:3, 3]  = [t.transform.translation.x,
                       t.transform.translation.y,
                       t.transform.translation.z]
        return mat

    @staticmethod
    def _reject_outliers(pts: np.ndarray, sigma: float = 2.5) -> np.ndarray:
        centroid = np.median(pts, axis=0)
        dists    = np.linalg.norm(pts - centroid, axis=1)
        med_d    = np.median(dists)
        mad      = np.median(np.abs(dists - med_d))
        return pts[dists <= med_d + sigma * (1.4826 * mad + 1e-6)]

    @staticmethod
    def _rigid_inverse(mat: np.ndarray) -> np.ndarray:
        inv         = np.eye(4)
        R_T         = mat[:3, :3].T
        inv[:3, :3] = R_T
        inv[:3, 3]  = -(R_T @ mat[:3, 3])
        return inv

    def _fit_obb(self, pts: np.ndarray,
                 prev_quat: Optional[np.ndarray] = None) -> OBB:
        mean     = pts.mean(axis=0)
        centered = pts - mean
        cov      = np.cov(centered, rowvar=False)
        eigvals, vecs = np.linalg.eigh(cov)
        eigvals = eigvals[::-1]
        vecs    = vecs[:, ::-1]
        if np.linalg.det(vecs) < 0:
            vecs[:, 2] = -vecs[:, 2]

        if eigvals[2] < self.eig_flat_thr:
            vecs[:, 2] = np.array([0.0, 0.0, 1.0])
            vecs[:, 1] = np.cross(vecs[:, 2], vecs[:, 0])
            vecs[:, 1] /= np.linalg.norm(vecs[:, 1]) + 1e-9

        if prev_quat is not None:
            ratio = (eigvals[0] - eigvals[1]) / (eigvals[0] + 1e-9)
            if ratio < self.eig_cyl_ratio:
                proj   = centered @ vecs
                mn, mx = proj.min(axis=0), proj.max(axis=0)
                size   = mx - mn
                center = mean + vecs @ ((mx + mn) / 2.0)
                return OBB(center, size, prev_quat)

        proj   = centered @ vecs
        mn, mx = proj.min(axis=0), proj.max(axis=0)
        size   = mx - mn
        center = mean + vecs @ ((mx + mn) / 2.0)
        quat   = R.from_matrix(vecs).as_quat()
        return OBB(center, size, quat)

    def _remove_plane(self, pts: np.ndarray) -> np.ndarray:
        if not self.ransac_enabled or len(pts) < self.ransac_min_pts:
            return pts

        self._scratch_pcd.points = o3d.utility.Vector3dVector(pts)
        try:
            _, inlier_idx = self._scratch_pcd.segment_plane(
                distance_threshold=self.ransac_dist_thr,
                ransac_n=3,
                num_iterations=self.ransac_n_iter,
            )
        except Exception:
            return pts

        mask = np.ones(len(pts), dtype=bool)
        mask[inlier_idx] = False
        remaining = pts[mask]

        if len(remaining) < self.dbscan_min_pts:
            return pts
        return remaining

    def _extract_robust_cluster(self, depth_img: np.ndarray, det2d,
                                cam_to_map: np.ndarray) -> Optional[ClusterResult]:
        img_h, img_w = depth_img.shape
        box_cx = det2d.bbox.center.position.x
        box_cy = det2d.bbox.center.position.y
        sx = det2d.bbox.size_x * 0.8
        sy = det2d.bbox.size_y * 0.8

        u_min = max(0,     int(box_cx - sx / 2))
        u_max = min(img_w, int(box_cx + sx / 2))
        v_min = max(0,     int(box_cy - sy / 2))
        v_max = min(img_h, int(box_cy + sy / 2))

        roi = depth_img[v_min:v_max, u_min:u_max]
        valid = (np.isfinite(roi)
                 & (roi > self.min_depth_m)
                 & (roi < self.max_depth_m))
        if not np.any(valid):
            return None

        z      = roi[valid]
        vv, uu = np.where(valid)
        uu    += u_min
        vv    += v_min
        pts_cam = np.column_stack((
            (uu - self.cx) * z / self.fx,
            (vv - self.cy) * z / self.fy,
            z,
        ))

        self._scratch_pcd.points = o3d.utility.Vector3dVector(pts_cam)
        downsampled = self._scratch_pcd.voxel_down_sample(self.voxel_size)
        down_pts = np.asarray(downsampled.points)
        if len(down_pts) > self.max_dbscan_pts:
            self._scratch_pcd.points = o3d.utility.Vector3dVector(down_pts)
            downsampled = self._scratch_pcd.voxel_down_sample(
                self.voxel_size * 2.0)
            down_pts = np.asarray(downsampled.points)

        if len(down_pts) < self.dbscan_min_pts:
            return None

        z_median = float(np.median(down_pts[:, 2]))

        down_pts = self._remove_plane(down_pts)
        if len(down_pts) < self.dbscan_min_pts:
            return None
        eps_scale    = float(np.clip(z_median / 1.5, 0.6, 2.0))
        adaptive_eps = self.dbscan_eps * eps_scale

        self._scratch_pcd.points = o3d.utility.Vector3dVector(down_pts)
        labels = np.array(self._scratch_pcd.cluster_dbscan(
            eps=adaptive_eps,
            min_points=self.dbscan_min_pts,
            print_progress=False,
        ))
        if len(labels) == 0 or np.all(labels == -1):
            return None

        ray_dir = np.array([
            (box_cx - self.cx) / self.fx,
            (box_cy - self.cy) / self.fy,
            1.0,
        ])
        ray_dir /= np.linalg.norm(ray_dir)

        best_cluster: Optional[np.ndarray] = None
        best_score   = float('inf')
        n_candidates = 0

        for label in np.unique(labels[labels >= 0]):
            c_pts = down_pts[labels == label]
            if len(c_pts) < self.dbscan_min_pts:
                continue
            extents = c_pts.max(axis=0) - c_pts.min(axis=0)
            if np.linalg.norm(extents) > self.max_cluster_extent:
                continue

            n_candidates += 1
            norms     = np.linalg.norm(c_pts, axis=1, keepdims=True)
            unit_pts  = c_pts / np.maximum(norms, 1e-6)
            cross     = np.cross(unit_pts, ray_dir)
            ray_score = np.percentile(np.linalg.norm(cross, axis=1), 10)
            depth_score = np.std(c_pts @ ray_dir)
            score = ray_score + 0.5 * depth_score

            if score < best_score:
                best_score   = score
                best_cluster = c_pts

        if best_cluster is None:
            return None

        centroid_cam = best_cluster.mean(axis=0)

        homo    = np.hstack((best_cluster, np.ones((len(best_cluster), 1))))
        pts_map = (cam_to_map @ homo.T).T[:, :3]
        obb     = self._fit_obb(pts_map)

        return ClusterResult(
            obb=obb, pts_map=pts_map, pts_cam=best_cluster,
            n_candidates=n_candidates, centroid_cam=centroid_cam,
        )

    def _extract_from_mask(
        self,
        depth_img: np.ndarray,
        det2d,
        tracker_id: str,
        cam_to_map: np.ndarray,
    ) -> Optional[ClusterResult]:
        with self._mask_lock:
            mask_img = self._latest_mask

        tid_int = None
        try:
            tid_int = int(tracker_id)
        except (ValueError, TypeError):
            pass

        if (mask_img is not None
                and tid_int is not None
                and tid_int > 0
                and mask_img.shape == depth_img.shape):

            object_mask = (mask_img == tid_int)
            if np.any(object_mask):
                valid = (object_mask
                         & np.isfinite(depth_img)
                         & (depth_img > self.min_depth_m)
                         & (depth_img < self.max_depth_m))
                if np.any(valid):
                    z = depth_img[valid]
                    vv, uu = np.where(valid)
                    pts_cam = np.column_stack((
                        (uu - self.cx) * z / self.fx,
                        (vv - self.cy) * z / self.fy,
                        z,
                    ))
                    if len(pts_cam) >= self.min_cluster_points:
                        pts_cam = self._reject_outliers(pts_cam)
                        if len(pts_cam) >= self.min_cluster_points:
                            homo    = np.hstack((pts_cam,
                                                 np.ones((len(pts_cam), 1))))
                            pts_map = (cam_to_map @ homo.T).T[:, :3]
                            obb     = self._fit_obb(pts_map)
                            return ClusterResult(
                                obb=obb,
                                pts_map=pts_map,
                                pts_cam=pts_cam,
                                n_candidates=1,
                                centroid_cam=pts_cam.mean(axis=0),
                            )

        return self._extract_robust_cluster(depth_img, det2d, cam_to_map)

    def _get_marker_id(self, obj_id: str) -> int:
        if obj_id not in self._marker_id_map:
            self._marker_id_map[obj_id] = self._next_marker_id
            self._next_marker_id += 1
        return self._marker_id_map[obj_id]

    def _evict_lru_if_needed(self) -> None:
        while len(self._db) > self.max_db_size:
            evicted_id, _ = self._db.popitem(last=False)
            self._marker_id_map.pop(evicted_id, None)
            self.get_logger().warn(
                f'DB full; LRU evicted: {evicted_id}',
                throttle_duration_sec=5.0)

    def _accumulate_and_refit(self, frame_pts_list: list,
                              prev_quat: np.ndarray) -> tuple:
        all_pts = np.vstack(frame_pts_list)

        self._scratch_pcd.points = o3d.utility.Vector3dVector(all_pts)
        merged    = self._scratch_pcd.voxel_down_sample(self.voxel_size)
        clean_pts = self._reject_outliers(np.asarray(merged.points))
        if len(clean_pts) < self.dbscan_min_pts:
            return None, len(clean_pts)

        new_obb = self._fit_obb(clean_pts, prev_quat=prev_quat)
        return new_obb, len(clean_pts)

    def _sync_cb(self, depth_msg: Image, yolo_msg: Detection2DArray) -> None:
        if self.fx is None:
            return

        now_ns   = self.get_clock().now().nanoseconds
        stamp_ns = (depth_msg.header.stamp.sec * 10**9
                    + depth_msg.header.stamp.nanosec)
        age_s = (now_ns - stamp_ns) / 1e9
        self.get_logger().debug(
            f'[sync_cb] fired — depth_frame={depth_msg.header.frame_id} '
            f'age={age_s*1000:.1f}ms dets={len(yolo_msg.detections)}')
        if age_s > self.max_frame_age_s:
            self.get_logger().warn(
                f'[sync_cb] frame too old: {age_s*1000:.1f}ms > '
                f'{self.max_frame_age_s*1000:.0f}ms — dropping',
                throttle_duration_sec=1.0)
            with self._diag_lock:
                self._frame_drop_age += 1
            return

        t0 = time.perf_counter()
        dtype = self._DEPTH_ENCODINGS.get(depth_msg.encoding)
        if dtype is None:
            if not self._depth_encoding_logged:
                self.get_logger().error(
                    f'Unsupported depth encoding: "{depth_msg.encoding}".')
                self._depth_encoding_logged = True
            return
        if not self._depth_encoding_logged:
            self.get_logger().info(
                f'Depth encoding confirmed: {depth_msg.encoding}')
            self._depth_encoding_logged = True

        depth_img = np.frombuffer(depth_msg.data, dtype=dtype).reshape(
            (depth_msg.height, depth_msg.width))
        if dtype == np.uint16:
            depth_img = depth_img.astype(np.float32) / 1000.0
        with self._conf_lock:
            conf = self._latest_confidence

        if conf is not None and conf.shape == depth_img.shape:
            mask = conf < self.min_confidence
            if mask.any():
                depth_img = depth_img.copy()
                depth_img[mask] = np.nan
        t_decode = time.perf_counter() - t0
        t0 = time.perf_counter()
        cam_to_map = self._get_tf_matrix(
            self._target_frame, depth_msg.header.frame_id, depth_msg.header.stamp)
        t_tf = time.perf_counter() - t0
        if cam_to_map is None:
            self.get_logger().warn(
                '[sync_cb] TF lookup failed — dropping frame',
                throttle_duration_sec=1.0)
            with self._diag_lock:
                self._frame_drop_tf += 1
            return
        map_to_cam = self._rigid_inverse(cam_to_map)

        detections = yolo_msg.detections
        skip_ids: set = set()
        for i in range(len(detections)):
            for j in range(i + 1, len(detections)):
                if _iou_2d(detections[i], detections[j]) > self.iou_skip_thr:
                    si = detections[i].results[0].hypothesis.score \
                         if detections[i].results else 0.0
                    sj = detections[j].results[0].hypothesis.score \
                         if detections[j].results else 0.0
                    skip_ids.add(j if si >= sj else i)

        det_indices = sorted(
            range(len(detections)),
            key=lambda i: (
                -detections[i].results[0].hypothesis.score
                if detections[i].results else float('inf')
            ),
        )

        t_cluster_total = 0.0
        t_pca_total     = 0.0

        budget_start = time.perf_counter()
        budget_s     = self.per_frame_budget_ms / 1000.0
        budget_skipped_this_frame = 0

        for det_idx in det_indices:
            if det_idx in skip_ids:
                continue
            if (time.perf_counter() - budget_start) > budget_s:
                budget_skipped_this_frame += 1
                continue

            det2d = detections[det_idx]
            try:
                result         = det2d.results[0]
                tracker_id     = str(det2d.id)
                semantic_class = result.hypothesis.class_id
                confidence     = result.hypothesis.score
            except (IndexError, AttributeError) as e:
                self.get_logger().warn(f'[sync_cb] skipping det — bad fields: {e}',
                                       throttle_duration_sec=2.0)
                continue
            if confidence < self.min_det_score:
                continue
            self.get_logger().debug(
                f'[sync_cb] processing id={tracker_id} class={semantic_class} '
                f'conf={confidence:.2f}')
            t0 = time.perf_counter()
            cluster = self._extract_from_mask(
                depth_img, det2d, tracker_id, cam_to_map)
            t_cluster_total += time.perf_counter() - t0
            if cluster is None:
                continue

            self.dbg_cluster_pub.publish(
                self._make_pc2(cluster.pts_map, self._target_frame, depth_msg.header.stamp))
            t0 = time.perf_counter()
            with self._db_lock:
                if tracker_id not in self._db:
                    self.get_logger().info(
                        f'[db] NEW object id={tracker_id} class={semantic_class}')
                    first_obb = OBB(
                        center=cluster.obb.center,
                        size=np.full(3, self.initial_box_size_m),
                        quat=cluster.obb.quat,
                    )
                    self._db[tracker_id] = {
                        'obb':               first_obb,
                        'frame_pts':         deque([cluster.pts_map],
                                                   maxlen=self.sliding_window),
                        'class':             semantic_class,
                        'view_count':        1,
                        'refit_count':       0,
                        'miss_count':        0,
                        'last_pts':          len(cluster.pts_map),
                        'last_centroid_cam': cluster.centroid_cam,
                    }
                    self._evict_lru_if_needed()
                    work_for_this_id = None
                else:
                    obj = self._db[tracker_id]
                    self._db.move_to_end(tracker_id)

                    prev_centre_cam = map_to_cam @ np.array(
                        [*obj['obb'].center, 1.0])
                    cam_dist = np.linalg.norm(
                        cluster.centroid_cam - prev_centre_cam[:3])

                    if cam_dist > self.id_reuse_thr:
                        obj['obb']            = cluster.obb
                        obj['frame_pts']      = deque(
                            [cluster.pts_map], maxlen=self.sliding_window)
                        obj['view_count']     = 1
                        obj['refit_count']    = 0
                        obj['miss_count']     = 0
                        obj['last_centroid_cam'] = cluster.centroid_cam
                        work_for_this_id = None
                    else:
                        prev_c = obj.get('last_centroid_cam',
                                         cluster.centroid_cam)
                        drift = np.linalg.norm(cluster.centroid_cam - prev_c)
                        if drift > self.centroid_drift_thr:
                            obj['frame_pts'].clear()
                        obj['last_centroid_cam'] = cluster.centroid_cam
                        obj['frame_pts'].append(cluster.pts_map)
                        should_refit = (
                            obj['refit_count'] < self.pca_debounce
                            or obj['refit_count'] % max(
                                1, self.pca_debounce // 5) == 0
                        )
                        if should_refit:
                            work_for_this_id = (
                                tracker_id,
                                list(obj['frame_pts']),
                                obj['obb'].quat,
                            )
                        else:
                            work_for_this_id = None
                            obj['view_count'] += 1

            if work_for_this_id is not None:
                tid, frame_list, prev_quat = work_for_this_id
                new_obb, n_clean = self._accumulate_and_refit(
                    frame_list, prev_quat)

                with self._db_lock:
                    if tid in self._db:
                        obj = self._db[tid]
                        obj['last_pts'] = n_clean
                        if new_obb is not None:
                            obj['obb']         = new_obb
                            obj['refit_count'] += 1
                        obj['view_count'] += 1

            t_pca_total += time.perf_counter() - t0

        seen_ids = {
            str(det2d.id)
            for det2d in detections
            if det2d.results
            and det2d.results[0].hypothesis.score >= self.min_det_score
        }
        with self._db_lock:
            for oid, obj in self._db.items():
                obj['miss_count'] = (0 if oid in seen_ids
                                     else obj.get('miss_count', 0) + 1)
            ids_to_evict = [oid for oid, obj in self._db.items()
                            if obj['miss_count'] >= self.decay_miss_frames]
            for oid in ids_to_evict:
                self.get_logger().debug(f'Decay evicted: {oid}')
                del self._db[oid]
                self._marker_id_map.pop(oid, None)

        t0 = time.perf_counter()
        self._publish(depth_msg.header.stamp)
        t_publish = time.perf_counter() - t0

        with self._diag_lock:
            self._t_decode    += t_decode
            self._t_tf        += t_tf
            self._t_cluster   += t_cluster_total
            self._t_pca       += t_pca_total
            self._t_publish   += t_publish
            self._frame_count += 1
            self._budget_skipped_dets += budget_skipped_this_frame

    def _publish(self, stamp) -> None:
        det3d_arr = Detection3DArray()
        det3d_arr.header.frame_id = self._target_frame
        det3d_arr.header.stamp    = stamp
        marker_arr    = MarkerArray()
        label_markers = MarkerArray()
        frame_pts_data: list = []
        lifetime_forever = Duration(seconds=0).to_msg()

        with self._db_lock:
            current_ids = set(self._db.keys())
            for removed_id in self._prev_published_ids - current_ids:
                mid = self._marker_id_map.get(removed_id)
                if mid is not None:
                    for ns in ('apex_objects', 'apex_debug_labels',
                               'apex_orientation', 'apex_confirmed'):
                        dm                 = Marker()
                        dm.header.frame_id = self._target_frame
                        dm.header.stamp    = stamp
                        dm.ns              = ns
                        dm.id              = mid
                        dm.action          = Marker.DELETE
                        marker_arr.markers.append(dm)

            for obj_id, obj in self._db.items():
                obb      = obj['obb']
                mid      = self._get_marker_id(obj_id)
                promoted = obj['refit_count'] >= self.promotion_views
                lt       = lifetime_forever

                det3d        = Detection3D()
                det3d.header = det3d_arr.header
                det3d.bbox = BoundingBox3D(
                    center=Pose(
                        position=Point(x=float(obb.center[0]),
                                       y=float(obb.center[1]),
                                       z=float(obb.center[2])),
                        orientation=Quaternion(x=float(obb.quat[0]),
                                               y=float(obb.quat[1]),
                                               z=float(obb.quat[2]),
                                               w=float(obb.quat[3])),
                    ),
                    size=Vector3(x=float(obb.size[0]),
                                 y=float(obb.size[1]),
                                 z=float(obb.size[2])),
                )
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(obj['class'])
                hyp.hypothesis.score = float(min(1.0,
                    obj['refit_count'] / max(1, self.promotion_views)))
                det3d.results.append(hyp)
                det3d_arr.detections.append(det3d)

                m        = Marker()
                m.header = det3d_arr.header
                m.ns     = 'apex_objects'
                m.id     = mid
                m.type   = Marker.CUBE
                m.action = Marker.ADD
                m.pose   = det3d.bbox.center
                m.scale  = det3d.bbox.size
                if promoted:
                    m.color.r, m.color.g, m.color.b = 0.0, 1.0, 1.0
                else:
                    m.color.r, m.color.g, m.color.b = 1.0, 0.4, 0.0
                m.color.a = 0.15
                m.lifetime = lt
                marker_arr.markers.append(m)

                if promoted:
                    marker_arr.markers.append(
                        _make_wireframe_marker(
                            self._target_frame, stamp, 'apex_confirmed', mid,
                            obb, lt, r=0.0, g=1.0, b=1.0, line_width=0.010))

                R_mat    = R.from_quat(obb.quat).as_matrix()
                axis     = R_mat[:, 0]
                half_len = float(max(obb.size) * 0.6)
                cx = float(obb.center[0])
                cy = float(obb.center[1])
                cz = float(obb.center[2])
                arrow          = Marker()
                arrow.header   = det3d_arr.header
                arrow.ns       = 'apex_orientation'
                arrow.id       = mid
                arrow.type     = Marker.ARROW
                arrow.action   = Marker.ADD
                arrow.lifetime = lt
                arrow.scale.x  = 0.02
                arrow.scale.y  = 0.04
                arrow.scale.z  = 0.06
                arrow.color.r  = 1.0
                arrow.color.g  = 0.5
                arrow.color.b  = 0.0
                arrow.color.a  = 1.0
                arrow.points   = [
                    Point(x=cx, y=cy, z=cz),
                    Point(x=cx + float(axis[0]) * half_len,
                          y=cy + float(axis[1]) * half_len,
                          z=cz + float(axis[2]) * half_len),
                ]
                marker_arr.markers.append(arrow)

                tier  = '★ confirmed' if promoted else '○ accumulating'
                label = (f'{obj["class"]}  {tier}\n'
                         f'id:{obj_id[:6]}  pts:{obj.get("last_pts", 0)}')
                label_markers.markers.append(
                    _make_text_marker(self._target_frame, stamp,
                                      'apex_debug_labels', mid, obb.center,
                                      label, scale=0.12, lifetime=lt))

                if obj['frame_pts']:
                    frame_pts_data.append(
                        (obj_id, len(obj['frame_pts']), list(obj['frame_pts'])))

            self._prev_published_ids = current_ids

        accum_clouds = []
        for obj_id, n_frames, pts_list in frame_pts_data:
            pts = np.vstack(pts_list)
            self.get_logger().debug(
                f'[accum] id={obj_id[:6]} frames={n_frames} total_pts={len(pts)}')
            accum_clouds.append(pts)

        self.det3d_pub.publish(det3d_arr)
        self.marker_pub.publish(marker_arr)
        self.dbg_label_pub.publish(label_markers)
        if accum_clouds:
            self.dbg_accum_pub.publish(
                self._make_pc2(np.vstack(accum_clouds),
                               self._target_frame, stamp))

    def _publish_diagnostics(self) -> None:
        with self._diag_lock:
            fc = max(self._frame_count, 1)
            avg = {
                'decode_ms':       self._t_decode  / fc * 1e3,
                'tf_ms':           self._t_tf      / fc * 1e3,
                'cluster_ms':      self._t_cluster / fc * 1e3,
                'pca_ms':          self._t_pca     / fc * 1e3,
                'publish_ms':      self._t_publish / fc * 1e3,
                'frames':          self._frame_count,
                'drop_age':        self._frame_drop_age,
                'drop_tf':         self._frame_drop_tf,
                'budget_skipped':  self._budget_skipped_dets,
            }
            self._t_decode = self._t_tf = 0.0
            self._t_cluster = self._t_pca = self._t_publish = 0.0
            self._frame_count = self._frame_drop_age = 0
            self._frame_drop_tf = self._budget_skipped_dets = 0

        with self._db_lock:
            db_size  = len(self._db)
            promoted = sum(1 for o in self._db.values()
                           if o['refit_count'] >= self.promotion_views)
            total_pts = sum(o.get('last_pts', 0) for o in self._db.values())

        total_ms = sum(v for k, v in avg.items() if k.endswith('_ms'))
        level = (DiagnosticStatus.OK    if total_ms < 50  else
                 DiagnosticStatus.WARN  if total_ms < 100 else
                 DiagnosticStatus.ERROR)

        st = DiagnosticStatus()
        st.name = 'obb3d'
        st.hardware_id = 'zed2i'
        st.level = level
        st.message = (
            f'{"OK" if level == DiagnosticStatus.OK else "SLOW"} | '
            f'{avg["frames"]:.0f} fps | {total_ms:.1f} ms/frame | '
            f'{db_size} objs | budget_skip={avg["budget_skipped"]:.0f}')
        st.values = [
            KeyValue(key=k, value=str(round(v, 2) if isinstance(v, float) else v))
            for k, v in {
                'frames':           int(avg['frames']),
                'drop_age':         int(avg['drop_age']),
                'drop_tf':          int(avg['drop_tf']),
                'budget_skipped':   int(avg['budget_skipped']),
                'db_objects':       db_size,
                'promoted':         promoted,
                'total_pts':        total_pts,
                'avg_decode_ms':    avg['decode_ms'],
                'avg_tf_ms':        avg['tf_ms'],
                'avg_cluster_ms':   avg['cluster_ms'],
                'avg_pca_ms':       avg['pca_ms'],
                'avg_publish_ms':   avg['publish_ms'],
                'avg_total_ms':     total_ms,
            }.items()
        ]
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [st]
        self.diag_pub.publish(arr)

    def _get_object_cloud_cb(self, request, response):
        obj_id = request.object_id
        with self._db_lock:
            if obj_id not in self._db:
                response.found   = False
                response.message = f'Object {obj_id!r} not in database'
                return response
            obj = self._db[obj_id]
            if not obj['frame_pts']:
                response.found   = False
                response.message = f'Object {obj_id!r} has no accumulated points'
                return response
            pts_list = list(obj['frame_pts'])
            n_frames = len(pts_list)

        pts   = np.vstack(pts_list)
        stamp = self.get_clock().now().to_msg()
        response.cloud   = self._make_pc2(pts, self._target_frame, stamp)
        response.found   = True
        response.message = f'{len(pts)} pts from {n_frames} frames'
        return response

    def _dump_database_cb(self, _req, resp) -> Trigger.Response:
        with self._db_lock:
            dump = {}
            for obj_id, obj in self._db.items():
                obb = obj['obb']
                dump[obj_id] = {
                    'class':       str(obj['class']),
                    'view_count':  obj['view_count'],
                    'refit_count': obj['refit_count'],
                    'miss_count':  obj['miss_count'],
                    'promoted':    obj['refit_count'] >= self.promotion_views,
                    'n_pts':           obj.get('last_pts', 0),
                    'window_size':     len(obj.get('frame_pts', [])),
                    'center':          obb.center.tolist(),
                    'size':            obb.size.tolist(),
                    'quat':            obb.quat.tolist(),
                }
        resp.success = True
        resp.message = json.dumps(dump, indent=2)
        return resp

    @staticmethod
    def _make_pc2(pts: np.ndarray, frame_id: str, stamp) -> PointCloud2:
        if pts.dtype != np.float32:
            pts = pts.astype(np.float32)
        if not pts.flags['C_CONTIGUOUS']:
            pts = np.ascontiguousarray(pts)

        msg              = PointCloud2()
        msg.header.frame_id = frame_id
        msg.header.stamp    = stamp
        msg.height       = 1
        msg.width        = len(pts)
        msg.is_dense     = True
        msg.is_bigendian = False
        msg.point_step   = 12
        msg.row_step     = 12 * len(pts)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.data = pts.tobytes()
        return msg


def main() -> None:
    rclpy.init()
    node = OBB3D()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
