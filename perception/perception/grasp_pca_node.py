#!/bin/sh
"exec" "${PERCEPTION_VENV:-$HOME/perception-venv/bin/python3}" "$0" "$@"
"""
grasp_pca_node.py

Zero-VRAM geometric grasp baseline for Walkie. Pure numpy/scipy/open3d —
no GPU, no model, always ready. Serves as a fallback for simple convex
shapes AND as a comparison baseline against GraspNet (grasp_from_mask_node).

Pipeline (mirrors grasp_from_mask_node's cloud caching / mask extraction):
  mask + bbox-fallback → masked organized-cloud points (optical convention)
  → table-plane removal → voxel downsample → PCA → shape classification
  → per-shape single-grasp heuristic → score → debug cloud + markers.

Services:
  /grasp/pca_heuristic   walkie_perception/srv/PCAHeuristicGrasp
  /grasp/pca/status      std_srvs/srv/Trigger

This node produces ONE primary grasp per call (best-guess heuristic), not a
sampler — unlike GraspNet's many candidates.

────────────────────────────────────────────────────────────────────────────
Tuning guide
────────────────────────────────────────────────────────────────────────────
  Symptom                              Adjustment
  ──────────────────────────────────────────────────────────────────────────
  Bottle classified as elongated_flat  Lower  cylindrical_ratio21_min
  Apple classified as cylindrical      Lower  spherical_ratio10_min
  Fork grasp lands mid-tines           Increase utensil_handle_frac → 1.0
  Cylinder grasp too close to cap/base Move cylinder_grip_height_frac → 0.5
  Spherical approach direction wrong   Check the TF lookup log line — verify
                                        planning_frame is correct and the
                                        camera is present in the /tf tree
────────────────────────────────────────────────────────────────────────────
"""
import threading
import time
from collections import deque
from typing import Tuple

import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import tf2_ros
import tf2_geometry_msgs  # noqa: F401 — registers Pose transform support

from geometry_msgs.msg import Point, Pose, PoseArray
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from walkie_perception.srv import PCAHeuristicGrasp


# Semantic shape hints keyed by lowercased YOLO class name. If the request's
# object_class matches, that shape is used directly (PCA axes are still
# computed for grasp orientation). Easily editable.
CLASS_SHAPE_HINTS = {
    "bottle": "cylindrical",
    "can":    "cylindrical",
    "cup":    "cylindrical",
    "mug":    "cylindrical",
    "apple":  "spherical",
    "orange": "spherical",
    "ball":   "spherical",
    "fork":   "elongated_flat",
    "spoon":  "elongated_flat",
    "knife":  "elongated_flat",
}

# Shape → RGB for debug markers. Deliberately distinct from
# grasp_from_mask_node's green/yellow/orange so both can be viewed at once.
SHAPE_COLORS = {
    "cylindrical":    (0.2, 0.4, 1.0),   # blue
    "spherical":      (0.2, 1.0, 1.0),   # cyan
    "elongated_flat": (1.0, 0.2, 1.0),   # magenta
    "unknown":        (0.6, 0.6, 0.6),   # grey
}


def _pt(v) -> Point:
    return Point(x=float(v[0]), y=float(v[1]), z=float(v[2]))


def _unit(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else v


class GraspPCANode(Node):

    # Depth range gate on the forward (body X) axis, metres.
    RANGE_MIN_M = 0.1
    RANGE_MAX_M = 3.0

    _DYNAMIC_PARAMS = ("planning_frame", "max_gripper_width", "approach_dist_m")

    def __init__(self):
        super().__init__("grasp_pca")

        self._setup_parameters()
        self._init_state()
        self._create_ros_interfaces()

        self.get_logger().info(
            "GraspPCA ready (always standby, no GPU)\n"
            f"  source: cloud {self.cloud_topic}\n"
            f"  /grasp/pca_heuristic — single best-guess geometric grasp\n"
            f"  /grasp/pca/status    — readiness check"
        )

    # ── Setup ────────────────────────────────────────────────────────

    def _setup_parameters(self) -> None:
        params = [
            ("voxel_size_m",        0.005),
            ("cache_size",          10),
            ("min_points",          150),   # geometry needs less than a net
            ("remove_table_plane",  True),
            ("table_dist_thresh_m", 0.015),
            ("remove_bottom_pct",   0.10),  # secondary crop after RANSAC
            ("max_gripper_width",   0.08),  # default, overridden per-request
            ("planning_frame",      "base_footprint"),   # for up-vector lookup + pose transform
            ("approach_dist_m",     0.10),  # pre-grasp back-out distance
            ("tf_timeout_s",        0.3),
            ("cloud_topic",
             "/zed_head/zed_node/point_cloud/cloud_registered"),
            # ZED organized cloud is in the body frame (X-fwd); extracted
            # points are emitted in this optical frame after the body→optical
            # axis swap, matching grasp_from_mask_node.
            ("cloud_optical_frame", "zed_head_left_camera_frame_optical"),
            # Shape-classification thresholds. eigvals sorted descending:
            # lambda0 >= lambda1 >= lambda2. ratio_10 = l1/l0, ratio_21 = l2/l1.
            ("spherical_ratio10_min",   0.55),  # ratio_10 above → spherical
            ("cylindrical_ratio21_min", 0.45),  # ratio_21 above → cylindrical
            # else → elongated_flat
            ("cylinder_grip_height_frac", 0.50),  # 0=one end, 0.5=middle
            ("utensil_n_bins",     10),
            ("utensil_handle_frac", 0.75),  # grasp this far toward handle end
        ]
        for name, default in params:
            self.declare_parameter(name, default)
            if name not in self._DYNAMIC_PARAMS:
                setattr(self, name, self.get_parameter(name).value)

    def _init_state(self) -> None:
        self._cloud_cache = deque(maxlen=self.cache_size)
        self._cloud_lock  = threading.Lock()
        self._last_dbg_cloud = None
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

    def _create_ros_interfaces(self) -> None:
        qos_be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=5)

        self.create_subscription(
            PointCloud2, self.cloud_topic, self._cloud_cb, qos_be)

        self.create_service(
            PCAHeuristicGrasp, "/grasp/pca_heuristic", self._handle_request)
        self.create_service(
            Trigger, "/grasp/pca/status", self._handle_status)

        self.dbg_cloud_pub = self.create_publisher(
            PointCloud2, "/grasp/debug/pca_masked_cloud", 10)
        self.dbg_marker_pub = self.create_publisher(
            MarkerArray, "/grasp/debug/pca_markers", 10)

    # ── Subscribers ──────────────────────────────────────────────────

    def _cloud_cb(self, msg: PointCloud2) -> None:
        with self._cloud_lock:
            self._cloud_cache.append(msg)

    # ── Cloud / mask helpers (mirror grasp_from_mask_node) ───────────

    @staticmethod
    def _cloud_xyz(cloud_msg: PointCloud2) -> np.ndarray:
        """Organized cloud → (H, W, 3) float32 XYZ (first three fields)."""
        H, W = cloud_msg.height, cloud_msg.width
        buf = np.frombuffer(cloud_msg.data, dtype=np.uint8).reshape(
            H, W, cloud_msg.point_step)
        return buf[:, :, 0:12].copy().view(np.float32).reshape(H, W, 3)

    @staticmethod
    def _resize_mask(mask: np.ndarray, h: int, w: int) -> np.ndarray:
        if mask.shape == (h, w):
            return mask
        import cv2
        return cv2.resize(
            mask.astype(np.uint8), (w, h),
            interpolation=cv2.INTER_NEAREST).astype(bool)

    def _bbox_mask(self, H: int, W: int, bbox) -> np.ndarray:
        cx = int(bbox.center.position.x)
        cy = int(bbox.center.position.y)
        hw = max(1, int(bbox.size_x / 2))
        hh = max(1, int(bbox.size_y / 2))
        m = np.zeros((H, W), dtype=bool)
        m[max(0, cy - hh):min(H, cy + hh),
          max(0, cx - hw):min(W, cx + hw)] = True
        return m

    def _build_region_mask(self, request, ref_cloud: PointCloud2) -> np.ndarray:
        """Boolean pixel mask of the requested object (mask or bbox mode).

        Built in the mask's own coordinate space; _extract_from_cloud resizes
        it down to the organized cloud resolution."""
        if request.mask.data:
            H, W = request.mask.height, request.mask.width
        else:
            H, W = ref_cloud.height, ref_cloud.width

        if request.mask.data:
            if request.mask.encoding in ("16UC1", "mono16"):
                mask_dtype = np.uint16
            elif request.mask.encoding in ("mono8", "8UC1"):
                mask_dtype = np.uint8
            else:
                raise ValueError(
                    f"Expected mono8 or 16UC1 mask, got {request.mask.encoding}")
            mask_img = np.frombuffer(
                request.mask.data, dtype=mask_dtype
            ).reshape(request.mask.height, request.mask.width)
            if request.tracker_id > 0:
                object_mask = mask_img == request.tracker_id
                sel = f"tracker_id={request.tracker_id}"
            else:
                object_mask = mask_img > 0
                sel = "all non-zero pixels"

            if object_mask.sum() < 10:
                self.get_logger().warn(
                    f"Mask has {int(object_mask.sum())} pixels ({sel}) "
                    f"— falling back to bbox")
                object_mask = self._bbox_mask(H, W, request.bbox)
            else:
                self.get_logger().info(
                    f"mask mode: {int(object_mask.sum())} pixels ({sel})")
        else:
            object_mask = self._bbox_mask(H, W, request.bbox)
            self.get_logger().info(
                f"bbox mode: pixels={int(object_mask.sum())}")
        return object_mask

    def _extract_from_cloud(
        self, cloud_msg: PointCloud2, mask: np.ndarray
    ) -> np.ndarray:
        """Masked XYZ from ZED's organized cloud (body frame) → optical
        convention (X-right, Y-down, Z-fwd)."""
        xyz = self._cloud_xyz(cloud_msg)
        m = self._resize_mask(mask, cloud_msg.height, cloud_msg.width)
        pts = xyz[m]
        pts = pts[np.isfinite(pts).all(axis=1)]
        if len(pts) == 0:
            return np.empty((0, 3), dtype=np.float32)
        fwd = pts[:, 0]
        pts = pts[(fwd > self.RANGE_MIN_M) & (fwd < self.RANGE_MAX_M)]
        if len(pts) == 0:
            return np.empty((0, 3), dtype=np.float32)
        # body (X-fwd, Y-left, Z-up) → optical (X-right, Y-down, Z-fwd)
        opt = np.stack([-pts[:, 1], -pts[:, 2], pts[:, 0]], axis=1)
        return opt.astype(np.float32)

    def _make_pc2(self, pts: np.ndarray, frame_id: str, stamp) -> PointCloud2:
        if pts.dtype != np.float32:
            pts = pts.astype(np.float32)
        if not pts.flags["C_CONTIGUOUS"]:
            pts = np.ascontiguousarray(pts)
        msg = PointCloud2()
        msg.header.frame_id = frame_id
        msg.header.stamp    = stamp
        msg.height       = 1
        msg.width        = len(pts)
        msg.is_dense     = True
        msg.is_bigendian = False
        msg.point_step   = 12
        msg.row_step     = 12 * len(pts)
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.data = pts.tobytes()
        return msg

    # ── Point accumulation ───────────────────────────────────────────

    def _accumulate_points(
        self, request, cached: list, object_mask: np.ndarray
    ) -> Tuple[np.ndarray, int]:
        """Adaptive multi-frame extraction until min_points is reached."""
        if request.num_frames == 0:
            frames_to_try = sorted(set([1, 3, min(5, len(cached))]))
        else:
            frames_to_try = [min(request.num_frames, len(cached))]

        last_count = 0
        for n_frames in frames_to_try:
            all_pts = []
            for frame_msg in cached[-n_frames:]:
                p = self._extract_from_cloud(frame_msg, object_mask)
                if len(p) > 0:
                    all_pts.append(p)
            if not all_pts:
                continue
            merged = np.vstack(all_pts)
            last_count = len(merged)
            if last_count >= self.min_points:
                return merged, n_frames
            self.get_logger().info(
                f"{last_count} pts from {n_frames} frame(s) "
                f"(need {self.min_points}) — trying more frames")
        raise ValueError(
            f"Insufficient points: {last_count} after "
            f"{frames_to_try[-1]} frame(s) (need {self.min_points})")

    # ── Table removal + downsample ───────────────────────────────────

    def _remove_table(self, pts: np.ndarray) -> np.ndarray:
        """RANSAC plane removal + bottom crop. Optical Y is down, so the
        largest-Y points are the physical bottom. Falls back to the input
        cloud if a step would leave too few points."""
        if not self.remove_table_plane or len(pts) < 10:
            return pts

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=self.table_dist_thresh_m,
            ransac_n=3, num_iterations=100)
        inlier_ratio = len(inliers) / len(pts)
        if inlier_ratio > 0.05:
            kept = np.delete(pts, inliers, axis=0)
            if len(kept) >= self.min_points // 2:
                self.get_logger().info(
                    f"table plane: removed {len(inliers)} pts "
                    f"(ratio {inlier_ratio:.2f}), {len(kept)} remain")
                pts = kept
            else:
                self.get_logger().warn(
                    f"table removal would leave {len(kept)} pts — "
                    f"keeping pre-RANSAC cloud")

        # Secondary bottom crop (optical Y down → drop the largest-Y band).
        if self.remove_bottom_pct > 0 and len(pts) >= 10:
            y = pts[:, 1]
            y_cut = np.quantile(y, 1.0 - self.remove_bottom_pct)
            cropped = pts[y < y_cut]
            if len(cropped) >= self.min_points // 2:
                pts = cropped
            else:
                self.get_logger().warn(
                    "bottom crop would leave too few pts — skipping")
        return pts

    def _downsample(self, pts: np.ndarray) -> np.ndarray:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        return np.asarray(
            pcd.voxel_down_sample(self.voxel_size_m).points).astype(np.float32)

    # ── PCA + classification ─────────────────────────────────────────

    @staticmethod
    def _pca(pts: np.ndarray):
        """Returns (mean, centered, eigvals desc, eigvecs cols, r10, r21)."""
        mean = pts.mean(axis=0)
        centered = pts - mean
        cov = np.cov(centered, rowvar=False)
        eigvals, eigvecs = np.linalg.eigh(cov)   # ascending
        eigvals = eigvals[::-1]                  # λ0 ≥ λ1 ≥ λ2
        eigvecs = eigvecs[:, ::-1]
        if np.linalg.det(eigvecs) < 0:           # force right-handed
            eigvecs[:, 2] *= -1
        r10 = float(eigvals[1] / (eigvals[0] + 1e-9))
        r21 = float(eigvals[2] / (eigvals[1] + 1e-9))
        return mean, centered, eigvals, eigvecs, r10, r21

    def _classify_shape(self, request, r10: float, r21: float) -> str:
        hint = CLASS_SHAPE_HINTS.get(request.object_class.lower().strip())
        if hint:
            self.get_logger().info(
                f"shape={hint} (from object_class='{request.object_class}')")
            return hint
        if r10 > self.spherical_ratio10_min:
            shape = "spherical"
        elif r21 > self.cylindrical_ratio21_min:
            shape = "cylindrical"
        else:
            shape = "elongated_flat"
        self.get_logger().info(
            f"shape={shape} (from PCA ratios: ratio10={r10:.2f} ratio21={r21:.2f})")
        return shape

    # ── Per-shape grasp generation ───────────────────────────────────
    # Each returns (approach, closing, finger_axis, grasp_center, width,
    # shape_confidence) — all unnormalised intermediate vectors are fine;
    # _build_rotation re-orthonormalises.

    def _grasp_cylindrical(self, mean, centered, eigvals, eigvecs, r10, r21):
        major_axis = eigvecs[:, 0]   # cylinder long axis
        # Side grasp from the direction the camera sees it: camera forward
        # projected perpendicular to the major axis.
        cam_fwd = np.array([0.0, 0.0, 1.0])
        approach = cam_fwd - np.dot(cam_fwd, major_axis) * major_axis
        if np.linalg.norm(approach) < 1e-6:
            approach = eigvecs[:, 1]   # camera looking down the axis
        approach = _unit(approach)

        closing = _unit(np.cross(major_axis, approach))
        approach = _unit(np.cross(closing, major_axis))  # re-orthonormalise

        proj_closing = centered @ closing
        width = float(proj_closing.max() - proj_closing.min())

        proj_major = centered @ major_axis
        t = proj_major.min() + self.cylinder_grip_height_frac * (
            proj_major.max() - proj_major.min())
        grasp_center = mean + major_axis * t
        finger_axis = major_axis
        shape_confidence = float(np.clip(r21, 0, 1))
        return approach, closing, finger_axis, grasp_center, width, shape_confidence

    def _grasp_spherical(self, mean, centered, eigvals, eigvecs, r10, r21,
                         pts_frame_id, stamp):
        down_in_cam = self._down_vector_in_camera(pts_frame_id, stamp)
        approach = _unit(down_in_cam)   # gripper moves downward (top-down)

        closing = eigvecs[:, 1] - np.dot(eigvecs[:, 1], approach) * approach
        if np.linalg.norm(closing) < 1e-6:
            closing = eigvecs[:, 2] - np.dot(eigvecs[:, 2], approach) * approach
        closing = _unit(closing)

        proj_closing = centered @ closing
        width = float(proj_closing.max() - proj_closing.min())
        grasp_center = mean          # centroid — sphere is symmetric
        finger_axis = np.cross(approach, closing)
        shape_confidence = float(np.clip(r10, 0, 1))
        return approach, closing, finger_axis, grasp_center, width, shape_confidence

    def _grasp_elongated_flat(self, mean, centered, eigvals, eigvecs, r10, r21):
        major_axis = eigvecs[:, 0]    # length direction
        width_axes = eigvecs[:, 1:3]  # in-plane width + thickness

        proj_major = centered @ major_axis
        n_bins = int(self.utensil_n_bins)
        bin_edges = np.linspace(proj_major.min(), proj_major.max(), n_bins + 1)
        bin_widths, bin_centers_t = [], []
        for i in range(n_bins):
            m = (proj_major >= bin_edges[i]) & (proj_major < bin_edges[i + 1])
            if m.sum() < 3:
                bin_widths.append(np.nan)
            else:
                perp = centered[m] @ width_axes
                r = np.linalg.norm(perp, axis=1)
                bin_widths.append(float(np.percentile(r, 90) * 2))
            bin_centers_t.append((bin_edges[i] + bin_edges[i + 1]) / 2)
        bin_widths = np.array(bin_widths)

        # Narrower side is the handle.
        n_edge = max(1, n_bins // 3)
        first_w = np.nanmean(bin_widths[:n_edge])
        last_w  = np.nanmean(bin_widths[-n_edge:])
        if first_w <= last_w:
            handle_end_t = proj_major.min()
            handle_side  = "min-t"
        else:
            handle_end_t = proj_major.max()
            handle_side  = "max-t"

        t_grasp = self.utensil_handle_frac * handle_end_t
        grasp_center = mean + major_axis * t_grasp

        # Pinch the thickness (smallest eigvec): fingers contact the flat
        # top/bottom faces. Approach points into the scene (+Z optical).
        approach = eigvecs[:, 2].copy()
        if np.dot(approach, np.array([0, 0, 1])) < 0:
            approach = -approach
        closing = approach.copy()      # pinch direction == approach for flat grasp
        finger_axis = major_axis       # fingers run along the handle length

        near = np.abs(proj_major - t_grasp) < (
            (proj_major.max() - proj_major.min()) / n_bins)
        if near.sum() >= 3:
            thickness_proj = centered[near] @ eigvecs[:, 2]
            width = float(thickness_proj.max() - thickness_proj.min())
        else:
            width = float(2 * np.sqrt(max(eigvals[2], 1e-6)))

        contrast = abs(last_w - first_w) / max(last_w, first_w, 1e-6)
        shape_confidence = float(np.clip(contrast, 0, 1))

        profile = list(zip(np.round(bin_centers_t, 3), np.round(bin_widths, 3)))
        self.get_logger().info(f"width profile (t, width): {profile}")
        self.get_logger().info(
            f"handle side: {handle_side} | grasp at t={t_grasp:.3f}")
        return approach, closing, finger_axis, grasp_center, width, shape_confidence

    def _down_vector_in_camera(self, pts_frame_id: str, stamp) -> np.ndarray:
        """'Down' (planning -Z) expressed in the camera optical frame the
        points live in. Falls back to optical +Y (≈ down for a forward-facing
        optical frame) on TF failure."""
        planning_frame = self.get_parameter("planning_frame").value
        down_in_base = np.array([0.0, 0.0, -1.0])
        for tf_stamp in (stamp, rclpy.time.Time()):
            try:
                tf = self._tf_buffer.lookup_transform(
                    planning_frame, pts_frame_id, tf_stamp,
                    timeout=Duration(seconds=self.tf_timeout_s))
                q = tf.transform.rotation
                R_base_cam = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
                return R_base_cam.T @ down_in_base
            except Exception as e:
                self.get_logger().warn(f"TF down-vector lookup failed: {e}")
        self.get_logger().warn(
            "Spherical grasp: TF unavailable — assuming optical +Y is 'down' "
            "(valid for a forward-looking optical frame; verify in RViz)")
        return np.array([0.0, 1.0, 0.0])

    @staticmethod
    def _build_rotation(approach, closing) -> np.ndarray:
        """Orthonormal right-handed rotation matrix with columns
        [approach, closing, finger_axis]."""
        col0 = _unit(approach)
        col1 = closing - np.dot(closing, col0) * col0
        if np.linalg.norm(col1) < 1e-6:
            # Degenerate (closing ∥ approach, e.g. flat-pinch): pick any
            # perpendicular axis.
            tmp = np.array([1.0, 0.0, 0.0])
            if abs(np.dot(tmp, col0)) > 0.9:
                tmp = np.array([0.0, 1.0, 0.0])
            col1 = tmp - np.dot(tmp, col0) * col0
        col1 = _unit(col1)
        col2 = np.cross(col0, col1)
        return np.column_stack([col0, col1, col2])

    # ── Service: status ──────────────────────────────────────────────

    def _handle_status(self, _req, resp) -> Trigger.Response:
        with self._cloud_lock:
            n = len(self._cloud_cache)
        resp.success = True
        resp.message = (
            f"PCA heuristic node ready (always standby, no GPU) | "
            f"cached_frames={n}/{self.cache_size}")
        self.get_logger().info(resp.message)
        return resp

    # ── Service: pca_heuristic ───────────────────────────────────────

    def _handle_request(self, request, response):
        t0 = time.perf_counter()
        try:
            return self._run(request, response, t0)
        except ValueError as e:
            response.success = False
            response.message = str(e)
            self.get_logger().warn(
                f"{response.message} (compute {(time.perf_counter()-t0)*1e3:.2f} ms)")
            return response

    def _run(self, request, response, t0):
        with self._cloud_lock:
            cached = list(self._cloud_cache)
        if not cached:
            raise ValueError(
                f"No frames cached — is ZED running? ({self.cloud_topic})")

        object_mask = self._build_region_mask(request, cached[-1])
        pts, frames_used = self._accumulate_points(request, cached, object_mask)
        points_extracted = len(pts)

        pts = self._remove_table(pts)
        pts = self._downsample(pts)
        if len(pts) < self.min_points:
            raise ValueError(
                f"Insufficient points after table removal/downsample: "
                f"{len(pts)} (need {self.min_points})")

        last = cached[-1]
        pts_frame_id = self.cloud_optical_frame
        stamp = last.header.stamp

        # ── PCA ───────────────────────────────────────────────────
        mean, centered, eigvals, eigvecs, r10, r21 = self._pca(pts)
        response.pca_eigenvalues = eigvals.astype(np.float32).tolist()
        response.pca_axes = eigvecs.flatten(order="C").astype(np.float32).tolist()

        shape = self._classify_shape(request, r10, r21)
        response.detected_shape = shape

        # ── Grasp by shape ────────────────────────────────────────
        if shape == "cylindrical":
            grasp = self._grasp_cylindrical(mean, centered, eigvals, eigvecs, r10, r21)
        elif shape == "spherical":
            grasp = self._grasp_spherical(mean, centered, eigvals, eigvecs, r10, r21,
                                          pts_frame_id, stamp)
        else:
            grasp = self._grasp_elongated_flat(mean, centered, eigvals, eigvecs, r10, r21)
        approach, closing, _finger_axis, grasp_center, width, shape_conf = grasp

        rot = self._build_rotation(approach, closing)
        quat = R.from_matrix(rot).as_quat()   # [x, y, z, w]

        # ── Width feasibility score ───────────────────────────────
        max_w = self.get_parameter("max_gripper_width").value
        if request.max_gripper_width > 0:
            max_w = request.max_gripper_width
        if width <= max_w * 0.9:
            width_feasibility = 1.0
        else:
            width_feasibility = max(
                0.0, 1.0 - (width - max_w * 0.9) / (max_w * 0.5))
        final_score = float(np.clip(shape_conf * width_feasibility, 0.0, 1.0))

        # ── Build response ────────────────────────────────────────
        # Camera-frame pose is intermediate only (local, not returned); the
        # consumer uses poses_base in the planning frame.
        pose = Pose()
        pose.position.x = float(grasp_center[0])
        pose.position.y = float(grasp_center[1])
        pose.position.z = float(grasp_center[2])
        pose.orientation.x = float(quat[0])
        pose.orientation.y = float(quat[1])
        pose.orientation.z = float(quat[2])
        pose.orientation.w = float(quat[3])

        cam_poses = PoseArray()
        cam_poses.header.frame_id = pts_frame_id
        cam_poses.header.stamp    = stamp
        cam_poses.poses.append(pose)
        response.scores = [final_score]
        response.widths = [float(width)]
        response.success = True
        response.message = (
            f"shape={shape} | score={final_score:.2f} | "
            f"width={width*100:.1f}cm | "
            f"eigvals=[{eigvals[0]:.4f},{eigvals[1]:.4f},{eigvals[2]:.4f}] | "
            f"ratio10={r10:.2f} ratio21={r21:.2f}")
        # compute_ms is the pure-geometry budget (excludes the TF lookup below);
        # logged to the terminal rather than returned in the srv.
        compute_ms = (time.perf_counter() - t0) * 1e3
        self.get_logger().info(
            f"{response.message} | {points_extracted} pts extracted | "
            f"{frames_used} frame(s) | compute {compute_ms:.2f} ms")

        # ── Transform to planning frame (parity with GraspFromMask) ──
        self._transform_to_planning_frame(response, cam_poses, pts, pts_frame_id, stamp)

        # ── Debug viz ─────────────────────────────────────────────
        self._publish_debug(pts, pts_frame_id, stamp, shape,
                            grasp_center, approach, mean, eigvecs[:, 0], eigvals)
        return response

    # ── Planning-frame transform (mirrors grasp_from_mask_node) ──────

    def _transform_to_planning_frame(self, response, cam_poses, pts, frame_id, stamp):
        """Transform grasp poses + object bbox into the planning frame, with
        the same −90° EE-pitch alignment and approach back-out as
        grasp_from_mask_node so both services drive the arm identically.

        Tries the cloud stamp first, falls back to latest TF; on total TF
        failure poses_base is left empty."""
        planning_frame = self.get_parameter("planning_frame").value
        response.planning_frame = planning_frame
        response.poses_base.header.frame_id          = planning_frame
        response.poses_base.header.stamp             = stamp
        response.approach_poses_base.header.frame_id = planning_frame
        response.approach_poses_base.header.stamp    = stamp

        try:
            self._apply_planning_tf(response, cam_poses, pts, frame_id, planning_frame, stamp)
            self.get_logger().info(
                f"Transformed {len(response.poses_base.poses)} pose(s) → {planning_frame}")
        except Exception as e:
            self.get_logger().warn(
                f"TF at stamp failed ({e}), retrying with latest transform…")
            try:
                self._apply_planning_tf(
                    response, cam_poses, pts, frame_id, planning_frame, rclpy.time.Time())
                self.get_logger().info(
                    f"Transformed {len(response.poses_base.poses)} pose(s) → "
                    f"{planning_frame} (latest TF)")
            except Exception as e2:
                self.get_logger().warn(
                    f"TF {frame_id} → {planning_frame} unavailable: {e2}. "
                    f"poses_base will be empty.")

    def _apply_planning_tf(self, response, cam_poses, pts, frame_id, planning_frame, tf_stamp):
        tf = self._tf_buffer.lookup_transform(
            planning_frame, frame_id, tf_stamp,
            timeout=Duration(seconds=self.tf_timeout_s))

        # Transform the point cloud to the planning frame for the AABB.
        tf_rot = R.from_quat([
            tf.transform.rotation.x, tf.transform.rotation.y,
            tf.transform.rotation.z, tf.transform.rotation.w,
        ])
        tf_trans = np.array([
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z,
        ])
        pts_plan = pts @ tf_rot.as_matrix().T + tf_trans

        x_min, x_max = float(pts_plan[:, 0].min()), float(pts_plan[:, 0].max())
        y_min, y_max = float(pts_plan[:, 1].min()), float(pts_plan[:, 1].max())
        z_min, z_max = float(pts_plan[:, 2].min()), float(pts_plan[:, 2].max())

        response.object_size.x = x_max - x_min
        response.object_size.y = y_max - y_min
        response.object_size.z = z_max - z_min
        response.object_bbox_pose.position.x = (x_min + x_max) / 2.0
        response.object_bbox_pose.position.y = (y_min + y_max) / 2.0
        response.object_bbox_pose.position.z = (z_min + z_max) / 2.0
        response.object_bbox_pose.orientation.w = 1.0

        del response.poses_base.poses[:]
        del response.approach_poses_base.poses[:]
        del response.height_below_grasp[:]
        del response.height_above_grasp[:]

        # EE alignment: rotate −90° about Y in the planning (world) frame so the
        # gripper approach points along the arm EE's local Z. Pre-multiply.
        ee_pitch = R.from_euler("y", -90.0, degrees=True)
        approach_dist = self.get_parameter("approach_dist_m").value

        for pose in cam_poses.poses:
            # Jazzy: do_transform_pose takes a bare Pose and returns a Pose.
            p = tf2_geometry_msgs.do_transform_pose(pose, tf)
            q_in = R.from_quat([p.orientation.x, p.orientation.y,
                                p.orientation.z, p.orientation.w])
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (
                float(v) for v in (ee_pitch * q_in).as_quat())
            response.poses_base.poses.append(p)
            response.height_below_grasp.append(float(p.position.z - z_min))
            response.height_above_grasp.append(float(z_max - p.position.z))

            # Approach pose: same orientation, backed out along approach axis
            # (col-0 of the rotation matrix).
            rot = R.from_quat([p.orientation.x, p.orientation.y,
                               p.orientation.z, p.orientation.w]).as_matrix()
            approach_axis = rot[:, 0]
            ap = Pose()
            ap.position.x  = p.position.x - float(approach_axis[0]) * approach_dist
            ap.position.y  = p.position.y - float(approach_axis[1]) * approach_dist
            ap.position.z  = p.position.z - float(approach_axis[2]) * approach_dist
            ap.orientation = p.orientation
            response.approach_poses_base.poses.append(ap)

    # ── Debug publishing ─────────────────────────────────────────────

    def _publish_debug(self, pts, frame_id, stamp, shape, grasp_center,
                       approach, mean, major_axis, eigvals):
        dbg = self._make_pc2(pts, frame_id, stamp)
        self.dbg_cloud_pub.publish(dbg)
        self._last_dbg_cloud = dbg

        color = SHAPE_COLORS.get(shape, SHAPE_COLORS["unknown"])
        ma = MarkerArray()
        lt = Duration(seconds=0).to_msg()

        clr = Marker()
        clr.action = Marker.DELETEALL
        clr.ns     = "pca_heuristic"
        ma.markers.append(clr)

        # Approach arrow (8 cm) from the grasp centre.
        arrow = Marker()
        arrow.header.frame_id = frame_id
        arrow.header.stamp    = stamp
        arrow.ns   = "pca_heuristic"
        arrow.id   = 0
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.lifetime = lt
        arrow.scale.x = 0.008   # shaft diameter
        arrow.scale.y = 0.016   # head diameter
        arrow.scale.z = 0.0
        arrow.color.a = 0.95
        arrow.color.r, arrow.color.g, arrow.color.b = color
        arrow.points = [_pt(grasp_center),
                        _pt(grasp_center + _unit(approach) * 0.08)]
        ma.markers.append(arrow)

        # Major axis as a thin line through the centroid (sanity check on PCA).
        half = float(np.sqrt(max(eigvals[0], 1e-9))) * 2.0
        axis = Marker()
        axis.header.frame_id = frame_id
        axis.header.stamp    = stamp
        axis.ns   = "pca_heuristic"
        axis.id   = 1
        axis.type = Marker.LINE_LIST
        axis.action = Marker.ADD
        axis.lifetime = lt
        axis.scale.x = 0.003
        axis.color.a = 0.9
        axis.color.r, axis.color.g, axis.color.b = color
        axis.pose.orientation.w = 1.0
        axis.points = [_pt(mean - major_axis * half),
                       _pt(mean + major_axis * half)]
        ma.markers.append(axis)

        self.dbg_marker_pub.publish(ma)


def main():
    rclpy.init()
    node = GraspPCANode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
