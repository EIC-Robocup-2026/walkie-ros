#!/bin/sh
"exec" "${PERCEPTION_VENV:-$HOME/perception-venv/bin/python3}" "$0" "$@"
"""
grasp_from_mask_node.py

ROS 2 service node wrapping GraspNet-1Billion.
Receives YOLO segmentation mask, unprojects masked depth pixels to 3D,
runs GraspNet inference, returns grasp poses.

Services:
  /grasp/from_mask   walkie_perception/srv/GraspFromMask
  /grasp/standby     std_srvs/srv/SetBool
  /grasp/status      std_srvs/srv/Trigger

Standby semantics:
  standby true  = model IS loaded in GPU (normal operating state)
  standby false = model unloaded from GPU (VRAM freed for other tasks)
"""
import gc
import os
import sys
import threading
import time
from collections import deque
from enum import Enum
from typing import Optional, Tuple

# graspnet-baseline is not installed as a package; add its root to sys.path
_GRASPNET_ROOT = os.path.expanduser("~/graspnet-baseline")
if _GRASPNET_ROOT not in sys.path:
    sys.path.insert(0, _GRASPNET_ROOT)

import cv2
import numpy as np
import open3d as o3d
import torch
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import tf2_ros
import tf2_geometry_msgs  # noqa: F401 — registers PoseStamped transform support

from geometry_msgs.msg import Point, Pose, PoseArray
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import Marker, MarkerArray

from models.graspnet import GraspNet, pred_decode
from graspnetAPI.grasp import GraspGroup

from walkie_perception.srv import GraspFromMask


class NodeState(Enum):
    LOADING  = "loading"
    STANDBY  = "standby"
    ACTIVE   = "active"
    UNLOADED = "unloaded"


class PipelineAbort(Exception):
    """Abort the request pipeline with a user-facing failure message."""


def _pt(v) -> Point:
    return Point(x=float(v[0]), y=float(v[1]), z=float(v[2]))


class GraspFromMaskNode(Node):

    # Depth range gate applied to extracted points (forward axis), metres.
    RANGE_MIN_M = 0.1
    RANGE_MAX_M = 3.0
    # Gripper marker geometry.
    MARKER_PALM_DEPTH_M   = 0.02  # palm sits 2 cm back from grasp centre
    MARKER_APPROACH_LEN_M = 0.06

    # These are read with get_parameter() on every request so they stay
    # changeable at runtime via `ros2 param set`.
    _DYNAMIC_PARAMS = ("planning_frame", "approach_dist_m")

    def __init__(self):
        super().__init__("grasp_from_mask")

        self._setup_parameters()
        self._init_runtime_state()
        self._check_gpu()

        self._load_model()
        self._set_state(NodeState.STANDBY)
        self.get_logger().info(
            f"Model ready (standby). VRAM: {self._vram_info()}"
        )

        self._create_ros_interfaces()
        self.get_logger().info(
            f"GraspFromMask ready\n"
            f"  source: {'cloud ' + self.cloud_topic if self.use_cloud else 'depth ' + self.depth_topic}\n"
            f"  info  : {self.info_topic}\n"
            f"  /grasp/from_mask  — request a grasp\n"
            f"  /grasp/standby    — true=load GPU, false=unload\n"
            f"  /grasp/status     — check state and VRAM"
        )

    # ── Setup ────────────────────────────────────────────────────────

    def _setup_parameters(self) -> None:
        params = [
            ("checkpoint_path",
             os.path.expanduser("~/graspnet-baseline/logs/log_rs/checkpoint-rs.tar")),
            ("num_point",    10000),
            ("num_view",     300),
            ("voxel_size_m", 0.005),
            ("cache_size",   10),
            ("min_points",   200),
            ("debug_cloud",  True),
            # Continuously republish the last masked cloud so it stays visible
            # in RViz between requests (the cloud is otherwise one-shot per
            # request).
            ("debug_cloud_continuous", True),
            ("debug_cloud_rate_hz",    5.0),
            ("use_mask",     True),
            ("outlier_removal",      True),
            ("outlier_nb_neighbors", 20),
            ("outlier_std_ratio",    2.0),
            ("cluster_filter",       True),
            ("cluster_eps",          0.02),
            ("cluster_min_samples",  10),
            # Depth-path edge / flying-pixel filter — reproduces the neighbour-
            # disagreement rejection ZED already applies to its point cloud, so
            # the depth path can keep the full-resolution mask without the
            # silhouette "skin" of edge points. Only used when use_cloud=false.
            ("depth_edge_filter",    True),
            ("depth_edge_tol_m",     0.03),
            ("depth_edge_radius",    1),
            ("depth_edge_min_ratio", 0.6),
            # Shrink the region mask by N pixels (at image resolution) before
            # extraction. Optional — for loose masks only; off by default since
            # the cloud-validity gate is the right tool for ZED edge bleed.
            # 0 = disabled.
            ("mask_erosion_px",      0),
            # Depth path only: reject depth pixels that have NO point in the
            # ZED cloud (the cloud already drops ZED's low-confidence edge
            # pixels, the raw depth image keeps them). Removes exactly the
            # points absent from the cloud topic. Subscribes to the cloud even
            # in depth mode.
            ("gate_by_cloud",        True),
            ("depth_topic", "/zed_head/zed_node/depth/depth_registered"),
            ("info_topic",  "/zed_head/zed_node/depth/camera_info"),
            # Point-cloud source: use ZED's organized cloud instead of
            # unprojecting the depth image (uses ZED's own intrinsics/
            # rectification, no fx/fy).
            ("use_cloud",   False),
            ("cloud_topic", "/zed_head/zed_node/point_cloud/cloud_registered"),
            # ZED cloud is in the body frame (X-fwd); poses are emitted in this
            # optical frame after the body→optical axis swap GraspNet expects.
            ("cloud_optical_frame", "zed_head_left_camera_frame_optical"),
            ("planning_frame",  "base_footprint"),
            ("approach_dist_m", 0.10),
        ]
        for name, default in params:
            self.declare_parameter(name, default)
            if name not in self._DYNAMIC_PARAMS:
                setattr(self, name, self.get_parameter(name).value)
        self.checkpoint_path = os.path.expanduser(self.checkpoint_path)

    def _init_runtime_state(self) -> None:
        self._state      = NodeState.LOADING
        self._state_lock = threading.Lock()
        self.net: Optional[GraspNet] = None

        # Depth / cloud frame caches.
        self._depth_cache = deque(maxlen=self.cache_size)
        self._depth_lock  = threading.Lock()
        self._cloud_cache = deque(maxlen=self.cache_size)
        self._cloud_lock  = threading.Lock()

        # Last masked debug cloud, republished continuously for RViz.
        self._last_dbg_cloud = None
        self._last_dbg_lock  = threading.Lock()

        # Camera intrinsics.
        self._fx = self._fy = self._cx = self._cy = None
        self._img_w = self._img_h = None   # full image dims (bbox coord space)
        self._info_lock = threading.Lock()
        self._depth_frame_id = "zed_left_camera_frame_optical"

        # Inference lock (one at a time).
        self._inference_lock = threading.Lock()

        # TF.
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

    def _check_gpu(self) -> None:
        if not torch.cuda.is_available():
            self.get_logger().fatal("CUDA not available — GPU required")
            raise RuntimeError("CUDA not available")
        self.get_logger().info(
            f"GPU: {torch.cuda.get_device_name(0)} | "
            f"sm_{torch.cuda.get_device_capability()[0]}"
            f"{torch.cuda.get_device_capability()[1]}"
        )
        self.get_logger().info(f"VRAM before load: {self._vram_info()}")

    def _create_ros_interfaces(self) -> None:
        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, depth=5
        )

        self.create_subscription(
            CameraInfo, self.info_topic, self._info_cb, qos_be)
        if self.use_cloud:
            self.create_subscription(
                PointCloud2, self.cloud_topic, self._cloud_cb, qos_be)
        else:
            self.create_subscription(
                Image, self.depth_topic, self._depth_cb, qos_be)
            # Depth mode still subscribes to the cloud when gating, so we can
            # drop depth pixels the cloud doesn't have.
            if self.gate_by_cloud:
                self.create_subscription(
                    PointCloud2, self.cloud_topic, self._cloud_cb, qos_be)

        self.create_service(
            GraspFromMask, "/grasp/from_mask", self._handle_request)
        self.create_service(
            SetBool, "/grasp/standby", self._handle_standby)
        self.create_service(
            Trigger, "/grasp/status", self._handle_status)

        self.dbg_cloud_pub = self.create_publisher(
            PointCloud2, "/grasp/debug/masked_cloud", 10)
        self.dbg_marker_pub = self.create_publisher(
            MarkerArray, "/grasp/debug/grasp_markers", 10)

        # Continuous republish timer (keeps the last masked cloud alive in RViz).
        if self.debug_cloud and self.debug_cloud_continuous:
            period = 1.0 / max(self.debug_cloud_rate_hz, 0.1)
            self.create_timer(period, self._republish_dbg_cloud)

    # ── State helpers ────────────────────────────────────────────────

    def _get_state(self) -> NodeState:
        with self._state_lock:
            return self._state

    def _set_state(self, state: NodeState) -> None:
        with self._state_lock:
            self._state = state

    # ── GPU / model helpers ──────────────────────────────────────────

    def _vram_used_mb(self) -> int:
        free, total = torch.cuda.mem_get_info()
        return (total - free) // 1024**2

    def _vram_info(self) -> str:
        free, total = torch.cuda.mem_get_info()
        used = total - free
        return (
            f"{used // 1024**2} MB used / "
            f"{total // 1024**2} MB total "
            f"({free // 1024**2} MB free)"
        )

    def _load_model(self) -> None:
        self.get_logger().info(
            f"Loading checkpoint: {self.checkpoint_path}")
        t0 = time.perf_counter()
        self.net = GraspNet(
            input_feature_dim=0,
            num_view=self.num_view,
            num_angle=12,
            num_depth=4,
            cylinder_radius=0.05,
            hmin=-0.02,
            hmax_list=[0.01, 0.02, 0.03, 0.04],
            is_training=False,
        )
        ckpt = torch.load(self.checkpoint_path, map_location="cuda")
        self.net.load_state_dict(ckpt["model_state_dict"])
        self.net = self.net.cuda().eval()
        load_ms = (time.perf_counter() - t0) * 1e3
        self.get_logger().info(
            f"Checkpoint loaded in {load_ms:.0f} ms. "
            f"VRAM: {self._vram_info()}"
        )
        self._warmup()

    def _warmup(self) -> None:
        self.get_logger().info("Running warm-up inference...")
        dummy = torch.randn(1, self.num_point, 3).cuda()
        with torch.no_grad():
            pred_decode(self.net({"point_clouds": dummy}))
        torch.cuda.synchronize()
        self.get_logger().info(
            f"Warm-up done. VRAM: {self._vram_info()}")

    def _unload_model(self) -> int:
        """Free the model from GPU; returns MB of VRAM freed."""
        used_before = self._vram_used_mb()
        self.net = self.net.cpu()
        del self.net
        self.net = None
        gc.collect()
        torch.cuda.empty_cache()
        return used_before - self._vram_used_mb()

    # ── Image / cloud decoding helpers ───────────────────────────────

    @staticmethod
    def _decode_depth(depth_msg: Image) -> np.ndarray:
        """Depth Image message → (H, W) float32 metres."""
        if depth_msg.encoding == "16UC1":
            return np.frombuffer(
                depth_msg.data, dtype=np.uint16
            ).reshape(depth_msg.height, depth_msg.width).astype(np.float32) / 1000.0
        return np.frombuffer(
            depth_msg.data, dtype=np.float32
        ).reshape(depth_msg.height, depth_msg.width)

    @staticmethod
    def _cloud_xyz(cloud_msg: PointCloud2) -> np.ndarray:
        """Organized cloud → (H, W, 3) float32 XYZ.

        x,y,z are the first three float32 fields (offsets 0,4,8)."""
        H, W = cloud_msg.height, cloud_msg.width
        buf = np.frombuffer(cloud_msg.data, dtype=np.uint8).reshape(
            H, W, cloud_msg.point_step)
        return buf[:, :, 0:12].copy().view(np.float32).reshape(H, W, 3)

    @staticmethod
    def _resize_mask(mask: np.ndarray, h: int, w: int) -> np.ndarray:
        """Nearest-neighbour resize of a boolean mask to (h, w)."""
        if mask.shape == (h, w):
            return mask
        return cv2.resize(
            mask.astype(np.uint8), (w, h),
            interpolation=cv2.INTER_NEAREST,
        ).astype(bool)

    def _make_pc2(self, pts: np.ndarray, frame_id: str, stamp) -> PointCloud2:
        if pts.dtype != np.float32:
            pts = pts.astype(np.float32)
        if not pts.flags["C_CONTIGUOUS"]:
            pts = np.ascontiguousarray(pts)
        msg = PointCloud2()
        msg.header.frame_id = frame_id
        msg.header.stamp    = stamp
        msg.height    = 1
        msg.width     = len(pts)
        msg.is_dense  = True
        msg.is_bigendian = False
        msg.point_step   = 12
        msg.row_step     = 12 * len(pts)
        msg.fields = [
            PointField(name="x", offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8,
                       datatype=PointField.FLOAT32, count=1),
        ]
        msg.data = pts.tobytes()
        return msg

    # ── Point filtering ──────────────────────────────────────────────

    def _remove_outliers(self, pts: np.ndarray) -> np.ndarray:
        """Density-based outlier removal with depth weighting.

        Points are normalised by (z_min / z) before statistical outlier
        removal so near-camera points are treated as denser and harder to
        remove, while sparse far-field noise is eliminated more aggressively.
        """
        if len(pts) <= self.outlier_nb_neighbors:
            return pts

        z = pts[:, 2]
        z_min = float(z.min())
        if z_min <= 0:
            z_min = 0.01

        # Scale each point toward origin proportionally to its depth.
        # Near points (z ≈ z_min) → weight ≈ 1.0 (unchanged).
        # Far points (z >> z_min) → weight < 1.0 (compressed).
        weights = (z_min / np.clip(z, z_min, None)).reshape(-1, 1)
        pts_norm = pts * weights

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts_norm)
        _, inlier_idx = pcd.remove_statistical_outlier(
            nb_neighbors=self.outlier_nb_neighbors,
            std_ratio=self.outlier_std_ratio,
        )
        return pts[inlier_idx]

    def _keep_dominant_cluster(self, pts: np.ndarray) -> np.ndarray:
        """DBSCAN clustering — keep the cluster with the most points.

        Clusters are scored by point count weighted by mean proximity to the
        camera (1/z), so a dense near cluster beats a slightly larger far one.
        Noise label (-1) is always discarded.
        """
        if len(pts) < self.cluster_min_samples:
            return pts

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        labels = np.array(pcd.cluster_dbscan(
            eps=self.cluster_eps,
            min_points=self.cluster_min_samples,
            print_progress=False,
        ))

        unique_labels = set(labels) - {-1}
        if not unique_labels:
            self.get_logger().warn(
                "cluster_filter: no clusters found — returning all points")
            return pts

        # Score each cluster: count * mean(1/z) gives density × nearness
        best_label, best_score = -1, -1.0
        for lbl in unique_labels:
            mask = labels == lbl
            cluster_pts = pts[mask]
            count = mask.sum()
            mean_nearness = float(np.mean(1.0 / np.clip(cluster_pts[:, 2], 0.01, None)))
            score = count * mean_nearness
            if score > best_score:
                best_score = score
                best_label = lbl

        result = pts[labels == best_label]
        n_removed = len(pts) - len(result)
        n_clusters = len(unique_labels)
        self.get_logger().info(
            f"cluster_filter: {n_clusters} cluster(s), kept label={best_label} "
            f"({len(result)} pts), removed {n_removed} pts"
        )
        return result

    def _filter_points(self, pts: np.ndarray) -> np.ndarray:
        """Voxel-downsample, then optional outlier removal and clustering.

        Downsampling first keeps outlier removal and DBSCAN cheap."""
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        pts = np.asarray(
            pcd.voxel_down_sample(self.voxel_size_m).points
        ).astype(np.float32)

        if self.outlier_removal:
            before = len(pts)
            pts = self._remove_outliers(pts)
            self.get_logger().info(
                f"outlier removal: {before} → {len(pts)} pts "
                f"(removed {before - len(pts)})"
            )

        if self.cluster_filter:
            pts = self._keep_dominant_cluster(pts)

        return pts

    # ── Region mask ──────────────────────────────────────────────────

    def _bbox_mask(self, H: int, W: int, bbox) -> np.ndarray:
        cx = int(bbox.center.position.x)
        cy = int(bbox.center.position.y)
        hw = max(1, int(bbox.size_x / 2))
        hh = max(1, int(bbox.size_y / 2))
        m = np.zeros((H, W), dtype=bool)
        m[max(0, cy - hh):min(H, cy + hh),
          max(0, cx - hw):min(W, cx + hw)] = True
        return m

    def _region_dims(self, request, cached: list) -> Tuple[int, int]:
        """Dimensions of the IMAGE coordinate space the region mask lives in.

        The mask is built at image resolution (where mask and bbox pixels
        live), then _extract_from_cloud / _unproject resize it down to the
        cloud / depth resolution. Using the cache frame's dims here would be
        wrong for the cloud (256×448 ≠ 1920×1080 bbox space)."""
        if request.mask.data:
            return request.mask.height, request.mask.width
        if not self.use_cloud:
            depth_ref = cached[-1]
            return depth_ref.height, depth_ref.width
        with self._info_lock:
            H, W = self._img_h, self._img_w
        if H is None:
            raise PipelineAbort(
                "No image dimensions yet (camera_info) — needed for "
                "bbox mode with the cloud source")
        return H, W

    def _build_region_mask(self, request, cached: list) -> np.ndarray:
        """Boolean pixel mask of the requested object (mask or bbox mode)."""
        H, W = self._region_dims(request, cached)

        if self.use_mask and request.mask.data:
            if request.mask.encoding in ("16UC1", "mono16"):
                mask_dtype = np.uint16
            elif request.mask.encoding in ("mono8", "8UC1"):
                mask_dtype = np.uint8
            else:
                raise PipelineAbort(
                    f"Expected mono8 or 16UC1 mask, got {request.mask.encoding}")

            mask_img = np.frombuffer(
                request.mask.data, dtype=mask_dtype
            ).reshape(request.mask.height, request.mask.width)
            # tracker_id > 0 selects that label in a multi-object mask;
            # otherwise treat the mask as single-object (any non-zero pixel).
            if request.tracker_id > 0:
                object_mask = mask_img == request.tracker_id
                sel = f"tracker_id={request.tracker_id}"
            else:
                object_mask = mask_img > 0
                sel = "all non-zero pixels"

            if object_mask.sum() < 10:
                self.get_logger().warn(
                    f"Mask has {int(object_mask.sum())} pixels ({sel}) "
                    f"— falling back to bbox"
                )
                object_mask = self._bbox_mask(H, W, request.bbox)
            else:
                self.get_logger().info(
                    f"mask mode: {int(object_mask.sum())} pixels ({sel})")
        else:
            object_mask = self._bbox_mask(H, W, request.bbox)
            self.get_logger().info(
                f"bbox mode: centre=({int(request.bbox.center.position.x)},"
                f"{int(request.bbox.center.position.y)}) "
                f"size={int(request.bbox.size_x)}x{int(request.bbox.size_y)} "
                f"pixels={object_mask.sum()}"
            )

        return self._erode_region(object_mask)

    def _erode_region(self, object_mask: np.ndarray) -> np.ndarray:
        """Erode the region to drop the silhouette boundary band.

        Edge flying pixels and loose-mask background bleed concentrate in the
        rim just inside the mask edge; eroding removes it. Skipped if it would
        leave the object too small."""
        if self.mask_erosion_px <= 0 or object_mask.sum() == 0:
            return object_mask
        k = 2 * int(self.mask_erosion_px) + 1
        eroded = cv2.erode(
            object_mask.astype(np.uint8),
            np.ones((k, k), np.uint8), iterations=1).astype(bool)
        if eroded.sum() >= 10:
            self.get_logger().info(
                f"mask erosion: {int(object_mask.sum())} → "
                f"{int(eroded.sum())} px (−{self.mask_erosion_px}px rim)")
            return eroded
        self.get_logger().warn(
            f"mask erosion would leave {int(eroded.sum())} px — "
            f"skipping (object too small)")
        return object_mask

    # ── Point extraction ─────────────────────────────────────────────

    def _edge_agreement(self, depth: np.ndarray, valid: np.ndarray) -> np.ndarray:
        """Neighbour-disagreement / flying-pixel filter.

        Keeps a valid pixel only if its depth agrees (within tol) with at least
        a minimum fraction of its valid neighbours. This drops the thin skin of
        stereo flying pixels along depth discontinuities — the same artefact
        ZED's point cloud already rejects but the raw depth image keeps.
        """
        r   = max(1, int(self.depth_edge_radius))
        tol = float(self.depth_edge_tol_m)
        # Sentinel for invalid pixels so they never count as "agreeing".
        d = np.where(valid, depth, -1000.0).astype(np.float32)

        agree = np.zeros(depth.shape, dtype=np.int16)
        n_neighbors = 0
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx == 0 and dy == 0:
                    continue
                n_neighbors += 1
                sd = np.roll(np.roll(d, dy, axis=0), dx, axis=1)
                sv = np.roll(np.roll(valid, dy, axis=0), dx, axis=1)
                agree += (sv & (np.abs(sd - d) < tol)).astype(np.int16)

        min_agree = int(np.ceil(n_neighbors * self.depth_edge_min_ratio))
        return valid & (agree >= min_agree)

    def _cloud_validity(self, cloud_msg: PointCloud2,
                        h_out: int, w_out: int) -> np.ndarray:
        """Boolean (h_out, w_out) mask of pixels where the ZED cloud has a
        finite point, upsampled from the (lower-res) organized cloud. Used to
        drop depth pixels the cloud rejected as low-confidence/edge."""
        xyz = self._cloud_xyz(cloud_msg)
        valid = np.isfinite(xyz).all(axis=2)
        return self._resize_mask(valid, h_out, w_out)

    def _unproject(
        self,
        depth_msg: Image,
        mask: np.ndarray,
        cloud_valid: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """Unproject masked depth pixels to XYZ using pinhole model."""
        depth = self._decode_depth(depth_msg)
        m = self._resize_mask(mask, depth_msg.height, depth_msg.width)

        # Global validity (finite + range gate) over the whole frame, so the
        # edge filter can inspect each pixel's neighbourhood before masking.
        valid2d = (np.isfinite(depth)
                   & (depth > self.RANGE_MIN_M) & (depth < self.RANGE_MAX_M))
        if self.depth_edge_filter:
            valid2d = self._edge_agreement(depth, valid2d)
        # Drop depth pixels the ZED cloud doesn't have (its confidence/edge filter).
        if cloud_valid is not None:
            valid2d &= cloud_valid

        sel = m & valid2d
        vs, us = np.where(sel)
        z  = depth[vs, us]
        us = us.astype(np.float32)
        vs = vs.astype(np.float32)

        if len(z) == 0:
            return np.empty((0, 3), dtype=np.float32)

        with self._info_lock:
            fx, fy, cx, cy = self._fx, self._fy, self._cx, self._cy

        x = (us - cx) * z / fx
        y = (vs - cy) * z / fy
        return np.stack([x, y, z], axis=1).astype(np.float32)

    def _extract_from_cloud(
        self,
        cloud_msg: PointCloud2,
        mask: np.ndarray,
    ) -> np.ndarray:
        """Pull masked XYZ from ZED's organized cloud (body frame) and convert
        to the optical convention (X-right, Y-down, Z-fwd) GraspNet expects."""
        xyz = self._cloud_xyz(cloud_msg)
        m = self._resize_mask(mask, cloud_msg.height, cloud_msg.width)

        pts = xyz[m]
        pts = pts[np.isfinite(pts).all(axis=1)]
        if len(pts) == 0:
            return np.empty((0, 3), dtype=np.float32)

        # Range gate on forward axis (body X) to match the depth path.
        fwd = pts[:, 0]
        pts = pts[(fwd > self.RANGE_MIN_M) & (fwd < self.RANGE_MAX_M)]
        if len(pts) == 0:
            return np.empty((0, 3), dtype=np.float32)

        # body (X-fwd, Y-left, Z-up) → optical (X-right, Y-down, Z-fwd)
        opt = np.stack([-pts[:, 1], -pts[:, 2], pts[:, 0]], axis=1)
        return opt.astype(np.float32)

    # ── Subscribers ──────────────────────────────────────────────────

    def _info_cb(self, msg: CameraInfo) -> None:
        with self._info_lock:
            if self._fx is not None:
                return
            self._fx = msg.k[0]
            self._fy = msg.k[4]
            self._cx = msg.k[2]
            self._cy = msg.k[5]
            self._img_w = msg.width
            self._img_h = msg.height
            self._depth_frame_id = msg.header.frame_id
        self.get_logger().info(
            f"Camera info: fx={self._fx:.1f} fy={self._fy:.1f} "
            f"cx={self._cx:.1f} cy={self._cy:.1f}"
        )

    def _depth_cb(self, msg: Image) -> None:
        with self._depth_lock:
            self._depth_cache.append(msg)

    def _cloud_cb(self, msg: PointCloud2) -> None:
        with self._cloud_lock:
            self._cloud_cache.append(msg)

    # ── Debug cloud publishing ───────────────────────────────────────

    def _publish_debug_cloud(self, pts: np.ndarray, frame_id: str, stamp) -> None:
        if not self.debug_cloud:
            return
        msg = self._make_pc2(pts, frame_id=frame_id, stamp=stamp)
        self.dbg_cloud_pub.publish(msg)
        with self._last_dbg_lock:
            self._last_dbg_cloud = msg

    def _republish_dbg_cloud(self) -> None:
        """Re-emit the last masked cloud with a fresh stamp so it stays
        visible in RViz between requests."""
        with self._last_dbg_lock:
            msg = self._last_dbg_cloud
        if msg is None:
            return
        msg.header.stamp = self.get_clock().now().to_msg()
        self.dbg_cloud_pub.publish(msg)

    # ── Service: status ──────────────────────────────────────────────

    def _handle_status(self, _req, resp) -> Trigger.Response:
        state = self._get_state()
        if self.use_cloud:
            with self._cloud_lock:
                n_frames = len(self._cloud_cache)
        else:
            with self._depth_lock:
                n_frames = len(self._depth_cache)
        with self._info_lock:
            has_info = self._fx is not None
        resp.success = True
        resp.message = (
            f"state={state.value} | "
            f"VRAM: {self._vram_info()} | "
            f"cached_frames={n_frames}/{self.cache_size} | "
            f"camera_info={'ok' if has_info else 'waiting'}"
        )
        self.get_logger().info(resp.message)
        return resp

    # ── Service: standby ─────────────────────────────────────────────

    def _handle_standby(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        if request.data:
            return self._standby_load(response)
        return self._standby_unload(response)

    def _standby_load(self, response: SetBool.Response) -> SetBool.Response:
        state = self._get_state()
        if state in (NodeState.STANDBY, NodeState.ACTIVE, NodeState.LOADING):
            response.success = True
            response.message = (
                f"Already in state={state.value}. "
                f"VRAM: {self._vram_info()}"
            )
            return response

        self._set_state(NodeState.LOADING)
        try:
            self._load_model()
            self._set_state(NodeState.STANDBY)
            response.success = True
            response.message = (
                f"Model loaded and ready. VRAM: {self._vram_info()}")
            self.get_logger().info(response.message)
        except Exception as exc:
            self._set_state(NodeState.UNLOADED)
            response.success = False
            response.message = f"Load failed: {exc}"
        return response

    def _standby_unload(self, response: SetBool.Response) -> SetBool.Response:
        state = self._get_state()
        if state == NodeState.UNLOADED:
            response.success = True
            response.message = f"Already unloaded. VRAM: {self._vram_info()}"
            return response
        if state == NodeState.ACTIVE:
            response.success = False
            response.message = "Inference running — wait for it to finish"
            return response
        if state == NodeState.LOADING:
            response.success = False
            response.message = "Still loading — try again shortly"
            return response

        self._set_state(NodeState.ACTIVE)
        try:
            freed_mb = self._unload_model()
            self._set_state(NodeState.UNLOADED)
            response.success = True
            response.message = (
                f"Model unloaded. Freed ~{freed_mb} MB. "
                f"VRAM: {self._vram_info()}"
            )
            self.get_logger().info(response.message)
        except Exception as exc:
            self._set_state(NodeState.STANDBY)
            response.success = False
            response.message = f"Unload failed: {exc}"
        return response

    # ── Service: from_mask ───────────────────────────────────────────

    def _handle_request(
        self,
        request: GraspFromMask.Request,
        response: GraspFromMask.Response,
    ) -> GraspFromMask.Response:

        t_start = time.perf_counter()

        state = self._get_state()
        if state == NodeState.UNLOADED:
            response.success = False
            response.message = (
                "Model is unloaded. "
                "Call: ros2 service call /grasp/standby "
                "std_srvs/srv/SetBool '{data: true}'"
            )
            return response
        if state == NodeState.LOADING:
            response.success = False
            response.message = "Model is still loading — try again shortly"
            return response
        if state == NodeState.ACTIVE:
            response.success = False
            response.message = "Another inference is running — try again shortly"
            return response

        self._set_state(NodeState.ACTIVE)
        try:
            response = self._run_pipeline(request, response, t_start)
        except PipelineAbort as abort:
            response.success = False
            response.message = str(abort)
        finally:
            self._set_state(NodeState.STANDBY)

        return response

    # ── Pipeline ─────────────────────────────────────────────────────

    def _run_pipeline(
        self,
        request: GraspFromMask.Request,
        response: GraspFromMask.Response,
        t_start: float,
    ) -> GraspFromMask.Response:

        cached      = self._snapshot_frames()
        object_mask = self._build_region_mask(request, cached)
        cloud_valid = self._depth_path_cloud_gate(cached)

        pts, actual_frames = self._accumulate_points(
            request, cached, object_mask, cloud_valid)
        points_extracted = len(pts)

        pts = self._sample_points(pts)
        points_fed = len(pts)

        # Optical-convention frame for the extracted points: the cloud's own
        # header frame is the body frame, so override it when using the cloud.
        last = cached[-1]
        if self.use_cloud:
            frame_id = self.cloud_optical_frame
        else:
            frame_id = last.header.frame_id or self._depth_frame_id
        stamp = last.header.stamp

        self._publish_debug_cloud(pts, frame_id, stamp)

        raw_grasps, inference_ms = self._run_graspnet(pts)
        grasps_raw = len(raw_grasps)

        gg = self._select_grasps(raw_grasps, request)
        if len(gg) == 0:
            raise PipelineAbort(
                f"No grasps above threshold {request.score_threshold} "
                f"(raw: {grasps_raw})"
            )

        # Camera-frame poses are intermediate only (kept local, not returned);
        # downstream consumers use poses_base in the planning frame.
        cam_poses = self._build_camera_poses(gg, response, frame_id, stamp)
        planning_frame = self._transform_to_planning_frame(
            response, cam_poses, pts, frame_id, stamp)
        self._publish_grasp_markers(gg, frame_id, stamp, response, planning_frame)

        total_ms = (time.perf_counter() - t_start) * 1e3
        response.success  = True
        response.message  = (
            f"OK | {len(gg)} grasps | top score {gg.scores[0]:.3f} | "
            f"top width {gg.widths[0]*100:.1f} cm"
        )
        # Timing / point counts are logged here rather than returned in the srv.
        self.get_logger().info(
            f"{response.message} | {points_extracted} pts extracted | "
            f"{points_fed} fed | {actual_frames} frame(s) | "
            f"raw grasps {grasps_raw} | infer {inference_ms:.0f} ms | "
            f"total {total_ms:.0f} ms"
        )
        return response

    def _snapshot_frames(self) -> list:
        """Snapshot the active frame cache; abort if prerequisites missing."""
        with self._info_lock:
            has_info = self._fx is not None

        # Cloud path carries its own geometry — only the depth path needs info.
        if not self.use_cloud and not has_info:
            raise PipelineAbort(
                f"Camera info not received yet — is ZED running? "
                f"({self.info_topic})"
            )

        if self.use_cloud:
            with self._cloud_lock:
                cached = list(self._cloud_cache)
            src_topic = self.cloud_topic
        else:
            with self._depth_lock:
                cached = list(self._depth_cache)
            src_topic = self.depth_topic

        if len(cached) == 0:
            raise PipelineAbort(
                f"No frames cached — is ZED running? ({src_topic})"
            )
        return cached

    def _depth_path_cloud_gate(self, cached: list) -> Optional[np.ndarray]:
        """Validity mask from the latest ZED cloud so the depth path can drop
        pixels the cloud rejected (its confidence/edge filter). This is exactly
        the set of points present in the depth image but not the cloud."""
        if self.use_cloud or not self.gate_by_cloud:
            return None
        with self._cloud_lock:
            cl = self._cloud_cache[-1] if self._cloud_cache else None
        if cl is None:
            self.get_logger().warn(
                "gate_by_cloud on but no cloud received yet — skipping cloud gate "
                f"({self.cloud_topic})")
            return None
        ref = cached[-1]
        return self._cloud_validity(cl, ref.height, ref.width)

    def _accumulate_points(
        self,
        request: GraspFromMask.Request,
        cached: list,
        object_mask: np.ndarray,
        cloud_valid: Optional[np.ndarray],
    ) -> Tuple[np.ndarray, int]:
        """Extract + filter masked points, adaptively merging more frames
        until min_points is reached. Returns (points, frames_used)."""
        if request.num_frames == 0:
            frames_to_try = sorted(set([1, 3, min(5, len(cached))]))
        else:
            frames_to_try = [min(request.num_frames, len(cached))]

        last_count = 0
        for n_frames in frames_to_try:
            all_pts = []
            for frame_msg in cached[-n_frames:]:
                if self.use_cloud:
                    p = self._extract_from_cloud(frame_msg, object_mask)
                else:
                    p = self._unproject(frame_msg, object_mask, cloud_valid)
                if len(p) > 0:
                    all_pts.append(p)

            if not all_pts:
                continue

            merged = self._filter_points(np.vstack(all_pts))
            last_count = len(merged)
            if last_count >= self.min_points:
                return merged, n_frames

            self.get_logger().info(
                f"{last_count} pts from {n_frames} frame(s) "
                f"(need {self.min_points}) — trying more frames"
            )

        raise PipelineAbort(
            f"Insufficient points: {last_count} "
            f"after {frames_to_try[-1]} frame(s) "
            f"(need {self.min_points})"
        )

    def _sample_points(self, pts: np.ndarray) -> np.ndarray:
        """Random-sample (with replacement if short) to exactly num_point."""
        replace = len(pts) < self.num_point
        idx = np.random.choice(len(pts), self.num_point, replace=replace)
        return pts[idx]

    def _run_graspnet(self, pts: np.ndarray) -> Tuple[object, float]:
        """GraspNet inference. Returns (decoded predictions, inference ms)."""
        cloud_t = torch.from_numpy(pts).unsqueeze(0).cuda()
        t_inf = time.perf_counter()
        with self._inference_lock:
            with torch.no_grad():
                ep    = self.net({"point_clouds": cloud_t})
                preds = pred_decode(ep)
        torch.cuda.synchronize()
        return preds[0], (time.perf_counter() - t_inf) * 1e3

    def _select_grasps(
        self, raw_grasps, request: GraspFromMask.Request
    ) -> GraspGroup:
        """Sort + NMS + score-threshold + trim to max_grasps."""
        gg = GraspGroup(raw_grasps.cpu().numpy())
        gg = gg.sort_by_score()[:2000].nms()

        if request.score_threshold > 0:
            gg = gg[gg.scores >= request.score_threshold]
        if len(gg) == 0:
            return gg

        n = request.max_grasps if request.max_grasps > 0 else min(len(gg), 20)
        return gg[:n]

    def _build_camera_poses(
        self, gg: GraspGroup, response: GraspFromMask.Response,
        frame_id: str, stamp,
    ) -> PoseArray:
        """Build camera-frame grasp poses (returned locally, not in the srv)
        and fill scores/widths on the response."""
        cam_poses = PoseArray()
        cam_poses.header.frame_id = frame_id
        cam_poses.header.stamp    = stamp

        for i in range(len(gg)):
            quat = R.from_matrix(gg.rotation_matrices[i]).as_quat()
            trans = gg.translations[i]

            pose = Pose()
            pose.position.x    = float(trans[0])
            pose.position.y    = float(trans[1])
            pose.position.z    = float(trans[2])
            pose.orientation.x = float(quat[0])
            pose.orientation.y = float(quat[1])
            pose.orientation.z = float(quat[2])
            pose.orientation.w = float(quat[3])
            cam_poses.poses.append(pose)
            response.scores.append(float(gg.scores[i]))
            response.widths.append(float(gg.widths[i]))
        return cam_poses

    def _transform_to_planning_frame(
        self, response: GraspFromMask.Response, cam_poses: PoseArray,
        pts: np.ndarray, frame_id: str, stamp,
    ) -> str:
        """Transform grasp poses + object bbox into the planning frame.

        Tries the cloud stamp first, falls back to the latest transform; on
        total TF failure poses_base is left empty. Returns the planning frame
        (read dynamically so it stays changeable via ros2 param set)."""
        planning_frame = self.get_parameter("planning_frame").value
        response.planning_frame = planning_frame
        response.poses_base.header.frame_id          = planning_frame
        response.poses_base.header.stamp             = stamp
        response.approach_poses_base.header.frame_id = planning_frame
        response.approach_poses_base.header.stamp    = stamp

        try:
            self._apply_planning_tf(response, cam_poses, pts, frame_id, planning_frame, stamp)
            self.get_logger().info(
                f"Transformed {len(response.poses_base.poses)} poses → {planning_frame}")
        except Exception as e:
            self.get_logger().warn(
                f"TF at stamp failed ({e}), retrying with latest transform…")
            try:
                self._apply_planning_tf(
                    response, cam_poses, pts, frame_id, planning_frame, rclpy.time.Time())
                self.get_logger().info(
                    f"Transformed {len(response.poses_base.poses)} poses → "
                    f"{planning_frame} (latest TF)")
            except Exception as e2:
                self.get_logger().warn(
                    f"TF {frame_id} → {planning_frame} unavailable: {e2}. "
                    f"poses_base will be empty.")
        return planning_frame

    def _apply_planning_tf(
        self, response: GraspFromMask.Response, cam_poses: PoseArray,
        pts: np.ndarray, frame_id: str, planning_frame: str, tf_stamp,
    ) -> None:
        tf = self._tf_buffer.lookup_transform(
            planning_frame, frame_id, tf_stamp,
            timeout=Duration(seconds=0.1))

        # Transform the point cloud to planning frame for bounding box.
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

        response.poses_base.poses.clear()
        response.approach_poses_base.poses.clear()
        response.height_below_grasp.clear()
        response.height_above_grasp.clear()

        # EE alignment: after transforming into the planning frame, rotate
        # −90° about Y in that (base_footprint) world frame so the gripper
        # approach points along the arm EE's local Z. Pre-multiply = world frame.
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

            # Approach pose: same orientation, backed out along approach axis.
            # Col-0 of the rotation matrix is the approach direction.
            rot = R.from_quat([p.orientation.x, p.orientation.y,
                               p.orientation.z, p.orientation.w]).as_matrix()
            approach_axis = rot[:, 0]
            ap = Pose()
            ap.position.x    = p.position.x - float(approach_axis[0]) * approach_dist
            ap.position.y    = p.position.y - float(approach_axis[1]) * approach_dist
            ap.position.z    = p.position.z - float(approach_axis[2]) * approach_dist
            ap.orientation   = p.orientation
            response.approach_poses_base.poses.append(ap)

    # ── Visualisation markers ────────────────────────────────────────

    def _publish_grasp_markers(
        self, gg: GraspGroup, frame_id: str, stamp,
        response: GraspFromMask.Response, planning_frame: str,
    ) -> None:
        """Gripper line markers (camera frame) + AABB cube (planning frame)."""
        ma = MarkerArray()
        lt = Duration(seconds=0).to_msg()

        # Clear previous markers
        clr = Marker()
        clr.action = Marker.DELETEALL
        clr.ns     = "gripper"
        ma.markers.append(clr)

        for i in range(min(10, len(gg))):
            ma.markers.append(
                self._gripper_marker(gg, i, frame_id, stamp, lt))

        if response.object_size.x > 0:
            bbox_m = Marker()
            bbox_m.header.frame_id = planning_frame
            bbox_m.header.stamp    = stamp
            bbox_m.ns              = "object_bbox"
            bbox_m.id              = 0
            bbox_m.type            = Marker.CUBE
            bbox_m.action          = Marker.ADD
            bbox_m.lifetime        = lt
            bbox_m.pose            = response.object_bbox_pose
            bbox_m.scale.x         = float(response.object_size.x)
            bbox_m.scale.y         = float(response.object_size.y)
            bbox_m.scale.z         = float(response.object_size.z)
            bbox_m.color.r         = 0.0
            bbox_m.color.g         = 0.8
            bbox_m.color.b         = 1.0
            bbox_m.color.a         = 0.25   # semi-transparent
            ma.markers.append(bbox_m)

        self.dbg_marker_pub.publish(ma)

    def _gripper_marker(
        self, gg: GraspGroup, i: int, frame_id: str, stamp, lifetime,
    ) -> Marker:
        rot   = gg.rotation_matrices[i]   # (3,3)
        trans = gg.translations[i]
        width = float(gg.widths[i])
        sc    = gg.scores[i]

        # GraspNet frame: col-0 = approach (x), col-1 = spread/width (y), col-2 = height (z)
        approach = rot[:, 0]
        spread   = rot[:, 1]

        # Geometry points
        palm      = trans - approach * self.MARKER_PALM_DEPTH_M
        l_tip     = trans + spread * (width / 2)
        r_tip     = trans - spread * (width / 2)
        l_base    = palm  + spread * (width / 2)
        r_base    = palm  - spread * (width / 2)
        approach_tip = trans + approach * self.MARKER_APPROACH_LEN_M

        if sc > 0.5:
            r, g, b = 0.0, 1.0, 0.2
        elif sc > 0.3:
            r, g, b = 1.0, 1.0, 0.0
        else:
            r, g, b = 1.0, 0.5, 0.0

        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp    = stamp
        m.ns              = "gripper"
        m.id              = i
        m.type            = Marker.LINE_LIST
        m.action          = Marker.ADD
        m.lifetime        = lifetime
        m.scale.x         = 0.004
        m.color.a         = 0.9
        m.color.r, m.color.g, m.color.b = r, g, b
        m.pose.orientation.w = 1.0
        m.points = [
            _pt(l_base),  _pt(l_tip),          # left finger
            _pt(r_base),  _pt(r_tip),          # right finger
            _pt(l_base),  _pt(r_base),         # palm bar
            _pt(trans),   _pt(approach_tip),   # approach direction
        ]
        return m


def main():
    rclpy.init()
    node = GraspFromMaskNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
