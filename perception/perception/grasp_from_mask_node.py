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
from typing import Optional

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

from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped, Quaternion, Vector3
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


class GraspFromMaskNode(Node):

    def __init__(self):
        super().__init__("grasp_from_mask")

        # ── Parameters ───────────────────────────────────────────────
        self.declare_parameter(
            "checkpoint_path",
            os.path.expanduser(
                "~/graspnet-baseline/logs/log_rs/checkpoint-rs.tar"
            ),
        )
        self.declare_parameter("num_point",    10000)
        self.declare_parameter("num_view",     300)
        self.declare_parameter("voxel_size_m", 0.005)
        self.declare_parameter("cache_size",   10)
        self.declare_parameter("min_points",   300)
        self.declare_parameter("debug_cloud",  True)
        self.declare_parameter("use_mask",     True)
        self.declare_parameter("outlier_removal",      True)
        self.declare_parameter("outlier_nb_neighbors", 20)
        self.declare_parameter("outlier_std_ratio",    2.0)
        self.declare_parameter("cluster_filter",       True)
        self.declare_parameter("cluster_eps",          0.02)
        self.declare_parameter("cluster_min_samples",  10)
        self.declare_parameter("depth_topic",
            "/zed_head/zed_node/depth/depth_registered")
        self.declare_parameter("info_topic",
            "/zed_head/zed_node/depth/camera_info")
        self.declare_parameter("planning_frame", "base_footprint")

        p = self.get_parameter
        self.checkpoint_path = os.path.expanduser(p("checkpoint_path").value)
        self.num_point    = p("num_point").value
        self.num_view     = p("num_view").value
        self.voxel_size_m = p("voxel_size_m").value
        self.cache_size   = p("cache_size").value
        self.min_points   = p("min_points").value
        self.debug_cloud  = p("debug_cloud").value
        self.use_mask     = p("use_mask").value
        self.outlier_removal      = p("outlier_removal").value
        self.outlier_nb_neighbors = p("outlier_nb_neighbors").value
        self.outlier_std_ratio    = p("outlier_std_ratio").value
        self.cluster_filter      = p("cluster_filter").value
        self.cluster_eps         = p("cluster_eps").value
        self.cluster_min_samples = p("cluster_min_samples").value
        self.depth_topic  = p("depth_topic").value
        self.info_topic   = p("info_topic").value
        # planning_frame is read dynamically each call — changeable via ros2 param set

        # ── State ────────────────────────────────────────────────────
        self._state      = NodeState.LOADING
        self._state_lock = threading.Lock()
        self.net: Optional[GraspNet] = None

        # ── Depth cache ──────────────────────────────────────────────
        self._depth_cache = deque(maxlen=self.cache_size)
        self._depth_lock  = threading.Lock()

        # ── Camera intrinsics ────────────────────────────────────────
        self._fx = self._fy = self._cx = self._cy = None
        self._info_lock = threading.Lock()
        self._depth_frame_id = "zed_left_camera_frame_optical"

        # ── Inference lock (one at a time) ───────────────────────────
        self._inference_lock = threading.Lock()

        # ── TF ───────────────────────────────────────────────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Check GPU ────────────────────────────────────────────────
        if not torch.cuda.is_available():
            self.get_logger().fatal("CUDA not available — GPU required")
            raise RuntimeError("CUDA not available")
        self.get_logger().info(
            f"GPU: {torch.cuda.get_device_name(0)} | "
            f"sm_{torch.cuda.get_device_capability()[0]}"
            f"{torch.cuda.get_device_capability()[1]}"
        )
        self.get_logger().info(f"VRAM before load: {self._vram_info()}")

        # ── Load model ───────────────────────────────────────────────
        self._load_model()
        with self._state_lock:
            self._state = NodeState.STANDBY
        self.get_logger().info(
            f"Model ready (standby). VRAM: {self._vram_info()}"
        )

        # ── QoS ──────────────────────────────────────────────────────
        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, depth=5
        )

        # ── Subscriptions ─────────────────────────────────────────────
        self.create_subscription(
            CameraInfo, self.info_topic, self._info_cb, qos_be)
        self.create_subscription(
            Image, self.depth_topic, self._depth_cb, qos_be)

        # ── Services ──────────────────────────────────────────────────
        self.create_service(
            GraspFromMask, "/grasp/from_mask", self._handle_request)
        self.create_service(
            SetBool, "/grasp/standby", self._handle_standby)
        self.create_service(
            Trigger, "/grasp/status", self._handle_status)

        # ── Publishers ────────────────────────────────────────────────
        self.dbg_cloud_pub = self.create_publisher(
            PointCloud2, "/grasp/debug/masked_cloud", 10)
        self.dbg_marker_pub = self.create_publisher(
            MarkerArray, "/grasp/debug/grasp_markers", 10)

        self.get_logger().info(
            f"GraspFromMask ready\n"
            f"  depth : {self.depth_topic}\n"
            f"  info  : {self.info_topic}\n"
            f"  /grasp/from_mask  — request a grasp\n"
            f"  /grasp/standby    — true=load GPU, false=unload\n"
            f"  /grasp/status     — check state and VRAM"
        )

    # ── Helpers ──────────────────────────────────────────────────────

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

    def _bbox_mask(self, H: int, W: int, bbox) -> np.ndarray:
        cx = int(bbox.center.position.x)
        cy = int(bbox.center.position.y)
        hw = max(1, int(bbox.size_x / 2))
        hh = max(1, int(bbox.size_y / 2))
        m = np.zeros((H, W), dtype=bool)
        m[max(0, cy - hh):min(H, cy + hh),
          max(0, cx - hw):min(W, cx + hw)] = True
        return m

    def _unproject(
        self,
        depth_msg: Image,
        mask: np.ndarray,
    ) -> np.ndarray:
        """Unproject masked depth pixels to XYZ using pinhole model."""
        if depth_msg.encoding == "16UC1":
            depth = np.frombuffer(
                depth_msg.data, dtype=np.uint16
            ).reshape(depth_msg.height, depth_msg.width).astype(np.float32) / 1000.0
        else:
            depth = np.frombuffer(
                depth_msg.data, dtype=np.float32
            ).reshape(depth_msg.height, depth_msg.width)

        m = mask
        if m.shape != (depth_msg.height, depth_msg.width):
            m = cv2.resize(
                m.astype(np.uint8),
                (depth_msg.width, depth_msg.height),
                interpolation=cv2.INTER_NEAREST,
            ).astype(bool)

        vs, us = np.where(m)
        z = depth[vs, us]
        valid = np.isfinite(z) & (z > 0.1) & (z < 3.0)
        us = us[valid].astype(np.float32)
        vs = vs[valid].astype(np.float32)
        z  = z[valid]

        if len(z) == 0:
            return np.empty((0, 3), dtype=np.float32)

        with self._info_lock:
            fx, fy, cx, cy = self._fx, self._fy, self._cx, self._cy

        x = (us - cx) * z / fx
        y = (vs - cy) * z / fy
        return np.stack([x, y, z], axis=1).astype(np.float32)

    # ── Subscribers ──────────────────────────────────────────────────

    def _info_cb(self, msg: CameraInfo) -> None:
        with self._info_lock:
            if self._fx is not None:
                return
            self._fx = msg.k[0]
            self._fy = msg.k[4]
            self._cx = msg.k[2]
            self._cy = msg.k[5]
            self._depth_frame_id = msg.header.frame_id
        self.get_logger().info(
            f"Camera info: fx={self._fx:.1f} fy={self._fy:.1f} "
            f"cx={self._cx:.1f} cy={self._cy:.1f}"
        )

    def _depth_cb(self, msg: Image) -> None:
        with self._depth_lock:
            self._depth_cache.append(msg)

    # ── Service: status ──────────────────────────────────────────────

    def _handle_status(self, _req, resp) -> Trigger.Response:
        with self._state_lock:
            state = self._state
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

        with self._state_lock:
            state = self._state

        if request.data:
            if state in (NodeState.STANDBY, NodeState.ACTIVE, NodeState.LOADING):
                response.success = True
                response.message = (
                    f"Already in state={state.value}. "
                    f"VRAM: {self._vram_info()}"
                )
                return response
            with self._state_lock:
                self._state = NodeState.LOADING
            try:
                self._load_model()
                with self._state_lock:
                    self._state = NodeState.STANDBY
                response.success = True
                response.message = (
                    f"Model loaded and ready. VRAM: {self._vram_info()}")
                self.get_logger().info(response.message)
            except Exception as exc:
                with self._state_lock:
                    self._state = NodeState.UNLOADED
                response.success = False
                response.message = f"Load failed: {exc}"

        else:
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

            with self._state_lock:
                self._state = NodeState.ACTIVE
            try:
                vram_before = (
                    torch.cuda.mem_get_info()[1] - torch.cuda.mem_get_info()[0]
                ) // 1024**2
                self.net = self.net.cpu()
                del self.net
                self.net = None
                gc.collect()
                torch.cuda.empty_cache()
                vram_after = (
                    torch.cuda.mem_get_info()[1] - torch.cuda.mem_get_info()[0]
                ) // 1024**2
                with self._state_lock:
                    self._state = NodeState.UNLOADED
                response.success = True
                response.message = (
                    f"Model unloaded. Freed ~{vram_after - vram_before} MB. "
                    f"VRAM: {self._vram_info()}"
                )
                self.get_logger().info(response.message)
            except Exception as exc:
                with self._state_lock:
                    self._state = NodeState.STANDBY
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

        with self._state_lock:
            state = self._state

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

        with self._state_lock:
            self._state = NodeState.ACTIVE
        try:
            response = self._run_pipeline(request, response, t_start)
        finally:
            with self._state_lock:
                self._state = NodeState.STANDBY

        return response

    def _run_pipeline(
        self,
        request: GraspFromMask.Request,
        response: GraspFromMask.Response,
        t_start: float,
    ) -> GraspFromMask.Response:

        # ── Check prerequisites ───────────────────────────────────
        with self._info_lock:
            has_info = self._fx is not None

        if not has_info:
            response.success = False
            response.message = (
                f"Camera info not received yet — is ZED running? "
                f"({self.info_topic})"
            )
            return response

        with self._depth_lock:
            cached = list(self._depth_cache)

        if len(cached) == 0:
            response.success = False
            response.message = (
                f"No depth frames cached — is ZED running? "
                f"({self.depth_topic})"
            )
            return response

        # ── Build region mask (mask or bbox) ─────────────────────
        depth_ref = cached[-1]
        H, W = depth_ref.height, depth_ref.width

        if self.use_mask and request.mask.data:
            if request.mask.encoding != "16UC1":
                response.success = False
                response.message = (
                    f"Expected 16UC1 mask, got {request.mask.encoding}")
                return response

            mask_img = np.frombuffer(
                request.mask.data, dtype=np.uint16
            ).reshape(request.mask.height, request.mask.width)
            object_mask = mask_img == request.tracker_id

            if object_mask.sum() < 10:
                self.get_logger().warn(
                    f"Mask has {object_mask.sum()} pixels for "
                    f"tracker_id={request.tracker_id} — falling back to bbox"
                )
                object_mask = self._bbox_mask(H, W, request.bbox)
            else:
                self.get_logger().info(
                    f"mask mode: {object_mask.sum()} pixels for "
                    f"tracker_id={request.tracker_id}"
                )
        else:
            object_mask = self._bbox_mask(H, W, request.bbox)
            self.get_logger().info(
                f"bbox mode: centre=({int(request.bbox.center.position.x)},"
                f"{int(request.bbox.center.position.y)}) "
                f"size={int(request.bbox.size_x)}x{int(request.bbox.size_y)} "
                f"pixels={object_mask.sum()}"
            )

        # ── Adaptive frame accumulation ───────────────────────────
        if request.num_frames == 0:
            frames_to_try = sorted(set([1, 3, min(5, len(cached))]))
        else:
            frames_to_try = [min(request.num_frames, len(cached))]

        pts = None
        actual_frames = 0

        for n_frames in frames_to_try:
            frames = cached[-n_frames:]
            all_pts = []

            for depth_msg in frames:
                p = self._unproject(depth_msg, object_mask)
                if len(p) > 0:
                    all_pts.append(p)

            if not all_pts:
                continue

            merged = np.vstack(all_pts)
            actual_frames = n_frames

            # Downsample early so outlier removal and DBSCAN run on a small cloud.
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(merged)
            merged = np.asarray(
                pcd.voxel_down_sample(self.voxel_size_m).points
            ).astype(np.float32)

            if self.outlier_removal:
                before = len(merged)
                merged = self._remove_outliers(merged)
                self.get_logger().info(
                    f"outlier removal: {before} → {len(merged)} pts "
                    f"(removed {before - len(merged)})"
                )

            if self.cluster_filter:
                merged = self._keep_dominant_cluster(merged)

            if len(merged) >= self.min_points:
                pts = merged
                break

            self.get_logger().info(
                f"{len(merged)} pts from {n_frames} frame(s) "
                f"(need {self.min_points}) — trying more frames"
            )

        if pts is None or len(pts) < self.min_points:
            response.success = False
            response.message = (
                f"Insufficient points: "
                f"{len(pts) if pts is not None else 0} "
                f"after {frames_to_try[-1]} frame(s) "
                f"(need {self.min_points})"
            )
            return response

        response.points_extracted = len(pts)
        response.frames_used      = actual_frames

        # ── Sample to num_point ───────────────────────────────────
        if len(pts) >= self.num_point:
            idx = np.random.choice(len(pts), self.num_point, replace=False)
        else:
            idx = np.random.choice(len(pts), self.num_point, replace=True)
        pts = pts[idx]
        response.points_fed = len(pts)

        # ── Publish debug cloud ───────────────────────────────────
        if self.debug_cloud:
            last = cached[-1]
            self.dbg_cloud_pub.publish(
                self._make_pc2(pts,
                               frame_id=last.header.frame_id or self._depth_frame_id,
                               stamp=last.header.stamp)
            )

        # ── GraspNet inference ────────────────────────────────────
        cloud_t = torch.from_numpy(pts).unsqueeze(0).cuda()
        t_inf = time.perf_counter()

        with self._inference_lock:
            with torch.no_grad():
                ep    = self.net({"point_clouds": cloud_t})
                preds = pred_decode(ep)
        torch.cuda.synchronize()

        response.inference_ms = (time.perf_counter() - t_inf) * 1e3
        response.grasps_raw   = len(preds[0])

        # ── Post-process ──────────────────────────────────────────
        gg = GraspGroup(preds[0].cpu().numpy())
        gg = gg.sort_by_score()[:2000].nms()

        if request.score_threshold > 0:
            gg = gg[gg.scores >= request.score_threshold]

        if len(gg) == 0:
            response.success = False
            response.message = (
                f"No grasps above threshold {request.score_threshold} "
                f"(raw: {response.grasps_raw})"
            )
            return response

        n = request.max_grasps if request.max_grasps > 0 else min(len(gg), 20)
        gg = gg[:n]
        response.grasps_returned = len(gg)

        # ── Build pose response ───────────────────────────────────
        last = cached[-1]
        frame_id = last.header.frame_id or self._depth_frame_id
        stamp    = last.header.stamp
        response.poses.header.frame_id = frame_id
        response.poses.header.stamp    = stamp

        for i in range(len(gg)):
            rot   = gg.rotation_matrices[i]
            trans = gg.translations[i]
            quat  = R.from_matrix(rot).as_quat()

            pose = Pose()
            pose.position.x    = float(trans[0])
            pose.position.y    = float(trans[1])
            pose.position.z    = float(trans[2])
            pose.orientation.x = float(quat[0])
            pose.orientation.y = float(quat[1])
            pose.orientation.z = float(quat[2])
            pose.orientation.w = float(quat[3])
            response.poses.poses.append(pose)
            response.scores.append(float(gg.scores[i]))
            response.widths.append(float(gg.widths[i]))

        # ── Transform poses to planning frame ────────────────────────
        planning_frame = self.get_parameter("planning_frame").value
        response.planning_frame = planning_frame
        response.poses_base.header.frame_id = planning_frame
        response.poses_base.header.stamp    = stamp

        # EE alignment: after transforming into the planning frame, rotate
        # −90° about Y in that (base_footprint) world frame so the gripper
        # approach points along the arm EE's local Z. Pre-multiply = world frame.
        ee_pitch = R.from_euler("y", -90.0, degrees=True)
        # Z compensation: shift grasp along base_footprint +Z (calibration).
        z_offset_m = 0.025

        def _do_tf(tf_stamp):
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
            response.height_below_grasp.clear()
            response.height_above_grasp.clear()

            for pose in response.poses.poses:
                # Jazzy: do_transform_pose takes a bare Pose and returns a Pose.
                p = tf2_geometry_msgs.do_transform_pose(pose, tf)
                q_in = R.from_quat([p.orientation.x, p.orientation.y,
                                    p.orientation.z, p.orientation.w])
                p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (
                    float(v) for v in (ee_pitch * q_in).as_quat())
                p.position.z += z_offset_m
                response.poses_base.poses.append(p)
                response.height_below_grasp.append(float(p.position.z - z_min))
                response.height_above_grasp.append(float(z_max - p.position.z))

        try:
            _do_tf(stamp)
            self.get_logger().info(
                f"Transformed {len(response.poses_base.poses)} poses → {planning_frame}")
        except Exception as e:
            self.get_logger().warn(
                f"TF at stamp failed ({e}), retrying with latest transform…")
            try:
                _do_tf(rclpy.time.Time())
                self.get_logger().info(
                    f"Transformed {len(response.poses_base.poses)} poses → "
                    f"{planning_frame} (latest TF)")
            except Exception as e2:
                self.get_logger().warn(
                    f"TF {frame_id} → {planning_frame} unavailable: {e2}. "
                    f"poses_base will be empty.")

        # ── Publish gripper visualisation markers ─────────────────
        ma  = MarkerArray()
        lt  = Duration(seconds=0).to_msg()

        # Clear previous markers
        clr = Marker()
        clr.action = Marker.DELETEALL
        clr.ns     = "gripper"
        ma.markers.append(clr)

        finger_len = 0.04   # 4 cm fingers
        palm_depth = 0.02   # palm sits 2 cm back from grasp centre

        def _pt(v):
            return Point(x=float(v[0]), y=float(v[1]), z=float(v[2]))

        for i in range(min(10, len(gg))):
            rot   = gg.rotation_matrices[i]   # (3,3)
            trans = gg.translations[i]
            width = float(gg.widths[i])
            sc    = gg.scores[i]

            # GraspNet frame: col-0 = approach (x), col-1 = spread/width (y), col-2 = height (z)
            approach = rot[:, 0]
            spread   = rot[:, 1]

            # Geometry points
            palm      = trans - approach * palm_depth
            l_tip     = trans + spread * (width / 2)
            r_tip     = trans - spread * (width / 2)
            l_base    = palm  + spread * (width / 2)
            r_base    = palm  - spread * (width / 2)
            approach_tip = trans + approach * 0.06

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
            m.lifetime        = lt
            m.scale.x         = 0.004
            m.color.a         = 0.9
            m.color.r, m.color.g, m.color.b = r, g, b
            m.pose.orientation.w = 1.0
            m.points = [
                _pt(l_base),  _pt(l_tip),         # left finger
                _pt(r_base),  _pt(r_tip),          # right finger
                _pt(l_base),  _pt(r_base),         # palm bar
                _pt(trans),   _pt(approach_tip),   # approach direction
            ]
            ma.markers.append(m)

        self.dbg_marker_pub.publish(ma)

        # ── Final response ────────────────────────────────────────
        response.total_ms = (time.perf_counter() - t_start) * 1e3
        response.success  = True
        response.message  = (
            f"OK | "
            f"{response.grasps_returned} grasps | "
            f"{response.points_extracted} pts extracted | "
            f"{actual_frames} frame(s) | "
            f"infer {response.inference_ms:.0f} ms | "
            f"total {response.total_ms:.0f} ms | "
            f"top score {gg.scores[0]:.3f} | "
            f"top width {gg.widths[0]*100:.1f} cm"
        )
        self.get_logger().info(response.message)
        return response


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
