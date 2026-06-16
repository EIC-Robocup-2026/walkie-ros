#!/bin/sh
"exec" "${PERCEPTION_VENV:-$HOME/perception-venv/bin/python3}" "$0" "$@"
"""
grasp_pos_node.py

"Absolute best" grasp-position finder for Walkie. Combines a live, in-distribution
ZED camera view (subscribed here) with the segmented, near-complete per-object
cloud supplied in the request:

  1. Crop the LIVE cloud to the object's box (+margin) and feed THAT to GraspNet
     (in-distribution → calibrated scores, with table/neighbour collision context).
  2. Keep grasps whose centre lies inside the object box (OBB containment).
  3. Validate each grasp against the supplied object cloud's surface normals
     (antipodal quality, including the occluded far contact a single view misses),
     refining the opening width and centre.
  4. Rank by GraspNet quality + antipodal quality + soft preferences.

The only thing fed to GraspNet is the cropped live cloud (XYZ, num_point points,
camera-optical frame). The object cloud is used for the crop box, containment
selection, and antipodal validation — never fed raw (it would be out-of-distribution).

Services:
  /grasp/pos          walkie_perception/srv/GraspPos
  /grasp_pos/standby  std_srvs/srv/SetBool   (true=load GPU, false=unload)
  /grasp_pos/status   std_srvs/srv/Trigger
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

import numpy as np
import open3d as o3d
import torch
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import tf2_ros

from geometry_msgs.msg import Point, Pose
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import Marker, MarkerArray

from models.graspnet import GraspNet, pred_decode
from graspnetAPI.grasp import GraspGroup

from walkie_perception.srv import GraspPos


class NodeState(Enum):
    LOADING  = "loading"
    STANDBY  = "standby"
    ACTIVE   = "active"
    UNLOADED = "unloaded"


class PipelineAbort(Exception):
    """Abort the request pipeline with a user-facing failure message."""


def _pt(v) -> Point:
    return Point(x=float(v[0]), y=float(v[1]), z=float(v[2]))


def _unit(v: np.ndarray):
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-9 else None


class GraspPosNode(Node):

    # Live-cloud forward-axis (body X) range gate, metres.
    RANGE_MIN_M = 0.1
    RANGE_MAX_M = 3.0
    MARKER_PALM_DEPTH_M   = 0.02
    MARKER_APPROACH_LEN_M = 0.06

    _DYNAMIC_PARAMS = ("planning_frame", "approach_dist_m")

    def __init__(self):
        super().__init__("grasp_pos")

        self._setup_parameters()
        self._init_runtime_state()
        self._check_gpu()

        self._load_model()
        self._set_state(NodeState.STANDBY)
        self.get_logger().info(
            f"Model ready (standby). VRAM: {self._vram_info()}")

        self._create_ros_interfaces()
        self.get_logger().info(
            "GraspPos ready\n"
            f"  live feed : {self.cloud_topic}\n"
            "  /grasp/pos         — find grasp(s): live crop + GraspNet + antipodal validation\n"
            "  /grasp_pos/standby — true=load GPU, false=unload\n"
            "  /grasp_pos/status  — check state and VRAM")

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
            ("live_merge_frames", 3),     # live frames merged to fill holes
            ("crop_margin_m", 0.12),      # default crop margin if request <= 0
            ("select_margin_m", 0.02),    # OBB containment slack for selection
            ("outlier_removal",      True),
            ("outlier_nb_neighbors", 20),
            ("outlier_std_ratio",    2.0),
            ("rerank_pool_size",     200),
            # Antipodal validation (against the supplied object cloud).
            ("normal_radius_m",      0.02),
            ("normal_max_nn",        30),
            ("antipodal_r_tol_m",    0.01),   # contact search tube radius
            ("antipodal_min_pts",    4),
            ("width_clearance_m",    0.01),
            ("max_gripper_width_m",  0.10),
            ("debug_cloud",  True),
            ("cloud_topic",
             "/zed_head/zed_node/point_cloud/cloud_registered"),
            ("cloud_optical_frame", "zed_head_left_camera_frame_optical"),
            ("planning_frame",  "base_footprint"),
            ("approach_dist_m", 0.10),
            ("tf_timeout_s",    0.3),
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

        self._cloud_cache = deque(maxlen=self.cache_size)
        self._cloud_lock  = threading.Lock()

        self._inference_lock = threading.Lock()

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

    def _check_gpu(self) -> None:
        if not torch.cuda.is_available():
            self.get_logger().fatal("CUDA not available — GPU required")
            raise RuntimeError("CUDA not available")
        self.get_logger().info(
            f"GPU: {torch.cuda.get_device_name(0)} | "
            f"sm_{torch.cuda.get_device_capability()[0]}"
            f"{torch.cuda.get_device_capability()[1]}")
        self.get_logger().info(f"VRAM before load: {self._vram_info()}")

    def _create_ros_interfaces(self) -> None:
        qos_be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=5)
        self.create_subscription(
            PointCloud2, self.cloud_topic, self._cloud_cb, qos_be)

        self.create_service(GraspPos, "/grasp/pos", self._handle_request)
        self.create_service(SetBool, "/grasp_pos/standby", self._handle_standby)
        self.create_service(Trigger, "/grasp_pos/status", self._handle_status)

        self.dbg_cloud_pub = self.create_publisher(
            PointCloud2, "/grasp/debug/pos_crop_cloud", 10)
        self.dbg_marker_pub = self.create_publisher(
            MarkerArray, "/grasp/debug/pos_grasp_markers", 10)

    # ── State / GPU helpers ──────────────────────────────────────────

    def _get_state(self) -> NodeState:
        with self._state_lock:
            return self._state

    def _set_state(self, state: NodeState) -> None:
        with self._state_lock:
            self._state = state

    def _vram_used_mb(self) -> int:
        free, total = torch.cuda.mem_get_info()
        return (total - free) // 1024**2

    def _vram_info(self) -> str:
        free, total = torch.cuda.mem_get_info()
        used = total - free
        return (f"{used // 1024**2} MB used / {total // 1024**2} MB total "
                f"({free // 1024**2} MB free)")

    def _load_model(self) -> None:
        self.get_logger().info(f"Loading checkpoint: {self.checkpoint_path}")
        t0 = time.perf_counter()
        self.net = GraspNet(
            input_feature_dim=0, num_view=self.num_view, num_angle=12,
            num_depth=4, cylinder_radius=0.05, hmin=-0.02,
            hmax_list=[0.01, 0.02, 0.03, 0.04], is_training=False)
        ckpt = torch.load(self.checkpoint_path, map_location="cuda")
        self.net.load_state_dict(ckpt["model_state_dict"])
        self.net = self.net.cuda().eval()
        self.get_logger().info(
            f"Checkpoint loaded in {(time.perf_counter()-t0)*1e3:.0f} ms. "
            f"VRAM: {self._vram_info()}")
        self._warmup()

    def _warmup(self) -> None:
        self.get_logger().info("Running warm-up inference...")
        dummy = torch.randn(1, self.num_point, 3).cuda()
        with torch.no_grad():
            pred_decode(self.net({"point_clouds": dummy}))
        torch.cuda.synchronize()
        self.get_logger().info(f"Warm-up done. VRAM: {self._vram_info()}")

    def _unload_model(self) -> int:
        used_before = self._vram_used_mb()
        self.net = self.net.cpu()
        del self.net
        self.net = None
        gc.collect()
        torch.cuda.empty_cache()
        return used_before - self._vram_used_mb()

    # ── Subscriber ───────────────────────────────────────────────────

    def _cloud_cb(self, msg: PointCloud2) -> None:
        with self._cloud_lock:
            self._cloud_cache.append(msg)

    # ── Cloud decoding ───────────────────────────────────────────────

    @staticmethod
    def _organized_xyz(cloud_msg: PointCloud2) -> np.ndarray:
        """Organized cloud → (H, W, 3) float32 XYZ (first three fields)."""
        H, W = cloud_msg.height, cloud_msg.width
        buf = np.frombuffer(cloud_msg.data, dtype=np.uint8).reshape(
            H, W, cloud_msg.point_step)
        return buf[:, :, 0:12].copy().view(np.float32).reshape(H, W, 3)

    def _live_optical_points(self, cached: list) -> np.ndarray:
        """Merge the last live frames → (N,3) in the optical convention
        (X-right, Y-down, Z-fwd). ZED organized cloud is body frame (X-fwd)."""
        n = min(self.live_merge_frames, len(cached))
        out = []
        for msg in cached[-n:]:
            xyz = self._organized_xyz(msg).reshape(-1, 3)
            xyz = xyz[np.isfinite(xyz).all(axis=1)]
            fwd = xyz[:, 0]
            xyz = xyz[(fwd > self.RANGE_MIN_M) & (fwd < self.RANGE_MAX_M)]
            if len(xyz):
                # body (X-fwd, Y-left, Z-up) → optical (X-right, Y-down, Z-fwd)
                out.append(np.stack([-xyz[:, 1], -xyz[:, 2], xyz[:, 0]], axis=1))
        if not out:
            return np.empty((0, 3), dtype=np.float32)
        return np.vstack(out).astype(np.float32)

    @staticmethod
    def _cloud_to_xyz(cloud_msg: PointCloud2) -> np.ndarray:
        """Any PointCloud2 → (N,3) float32 XYZ, finite only (by field offset)."""
        offs = {f.name: f.offset for f in cloud_msg.fields
                if f.name in ("x", "y", "z")}
        if not all(k in offs for k in ("x", "y", "z")):
            raise PipelineAbort(
                f"object_cloud missing x/y/z fields "
                f"({[f.name for f in cloud_msg.fields]})")
        n = cloud_msg.width * cloud_msg.height
        if n == 0:
            return np.empty((0, 3), dtype=np.float32)
        buf = np.frombuffer(cloud_msg.data, dtype=np.uint8).reshape(
            n, cloud_msg.point_step)
        xyz = np.stack([buf[:, offs[a]:offs[a] + 4].copy().view(np.float32).reshape(n)
                        for a in ("x", "y", "z")], axis=1)
        return xyz[np.isfinite(xyz).all(axis=1)].astype(np.float32)

    def _make_pc2(self, pts: np.ndarray, frame_id: str, stamp) -> PointCloud2:
        if pts.dtype != np.float32:
            pts = pts.astype(np.float32)
        if not pts.flags["C_CONTIGUOUS"]:
            pts = np.ascontiguousarray(pts)
        msg = PointCloud2()
        msg.header.frame_id = frame_id
        msg.header.stamp    = stamp
        msg.height, msg.width = 1, len(pts)
        msg.is_dense, msg.is_bigendian = True, False
        msg.point_step, msg.row_step = 12, 12 * len(pts)
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)]
        msg.data = pts.tobytes()
        return msg

    # ── TF helper ────────────────────────────────────────────────────

    def _lookup_RT(self, target: str, source: str, stamp):
        """target ← source as (R 3x3, t 3). Tries stamp then latest; None on fail."""
        for tf_stamp, label in ((stamp, "stamp"), (rclpy.time.Time(), "latest")):
            try:
                tf = self._tf_buffer.lookup_transform(
                    target, source, tf_stamp,
                    timeout=Duration(seconds=self.tf_timeout_s))
                q = tf.transform.rotation
                t = tf.transform.translation
                return (R.from_quat([q.x, q.y, q.z, q.w]).as_matrix(),
                        np.array([t.x, t.y, t.z]))
            except Exception as e:
                self.get_logger().warn(
                    f"TF {source} → {target} ({label}) failed: {e}")
        return None

    # ── Filtering ────────────────────────────────────────────────────

    def _filter(self, pts: np.ndarray) -> np.ndarray:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        pts = np.asarray(
            pcd.voxel_down_sample(self.voxel_size_m).points).astype(np.float32)
        if self.outlier_removal and len(pts) > self.outlier_nb_neighbors:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pts)
            _, idx = pcd.remove_statistical_outlier(
                nb_neighbors=self.outlier_nb_neighbors,
                std_ratio=self.outlier_std_ratio)
            pts = pts[idx]
        return pts

    def _sample(self, pts: np.ndarray) -> np.ndarray:
        replace = len(pts) < self.num_point
        idx = np.random.choice(len(pts), self.num_point, replace=replace)
        return pts[idx]

    # ── Antipodal validation against the object cloud ────────────────

    def _estimate_normals(self, obj_pts: np.ndarray) -> np.ndarray:
        """Per-point normals oriented outward from the object centroid."""
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(obj_pts)
        pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
            radius=self.normal_radius_m, max_nn=self.normal_max_nn))
        n = np.asarray(pcd.normals)
        d = obj_pts - obj_pts.mean(axis=0)
        flip = (n * d).sum(axis=1) < 0
        n[flip] *= -1.0
        return n

    def _antipodal(self, center, closing, obj_pts, obj_normals, gn_width):
        """Antipodal quality + refined (width, centre) for a grasp, evaluated
        on the object surface. Returns (score[0,1], width, centre)."""
        u = _unit(np.asarray(closing, dtype=np.float64))
        if u is None:
            return 0.5, gn_width, center
        rel = obj_pts - center
        s = rel @ u
        perp = np.linalg.norm(rel - np.outer(s, u), axis=1)
        near = perp < self.antipodal_r_tol_m
        if int(near.sum()) < self.antipodal_min_pts:
            return 0.5, gn_width, center        # neutral, no refine
        s_n = s[near]; n_n = obj_normals[near]
        ip, im = int(np.argmax(s_n)), int(np.argmin(s_n))
        a_plus  = float(np.dot(_unit(n_n[ip]) if _unit(n_n[ip]) is not None
                               else n_n[ip],  u))
        a_minus = float(np.dot(_unit(n_n[im]) if _unit(n_n[im]) is not None
                               else n_n[im], -u))
        score = float(np.clip(a_plus, 0, 1) * np.clip(a_minus, 0, 1))
        contact_w = float(s_n[ip] - s_n[im])
        width = float(min(contact_w + self.width_clearance_m, self.max_gripper_width_m))
        centre = center + u * ((s_n[ip] + s_n[im]) / 2.0)
        return score, width, centre

    # ── Services: status / standby ───────────────────────────────────

    def _handle_status(self, _req, resp) -> Trigger.Response:
        with self._cloud_lock:
            n = len(self._cloud_cache)
        resp.success = True
        resp.message = (f"state={self._get_state().value} | "
                        f"VRAM: {self._vram_info()} | "
                        f"live_frames={n}/{self.cache_size}")
        self.get_logger().info(resp.message)
        return resp

    def _handle_standby(self, request, response):
        if request.data:
            return self._standby_load(response)
        return self._standby_unload(response)

    def _standby_load(self, response):
        state = self._get_state()
        if state in (NodeState.STANDBY, NodeState.ACTIVE, NodeState.LOADING):
            response.success = True
            response.message = f"Already {state.value}. VRAM: {self._vram_info()}"
            return response
        self._set_state(NodeState.LOADING)
        try:
            self._load_model()
            self._set_state(NodeState.STANDBY)
            response.success = True
            response.message = f"Model loaded. VRAM: {self._vram_info()}"
        except Exception as exc:
            self._set_state(NodeState.UNLOADED)
            response.success = False
            response.message = f"Load failed: {exc}"
        return response

    def _standby_unload(self, response):
        state = self._get_state()
        if state == NodeState.UNLOADED:
            response.success = True
            response.message = f"Already unloaded. VRAM: {self._vram_info()}"
            return response
        if state in (NodeState.ACTIVE, NodeState.LOADING):
            response.success = False
            response.message = f"Busy ({state.value}) — try again shortly"
            return response
        self._set_state(NodeState.ACTIVE)
        try:
            freed = self._unload_model()
            self._set_state(NodeState.UNLOADED)
            response.success = True
            response.message = f"Unloaded. Freed ~{freed} MB. VRAM: {self._vram_info()}"
        except Exception as exc:
            self._set_state(NodeState.STANDBY)
            response.success = False
            response.message = f"Unload failed: {exc}"
        return response

    # ── Service: /grasp/pos ──────────────────────────────────────────

    def _handle_request(self, request, response):
        t0 = time.perf_counter()
        state = self._get_state()
        if state == NodeState.UNLOADED:
            response.success = False
            response.message = ("Model unloaded. Call /grasp_pos/standby "
                                "std_srvs/srv/SetBool '{data: true}'")
            return response
        if state in (NodeState.LOADING, NodeState.ACTIVE):
            response.success = False
            response.message = f"Busy ({state.value}) — try again shortly"
            return response

        self._set_state(NodeState.ACTIVE)
        try:
            response = self._run_pipeline(request, response, t0)
        except PipelineAbort as abort:
            response.success = False
            response.message = str(abort)
        finally:
            self._set_state(NodeState.STANDBY)
        return response

    # ── Pipeline ─────────────────────────────────────────────────────

    def _run_pipeline(self, request, response, t0):
        # 0. Object cloud (segment + validation surface) ----------------
        obj_msg = request.object_cloud
        obj_frame = obj_msg.header.frame_id or "map"
        obj_pts = self._cloud_to_xyz(obj_msg)
        if len(obj_pts) < self.antipodal_min_pts:
            raise PipelineAbort(
                f"object_cloud has too few points ({len(obj_pts)})")

        # 1. Live view snapshot → optical points ------------------------
        with self._cloud_lock:
            cached = list(self._cloud_cache)
        if not cached:
            raise PipelineAbort(
                f"No live frames cached — is ZED running? ({self.cloud_topic})")
        live_opt = self._live_optical_points(cached)
        opt_frame = self.cloud_optical_frame
        stamp = cached[-1].header.stamp
        if len(live_opt) == 0:
            raise PipelineAbort("Live view produced no valid points")

        # 2. Bring the object into the optical frame (crop box + normals)
        RT = self._lookup_RT(opt_frame, obj_frame, stamp)
        if RT is None:
            raise PipelineAbort(
                f"TF {obj_frame} → {opt_frame} unavailable — cannot relate "
                f"object to camera")
        R_oo, t_oo = RT
        obj_opt = obj_pts @ R_oo.T + t_oo
        obj_min, obj_max = obj_opt.min(axis=0), obj_opt.max(axis=0)

        # 3. Crop the LIVE cloud to the object box + margin -------------
        margin = request.crop_margin_m if request.crop_margin_m > 0 else self.crop_margin_m
        lo, hi = obj_min - margin, obj_max + margin
        m = np.all((live_opt >= lo) & (live_opt <= hi), axis=1)
        crop = live_opt[m]
        if len(crop) < self.min_points:
            raise PipelineAbort(
                f"Live view has too few points in the object region "
                f"({len(crop)}; need {self.min_points}) — object occluded or moved?")
        crop = self._filter(crop)
        if len(crop) < self.min_points:
            raise PipelineAbort(
                f"Too few points after filtering ({len(crop)})")
        pts = self._sample(crop)

        if self.debug_cloud:
            self.dbg_cloud_pub.publish(self._make_pc2(pts, opt_frame, stamp))

        # 4. GraspNet on the in-distribution live crop ------------------
        raw, infer_ms = self._run_graspnet(pts)
        grasps_raw = len(raw)
        gg = self._select_grasps(raw, request)
        if len(gg) == 0:
            raise PipelineAbort(
                f"No grasps above threshold {request.score_threshold} "
                f"(raw: {grasps_raw})")

        # 5. OBB containment + antipodal validation (optical frame) -----
        obj_normals = self._estimate_normals(obj_opt)
        sel_lo = obj_min - self.select_margin_m
        sel_hi = obj_max + self.select_margin_m

        cand = []   # dicts of optical-frame data for kept grasps
        for i in range(len(gg)):
            c = np.asarray(gg.translations[i], dtype=np.float64)
            if not np.all((c >= sel_lo) & (c <= sel_hi)):
                continue                              # grasp not on this object
            rot = np.asarray(gg.rotation_matrices[i], dtype=np.float64)
            closing = rot[:, 1]                       # GraspNet col-1 = closing/spread
            anti, width, c_ref = self._antipodal(
                c, closing, obj_opt, obj_normals, float(gg.widths[i]))
            cand.append({"idx": i, "center": c_ref, "rot": rot,
                         "gn": float(gg.scores[i]), "anti": anti, "width": width})

        if not cand:
            raise PipelineAbort(
                f"GraspNet found {len(gg)} grasps but none lay on the object "
                f"(OBB containment) — check the object cloud / crop margin")

        # 6. Transform candidates to the planning frame -----------------
        planning_frame = self.get_parameter("planning_frame").value
        RT_po = self._lookup_RT(planning_frame, opt_frame, stamp)
        response.planning_frame = planning_frame
        response.poses_base.header.frame_id          = planning_frame
        response.poses_base.header.stamp             = stamp
        response.approach_poses_base.header.frame_id = planning_frame
        response.approach_poses_base.header.stamp    = stamp
        if RT_po is None:
            raise PipelineAbort(
                f"TF {opt_frame} → {planning_frame} unavailable")
        R_po, t_po = RT_po

        # Object AABB in planning frame (from the supplied object cloud).
        obj_plan = obj_opt @ R_po.T + t_po
        z_min, z_max = float(obj_plan[:, 2].min()), float(obj_plan[:, 2].max())
        x_min, x_max = float(obj_plan[:, 0].min()), float(obj_plan[:, 0].max())
        y_min, y_max = float(obj_plan[:, 1].min()), float(obj_plan[:, 1].max())
        response.object_size.x = x_max - x_min
        response.object_size.y = y_max - y_min
        response.object_size.z = z_max - z_min
        response.object_bbox_pose.position.x = (x_min + x_max) / 2.0
        response.object_bbox_pose.position.y = (y_min + y_max) / 2.0
        response.object_bbox_pose.position.z = (z_min + z_max) / 2.0
        response.object_bbox_pose.orientation.w = 1.0

        ee_pitch = R.from_euler("y", -90.0, degrees=True).as_matrix()
        for c in cand:
            R_final = ee_pitch @ (R_po @ c["rot"])
            pos = R_po @ c["center"] + t_po
            c["pos_plan"] = pos
            c["R_plan"] = R_final
            c["approach"] = R_final[:, 0]
            c["frac"] = ((pos[2] - z_min) / (z_max - z_min)
                         if (z_max - z_min) > 1e-6 else 0.5)

        # 7. Composite ranking ------------------------------------------
        order = self._rank(request, cand)
        n_out = request.max_grasps if request.max_grasps > 0 else min(len(cand), 20)
        order = order[:n_out]

        approach_dist = self.get_parameter("approach_dist_m").value
        for j in order:
            c = cand[j]
            q = R.from_matrix(c["R_plan"]).as_quat()
            p = Pose()
            p.position.x, p.position.y, p.position.z = (
                float(c["pos_plan"][0]), float(c["pos_plan"][1]), float(c["pos_plan"][2]))
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (
                float(q[0]), float(q[1]), float(q[2]), float(q[3]))
            response.poses_base.poses.append(p)
            response.scores.append(float(c["gn"]))
            response.antipodal_scores.append(float(c["anti"]))
            response.widths.append(float(c["width"]))
            response.height_below_grasp.append(float(p.position.z - z_min))
            response.height_above_grasp.append(float(z_max - p.position.z))

            axis = c["approach"]
            ap = Pose()
            ap.position.x = p.position.x - float(axis[0]) * approach_dist
            ap.position.y = p.position.y - float(axis[1]) * approach_dist
            ap.position.z = p.position.z - float(axis[2]) * approach_dist
            ap.orientation = p.orientation
            response.approach_poses_base.poses.append(ap)

        self._publish_markers([cand[j] for j in order], opt_frame, stamp,
                              response, planning_frame)

        top = cand[order[0]]
        response.success = True
        response.message = (
            f"OK | {len(order)} grasp(s) | top gn {top['gn']:.3f} "
            f"antipodal {top['anti']:.2f} width {top['width']*100:.1f}cm")
        self.get_logger().info(
            f"{response.message} | live crop {len(crop)} pts | "
            f"{len(cand)}/{len(gg)} on-object | raw {grasps_raw} | "
            f"infer {infer_ms:.0f} ms | total {(time.perf_counter()-t0)*1e3:.0f} ms")
        return response

    def _run_graspnet(self, pts: np.ndarray) -> Tuple[object, float]:
        cloud_t = torch.from_numpy(pts).unsqueeze(0).cuda()
        t = time.perf_counter()
        with self._inference_lock:
            with torch.no_grad():
                preds = pred_decode(self.net({"point_clouds": cloud_t}))
        torch.cuda.synchronize()
        return preds[0], (time.perf_counter() - t) * 1e3

    def _select_grasps(self, raw, request) -> GraspGroup:
        gg = GraspGroup(raw.cpu().numpy())
        gg = gg.sort_by_score()[:2000].nms()
        if request.score_threshold > 0:
            gg = gg[gg.scores >= request.score_threshold]
        if len(gg) == 0:
            return gg
        return gg[:max(1, int(self.rerank_pool_size))]

    def _rank(self, request, cand):
        """Order candidate indices best-first by quality + antipodal + prefs.
        All-zero weights → default to quality + antipodal (the 'best' default)."""
        w_q = float(request.quality_weight)
        w_n = float(request.antipodal_weight)
        w_a = float(request.approach_weight)
        w_p = float(request.position_weight)
        w_w = float(request.width_weight)
        w_r = float(request.region_weight)
        if (w_q + w_n + w_a + w_p + w_w + w_r) <= 0.0:
            w_q, w_n = 1.0, 1.0

        score = (w_q * np.array([c["gn"] for c in cand])
                 + w_n * np.array([c["anti"] for c in cand]))

        pref_a = _unit(np.array([request.preferred_approach.x,
                                 request.preferred_approach.y,
                                 request.preferred_approach.z]))
        if w_a > 0 and pref_a is not None:
            score += w_a * np.array([
                (float(np.dot(_unit(c["approach"]), pref_a)) + 1.0) / 2.0
                for c in cand])

        if w_p > 0:
            tgt = np.array([request.preferred_position.x,
                            request.preferred_position.y,
                            request.preferred_position.z])
            fall = request.position_falloff_m if request.position_falloff_m > 0 else 0.05
            score += w_p * np.array([
                np.exp(-np.linalg.norm(c["pos_plan"] - tgt) / fall) for c in cand])

        if w_w > 0 and request.preferred_width > 0:
            pw = float(request.preferred_width)
            score += w_w * np.array([
                np.clip(1.0 - abs(c["width"] - pw) / pw, 0.0, 1.0) for c in cand])

        if w_r > 0:
            rf = float(np.clip(request.region_frac, 0.0, 1.0))
            score += w_r * np.array([
                np.clip(1.0 - abs(c["frac"] - rf), 0.0, 1.0) for c in cand])

        return list(np.argsort(-score))

    # ── Debug markers ────────────────────────────────────────────────

    def _publish_markers(self, cand, opt_frame, stamp, response, planning_frame):
        ma = MarkerArray()
        lt = Duration(seconds=0).to_msg()
        clr = Marker(); clr.action = Marker.DELETEALL; clr.ns = "grasp_pos"
        ma.markers.append(clr)

        for i, c in enumerate(cand[:10]):
            center = c["center"]; rot = c["rot"]; width = c["width"]
            approach = rot[:, 0]; spread = rot[:, 1]
            palm = center - approach * self.MARKER_PALM_DEPTH_M
            sc = c["anti"]
            r, g, b = ((0.0, 1.0, 0.2) if sc > 0.6 else
                       (1.0, 1.0, 0.0) if sc > 0.3 else (1.0, 0.5, 0.0))
            m = Marker()
            m.header.frame_id = opt_frame
            m.header.stamp = stamp
            m.ns = "grasp_pos"; m.id = i
            m.type = Marker.LINE_LIST; m.action = Marker.ADD; m.lifetime = lt
            m.scale.x = 0.004; m.color.a = 0.9
            m.color.r, m.color.g, m.color.b = r, g, b
            m.pose.orientation.w = 1.0
            m.points = [
                _pt(palm + spread * (width / 2)), _pt(center + spread * (width / 2)),
                _pt(palm - spread * (width / 2)), _pt(center - spread * (width / 2)),
                _pt(palm + spread * (width / 2)), _pt(palm - spread * (width / 2)),
                _pt(center), _pt(center + approach * self.MARKER_APPROACH_LEN_M)]
            ma.markers.append(m)

        if response.object_size.x > 0:
            bm = Marker()
            bm.header.frame_id = planning_frame
            bm.header.stamp = stamp
            bm.ns = "object_bbox"; bm.id = 0
            bm.type = Marker.CUBE; bm.action = Marker.ADD; bm.lifetime = lt
            bm.pose = response.object_bbox_pose
            bm.scale.x = float(response.object_size.x)
            bm.scale.y = float(response.object_size.y)
            bm.scale.z = float(response.object_size.z)
            bm.color.r, bm.color.g, bm.color.b, bm.color.a = 0.0, 0.8, 1.0, 0.25
            ma.markers.append(bm)

        self.dbg_marker_pub.publish(ma)


def main():
    rclpy.init()
    node = GraspPosNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
