#!/bin/sh
"exec" "${PERCEPTION_VENV:-$HOME/perception-venv/bin/python3}" "$0" "$@"
"""
grasp_from_cloud_node.py

ROS 2 service node wrapping GraspNet-1Billion, fed by a point cloud supplied
directly in the request (e.g. a segmented per-object cloud from obb_3d's 3D
object mapping) rather than a YOLO mask over the live camera cloud.

Differences from grasp_from_mask_node.py:
  • Input is a sensor_msgs/PointCloud2 in the request — no camera, no mask, no
    depth unprojection, no organized-cloud assumption.
  • The cloud's header.frame_id is the source frame (obb_3d stamps it 'map').
    Points are fed to GraspNet AS-IS (no body→optical axis swap), then grasps
    are TF-transformed into the planning frame, exactly like the mask node.
  • No forward-axis range gate and no depth-nearness cluster weighting — those
    are camera-frame heuristics and meaningless for a world-frame mapped cloud.

Services:
  /grasp/from_cloud     walkie_perception/srv/GraspFromCloud
  /grasp_cloud/standby  std_srvs/srv/SetBool   (true=load GPU, false=unload)
  /grasp_cloud/status   std_srvs/srv/Trigger

Standby semantics:
  standby true  = model IS loaded in GPU (normal operating state)
  standby false = model unloaded from GPU (VRAM freed for other tasks)
"""
import gc
import os
import sys
import threading
import time
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

import tf2_ros
import tf2_geometry_msgs  # noqa: F401 — registers Pose transform support

from geometry_msgs.msg import Point, Pose
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import Marker, MarkerArray

from models.graspnet import GraspNet, pred_decode
from graspnetAPI.grasp import GraspGroup

from walkie_perception.srv import GraspFromCloud


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
    """Return v normalised, or None if it is (near) zero length."""
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-9 else None


class GraspFromCloudNode(Node):

    # Gripper marker geometry.
    MARKER_PALM_DEPTH_M   = 0.02  # palm sits 2 cm back from grasp centre
    MARKER_APPROACH_LEN_M = 0.06

    # Read with get_parameter() on every request so they stay changeable at
    # runtime via `ros2 param set`.
    _DYNAMIC_PARAMS = ("planning_frame", "approach_dist_m")

    def __init__(self):
        super().__init__("grasp_from_cloud")

        self._setup_parameters()
        self._init_runtime_state()
        self._check_gpu()

        self._load_model()
        self._set_state(NodeState.STANDBY)
        self.get_logger().info(
            f"Model ready (standby). VRAM: {self._vram_info()}")

        self._create_ros_interfaces()
        self.get_logger().info(
            "GraspFromCloud ready\n"
            "  source: PointCloud2 in the request (frame from cloud header)\n"
            "  /grasp/from_cloud    — request a grasp\n"
            "  /grasp_cloud/standby — true=load GPU, false=unload\n"
            "  /grasp_cloud/status  — check state and VRAM")

    # ── Setup ────────────────────────────────────────────────────────

    def _setup_parameters(self) -> None:
        params = [
            ("checkpoint_path",
             os.path.expanduser("~/graspnet-baseline/logs/log_rs/checkpoint-rs.tar")),
            ("num_point",    10000),
            ("num_view",     300),
            ("voxel_size_m", 0.005),
            ("min_points",   200),
            # A mapped per-object cloud is already segmented, so outlier removal
            # is light and clustering is off by default. Enable cluster_filter
            # if the supplied cloud still carries background/neighbour points.
            ("outlier_removal",      True),
            ("outlier_nb_neighbors", 20),
            ("outlier_std_ratio",    2.0),
            ("cluster_filter",       False),
            ("cluster_eps",          0.02),
            ("cluster_min_samples",  10),
            # Candidate pool kept after NMS for preference re-ranking; the final
            # response is trimmed to max_grasps from this pool.
            ("rerank_pool_size",     200),
            ("debug_cloud",  True),
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
            f"{torch.cuda.get_device_capability()[1]}")
        self.get_logger().info(f"VRAM before load: {self._vram_info()}")

    def _create_ros_interfaces(self) -> None:
        self.create_service(
            GraspFromCloud, "/grasp/from_cloud", self._handle_request)
        self.create_service(
            SetBool, "/grasp_cloud/standby", self._handle_standby)
        self.create_service(
            Trigger, "/grasp_cloud/status", self._handle_status)

        self.dbg_cloud_pub = self.create_publisher(
            PointCloud2, "/grasp/debug/cloud_input", 10)
        self.dbg_marker_pub = self.create_publisher(
            MarkerArray, "/grasp/debug/cloud_grasp_markers", 10)

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
            f"({free // 1024**2} MB free)")

    def _load_model(self) -> None:
        self.get_logger().info(f"Loading checkpoint: {self.checkpoint_path}")
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
            f"Checkpoint loaded in {load_ms:.0f} ms. VRAM: {self._vram_info()}")
        self._warmup()

    def _warmup(self) -> None:
        self.get_logger().info("Running warm-up inference...")
        dummy = torch.randn(1, self.num_point, 3).cuda()
        with torch.no_grad():
            pred_decode(self.net({"point_clouds": dummy}))
        torch.cuda.synchronize()
        self.get_logger().info(f"Warm-up done. VRAM: {self._vram_info()}")

    def _unload_model(self) -> int:
        """Free the model from GPU; returns MB of VRAM freed."""
        used_before = self._vram_used_mb()
        self.net = self.net.cpu()
        del self.net
        self.net = None
        gc.collect()
        torch.cuda.empty_cache()
        return used_before - self._vram_used_mb()

    # ── Cloud decoding ───────────────────────────────────────────────

    @staticmethod
    def _cloud_to_xyz(cloud_msg: PointCloud2) -> np.ndarray:
        """Any PointCloud2 (organized or not) → (N, 3) float32 XYZ, finite only.

        Reads the x/y/z float32 fields by their declared offsets, so it works
        regardless of point_step or extra fields (rgb, normals…)."""
        offsets = {f.name: f.offset for f in cloud_msg.fields
                   if f.name in ("x", "y", "z")}
        if not all(k in offsets for k in ("x", "y", "z")):
            raise PipelineAbort(
                f"cloud missing x/y/z fields (have: "
                f"{[f.name for f in cloud_msg.fields]})")

        n = cloud_msg.width * cloud_msg.height
        if n == 0:
            return np.empty((0, 3), dtype=np.float32)
        buf = np.frombuffer(cloud_msg.data, dtype=np.uint8).reshape(
            n, cloud_msg.point_step)

        def _col(name: str) -> np.ndarray:
            o = offsets[name]
            return buf[:, o:o + 4].copy().view(np.float32).reshape(n)

        xyz = np.stack([_col("x"), _col("y"), _col("z")], axis=1)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        return xyz.astype(np.float32)

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
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.data = pts.tobytes()
        return msg

    # ── Point filtering ──────────────────────────────────────────────

    def _filter_points(self, pts: np.ndarray) -> np.ndarray:
        """Voxel-downsample, then optional plain statistical outlier removal and
        largest-cluster DBSCAN. No depth weighting (world-frame cloud)."""
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        pts = np.asarray(
            pcd.voxel_down_sample(self.voxel_size_m).points).astype(np.float32)

        if self.outlier_removal and len(pts) > self.outlier_nb_neighbors:
            before = len(pts)
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pts)
            _, idx = pcd.remove_statistical_outlier(
                nb_neighbors=self.outlier_nb_neighbors,
                std_ratio=self.outlier_std_ratio)
            pts = pts[idx]
            self.get_logger().info(
                f"outlier removal: {before} → {len(pts)} pts "
                f"(removed {before - len(pts)})")

        if self.cluster_filter and len(pts) >= self.cluster_min_samples:
            pts = self._keep_largest_cluster(pts)
        return pts

    def _keep_largest_cluster(self, pts: np.ndarray) -> np.ndarray:
        """DBSCAN — keep the cluster with the most points (count only; the
        world-frame cloud has no camera to weight nearness by)."""
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        labels = np.array(pcd.cluster_dbscan(
            eps=self.cluster_eps,
            min_points=self.cluster_min_samples,
            print_progress=False))
        unique = set(labels) - {-1}
        if not unique:
            self.get_logger().warn(
                "cluster_filter: no clusters found — returning all points")
            return pts
        best = max(unique, key=lambda lbl: int((labels == lbl).sum()))
        result = pts[labels == best]
        self.get_logger().info(
            f"cluster_filter: {len(unique)} cluster(s), kept label={best} "
            f"({len(result)} pts), removed {len(pts) - len(result)} pts")
        return result

    def _sample_points(self, pts: np.ndarray) -> np.ndarray:
        """Random-sample (with replacement if short) to exactly num_point."""
        replace = len(pts) < self.num_point
        idx = np.random.choice(len(pts), self.num_point, replace=replace)
        return pts[idx]

    # ── Service: status ──────────────────────────────────────────────

    def _handle_status(self, _req, resp) -> Trigger.Response:
        state = self._get_state()
        resp.success = True
        resp.message = (
            f"state={state.value} | VRAM: {self._vram_info()} | "
            f"source=request cloud")
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
                f"Already in state={state.value}. VRAM: {self._vram_info()}")
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
                f"Model unloaded. Freed ~{freed_mb} MB. VRAM: {self._vram_info()}")
            self.get_logger().info(response.message)
        except Exception as exc:
            self._set_state(NodeState.STANDBY)
            response.success = False
            response.message = f"Unload failed: {exc}"
        return response

    # ── Service: from_cloud ──────────────────────────────────────────

    def _handle_request(
        self,
        request: GraspFromCloud.Request,
        response: GraspFromCloud.Response,
    ) -> GraspFromCloud.Response:

        t_start = time.perf_counter()

        state = self._get_state()
        if state == NodeState.UNLOADED:
            response.success = False
            response.message = (
                "Model is unloaded. Call: ros2 service call "
                "/grasp_cloud/standby std_srvs/srv/SetBool '{data: true}'")
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
        request: GraspFromCloud.Request,
        response: GraspFromCloud.Response,
        t_start: float,
    ) -> GraspFromCloud.Response:

        cloud_msg = request.cloud
        frame_id = cloud_msg.header.frame_id or "map"
        stamp = cloud_msg.header.stamp

        pts = self._cloud_to_xyz(cloud_msg)
        points_in = len(pts)
        if points_in == 0:
            raise PipelineAbort("Request cloud has no finite points")

        pts = self._filter_points(pts)
        if len(pts) < self.min_points:
            raise PipelineAbort(
                f"Insufficient points after filtering: {len(pts)} "
                f"(need {self.min_points})")

        pts = self._sample_points(pts)
        points_fed = len(pts)

        if self.debug_cloud:
            self.dbg_cloud_pub.publish(self._make_pc2(pts, frame_id, stamp))

        raw_grasps, inference_ms = self._run_graspnet(pts)
        grasps_raw = len(raw_grasps)

        gg = self._select_grasps(raw_grasps, request)
        if len(gg) == 0:
            raise PipelineAbort(
                f"No grasps above threshold {request.score_threshold} "
                f"(raw: {grasps_raw})")

        # Transform the candidate pool into the planning frame, re-rank it by
        # the request's preferences (approach/position/width/region), trim to
        # max_grasps, and fill the response. gg is reordered to match so the
        # debug markers reflect the returned ranking.
        planning_frame, gg = self._transform_and_rank(
            request, gg, pts, frame_id, stamp, response)
        self._publish_grasp_markers(gg, frame_id, stamp, response, planning_frame)

        total_ms = (time.perf_counter() - t_start) * 1e3
        response.success = True
        response.message = (
            f"OK | {len(gg)} grasps | top score {gg.scores[0]:.3f} | "
            f"top width {gg.widths[0]*100:.1f} cm")
        # Timing / point counts are logged here rather than returned in the srv.
        self.get_logger().info(
            f"{response.message} | src frame '{frame_id}' | "
            f"{points_in} pts in | {points_fed} fed | "
            f"raw grasps {grasps_raw} | infer {inference_ms:.0f} ms | "
            f"total {total_ms:.0f} ms")
        return response

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
        self, raw_grasps, request: GraspFromCloud.Request
    ) -> GraspGroup:
        """Sort + NMS + score-threshold, then keep a pool for re-ranking.

        Trimming to max_grasps happens after preference re-ranking, so this
        keeps up to rerank_pool_size candidates (sorted by GraspNet score)."""
        gg = GraspGroup(raw_grasps.cpu().numpy())
        gg = gg.sort_by_score()[:2000].nms()

        if request.score_threshold > 0:
            gg = gg[gg.scores >= request.score_threshold]
        if len(gg) == 0:
            return gg

        return gg[:max(1, int(self.rerank_pool_size))]

    def _lookup_planning_tf(self, frame_id: str, planning_frame: str, stamp):
        """TF frame_id → planning_frame: try the cloud stamp, fall back to the
        latest. Returns the TransformStamped or None on total failure."""
        for tf_stamp, label in ((stamp, "stamp"), (rclpy.time.Time(), "latest")):
            try:
                return self._tf_buffer.lookup_transform(
                    planning_frame, frame_id, tf_stamp,
                    timeout=Duration(seconds=0.1))
            except Exception as e:
                self.get_logger().warn(
                    f"TF {frame_id} → {planning_frame} ({label}) failed: {e}")
        return None

    def _transform_and_rank(
        self, request: GraspFromCloud.Request, gg: GraspGroup,
        pts: np.ndarray, frame_id: str, stamp,
        response: GraspFromCloud.Response,
    ) -> Tuple[str, GraspGroup]:
        """Transform the candidate pool into the planning frame, re-rank by the
        request's preferences, trim to max_grasps, and fill the response.

        Returns (planning_frame, gg_ranked). On TF failure poses_base is left
        empty and gg is returned trimmed by GraspNet order (for debug markers)."""
        planning_frame = self.get_parameter("planning_frame").value
        response.planning_frame = planning_frame
        response.poses_base.header.frame_id          = planning_frame
        response.poses_base.header.stamp             = stamp
        response.approach_poses_base.header.frame_id = planning_frame
        response.approach_poses_base.header.stamp    = stamp

        n_out = request.max_grasps if request.max_grasps > 0 else min(len(gg), 20)

        tf = self._lookup_planning_tf(frame_id, planning_frame, stamp)
        if tf is None:
            self.get_logger().warn(
                f"TF {frame_id} → {planning_frame} unavailable. "
                f"poses_base will be empty.")
            return planning_frame, gg[:n_out]

        # Object AABB in the planning frame.
        tf_rot = R.from_quat([
            tf.transform.rotation.x, tf.transform.rotation.y,
            tf.transform.rotation.z, tf.transform.rotation.w])
        tf_trans = np.array([
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z])
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

        # EE alignment: rotate −90° about Y in the planning frame so the gripper
        # approach points along the arm EE's local Z. Pre-multiply = world frame.
        ee_pitch = R.from_euler("y", -90.0, degrees=True)

        poses_plan, approach_axes = [], []
        for i in range(len(gg)):
            quat = R.from_matrix(gg.rotation_matrices[i]).as_quat()
            trans = gg.translations[i]
            cp = Pose()
            cp.position.x, cp.position.y, cp.position.z = (
                float(trans[0]), float(trans[1]), float(trans[2]))
            cp.orientation.x, cp.orientation.y, cp.orientation.z, cp.orientation.w = (
                float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))

            p = tf2_geometry_msgs.do_transform_pose(cp, tf)
            q_in = R.from_quat([p.orientation.x, p.orientation.y,
                                p.orientation.z, p.orientation.w])
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (
                float(v) for v in (ee_pitch * q_in).as_quat())
            rot = R.from_quat([p.orientation.x, p.orientation.y,
                               p.orientation.z, p.orientation.w]).as_matrix()
            poses_plan.append(p)
            approach_axes.append(rot[:, 0])   # col-0 = approach/travel direction

        order = self._rank_candidates(
            request, gg, poses_plan, approach_axes, z_min, z_max)[:n_out]

        # Fill the response in ranked order.
        del response.poses_base.poses[:]
        del response.approach_poses_base.poses[:]
        del response.scores[:]
        del response.widths[:]
        del response.height_below_grasp[:]
        del response.height_above_grasp[:]
        approach_dist = self.get_parameter("approach_dist_m").value

        for idx in order:
            p = poses_plan[idx]
            axis = approach_axes[idx]
            response.poses_base.poses.append(p)
            response.scores.append(float(gg.scores[idx]))
            response.widths.append(float(gg.widths[idx]))
            response.height_below_grasp.append(float(p.position.z - z_min))
            response.height_above_grasp.append(float(z_max - p.position.z))

            ap = Pose()
            ap.position.x  = p.position.x - float(axis[0]) * approach_dist
            ap.position.y  = p.position.y - float(axis[1]) * approach_dist
            ap.position.z  = p.position.z - float(axis[2]) * approach_dist
            ap.orientation = p.orientation
            response.approach_poses_base.poses.append(ap)

        self.get_logger().info(
            f"Transformed + ranked {len(order)} pose(s) → {planning_frame}")
        return planning_frame, gg[list(order)]

    def _rank_candidates(
        self, request: GraspFromCloud.Request, gg: GraspGroup,
        poses_plan, approach_axes, z_min: float, z_max: float,
    ):
        """Return candidate indices ordered best-first by a weighted blend of
        GraspNet quality and the request's preferences (all normalised to
        [0,1]). If every weight is 0, keep GraspNet's own order."""
        w_q  = float(request.quality_weight)
        w_a  = float(request.approach_weight)
        w_p  = float(request.position_weight)
        w_w  = float(request.width_weight)
        w_r  = float(request.region_weight)
        n = len(poses_plan)

        if (w_q + w_a + w_p + w_w + w_r) <= 0.0:
            return list(range(n))   # GraspNet order (already score-sorted)

        gn = np.array([float(gg.scores[i]) for i in range(n)], dtype=np.float64)
        score = w_q * gn

        pref_a = _unit(np.array([request.preferred_approach.x,
                                 request.preferred_approach.y,
                                 request.preferred_approach.z]))
        if w_a > 0 and pref_a is not None:
            align = np.array([float(np.dot(_unit(a) if _unit(a) is not None
                                           else a, pref_a))
                              for a in approach_axes])
            score += w_a * ((align + 1.0) / 2.0)        # [0,1]
        elif w_a > 0:
            self.get_logger().warn(
                "approach_weight>0 but preferred_approach is zero — ignoring")

        if w_p > 0:
            tgt = np.array([request.preferred_position.x,
                            request.preferred_position.y,
                            request.preferred_position.z])
            fall = request.position_falloff_m if request.position_falloff_m > 0 else 0.05
            d = np.array([np.linalg.norm(
                np.array([p.position.x, p.position.y, p.position.z]) - tgt)
                for p in poses_plan])
            score += w_p * np.exp(-d / fall)            # [0,1]

        if w_w > 0 and request.preferred_width > 0:
            pw = float(request.preferred_width)
            wid = np.array([float(gg.widths[i]) for i in range(n)])
            score += w_w * np.clip(1.0 - np.abs(wid - pw) / pw, 0.0, 1.0)
        elif w_w > 0:
            self.get_logger().warn(
                "width_weight>0 but preferred_width<=0 — ignoring")

        if w_r > 0 and (z_max - z_min) > 1e-6:
            rf = float(np.clip(request.region_frac, 0.0, 1.0))
            frac = np.array([(p.position.z - z_min) / (z_max - z_min)
                             for p in poses_plan])
            score += w_r * np.clip(1.0 - np.abs(frac - rf), 0.0, 1.0)

        return list(np.argsort(-score))

    # ── Visualisation markers ────────────────────────────────────────

    def _publish_grasp_markers(
        self, gg: GraspGroup, frame_id: str, stamp,
        response: GraspFromCloud.Response, planning_frame: str,
    ) -> None:
        """Gripper line markers (cloud frame) + AABB cube (planning frame)."""
        ma = MarkerArray()
        lt = Duration(seconds=0).to_msg()

        clr = Marker()
        clr.action = Marker.DELETEALL
        clr.ns     = "gripper"
        ma.markers.append(clr)

        for i in range(min(10, len(gg))):
            ma.markers.append(self._gripper_marker(gg, i, frame_id, stamp, lt))

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
            bbox_m.color.a         = 0.25
            ma.markers.append(bbox_m)

        self.dbg_marker_pub.publish(ma)

    def _gripper_marker(
        self, gg: GraspGroup, i: int, frame_id: str, stamp, lifetime,
    ) -> Marker:
        rot   = gg.rotation_matrices[i]
        trans = gg.translations[i]
        width = float(gg.widths[i])
        sc    = gg.scores[i]

        # GraspNet frame: col-0 = approach (x), col-1 = spread/width (y).
        approach = rot[:, 0]
        spread   = rot[:, 1]

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
    node = GraspFromCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
