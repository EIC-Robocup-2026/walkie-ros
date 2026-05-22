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
import threading
import time
from collections import deque
from enum import Enum
from typing import Optional

import cv2
import numpy as np
import open3d as o3d
import torch
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Point, Pose, PoseArray
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import Marker, MarkerArray

from models.graspnet import GraspNet, pred_decode
from graspnetAPI import GraspGroup

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
        self.declare_parameter("voxel_size_m", 0.0001)
        self.declare_parameter("cache_size",   10)
        self.declare_parameter("min_points",   300)
        self.declare_parameter("debug_cloud",  True)
        self.declare_parameter("depth_topic",
            "/zed_head/zed_node/depth/depth_registered")
        self.declare_parameter("info_topic",
            "/zed_head/zed_node/depth/camera_info")

        p = self.get_parameter
        self.checkpoint_path = os.path.expanduser(p("checkpoint_path").value)
        self.num_point    = p("num_point").value
        self.num_view     = p("num_view").value
        self.voxel_size_m = p("voxel_size_m").value
        self.cache_size   = p("cache_size").value
        self.min_points   = p("min_points").value
        self.debug_cloud  = p("debug_cloud").value
        self.depth_topic  = p("depth_topic").value
        self.info_topic   = p("info_topic").value

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

    def _unproject(
        self,
        depth_msg: Image,
        mask: np.ndarray,
    ) -> np.ndarray:
        """Unproject masked depth pixels to XYZ using pinhole model."""
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

        # ── Parse mask ────────────────────────────────────────────
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
                f"tracker_id={request.tracker_id} — using bbox fallback"
            )
            H, W = mask_img.shape
            cx = int(request.bbox.center.position.x)
            cy = int(request.bbox.center.position.y)
            hw = max(1, int(request.bbox.size_x / 2))
            hh = max(1, int(request.bbox.size_y / 2))
            object_mask = np.zeros((H, W), dtype=bool)
            object_mask[
                max(0, cy - hh) : min(H, cy + hh),
                max(0, cx - hw) : min(W, cx + hw),
            ] = True

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

        # ── Voxel downsample ──────────────────────────────────────
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        pcd = pcd.voxel_down_sample(self.voxel_size_m)
        pts = np.asarray(pcd.points).astype(np.float32)

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
        gg = gg.nms().sort_by_score()

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

        # ── Publish gripper visualisation markers ─────────────────
        ma  = MarkerArray()
        lt  = Duration(seconds=5).to_msg()

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

            # GraspNet frame: col-0 = approach, col-2 = finger-spread axis
            approach = rot[:, 0]
            spread   = rot[:, 2]

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
