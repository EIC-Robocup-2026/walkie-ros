#!/bin/sh
"exec" "${PERCEPTION_VENV:-$HOME/perception-venv/bin/python3}" "$0" "$@"
"""
test_grasp_compare.py — Call GraspNet (/grasp/from_mask) and the PCA
heuristic (/grasp/pca_heuristic) on the same object and print a side-by-side
comparison table.

Usage
─────
  ros2 run walkie_perception test_grasp_compare            # first detection
  ros2 run walkie_perception test_grasp_compare bottle     # by YOLO class
  ros2 run walkie_perception test_grasp_compare fork \\
      --masks /yolo/masks --dets /yolo/tracked_detections_2d

Both nodes publish to separate marker topics, so their grasps can be viewed
simultaneously in RViz:
  /grasp/debug/grasp_markers   (GraspNet,  green/yellow/orange)
  /grasp/debug/pca_markers     (PCA,       blue/cyan/magenta)
"""
import argparse
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

from walkie_perception.srv import GraspFromMask, PCAHeuristicGrasp


def _approach_axis(pose) -> np.ndarray:
    """Grasp approach = col-0 of the rotation matrix from the quaternion."""
    q = pose.orientation
    y2, z2 = 2 * q.y * q.y, 2 * q.z * q.z
    xy, xz = 2 * q.x * q.y, 2 * q.x * q.z
    wy, wz = 2 * q.w * q.y, 2 * q.w * q.z
    return np.array([1 - y2 - z2, xy + wz, xz - wy])   # first column of R


class GraspCompareNode(Node):
    def __init__(self, args):
        super().__init__("test_grasp_compare")
        self._args = args
        self._target_cls = args.object_class
        self._mask_msg = None
        self._found_tid = None
        self._bbox = None
        self._found_cls = None

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(Image, args.masks, self._mask_cb, qos)
        self.create_subscription(
            Detection2DArray, args.dets, self._det_cb, qos)

        self._gn_cli  = self.create_client(GraspFromMask, "/grasp/from_mask")
        self._pca_cli = self.create_client(
            PCAHeuristicGrasp, "/grasp/pca_heuristic")

    # ── subscribers ──────────────────────────────────────────────────

    def _mask_cb(self, msg: Image):
        self._mask_msg = msg

    def _det_cb(self, msg: Detection2DArray):
        if self._found_tid is not None:
            return
        for d in msg.detections:
            cls = d.results[0].hypothesis.class_id if d.results else ""
            tid = int(d.id) if d.id.lstrip("-").isdigit() else -1
            if tid <= 0:
                continue
            if self._target_cls is None or cls == self._target_cls:
                self._found_tid = tid
                self._bbox = d.bbox
                self._found_cls = cls
                return

    # ── wait helper ──────────────────────────────────────────────────

    def _wait(self, description, check_fn, timeout=20.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if check_fn():
                return True
        self.get_logger().error(f"Timeout waiting for {description}")
        return False

    def _call(self, client, req, name):
        future = client.call_async(req)
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        resp = future.result()
        if resp is None:
            print(f"  {name}: call failed (no response)")
        elif not resp.success:
            print(f"  {name}: {resp.message}")
        return resp

    # ── main ─────────────────────────────────────────────────────────

    def run(self):
        print("Waiting for /grasp/from_mask …")
        if not self._gn_cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("/grasp/from_mask not available")
            return
        print("Waiting for /grasp/pca_heuristic …")
        if not self._pca_cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("/grasp/pca_heuristic not available")
            return

        print("Waiting for mask frame …")
        if not self._wait("mask frame", lambda: self._mask_msg is not None):
            return

        target = f"class={self._target_cls}" if self._target_cls else "first object"
        print(f"Waiting for detection ({target}) …")
        if not self._wait(f"detection ({target})",
                          lambda: self._found_tid is not None):
            return

        tid, bbox, cls = self._found_tid, self._bbox, self._found_cls
        print(f"Target tracker_id={tid} class='{cls}' "
              f"bbox=({bbox.center.position.x:.0f},{bbox.center.position.y:.0f}) "
              f"{bbox.size_x:.0f}×{bbox.size_y:.0f}\n")

        # ── GraspNet ──────────────────────────────────────────────
        gn_req = GraspFromMask.Request()
        gn_req.mask            = self._mask_msg
        gn_req.tracker_id      = tid
        gn_req.bbox            = bbox
        gn_req.num_frames      = 0
        gn_req.score_threshold = 0.0
        gn_req.max_grasps      = 5
        print("Calling /grasp/from_mask …")
        gn = self._call(self._gn_cli, gn_req, "GraspNet")

        # ── PCA heuristic ─────────────────────────────────────────
        pca_req = PCAHeuristicGrasp.Request()
        pca_req.mask              = self._mask_msg
        pca_req.tracker_id        = tid
        pca_req.bbox              = bbox
        pca_req.num_frames        = 0
        pca_req.object_class      = cls or ""
        pca_req.max_gripper_width = 0.08
        print("Calling /grasp/pca_heuristic …")
        pca = self._call(self._pca_cli, pca_req, "PCA")

        self._print_table(gn, pca)

        print("\nView in RViz:")
        print("  /grasp/debug/grasp_markers (GraspNet, green/yellow/orange)")
        print("  /grasp/debug/pca_markers   (PCA, blue/cyan/magenta)")

    # ── table ────────────────────────────────────────────────────────

    @staticmethod
    def _fmt_vec(v, prec=3):
        return "(" + ", ".join(f"{x:+.{prec}f}" for x in v) + ")"

    def _print_table(self, gn, pca):
        gn_ok  = gn is not None and gn.success and len(gn.poses_base.poses) > 0
        pca_ok = pca is not None and pca.success and len(pca.poses_base.poses) > 0

        def gn_val(fn, default="—"):
            try:
                return fn() if gn_ok else default
            except Exception:
                return default

        def pca_val(fn, default="—"):
            try:
                return fn() if pca_ok else default
            except Exception:
                return default

        gn_pose  = gn.poses_base.poses[0] if gn_ok else None
        pca_pose = pca.poses_base.poses[0] if pca_ok else None

        rows = [
            # Position + approach axis are read from poses_base (planning frame)
            # — the service no longer returns camera-frame poses, and timing /
            # point counts are logged to each node's terminal, not returned.
            ("planning_frame",
             gn_val(lambda: gn.planning_frame),
             pca_val(lambda: pca.planning_frame)),
            ("grasp pos (x,y,z)",
             gn_val(lambda: self._fmt_vec([gn_pose.position.x, gn_pose.position.y,
                                           gn_pose.position.z])),
             pca_val(lambda: self._fmt_vec([pca_pose.position.x, pca_pose.position.y,
                                            pca_pose.position.z]))),
            ("approach axis",
             gn_val(lambda: self._fmt_vec(_approach_axis(gn_pose), 2)),
             pca_val(lambda: self._fmt_vec(_approach_axis(pca_pose), 2))),
            ("approach pos (x,y,z)",
             gn_val(lambda: self._fmt_vec([gn.approach_poses_base.poses[0].position.x,
                                           gn.approach_poses_base.poses[0].position.y,
                                           gn.approach_poses_base.poses[0].position.z])),
             pca_val(lambda: self._fmt_vec([pca.approach_poses_base.poses[0].position.x,
                                            pca.approach_poses_base.poses[0].position.y,
                                            pca.approach_poses_base.poses[0].position.z]))),
            ("width (cm)",
             gn_val(lambda: f"{gn.widths[0]*100:.1f}"),
             pca_val(lambda: f"{pca.widths[0]*100:.1f}")),
            ("score",
             gn_val(lambda: f"{gn.scores[0]:.3f}"),
             pca_val(lambda: f"{pca.scores[0]:.3f}")),
            ("object_size (xyz)",
             gn_val(lambda: self._fmt_vec([gn.object_size.x, gn.object_size.y,
                                           gn.object_size.z])),
             pca_val(lambda: self._fmt_vec([pca.object_size.x, pca.object_size.y,
                                            pca.object_size.z]))),
            ("detected_shape",
             "—",
             pca_val(lambda: pca.detected_shape)),
            ("eigvals",
             "—",
             pca_val(lambda: self._fmt_vec(list(pca.pca_eigenvalues), 4))),
        ]

        c0, c1, c2 = 20, 26, 26
        top = f"┌{'─'*(c0+2)}┬{'─'*(c1+2)}┬{'─'*(c2+2)}┐"
        sep = f"├{'─'*(c0+2)}┼{'─'*(c1+2)}┼{'─'*(c2+2)}┤"
        bot = f"└{'─'*(c0+2)}┴{'─'*(c1+2)}┴{'─'*(c2+2)}┘"
        print()
        print(top)
        print(f"│ {'':<{c0}} │ {'GraspNet (top1)':<{c1}} │ {'PCA Heuristic':<{c2}} │")
        print(sep)
        for label, gv, pv in rows:
            print(f"│ {label:<{c0}} │ {str(gv):<{c1}} │ {str(pv):<{c2}} │")
        print(bot)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("object_class", nargs="?", default=None,
                        help="YOLO class name to target (default: first detection)")
    parser.add_argument("--masks", default="/yolo/masks")
    parser.add_argument("--dets", default="/yolo/tracked_detections_2d")
    args = parser.parse_args(rclpy.utilities.remove_ros_args()[1:])

    rclpy.init()
    node = GraspCompareNode(args)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
