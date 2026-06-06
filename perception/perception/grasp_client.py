#!/bin/sh
"exec" "${PERCEPTION_VENV:-$HOME/perception-venv/bin/python3}" "$0" "$@"
"""
grasp_client.py — SDK-friendly ROS 2 client for /grasp/from_mask.

Designed for callers where YOLO runs outside ROS.  The caller provides:
  • mask   — numpy H×W array (bool or uint8), True/1 = object pixels
  • xywh   — (cx, cy, w, h) in pixels (YOLO format, pixel coords)

The client converts these to the service wire format and calls /grasp/from_mask.

Importable usage
────────────────
  import rclpy
  from walkie_perception.grasp_client import GraspClient

  rclpy.init()
  client = GraspClient()
  result = client.request_grasp(mask_array, xywh=(cx, cy, w, h))
  if result.success:
      pose = result.poses_base.poses[0]

CLI usage (quick test)
──────────────────────
  ros2 run walkie_perception grasp_client \\
    --mask /tmp/mask.npy --xywh 320 240 200 200
"""

import argparse
import sys

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D

from walkie_perception.srv import GraspFromMask


def _mask_to_image(mask: np.ndarray) -> Image:
    """Convert a binary H×W numpy mask to a 16UC1 sensor_msgs/Image.

    Non-zero pixels become value 1 (matched by tracker_id=1 in the request).
    """
    arr = np.where(mask.astype(bool), np.uint16(1), np.uint16(0))
    msg = Image()
    msg.height = arr.shape[0]
    msg.width = arr.shape[1]
    msg.encoding = "16UC1"
    msg.step = arr.shape[1] * 2
    msg.data = arr.tobytes()
    return msg


def _xywh_to_bbox(xywh) -> BoundingBox2D:
    """Convert (cx, cy, w, h) pixel coords to vision_msgs/BoundingBox2D."""
    cx, cy, w, h = (float(v) for v in xywh)
    bbox = BoundingBox2D()
    bbox.center.position.x = cx
    bbox.center.position.y = cy
    bbox.size_x = w
    bbox.size_y = h
    return bbox


class GraspClient(Node):
    """Thin ROS 2 client that wraps /grasp/from_mask for SDK callers.

    The caller supplies a raw numpy mask and a YOLO-style xywh bbox.
    tracker_id is fixed at 1 — the mask is encoded with that value internally.
    """

    def __init__(self, node_name: str = "grasp_client"):
        super().__init__(node_name)
        self._cli = self.create_client(GraspFromMask, "/grasp/from_mask")

    def wait_for_service(self, timeout_sec: float = 5.0) -> bool:
        return self._cli.wait_for_service(timeout_sec=timeout_sec)

    def request_grasp(
        self,
        mask: np.ndarray,
        xywh,
        score_threshold: float = 0.0,
        max_grasps: int = 5,
        num_frames: int = 0,
    ) -> GraspFromMask.Response:
        """Call /grasp/from_mask synchronously and return the response.

        Args:
            mask: H×W numpy array (bool or uint8). True/1 = object pixels.
            xywh: (cx, cy, w, h) in pixels — fallback if mask has < 10 pixels.
            score_threshold: minimum grasp quality score (0 = no filter).
            max_grasps: cap on returned poses (0 = up to 20).
            num_frames: depth frames to merge (0 = adaptive).

        Returns:
            GraspFromMask.Response — check .success and .poses_base.poses[0].
        """
        req = GraspFromMask.Request()
        req.mask = _mask_to_image(mask)
        req.tracker_id = 1
        req.bbox = _xywh_to_bbox(xywh)
        req.score_threshold = float(score_threshold)
        req.max_grasps = int(max_grasps)
        req.num_frames = int(num_frames)

        future = self._cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main():
    parser = argparse.ArgumentParser(description="grasp_client CLI test")
    parser.add_argument("--mask", required=True,
                        help="Path to .npy binary mask (H×W bool/uint8)")
    parser.add_argument("--xywh", nargs=4, type=float, required=True,
                        metavar=("CX", "CY", "W", "H"),
                        help="Bounding box centre and size in pixels")
    parser.add_argument("--score", type=float, default=0.0,
                        help="Minimum grasp score (default 0)")
    parser.add_argument("--grasps", type=int, default=5,
                        help="Max grasps to return (default 5)")
    parser.add_argument("--frames", type=int, default=0,
                        help="Depth frames to merge (default 0 = adaptive)")
    args = parser.parse_args()

    mask = np.load(args.mask)

    rclpy.init()
    client = GraspClient()

    if not client.wait_for_service(timeout_sec=5.0):
        client.get_logger().error("/grasp/from_mask not available")
        sys.exit(1)

    result = client.request_grasp(
        mask=mask,
        xywh=args.xywh,
        score_threshold=args.score,
        max_grasps=args.grasps,
        num_frames=args.frames,
    )

    if not result.success:
        print(f"[FAIL] {result.message}")
        sys.exit(1)

    print(f"[OK] {result.message}")
    print(f"  grasps returned : {result.grasps_returned}")
    print(f"  planning_frame  : {result.planning_frame}")
    for i, (pose, score, width) in enumerate(
        zip(result.poses_base.poses, result.scores, result.widths)
    ):
        p = pose.position
        print(f"  [{i}] score={score:.3f}  width={width:.3f}m  "
              f"pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f})")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
