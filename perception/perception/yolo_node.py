#!/usr/bin/env python3

import os
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
    Pose2D,
    Point2D,
)


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        # ── Parameters ────────────────────────────────────────────────────────
        from ament_index_python.packages import get_package_share_directory
        default_model = os.path.join(
            get_package_share_directory('perception'), 'models', 'yolo11l-seg.pt')
        self.declare_parameter('model_path', default_model)
        self.declare_parameter('image_topic', '/zed_head/zed_nodergb/color/rect/image')
        self.declare_parameter('confidence_threshold', 0.3)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('tracker_config', 'bytetrack.yaml')
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('class_filter', [])  # empty = all classes
        self.declare_parameter('device', 'cuda')      # 'cpu', 'cuda', 'cuda:0'
        self.declare_parameter('imgsz', 640)         # YOLO inference resolution

        p = self.get_parameter
        self.model_path     = p('model_path').value
        self.image_topic    = p('image_topic').value
        self.conf_thr       = p('confidence_threshold').value
        self.iou_thr        = p('iou_threshold').value
        self.tracker_cfg    = p('tracker_config').value
        self.publish_debug  = p('publish_debug_image').value
        self.class_filter   = p('class_filter').value
        self.device         = p('device').value
        self.imgsz          = p('imgsz').value

        # ── Lazy import ultralytics (heavy) ──────────────────────────────────
        # Done here rather than at module top so the import error is reported
        # AFTER ROS init, with a clear message about what to install.
        try:
            from ultralytics import YOLO
        except ImportError:
            self.get_logger().fatal(
                "ultralytics not installed. Run: pip install ultralytics")
            raise

        self.get_logger().info(f'Loading YOLO model: {self.model_path}')
        self.model = YOLO(self.model_path)
        self.get_logger().info(
            f'Model loaded. Classes: {len(self.model.names)} · device={self.device}')

        # ── Subscriptions / publishers ────────────────────────────────────────
        # BEST_EFFORT to drop frames if YOLO falls behind, never queue.
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=2)
        self.create_subscription(Image, self.image_topic,
                                 self._image_cb, qos)

        self.det_pub = self.create_publisher(
            Detection2DArray, '/yolo/tracked_detections_2d', 10)

        if self.publish_debug:
            self.dbg_pub = self.create_publisher(
                Image, '/yolo/debug_image', 5)

        # ── Mask publisher ────────────────────────────────────────────────────
        self.mask_pub = self.create_publisher(Image, '/yolo/masks', 10)

        # ── Depth stamp cache (for sync alignment with OBB3D) ─────────────────
        self._latest_depth_stamp = None
        self.create_subscription(
            Image,
            '/zed_head/zed_nodedepth/depth_registered',
            self._depth_stamp_cb,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=2),
        )

        # ── Stats ─────────────────────────────────────────────────────────────
        self._frame_count = 0
        self._inference_times = []
        self.create_timer(2.0, self._log_stats)

        self._bridge = CvBridge()
        self.get_logger().info(f'YOLO node ready. Listening on {self.image_topic}')

    def _depth_stamp_cb(self, msg: Image) -> None:
        """Cache latest depth frame stamp for sync alignment with OBB3D."""
        self._latest_depth_stamp = msg.header.stamp

    def _image_cb(self, msg: Image) -> None:
        """Run YOLO + ByteTrack on the incoming image."""
        try:
            img = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'Image decode failed: {e}',
                                   throttle_duration_sec=5.0)
            return

        # ── Run tracking inference ────────────────────────────────────────────
        # persist=True keeps the tracker state across calls (essential for IDs).
        # verbose=False suppresses ultralytics' per-frame stdout spam.
        t0 = time.perf_counter()
        try:
            results = self.model.track(
                img,
                persist=True,
                conf=self.conf_thr,
                iou=self.iou_thr,
                tracker=self.tracker_cfg,
                imgsz=self.imgsz,
                device=self.device,
                verbose=False,
            )
        except Exception as e:
            self.get_logger().error(f'YOLO inference failed: {e}',
                                    throttle_duration_sec=2.0)
            return
        inference_ms = (time.perf_counter() - t0) * 1e3
        self._inference_times.append(inference_ms)
        self._frame_count += 1

        # ── Build Detection2DArray ────────────────────────────────────────────
        det_msg = Detection2DArray()
        det_msg.header = msg.header
        # Use depth stamp so OBB3D synchronizer matches perfectly.
        if self._latest_depth_stamp is not None:
            det_msg.header.stamp = self._latest_depth_stamp

        if not results or len(results) == 0:
            self.det_pub.publish(det_msg)
            return

        result = results[0]
        if result.boxes is None or len(result.boxes) == 0:
            self.det_pub.publish(det_msg)
            return

        # Boxes come back as a Boxes object with xyxy, conf, cls, id attributes.
        # Each is a torch tensor; .cpu().numpy() gives us numpy arrays.
        xyxy = result.boxes.xyxy.cpu().numpy()       # (N, 4) [x1,y1,x2,y2]
        conf = result.boxes.conf.cpu().numpy()       # (N,)
        cls  = result.boxes.cls.cpu().numpy().astype(int)  # (N,)
        # Track IDs only exist when persist=True is used. Some detections may
        # not have an ID yet (just appeared, not yet tracked) — they show as None.
        ids  = (result.boxes.id.cpu().numpy().astype(int)
                if result.boxes.id is not None
                else np.full(len(xyxy), -1))

        for i in range(len(xyxy)):
            class_id = int(cls[i])
            class_name = self.model.names.get(class_id, str(class_id))

            # Optional class filter (e.g. only ['cup', 'bottle'])
            if self.class_filter and class_name not in self.class_filter:
                continue

            x1, y1, x2, y2 = xyxy[i]
            cx = float((x1 + x2) / 2.0)
            cy = float((y1 + y2) / 2.0)
            w  = float(x2 - x1)
            h  = float(y2 - y1)

            det = Detection2D()
            det.header = msg.header
            det.bbox = BoundingBox2D(
                center=Pose2D(position=Point2D(x=cx, y=cy)),
                size_x=w,
                size_y=h,
            )
            # Tracker ID lives on Detection2D.id in Jazzy's vision_msgs
            det.id = str(int(ids[i])) if ids[i] >= 0 else f'untracked_{i}'
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = class_name
            hyp.hypothesis.score    = float(conf[i])
            det.results.append(hyp)
            det_msg.detections.append(det)

        self.det_pub.publish(det_msg)

        # ── Build and publish segmentation label image ─────────────────────────
        # Only published when the seg model returns masks (yolo11n-seg.pt etc.)
        result = results[0] if results else None
        if result is not None and result.masks is not None and len(result.masks) > 0:
            label_img = np.zeros((msg.height, msg.width), dtype=np.uint16)
            for i in range(len(xyxy)):
                if ids[i] < 0:
                    continue
                tid = int(ids[i])
                if tid == 0:
                    continue  # 0 is background value — skip to avoid collision
                try:
                    poly = result.masks.xy[i]
                    if len(poly) < 3:
                        continue
                    pts_poly = np.array(poly, dtype=np.int32).reshape(-1, 1, 2)
                    cv2.fillPoly(label_img, [pts_poly], color=tid)
                except (IndexError, Exception):
                    continue
            mask_msg = Image()
            mask_msg.header = det_msg.header
            mask_msg.height = msg.height
            mask_msg.width = msg.width
            mask_msg.encoding = '16UC1'
            mask_msg.is_bigendian = False
            mask_msg.step = msg.width * 2
            mask_msg.data = label_img.tobytes()
            self.mask_pub.publish(mask_msg)

        # ── Optional debug image with overlays ────────────────────────────────
        if self.publish_debug:
            try:
                annotated = result.plot()  # ultralytics renders boxes + IDs
                dbg = Image()
                dbg.header   = msg.header
                dbg.height   = annotated.shape[0]
                dbg.width    = annotated.shape[1]
                dbg.encoding = 'bgr8'
                dbg.step     = annotated.shape[1] * 3
                dbg.data     = annotated.tobytes()
                self.dbg_pub.publish(dbg)
            except Exception as e:
                self.get_logger().debug(f'Debug image failed: {e}')

    def _log_stats(self) -> None:
        """Print rolling YOLO performance stats."""
        if not self._inference_times:
            return
        times = np.array(self._inference_times[-60:])  # last 60 frames
        self.get_logger().info(
            f'YOLO  fps={len(times)/2:.1f}  '
            f'mean={times.mean():.1f} ms  '
            f'p95={np.percentile(times, 95):.1f} ms  '
            f'frames={self._frame_count}')


def main():
    rclpy.init()
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
