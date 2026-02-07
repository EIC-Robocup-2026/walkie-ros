#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
import message_filters
from cv_bridge import CvBridge
import numpy as np
import cv2
from ultralytics import YOLO
import os
from ament_index_python.packages import get_package_share_directory
import tf2_ros
import tf2_geometry_msgs

class ObDetectionNode(Node):
    def __init__(self):
        super().__init__('ob_detection')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.is_active = False
        self.conf_threshold = 0.55
        self.target_classes = [0, 39, 41, 45] # Person, Bottle, Cup, Bowl
        
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        try:
            pkg_share = get_package_share_directory('perception')
            model_path = os.path.join(pkg_share, 'models', 'yolov8n.onnx')
            self.get_logger().info(f"Loading Model: {model_path}...")
            self.model = YOLO(model_path, task='detect')
        except Exception as e:
            self.get_logger().error(f"FATAL: Could not load model. Error: {e}")
            return

        self.cv_bridge = CvBridge()
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        rgb_topic = '/zed/zed_node/rgb/color/rect/image'
        depth_topic = '/zed/zed_node/depth/depth_registered'
        info_topic = '/zed/zed_node/rgb/color/rect/camera_info'
        
        self.create_subscription(CameraInfo, info_topic, self.info_callback, qos)

        self.img_sub = message_filters.Subscriber(self, Image, rgb_topic, qos_profile=qos)
        self.depth_sub = message_filters.Subscriber(self, Image, depth_topic, qos_profile=qos)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.img_sub, self.depth_sub], queue_size=10, slop=0.2)
        self.ts.registerCallback(self.sync_callback)

        self.det_pub = self.create_publisher(Detection3DArray, 'ob_detection/detections', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'ob_detection/markers', 10)
        self.debug_pub = self.create_publisher(Image, 'ob_detection/debug_image', 10)
        
        self.create_service(SetBool, 'ob_detection/toggle', self.toggle_callback)
        self.get_logger().info("Node Initialized.")

    def info_callback(self, msg):
        if self.fx is None:
            self.fx = msg.k[0]; self.fy = msg.k[4]; self.cx = msg.k[2]; self.cy = msg.k[5]
            self.get_logger().info(f"Camera Info Loaded: fx={self.fx:.2f}")

    def toggle_callback(self, req, res):
        self.is_active = req.data
        status = "STARTED" if self.is_active else "PAUSED"
        self.get_logger().info(f"System {status}")
        res.success = True; res.message = status
        return res

    def sync_callback(self, img_msg, depth_msg):
        if not self.is_active or self.fx is None: return

        try:
            cv_img = self.cv_bridge.imgmsg_to_cv2(img_msg, 'bgr8')
            cv_depth = self.cv_bridge.imgmsg_to_cv2(depth_msg, '32FC1')

            results = self.model(cv_img, verbose=False)
            
            det_array = Detection3DArray()
            det_array.header.frame_id = 'map' 
            det_array.header.stamp = img_msg.header.stamp
            
            marker_array = MarkerArray()
            marker_array.markers.append(Marker(action=Marker.DELETEALL))

            count = 0
            for r in results:
                for box in r.boxes:
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])

                    if cls_id not in self.target_classes or conf < self.conf_threshold:
                        continue

                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    u = int((x1 + x2) / 2)
                    v = int((y1 + y2) / 2)

                    if 0 <= u < cv_depth.shape[1] and 0 <= v < cv_depth.shape[0]:
                        z = float(cv_depth[v, u])
                        
                        if not np.isnan(z) and not np.isinf(z) and z > 0.2:
                            
                            x_cam = (u - self.cx) * z / self.fx
                            y_cam = (v - self.cy) * z / self.fy

                            pt_cam = PointStamped()
                            pt_cam.header.frame_id = img_msg.header.frame_id
                            
                            pt_cam.header.stamp = rclpy.time.Time(seconds=0).to_msg()
                            
                            pt_cam.point.x = x_cam
                            pt_cam.point.y = y_cam
                            pt_cam.point.z = z

                            try:
                                pt_map = self.tf_buffer.transform(
                                    pt_cam, 'map', timeout=rclpy.duration.Duration(seconds=0.05))
                                
                                det = Detection3D()
                                det.header = det_array.header
                                hyp = ObjectHypothesisWithPose()
                                hyp.hypothesis.class_id = self.model.names[cls_id]
                                hyp.hypothesis.score = conf
                                hyp.pose.pose.position = pt_map.point
                                hyp.pose.pose.orientation.w = 1.0

                                det.results.append(hyp)
                                det.bbox.center.position = pt_map.point
                                det.bbox.size.x = 0.2; det.bbox.size.y = 0.2; det.bbox.size.z = 0.2
                                det_array.detections.append(det)

                                m = Marker()
                                m.header = det_array.header
                                m.ns = "objects"; m.id = count; m.type = Marker.SPHERE
                                m.action = Marker.ADD
                                m.pose.position = pt_map.point
                                m.scale.x = 0.2; m.scale.y = 0.2; m.scale.z = 0.2
                                m.color.a = 1.0; m.color.g = 1.0 
                                marker_array.markers.append(m)
                                count += 1
                                
                            except Exception:
                                pass 

            if count > 0:
                self.det_pub.publish(det_array)
                self.marker_pub.publish(marker_array)

            annotated_frame = results[0].plot()
            self.debug_pub.publish(self.cv_bridge.cv2_to_imgmsg(annotated_frame, 'bgr8'))

        except Exception as e:
            self.get_logger().error(f"Processing Error: {e}")

def main():
    rclpy.init()
    node = ObDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()