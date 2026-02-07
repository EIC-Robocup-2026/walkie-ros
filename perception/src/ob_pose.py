#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection3DArray, Detection3D, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped

import tf2_ros
import tf2_geometry_msgs 
from cv_bridge import CvBridge
import numpy as np

class ObPoseNode(Node):
    def __init__(self):
        super().__init__('ob_detection')

        self.is_active = False 
        self.fx = self.fy = self.cx = self.cy = None
        self.latest_depth = None
        self.latest_header = None
        self.cv_bridge = CvBridge()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        self.create_subscription(CameraInfo, '/zed/zed_node/rgb/camera_info', self.info_callback, qos)
        self.create_subscription(Image, '/zed/zed_node/depth/depth_registered', self.depth_callback, qos)

        self.create_subscription(Detection2DArray, '/yolo/detections_2d', self.yolo_callback, 10)

        self.det3d_pub = self.create_publisher(Detection3DArray, 'ob_detection/detections_3d', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'ob_detection/markers', 10)
        
        self.create_service(SetBool, 'ob_detection/toggle', self.toggle_callback)
        self.get_logger().info("Absolute Position Node Ready. Waiting for TF (Map) and 2D Boxes...")

    def info_callback(self, msg):
        if self.fx is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.get_logger().info("Camera Calibration Loaded.")

    def depth_callback(self, msg):
        try:
            self.latest_depth = self.cv_bridge.imgmsg_to_cv2(msg, '32FC1')
            self.latest_header = msg.header
        except Exception: pass

    def toggle_callback(self, req, res):
        self.is_active = req.data
        status = "STARTED" if self.is_active else "PAUSED"
        self.get_logger().info(f"System {status}")
        res.success = True; res.message = status
        return res

    def get_absolute_position(self, x_cam, y_cam, z_cam, frame_id):
        """ 
        Converts (X,Y,Z) from Camera Frame -> Map Frame 
        Returns: (x_map, y_map, z_map) or None if failed
        """
        pt_cam = PointStamped()
        pt_cam.header.frame_id = frame_id

        # FIX: Set time to 0. This tells TF to use the LATEST available transform.
        pt_cam.header.stamp = rclpy.time.Time(seconds=0).to_msg()

        pt_cam.point.x = x_cam
        pt_cam.point.y = y_cam
        pt_cam.point.z = z_cam

        try:
            # Use a small timeout just in case
            pt_map = self.tf_buffer.transform(pt_cam, 'map', timeout=rclpy.duration.Duration(seconds=0.1))
            return pt_map.point.x, pt_map.point.y, pt_map.point.z
        except Exception as e:
            self.get_logger().warn(f"TF Failure: {e}")
            return None

        try:
            pt_map = self.tf_buffer.transform(pt_cam, 'map', timeout=rclpy.duration.Duration(seconds=0.05))
            return pt_map.point.x, pt_map.point.y, pt_map.point.z
        except Exception as e:
            return None

    def yolo_callback(self, msg):
        if not self.is_active: return
        if self.latest_depth is None or self.fx is None: return

        det3d_array = Detection3DArray()
        det3d_array.header.frame_id = 'map' 
        det3d_array.header.stamp = self.get_clock().now().to_msg()
        
        marker_array = MarkerArray()
        d = Marker(); d.action = Marker.DELETEALL; marker_array.markers.append(d)
        count = 0

        for det2d in msg.detections:
            u = int(det2d.bbox.center.x)
            v = int(det2d.bbox.center.y)
            label = det2d.results[0].hypothesis.class_id if det2d.results else "obj"
            score = det2d.results[0].hypothesis.score if det2d.results else 1.0

            if 0 <= u < self.latest_depth.shape[1] and 0 <= v < self.latest_depth.shape[0]:
                
                # 1. Get Depth (Camera Z)
                z = float(self.latest_depth[v, u])

                if not np.isnan(z) and not np.isinf(z) and z > 0.1:
                    
                    # 2. Camera Coordinates (Relative)
                    x_rel = (u - self.cx) * z / self.fx
                    y_rel = (v - self.cy) * z / self.fy

                    # 3. Transform to Absolute Coordinates (Map)
                    abs_pos = self.get_absolute_position(x_rel, y_rel, z, self.latest_header.frame_id)

                    if abs_pos:
                        map_x, map_y, map_z = abs_pos

                        # 4. Pack Message
                        det3d = Detection3D()
                        det3d.header = det3d_array.header 
                        
                        hyp = ObjectHypothesisWithPose()
                        hyp.hypothesis.class_id = label
                        hyp.hypothesis.score = score
                        
                        hyp.pose.pose.position.x = map_x
                        hyp.pose.pose.position.y = map_y
                        hyp.pose.pose.position.z = map_z
                        hyp.pose.pose.orientation.w = 1.0 
                        
                        det3d.results.append(hyp)
                        det3d.bbox.center = hyp.pose.pose.position
                        det3d.bbox.size.x = 0.2; det3d.bbox.size.y = 0.2; det3d.bbox.size.z = 0.2
                        det3d_array.detections.append(det3d)

                        # 5. Marker
                        m = Marker()
                        m.header = det3d_array.header
                        m.ns = "abs_detections"; m.id = count; m.type = Marker.SPHERE; m.action = Marker.ADD
                        m.pose.position = hyp.pose.pose.position
                        m.scale.x = 0.2; m.scale.y = 0.2; m.scale.z = 0.2
                        m.color.a = 1.0; m.color.g = 1.0 
                        marker_array.markers.append(m)
                        count += 1

        if count > 0:
            self.det3d_pub.publish(det3d_array)
            self.marker_pub.publish(marker_array)

def main():
    rclpy.init()
    node = ObPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()