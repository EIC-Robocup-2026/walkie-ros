#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseArray, Pose, PointStamped

import tf2_ros
import tf2_geometry_msgs 
import numpy as np
class ObPoseNode(Node):
    def __init__(self):
        super().__init__('ob_pose')

        self.is_active = True 
        self.fx = self.fy = self.cx = self.cy = None
        self.latest_depth = None
        self.latest_header = None
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        self.create_subscription(CameraInfo, '/zed/zed_node/depth/camera_info', self.info_callback, qos)
        self.create_subscription(Image, '/zed/zed_node/depth/depth_registered', self.depth_callback, qos)
        self.create_subscription(Detection2DArray, '/yolo/detections_2d', self.yolo_callback, 10)

        self.pose_pub = self.create_publisher(PoseArray, 'ob_detection/poses', 10)        
        self.marker_pub = self.create_publisher(MarkerArray, 'ob_detection/markers', 10)
        
        self.create_service(SetBool, 'ob_detection/toggle', self.toggle_callback)
        self.get_logger().info("Absolute Position Node Ready.")

    def info_callback(self, msg):
        if self.fx is None:
            self.fx = msg.k[0]; self.fy = msg.k[4]; self.cx = msg.k[2]; self.cy = msg.k[5]
            self.get_logger().info(f" Camera Info Loaded: fx={self.fx:.1f}, fy={self.fy:.1f}")

    def depth_callback(self, msg):
        try:
            dtype = np.float32
            if '16U' in msg.encoding: 
                dtype = np.uint16
            
            data = np.frombuffer(msg.data, dtype=dtype)
            self.latest_depth = data.reshape((msg.height, msg.width))
            self.latest_header = msg.header
            self.get_logger().info(f"received depth image ({msg.width}x{msg.height})", throttle_duration_sec=2.0)

        except Exception as e:
            self.get_logger().error(f"Depth conversion error: {e}")

    def toggle_callback(self, req, res):
        self.is_active = req.data
        status = "STARTED" if self.is_active else "PAUSED"
        res.success = True; res.message = status
        return res

    def get_absolute_position(self, x_cam, y_cam, z_cam, frame_id):
        pt_cam = PointStamped()
        pt_cam.header.frame_id = frame_id   
        pt_cam.header.stamp = rclpy.time.Time(seconds=0).to_msg()
        pt_cam.point.x = x_cam; pt_cam.point.y = y_cam; pt_cam.point.z = z_cam

        try:
            pt_map = self.tf_buffer.transform(pt_cam, 'map', timeout=rclpy.duration.Duration(seconds=0.1))
            return pt_map.point.x, pt_map.point.y, pt_map.point.z
        except Exception as e:
            self.get_logger().warn(f"TF Transform Failed: {e}", throttle_duration_sec=1.0)
            return None

    def get_robust_depth(self, u, v, depth_img, search_radius=5):
        """
        Searches a small window around (u, v) for valid depth values 
        and returns the median.
        """
        h, w = depth_img.shape
        
        u_min = max(0, u - search_radius)
        u_max = min(w, u + search_radius + 1)
        v_min = max(0, v - search_radius)
        v_max = min(h, v + search_radius + 1)

        roi = depth_img[v_min:v_max, u_min:u_max]
        
        valid_mask = np.isfinite(roi) & (roi > 0.1)
        valid_pixels = roi[valid_mask]

        if len(valid_pixels) == 0:
            return None
        return float(np.median(valid_pixels))

    def yolo_callback(self, msg):
        if not self.is_active or self.latest_depth is None or self.fx is None: return

        pose_array = PoseArray()
        pose_array.header.frame_id = 'map' 
        pose_array.header.stamp = self.get_clock().now().to_msg()
        
        marker_array = MarkerArray()
        d = Marker(); d.action = Marker.DELETEALL; marker_array.markers.append(d)
        count = 0

        for det2d in msg.detections:
            try:
                cx = det2d.bbox.center.x
                cy = det2d.bbox.center.y
            except AttributeError:
                try:
                    cx = det2d.bbox.center.position.x
                    cy = det2d.bbox.center.position.y
                except AttributeError:
                    continue
            
            u, v = int(cx), int(cy)
            m = None

            if 0 <= u < self.latest_depth.shape[1] and 0 <= v < self.latest_depth.shape[0]:
                
                z = self.get_robust_depth(u, v, self.latest_depth, search_radius=5)

                if z is not None:
                    x_rel = (u - self.cx) * z / self.fx
                    y_rel = (v - self.cy) * z / self.fy

                    abs_pos = self.get_absolute_position(x_rel, y_rel, z, self.latest_header.frame_id)

                    if abs_pos:
                        map_x, map_y, map_z = abs_pos

                        pose = Pose()
                        pose.position.x = map_x
                        pose.position.y = map_y
                        pose.position.z = map_z
                        pose.orientation.w = 1.0 
                        pose_array.poses.append(pose)

                        m = Marker()
                        m.header = pose_array.header
                        m.ns = "objects"; m.id = count; m.type = Marker.SPHERE; m.action = Marker.ADD
                        m.pose = pose
                        m.scale.x = 0.2; m.scale.y = 0.2; m.scale.z = 0.2
                        m.color.a = 1.0; m.color.g = 1.0 
                        marker_array.markers.append(m)
                        count += 1
                else:
                    self.get_logger().info(f"Could not find valid depth near x:{u} y:{v}", throttle_duration_sec=1.0)

            if m is None:
                pose = Pose()
                pose.position.x = 0.0
                pose.position.y = 0.0
                pose.position.z = 0.0
                pose.orientation.w = 10.0 
                pose_array.poses.append(pose)

        self.pose_pub.publish(pose_array)
        if count > 0:
            self.marker_pub.publish(marker_array)
            self.get_logger().info(f"Published {count} poses")

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