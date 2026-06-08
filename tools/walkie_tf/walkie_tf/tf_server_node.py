import rclpy
from rclpy.node import Node
import tf2_ros
from walkie_tf_interfaces.srv import GetTransform


class TFServerNode(Node):
    def __init__(self):
        super().__init__("walkie_tf_server")
        self._buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buffer, self)
        self.create_service(GetTransform, "get_transform", self._handle)
        self.get_logger().info("walkie_tf_server ready")

    def _handle(self, request, response):
        try:
            timeout_sec = request.timeout_sec if request.timeout_sec > 0 else 1.0
            t = self._buffer.lookup_transform(
                request.source_frame,
                request.target_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout_sec),
            )
            tr = t.transform.translation
            ro = t.transform.rotation
            response.success = True
            response.x, response.y, response.z = tr.x, tr.y, tr.z
            response.qx, response.qy, response.qz, response.qw = ro.x, ro.y, ro.z, ro.w
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TFServerNode()
    rclpy.spin(node)
    rclpy.shutdown()
