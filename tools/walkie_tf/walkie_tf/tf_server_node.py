import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import tf2_ros
from walkie_tf_interfaces.srv import GetTransform


class TFServerNode(Node):
    def __init__(self):
        super().__init__("walkie_tf_server")
        self._buffer = tf2_ros.Buffer()
        # TransformListener subscriptions stay in the node's default
        # (mutually-exclusive) callback group; the service runs in its own
        # reentrant group. With a MultiThreadedExecutor this lets the /tf
        # callbacks keep feeding the buffer while a service handler is blocked
        # inside lookup_transform's timeout, instead of starving it.
        self._listener = tf2_ros.TransformListener(self._buffer, self)
        self._service_cb_group = ReentrantCallbackGroup()
        self.create_service(
            GetTransform,
            "get_transform",
            self._handle,
            callback_group=self._service_cb_group,
        )
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
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
