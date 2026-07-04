#!/usr/bin/env python3
"""Republish a nav_msgs/Odometry topic as a TF broadcast.

Used ONLY during offline SLAM re-mapping from a recorded bag. The recorded `/tf`
already carries slam_toolbox's live `map->odom` (slam ran during the match), which
would fight a freshly-launched slam_toolbox. So the SLAM-rebuild recipe replays the
bag WITHOUT `/tf` and uses this node to reconstruct the `odom->base_footprint`
transform slam needs from the recorded wheel-odometry topic -- leaving exactly one
`map->odom` publisher (the new slam run).

    ros2 bag play <bag> --clock --topics /scan /tf_static /omni_wheel_drive_controller/odom
    python3 odom_to_tf.py
    ros2 launch robot_navigation mapping_Nav2_real.launch.py use_sim_time:=true

Frame-agnostic: it broadcasts msg.header.frame_id -> msg.child_frame_id verbatim, so
it works whether the controller emits odom->base_footprint or odom->base_link (the
base_link->base_footprint static link then comes from the replayed /tf_static).

The broadcast TF stamp is copied from the odometry message header, so it lines up
with the bag's `--clock` time regardless of this node's own use_sim_time setting.

Override the input topic with:
    python3 odom_to_tf.py --ros-args -p odom_topic:=/my/odom
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTf(Node):
    def __init__(self):
        super().__init__("odom_to_tf")
        topic = self.declare_parameter(
            "odom_topic", "/omni_wheel_drive_controller/odom"
        ).get_parameter_value().string_value
        self._br = TransformBroadcaster(self)
        self._sub = self.create_subscription(Odometry, topic, self._on_odom, 10)
        self.get_logger().info(f"Broadcasting odom->base TF from '{topic}'")

    def _on_odom(self, msg: Odometry):
        t = TransformStamped()
        t.header = msg.header                      # frame_id (usually 'odom') + stamp
        t.child_frame_id = msg.child_frame_id      # usually 'base_footprint' / 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._br.sendTransform(t)


def main():
    rclpy.init()
    node = OdomToTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
