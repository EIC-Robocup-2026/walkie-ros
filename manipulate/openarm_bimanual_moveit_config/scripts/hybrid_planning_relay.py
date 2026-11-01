#!/usr/bin/env python3
"""
Relays between the hybrid planning components (moveit_hybrid_planning 2.12.4) and the robot.

Global trajectory: the global planner publishes its MotionPlanResponse with an empty
trajectory_start. The local planner builds every waypoint on that start state, so it would
collision-check the joints outside the planning group at their defaults and, with a planar
virtual joint (moveit_only.launch.py map_frame), the robot at the map origin. This node fills
trajectory_start with the robot state now (/joint_states, and the virtual joint from TF
map_frame -> base_frame) and republishes the response for the local planner and the manager.

Local solution: the local planner publishes one trajectory_msgs/JointTrajectory with every
joint of its group (both_arms_lift: lift + both arms), but each joint_trajectory_controller
rejects joints it does not own. For each FollowJointTrajectory controller of
moveit_controllers.yaml, the joints that controller owns go to <controller>/joint_trajectory.
The local planner streams one waypoint per message, with its velocity and time_from_start 0.
A joint_trajectory_controller rejects a trajectory that ends with a non-zero velocity, so
only the positions are forwarded, and a point due at 0 s is given point_duration: the
controller then moves to each streamed waypoint over that time. The waypoints are 0.1 s
apart; reaching each in half that keeps the robot within the sampler's tolerance, so the
local planner rarely repeats a waypoint and aborts as "stuck" (which costs a replan).

Parameters:
  controllers_file   moveit_controllers.yaml (moveit_simple_controller_manager layout)
  global_input_topic   global planner output (default /global_trajectory)
  global_output_topic  local planner and manager input (default /hybrid_planning/global_trajectory)
  local_input_topic    local planner output (default /hybrid_planning/local_solution)
  point_duration     time_from_start of a point the planner sends due at 0 s [s] (default 0.05)
  map_frame          parent frame of the planar virtual joint; empty: fixed virtual joint
  base_frame         child link of the virtual joint (default base_footprint)
  virtual_joint      name of the SRDF virtual joint (default virtual_joint)
"""
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Transform
from moveit_msgs.msg import MotionPlanResponse
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def load_trajectory_controllers(path: str) -> dict:
    """{controller name: [joint names]} of the FollowJointTrajectory controllers in `path`."""
    with open(path) as f:
        config = yaml.safe_load(f)["moveit_simple_controller_manager"]
    return {name: list(config[name]["joints"]) for name in config["controller_names"]
            if config[name].get("type") == "FollowJointTrajectory"}


def _pick(values, indices):
    return [values[i] for i in indices]


class HybridPlanningRelay(Node):
    def __init__(self):
        super().__init__("hybrid_planning_relay")
        default_file = os.path.join(get_package_share_directory("openarm_bimanual_moveit_config"),
                                    "config", "moveit_controllers.yaml")
        controllers_file = self.declare_parameter("controllers_file", default_file).value
        global_input = self.declare_parameter("global_input_topic", "/global_trajectory").value
        global_output = self.declare_parameter("global_output_topic", "/hybrid_planning/global_trajectory").value
        local_input = self.declare_parameter("local_input_topic", "/hybrid_planning/local_solution").value
        point_duration = self.declare_parameter("point_duration", 0.05).value
        self._map_frame = self.declare_parameter("map_frame", "").value
        self._base_frame = self.declare_parameter("base_frame", "base_footprint").value
        self._virtual_joint = self.declare_parameter("virtual_joint", "virtual_joint").value
        self._point_duration = Duration(sec=int(point_duration), nanosec=int(point_duration % 1 * 1e9))

        self._joint_state = {}                              # name -> position
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)
        self._global_pub = self.create_publisher(MotionPlanResponse, global_output, 10)
        self.create_subscription(MotionPlanResponse, global_input, self._on_global_trajectory, 10)

        self._controllers = load_trajectory_controllers(controllers_file)
        self._controller_pubs = {name: self.create_publisher(JointTrajectory, f"/{name}/joint_trajectory", 10)
                                 for name in self._controllers}
        self._warned = set()
        self.create_subscription(JointTrajectory, local_input, self._on_local_solution, 10)
        self.get_logger().info(f"{global_input} -> {global_output} (with start state); {local_input} -> "
                               + ", ".join(f"/{n}/joint_trajectory" for n in self._controllers))

    def _on_joint_state(self, msg: JointState) -> None:
        self._joint_state.update(zip(msg.name, msg.position))

    def _on_global_trajectory(self, msg: MotionPlanResponse) -> None:
        start = msg.trajectory_start
        if not start.joint_state.name:
            start.joint_state.name = list(self._joint_state)
            start.joint_state.position = list(self._joint_state.values())
        if self._map_frame and self._virtual_joint not in start.multi_dof_joint_state.joint_names:
            try:
                tf = self._tf.lookup_transform(self._map_frame, self._base_frame, Time())
            except TransformException as e:
                self.get_logger().error(f"global trajectory without base pose: {e}")
            else:
                start.multi_dof_joint_state.header.frame_id = self._map_frame
                start.multi_dof_joint_state.joint_names.append(self._virtual_joint)
                start.multi_dof_joint_state.transforms.append(
                    Transform(translation=tf.transform.translation, rotation=tf.transform.rotation))
        self._global_pub.publish(msg)

    def _on_local_solution(self, msg: JointTrajectory) -> None:
        index = {name: i for i, name in enumerate(msg.joint_names)}
        for controller, joints in self._controllers.items():
            present = [j for j in joints if j in index]
            if not present:
                continue
            if len(present) != len(joints):
                # a joint_trajectory_controller rejects partial goals by default
                if controller not in self._warned:
                    self._warned.add(controller)
                    self.get_logger().warn(f"{controller}: trajectory has only {len(present)} of its "
                                           f"{len(joints)} joints, not forwarded")
                continue
            indices = [index[j] for j in joints]
            out = JointTrajectory(header=msg.header, joint_names=joints)
            out.points = [JointTrajectoryPoint(
                positions=_pick(p.positions, indices),
                time_from_start=p.time_from_start
                if p.time_from_start.sec or p.time_from_start.nanosec else self._point_duration)
                for p in msg.points]
            self._controller_pubs[controller].publish(out)


def main():
    rclpy.init()
    node = HybridPlanningRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
