"""
ROS 2 side of the full-body IK: reads the robot state and sends an IK solution for execution.

In:   /joint_states (lift + arm joints), TF map -> base_footprint (base pose; Nav2's frames).
Out:  lift + both arms -> MoveIt move_group, MoveGroup action /move_action, as one joint goal
      for the planning group both_arms_lift (plan and execute);
      x, y, yaw -> Nav2, NavigateToPose action /navigate_to_pose, as a goal pose in map.
Both goals are sent at the same time, so the base drives while the arms and lift move.

The walkie Nav2 default goal checker (general_goal_checker) stops within 0.1 m, too coarse for
the arms. While an IK base goal runs, its behavior tree is switched to precise_goal_checker
(0.02 m, 0.01 rad) through its goal_checker_selector topic, and back to the default afterwards.

The node spins in its own thread; every public method is thread-safe.
"""
import math
import threading

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .rotations import yaw_from_quat
from .walkie_model import WALKIE_JOINTS

MAP_FRAME = "map"
BASE_FRAME = "base_footprint"
MOVEIT_GROUP = "both_arms_lift"
MOVEIT_JOINT_TOLERANCE = 1e-3       # rad / m
# a base move smaller than this is not sent to Nav2 (it would only run its goal checker)
NAV_MIN_TRANSLATION = 0.01          # m
NAV_MIN_ROTATION = math.radians(1.0)
# goal checkers of robot_navigation's nav2_mppi_walkie_params.yaml, selected through the BT's
# GoalCheckerSelector (topic goal_checker_selector)
PRECISE_GOAL_CHECKER = "precise_goal_checker"
DEFAULT_GOAL_CHECKER = "general_goal_checker"

GOAL_STATUS = {GoalStatus.STATUS_SUCCEEDED: "succeeded", GoalStatus.STATUS_ABORTED: "aborted",
               GoalStatus.STATUS_CANCELED: "canceled"}
MOVEIT_ERRORS = {v: k for k, v in vars(MoveItErrorCodes).items() if k.isupper() and isinstance(v, int)}


class RobotBridge:
    def __init__(self, velocity_scaling: float = 0.5, acceleration_scaling: float = 0.3,
                 planning_time: float = 5.0, precise_base: bool = True):
        self.velocity_scaling = velocity_scaling
        self.acceleration_scaling = acceleration_scaling
        self.planning_time = planning_time
        self.precise_base = precise_base

        # rclpy's own SIGINT/SIGTERM handlers would stop ROS but leave the GUI open; the
        # entry script handles the signals by closing the GUI instead
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node = Node("full_body_ik", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self._lock = threading.Lock()
        self._joints = {}                                   # walkie joint name -> position
        self._status = {"MoveIt": "idle", "Nav2": "idle"}
        self._goal_handles = {}                             # "MoveIt"/"Nav2" -> active goal handle

        self.node.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self.node)
        self._moveit = ActionClient(self.node, MoveGroup, "move_action")
        self._nav = ActionClient(self.node, NavigateToPose, "navigate_to_pose")
        # latched like the selector's subscription, so the selection also reaches a BT started later
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._goal_checker = self.node.create_publisher(String, "goal_checker_selector", latched)

        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self.node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()

    def shutdown(self) -> None:
        self._executor.shutdown(timeout_sec=2.0)
        self._thread.join(timeout=2.0)          # spin must have returned before the node goes
        self.node.destroy_node()
        rclpy.try_shutdown()

    # ---- robot state ----

    def _on_joint_state(self, msg: JointState) -> None:
        with self._lock:
            self._joints.update(zip(msg.name, msg.position))

    def robot_state(self):
        """{IK joint name: value} of the robot now, or (None, reason) while something is missing."""
        with self._lock:
            joints = dict(self._joints)
        missing = [w for w in WALKIE_JOINTS.values() if w not in joints]
        if missing:
            return None, f"waiting for /joint_states ({len(missing)} joints missing)"
        try:
            tf = self._tf.lookup_transform(MAP_FRAME, BASE_FRAME, Time())
        except TransformException:
            return None, f"waiting for TF {MAP_FRAME} -> {BASE_FRAME}"
        t, q = tf.transform.translation, tf.transform.rotation
        state = {"x": t.x, "y": t.y, "yaw": yaw_from_quat(q.x, q.y, q.z, q.w)}
        state.update({name: float(joints[w]) for name, w in WALKIE_JOINTS.items()})
        return state, None

    # ---- execution ----

    def status_text(self) -> str:
        with self._lock:
            return "\n".join(f"{k}: {v}" for k, v in self._status.items())

    def _set_status(self, who: str, text: str) -> None:
        with self._lock:
            self._status[who] = text
        self.node.get_logger().info(f"{who}: {text}")

    def execute(self, command: dict, robot: dict | None) -> None:
        """
        Send the lift + arm joints of `command` to MoveIt and its base pose to Nav2, at once.
        robot: the robot state now; the base goal is skipped if it is (almost) there already.
        """
        self._send_moveit(command)
        if robot is not None and (
                math.hypot(command["x"] - robot["x"], command["y"] - robot["y"]) < NAV_MIN_TRANSLATION
                and abs(math.remainder(command["yaw"] - robot["yaw"], math.tau)) < NAV_MIN_ROTATION):
            self._set_status("Nav2", "base already at the goal, not sent")
        else:
            self._send_nav(command)

    def cancel(self) -> None:
        with self._lock:
            handles = list(self._goal_handles.items())
        for who, handle in handles:
            handle.cancel_goal_async()
            self._set_status(who, "cancel requested")

    def _send_moveit(self, command: dict) -> None:
        if not self._moveit.server_is_ready():
            self._set_status("MoveIt", "/move_action not available (is move_group running?)")
            return
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = MOVEIT_GROUP
        req.num_planning_attempts = 5
        req.allowed_planning_time = self.planning_time
        req.max_velocity_scaling_factor = self.velocity_scaling
        req.max_acceleration_scaling_factor = self.acceleration_scaling
        req.start_state.is_diff = True                      # plan from move_group's current state
        req.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=walkie_name, position=float(command[name]),
                            tolerance_above=MOVEIT_JOINT_TOLERANCE, tolerance_below=MOVEIT_JOINT_TOLERANCE,
                            weight=1.0)
            for name, walkie_name in WALKIE_JOINTS.items()])]
        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        self._set_status("MoveIt", "planning...")
        future = self._moveit.send_goal_async(goal)
        future.add_done_callback(lambda f: self._on_accepted("MoveIt", f, self._on_moveit_result))

    def _send_nav(self, command: dict) -> None:
        if not self._nav.server_is_ready():
            self._set_status("Nav2", "/navigate_to_pose not available (is Nav2 running?)")
            return
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = MAP_FRAME
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = float(command["x"]), float(command["y"])
        pose.pose.orientation.z = math.sin(command["yaw"] / 2)
        pose.pose.orientation.w = math.cos(command["yaw"] / 2)
        goal.pose = pose
        if self.precise_base:
            self._goal_checker.publish(String(data=PRECISE_GOAL_CHECKER))

        self._set_status("Nav2", f"goal x {command['x']:+.3f} y {command['y']:+.3f} "
                                 f"yaw {math.degrees(command['yaw']):+.1f} deg sent")
        future = self._nav.send_goal_async(goal)
        future.add_done_callback(lambda f: self._on_accepted("Nav2", f, self._on_nav_result))

    def _on_accepted(self, who: str, future, on_result) -> None:
        handle = future.result()
        if not handle.accepted:
            self._set_status(who, "goal rejected")
            if who == "Nav2":
                self._restore_goal_checker()
            return
        with self._lock:
            self._goal_handles[who] = handle
        if who == "MoveIt":
            self._set_status(who, "planning + executing...")
        else:
            self._set_status(who, "navigating...")
        handle.get_result_async().add_done_callback(lambda f: on_result(handle, f))

    def _finish(self, who: str, handle, text: str) -> None:
        with self._lock:
            if self._goal_handles.get(who) is handle:
                del self._goal_handles[who]
        self._set_status(who, text)

    def _on_moveit_result(self, handle, future) -> None:
        response = future.result()
        code = response.result.error_code.val
        if code == MoveItErrorCodes.SUCCESS:
            text = "succeeded"
        else:
            text = f"failed: {MOVEIT_ERRORS.get(code, code)}"
        self._finish("MoveIt", handle, text)

    def _on_nav_result(self, handle, future) -> None:
        response = future.result()
        text = GOAL_STATUS.get(response.status, f"status {response.status}")
        error = getattr(response.result, "error_code", 0)
        if error:
            text += f" (error {error}: {getattr(response.result, 'error_msg', '')})"
        self._finish("Nav2", handle, text)
        self._restore_goal_checker()

    def _restore_goal_checker(self) -> None:
        with self._lock:
            nav_active = "Nav2" in self._goal_handles
        if self.precise_base and not nav_active:
            self._goal_checker.publish(String(data=DEFAULT_GOAL_CHECKER))
