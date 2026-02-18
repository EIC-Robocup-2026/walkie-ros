#!/usr/bin/env python3
"""ROS2 node that receives end-effector pose, solves IK, and publishes joint
targets directly to a ros2_control JointTrajectoryController.

Subscribes to:
    /target_pose (geometry_msgs/PoseStamped) - Target end-effector pose (xyz + quaternion)
    /joint_states (sensor_msgs/JointState) - Current joint states for IK warm-start

Publishes to:
    /left_arm_controller/joint_trajectory (trajectory_msgs/JointTrajectory)
        - Single-point trajectory at configurable rate (default 50Hz)

No MoveIt dependency — publishes directly to ros2_control.
"""

import os
import sys
import threading
from typing import Optional

import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401 — registers PoseStamped transform type
import tf2_ros
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Add the parent directory to path so 'walkie_ik_servo' package can be imported
# When installed: ik_servo_node.py is at lib/walkie_ik_servo/ik_servo_node.py
#   and modules are at lib/walkie_ik_servo/walkie_ik_servo/*.py
# When running from source: ik_servo_node.py is at walkie_ik_servo/ik_servo_node.py
#   and modules are siblings, so parent dir contains the walkie_ik_servo/ package
pkg_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(pkg_dir)
if pkg_dir not in sys.path:
    sys.path.insert(0, pkg_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from walkie_ik_servo.ik_solver import DEFAULT_IK_CONFIG, WalkieArmIKSolver


def pose_to_se3(pose: Pose) -> np.ndarray:
    """Convert geometry_msgs/Pose to 4x4 SE3 homogeneous matrix.

    Args:
        pose: ROS Pose message with position (xyz) and orientation (quaternion xyzw).

    Returns:
        4x4 numpy array representing the SE3 transformation.
    """
    import pinocchio as pin

    # Pinocchio Quaternion constructor: (w, x, y, z)
    quat = pin.Quaternion(
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
    )
    rotation = quat.toRotationMatrix()

    T = np.eye(4)
    T[:3, :3] = rotation
    T[0, 3] = pose.position.x
    T[1, 3] = pose.position.y
    T[2, 3] = pose.position.z

    return T


class IKDirectControlNode(Node):
    """ROS2 node bridging end-effector pose commands to ros2_control.

    Pipeline:
        1. Receive target PoseStamped (xyz + quaternion) on /target_pose
        2. At configurable rate (default 50Hz), solve IK using Pinocchio
        3. Publish JointTrajectory with single waypoint to the controller
    """

    def __init__(self):
        super().__init__("ik_servo_node")

        # Declare parameters
        self._declare_parameters()

        # Read parameters
        urdf_path = self.get_parameter("urdf_path").get_parameter_value().string_value

        # Direct control parameters
        self.publish_rate = (
            self.get_parameter("publish_rate").get_parameter_value().double_value
        )
        command_topic = (
            self.get_parameter("command_topic").get_parameter_value().string_value
        )
        self.trajectory_duration = (
            self.get_parameter("trajectory_duration").get_parameter_value().double_value
        )

        # Read topic names
        target_pose_topic = (
            self.get_parameter("target_pose_topic").get_parameter_value().string_value
        )
        joint_states_topic = (
            self.get_parameter("joint_states_topic").get_parameter_value().string_value
        )

        # Read arm joint names
        self.arm_joint_names = list(
            self.get_parameter("arm_joint_names")
            .get_parameter_value()
            .string_array_value
        )

        # Read IK config from parameters
        ik_config = self._build_ik_config()

        # Resolve URDF path
        if not os.path.isabs(urdf_path):
            from ament_index_python.packages import get_package_share_directory

            pkg_share = get_package_share_directory("walkie_ik_servo")
            urdf_path = os.path.join(pkg_share, urdf_path)

        if not os.path.exists(urdf_path):
            self.get_logger().fatal(f"URDF not found: {urdf_path}")
            raise FileNotFoundError(f"URDF not found: {urdf_path}")

        self.get_logger().info(f"URDF path: {urdf_path}")

        # Initialize IK solver
        self.get_logger().info(
            "Initializing IK solver (this may take a few seconds)..."
        )
        self.ik_solver = WalkieArmIKSolver(urdf_path, ik_config)
        self.get_logger().info("IK solver ready")

        # IK solver reference frame (target poses are transformed into this frame)
        self.ik_frame = (
            self.get_parameter("ik_frame").get_parameter_value().string_value
        )
        self.tf_timeout = rclpy.duration.Duration(
            seconds=self.get_parameter("tf_timeout").get_parameter_value().double_value
        )

        # Position-only IK mode
        self.position_only = (
            self.get_parameter("position_only").get_parameter_value().bool_value
        )

        # TF2 buffer & listener for frame transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # State
        self._current_q = None  # Current joint positions from /joint_states
        self._target_pose_stamped = None  # Latest target PoseStamped (with frame_id)
        self._lock = threading.Lock()

        # QoS for joint states (best effort for real-time)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )

        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            target_pose_topic,
            self._target_pose_callback,
            10,
        )
        self.joint_state_sub = self.create_subscription(
            JointState,
            joint_states_topic,
            self._joint_state_callback,
            sensor_qos,
        )

        # Publisher for JointTrajectory
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            command_topic,
            10,
        )

        # Timer-based servo loop
        servo_period = 1.0 / self.publish_rate
        self.servo_timer = self.create_timer(servo_period, self._servo_loop)

        self.get_logger().info(
            f"IK Direct Control Node started:\n"
            f"  Subscribe:  {target_pose_topic} (PoseStamped)\n"
            f"  Subscribe:  {joint_states_topic} (JointState)\n"
            f"  Publish:    {command_topic} (JointTrajectory)\n"
            f"  Rate:       {self.publish_rate} Hz\n"
            f"  Duration:   {self.trajectory_duration} s\n"
            f"  IK frame:   {self.ik_frame}\n"
            f"  Pos-only:   {'ON' if self.position_only else 'OFF'}\n"
            f"  Collision:  {'ON' if ik_config.get('collision_check_enabled', True) else 'OFF'}\n"
            f"  Joints:     {self.arm_joint_names}"
        )

    # ------------------------------------------------------------------ #
    # Parameter declaration
    # ------------------------------------------------------------------ #

    def _declare_parameters(self):
        """Declare all ROS2 parameters with defaults."""
        self.declare_parameter("urdf_path", "urdf/gz_walkie_1arm.urdf")

        # Topic names
        self.declare_parameter("target_pose_topic", "/target_pose")
        self.declare_parameter("joint_states_topic", "/joint_states")

        # Direct ros2_control parameters
        self.declare_parameter("command_topic", "/left_arm_controller/joint_trajectory")
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("trajectory_duration", 0.02)

        # Arm joint names
        self.declare_parameter(
            "arm_joint_names",
            [
                "left_joint1",
                "left_joint2",
                "left_joint3",
                "left_joint4",
                "left_joint5",
                "left_joint6",
                "left_joint7",
            ],
        )

        # EE frame
        self.declare_parameter("ee_frame", "left_gripper_center")

        # IK solver reference frame — target poses are transformed into this frame
        # before solving. Must match the URDF root frame used by the reduced model.
        self.declare_parameter("ik_frame", "base_footprint")
        self.declare_parameter("tf_timeout", 0.1)  # seconds to wait for TF

        # Position-only IK mode — when True, the solver tracks only the XYZ
        # position of the target and preserves the current EE orientation.
        # This avoids orientation conflicts when the target quaternion doesn't
        # match the arm's natural kinematic orientation.
        self.declare_parameter("position_only", False)

        # IK solver params (Jacobian-based damped least squares)
        self.declare_parameter("ik_translation_weight", 1.0)
        self.declare_parameter("ik_rotation_weight", 0.3)
        self.declare_parameter("ik_damping", 1e-2)
        self.declare_parameter("ik_step_size", 1.0)
        self.declare_parameter("ik_max_iterations", 20)
        self.declare_parameter("ik_tolerance", 1e-3)

        # Filter weights
        self.declare_parameter("filter_weights", [0.4, 0.3, 0.2, 0.1])

        # Self-collision detection
        self.declare_parameter("collision_check_enabled", False)
        self.declare_parameter("collision_safety_margin", -0.005)
        self.declare_parameter("collision_max_backtrack", 3)

    def _build_ik_config(self) -> dict:
        """Build IK config dict from ROS2 parameters."""
        return {
            "ee_frame": self.get_parameter("ee_frame")
            .get_parameter_value()
            .string_value,
            "arm_joint_names": list(
                self.get_parameter("arm_joint_names")
                .get_parameter_value()
                .string_array_value
            ),
            "ik_translation_weight": self.get_parameter("ik_translation_weight")
            .get_parameter_value()
            .double_value,
            "ik_rotation_weight": self.get_parameter("ik_rotation_weight")
            .get_parameter_value()
            .double_value,
            "ik_damping": self.get_parameter("ik_damping")
            .get_parameter_value()
            .double_value,
            "ik_step_size": self.get_parameter("ik_step_size")
            .get_parameter_value()
            .double_value,
            "ik_max_iterations": self.get_parameter("ik_max_iterations")
            .get_parameter_value()
            .integer_value,
            "ik_tolerance": self.get_parameter("ik_tolerance")
            .get_parameter_value()
            .double_value,
            "filter_weights": list(
                self.get_parameter("filter_weights")
                .get_parameter_value()
                .double_array_value
            ),
            "collision_check_enabled": self.get_parameter("collision_check_enabled")
            .get_parameter_value()
            .bool_value,
            "collision_safety_margin": self.get_parameter("collision_safety_margin")
            .get_parameter_value()
            .double_value,
            "collision_max_backtrack": self.get_parameter("collision_max_backtrack")
            .get_parameter_value()
            .integer_value,
        }

    # ------------------------------------------------------------------ #
    # Subscriber callbacks
    # ------------------------------------------------------------------ #

    def _target_pose_callback(self, msg: PoseStamped) -> None:
        """Store the latest target pose (with frame_id) for the servo loop."""
        with self._lock:
            self._target_pose_stamped = msg

    def _joint_state_callback(self, msg: JointState) -> None:
        """Extract arm joint positions from JointState."""
        joint_map = {}
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                joint_map[name] = msg.position[i]

        q = np.zeros(len(self.arm_joint_names))
        found_all = True
        for i, name in enumerate(self.arm_joint_names):
            if name in joint_map:
                q[i] = joint_map[name]
            else:
                found_all = False

        if found_all:
            with self._lock:
                self._current_q = q

    # ------------------------------------------------------------------ #
    # Servo loop + trajectory publishing
    # ------------------------------------------------------------------ #

    def _servo_loop(self) -> None:
        """Timer callback: solve IK and publish JointTrajectory if target is set."""
        with self._lock:
            target_pose_stamped = self._target_pose_stamped
            current_q = self._current_q.copy() if self._current_q is not None else None

        if target_pose_stamped is None:
            return  # No target yet

        # Transform target pose into the IK solver's reference frame if needed
        source_frame = target_pose_stamped.header.frame_id
        if source_frame and source_frame != self.ik_frame:
            try:
                target_pose_stamped = self.tf_buffer.transform(
                    target_pose_stamped,
                    self.ik_frame,
                    timeout=self.tf_timeout,
                )
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as e:
                self.get_logger().warning(
                    f"TF transform {source_frame} -> {self.ik_frame} failed: {e}",
                    throttle_duration_sec=2.0,
                )
                return  # Skip this cycle

        # Convert Pose to SE3 matrix
        target_se3 = pose_to_se3(target_pose_stamped.pose)

        # Position-only mode: keep the target XYZ but replace orientation
        # with the current EE orientation from FK so the solver doesn't
        # fight to achieve an unreachable orientation.
        if self.position_only:
            # Use current_q if available; otherwise fall back to solver's
            # warm-start (init_data) which is the last successful solution.
            fk_q = current_q if current_q is not None else self.ik_solver.init_data
            current_se3 = self.ik_solver.get_forward_kinematics(fk_q)
            target_se3[:3, :3] = current_se3[:3, :3]

        # Solve IK
        try:
            q_target, _ = self.ik_solver.solve_ik(target_se3, current_q)
        except Exception as e:
            self.get_logger().error(f"IK solve failed: {e}", throttle_duration_sec=1.0)
            return

        # Publish to controller
        self._publish_joint_trajectory(q_target)

    def _publish_joint_trajectory(self, q_target: np.ndarray) -> None:
        """Publish a single-point JointTrajectory to the controller.

        Args:
            q_target: Target joint positions array of shape (n_joints,).
        """
        msg = JointTrajectory()
        msg.joint_names = self.arm_joint_names

        point = JointTrajectoryPoint()
        point.positions = q_target.tolist()

        # Convert trajectory_duration (float seconds) to Duration
        sec = int(self.trajectory_duration)
        nanosec = int((self.trajectory_duration - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)

        msg.points.append(point)
        self.traj_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IKDirectControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
