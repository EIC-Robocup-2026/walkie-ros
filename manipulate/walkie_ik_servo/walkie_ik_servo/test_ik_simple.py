#!/usr/bin/env python3
"""Solve IK for a goal pose, verify with FK, and optionally publish to ROS2.

Standalone mode (no ROS):
    python3 test_ik_simple.py
    python3 test_ik_simple.py --x 0.40 --y 0.20 --z 0.55

Publish to ROS2:
    python3 test_ik_simple.py --publish
    python3 test_ik_simple.py --publish --x 0.40 --y 0.20 --z 0.55
    python3 test_ik_simple.py --publish --x 0.40 --y 0.20 --z 0.55 --position-only
    python3 test_ik_simple.py --publish --x 0.40 --y 0.20 --z 0.55 --roll -1.5708 --yaw 1.5708

When --publish is given the script starts a ROS2 node and publishes PoseStamped
at --rate Hz until Ctrl-C.  Without --publish it just solves IK offline and
prints the result (no ROS dependency).
"""

import argparse
import math
import os
import sys

import numpy as np

# ------------------------------------------------------------------ #
# Path setup so the solver module can be imported
# ------------------------------------------------------------------ #
script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(script_dir)
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from walkie_ik_servo.ik_solver import WalkieArmIKSolver

# ------------------------------------------------------------------ #
# Helpers
# ------------------------------------------------------------------ #

URDF_PATH = os.path.normpath(
    os.path.join(script_dir, "..", "urdf", "gz_walkie_1arm.urdf")
)

SOLVER_CONFIG = {
    "ee_frame": "left_gripper_center",
    "arm_joint_names": [f"left_joint{i}" for i in range(1, 8)],
    "ik_translation_weight": 1.0,
    "ik_rotation_weight": 0.3,
    "ik_damping": 0.01,
    "ik_step_size": 1.0,
    "ik_max_iterations": 50,
    "ik_tolerance": 1e-4,
    "filter_weights": [1.0],
    "collision_check_enabled": False,
}


def make_se3(x, y, z, R=None):
    """Build a 4x4 SE3 matrix from position and optional 3x3 rotation."""
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z
    return T


def euler_to_quaternion(roll, pitch, yaw):
    """RPY (radians) -> [x, y, z, w] quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


def rpy_to_rotation_matrix(roll, pitch, yaw):
    """RPY (radians) -> 3x3 rotation matrix (ZYX convention)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def fmt_pos(T):
    """Format the translation part of a 4x4 SE3 matrix."""
    return f"({T[0, 3]:.4f}, {T[1, 3]:.4f}, {T[2, 3]:.4f})"


def print_header(title):
    print(f"\n{'=' * 60}")
    print(f"  {title}")
    print(f"{'=' * 60}")


# ------------------------------------------------------------------ #
# Solver setup
# ------------------------------------------------------------------ #


def load_solver():
    """Load the IK solver and return (solver, zero-config FK)."""
    if not os.path.exists(URDF_PATH):
        print(f"ERROR: URDF not found at {URDF_PATH}")
        sys.exit(1)
    print(f"URDF: {URDF_PATH}")
    print("Loading IK solver...")
    solver = WalkieArmIKSolver(URDF_PATH, SOLVER_CONFIG)
    q_zero = np.zeros(7)
    fk_zero = solver.get_forward_kinematics(q_zero)
    return solver, fk_zero


# ------------------------------------------------------------------ #
# Offline IK verification (no ROS)
# ------------------------------------------------------------------ #


def solve_and_verify(solver, fk_zero, x, y, z, roll, pitch, yaw, position_only):
    """Solve IK for the given goal and print results. Returns (q_sol, pos_err_mm)."""
    ee_pos = fk_zero[:3, 3]
    ee_rot = fk_zero[:3, :3]

    # Build target SE3
    if position_only:
        R = ee_rot
        print("  Mode:       position-only (using natural EE orientation)")
    else:
        R = rpy_to_rotation_matrix(roll, pitch, yaw)
        print(f"  Orientation: RPY({roll:.4f}, {pitch:.4f}, {yaw:.4f})")

    target = make_se3(x, y, z, R=R)
    print(f"  Target pos: {fmt_pos(target)}")

    # Solve
    q_zero = np.zeros(7)
    solver.reset()
    q_sol, _ = solver.solve_ik(target, q_zero)
    fk_sol = solver.get_forward_kinematics(q_sol)

    pos_err = np.linalg.norm(fk_sol[:3, 3] - target[:3, 3])
    print(f"  Solved q:   {np.array2string(q_sol, precision=4, suppress_small=True)}")
    print(f"  Reached:    {fmt_pos(fk_sol)}")
    print(f"  Pos error:  {pos_err * 1000:.2f} mm")

    if pos_err < 1.0e-3:
        print("  Status:     CONVERGED (< 1 mm)")
    elif pos_err < 5.0e-3:
        print("  Status:     OK (< 5 mm)")
    else:
        print(f"  Status:     WARNING — large error, target may be unreachable")

    return q_sol, pos_err * 1000


# ------------------------------------------------------------------ #
# ROS2 publisher
# ------------------------------------------------------------------ #


def publish_goal(args):
    """Start a ROS2 node that publishes the goal PoseStamped at a fixed rate."""
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node

    # Compute quaternion from RPY (even in position_only mode we need *some*
    # orientation in the message; the ik_servo_node will override it when
    # its own position_only flag is true).
    if args.position_only:
        # Use natural EE orientation: RPY(-pi/2, 0, pi/2)
        quat = euler_to_quaternion(-math.pi / 2, 0.0, math.pi / 2)
    else:
        quat = euler_to_quaternion(args.roll, args.pitch, args.yaw)

    rclpy.init()
    node = Node("test_ik_goal_publisher")
    pub = node.create_publisher(PoseStamped, args.topic, 10)

    msg = PoseStamped()
    msg.header.frame_id = args.frame
    msg.pose.position.x = args.x
    msg.pose.position.y = args.y
    msg.pose.position.z = args.z
    msg.pose.orientation.x = quat[0]
    msg.pose.orientation.y = quat[1]
    msg.pose.orientation.z = quat[2]
    msg.pose.orientation.w = quat[3]

    rate_hz = args.rate
    dt = 1.0 / rate_hz

    def _timer_cb():
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)

    node.create_timer(dt, _timer_cb)

    node.get_logger().info(
        f"Publishing goal to {args.topic} at {rate_hz} Hz\n"
        f"  Frame:    {args.frame}\n"
        f"  Position: ({args.x:.4f}, {args.y:.4f}, {args.z:.4f})\n"
        f"  Quat:     ({quat[0]:.4f}, {quat[1]:.4f}, {quat[2]:.4f}, {quat[3]:.4f})\n"
        f"  Pos-only: {args.position_only}\n"
        f"  Ctrl-C to stop"
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ------------------------------------------------------------------ #
# CLI
# ------------------------------------------------------------------ #


def parse_args():
    p = argparse.ArgumentParser(
        description="Solve IK for a goal pose and optionally publish to ROS2.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "examples:\n"
            "  python3 test_ik_simple.py                          # run offline IK tests\n"
            "  python3 test_ik_simple.py --x 0.40 --z 0.55       # solve specific pose\n"
            "  python3 test_ik_simple.py --publish                # publish default pose\n"
            "  python3 test_ik_simple.py --publish --x 0.40       # publish specific pose\n"
            "  python3 test_ik_simple.py --publish --position-only  # publish, pos-only\n"
        ),
    )

    # Goal pose
    p.add_argument(
        "--x",
        type=float,
        default=None,
        help="Target X  (default: zero-config EE X + 0.03)",
    )
    p.add_argument(
        "--y",
        type=float,
        default=None,
        help="Target Y  (default: zero-config EE Y + 0.05)",
    )
    p.add_argument(
        "--z",
        type=float,
        default=None,
        help="Target Z  (default: zero-config EE Z - 0.05)",
    )
    p.add_argument(
        "--roll",
        type=float,
        default=-math.pi / 2,
        help="Target roll  (default: -pi/2, natural EE)",
    )
    p.add_argument("--pitch", type=float, default=0.0, help="Target pitch (default: 0)")
    p.add_argument(
        "--yaw",
        type=float,
        default=math.pi / 2,
        help="Target yaw   (default: pi/2, natural EE)",
    )
    p.add_argument(
        "--position-only",
        action="store_true",
        help="Ignore orientation, solve position only (uses natural EE orientation)",
    )

    # ROS2 publish
    p.add_argument(
        "--publish",
        action="store_true",
        help="Publish PoseStamped to ROS2 after solving",
    )
    p.add_argument(
        "--topic",
        type=str,
        default="/target_pose",
        help="ROS2 topic to publish on (default: /target_pose)",
    )
    p.add_argument(
        "--frame",
        type=str,
        default="base_footprint",
        help="frame_id for PoseStamped (default: base_footprint)",
    )
    p.add_argument(
        "--rate", type=float, default=10.0, help="Publish rate in Hz (default: 10)"
    )

    return p.parse_args()


def main():
    args = parse_args()

    # Load solver and get zero-config FK
    solver, fk_zero = load_solver()
    ee_pos = fk_zero[:3, 3]

    print(
        f"Zero-config EE position: ({ee_pos[0]:.4f}, {ee_pos[1]:.4f}, {ee_pos[2]:.4f})"
    )

    # Fill in default target from zero-config + offset
    if args.x is None:
        args.x = float(ee_pos[0]) + 0.03
    if args.y is None:
        args.y = float(ee_pos[1]) + 0.05
    if args.z is None:
        args.z = float(ee_pos[2]) - 0.05

    # Offline IK solve + verify
    print_header("IK Solve")
    q_sol, err_mm = solve_and_verify(
        solver,
        fk_zero,
        args.x,
        args.y,
        args.z,
        args.roll,
        args.pitch,
        args.yaw,
        args.position_only,
    )

    # Publish to ROS2 if requested
    if args.publish:
        print_header("ROS2 Publish")
        publish_goal(args)
    else:
        print("\nTip: add --publish to send this pose to ROS2")


if __name__ == "__main__":
    main()
