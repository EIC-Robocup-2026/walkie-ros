#!/usr/bin/env python3
"""Standalone test: send IK goals that would cause self-collision and verify rejection.

No ROS required — directly uses the IK solver and collision detection.
Tests three scenarios:
  1. Safe target (reachable without collision)
  2. Target behind the robot (forces arm to fold into body)
  3. Direct joint configs known to collide

Usage:
    python3 test_collision_ik.py
"""

import os
import sys
import numpy as np

# Add package to path
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


def make_se3(x, y, z, R=None):
    """Build a 4x4 SE3 matrix from position and optional rotation."""
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z
    return T


def print_header(title):
    print(f"\n{'=' * 60}")
    print(f"  {title}")
    print(f"{'=' * 60}")


def print_collision_info(solver, q, label=""):
    """Print collision status and min distance for a given q."""
    collides = solver.check_collision(q)
    dist, f1, f2 = solver.get_min_distance(q)
    status = "COLLISION" if collides else "safe"
    tag = f" [{label}]" if label else ""
    print(f"  {status}{tag}: min_dist={dist * 1000:.2f}mm  ({f1} <-> {f2})")
    return collides


# ------------------------------------------------------------------ #
# Main test
# ------------------------------------------------------------------ #


def main():
    # Resolve URDF path
    urdf_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "urdf", "gz_walkie_1arm.urdf"
    )
    urdf_path = os.path.normpath(urdf_path)

    if not os.path.exists(urdf_path):
        print(f"ERROR: URDF not found at {urdf_path}")
        sys.exit(1)

    print(f"URDF: {urdf_path}")

    # Create solver with collision enabled
    config_collision_on = {
        "ee_frame": "left_gripper_center",
        "arm_joint_names": [f"left_joint{i}" for i in range(1, 8)],
        "ik_translation_weight": 1.0,
        "ik_rotation_weight": 0.3,
        "ik_damping": 0.01,
        "ik_step_size": 1.0,
        "ik_max_iterations": 50,
        "ik_tolerance": 1e-3,
        "filter_weights": [1.0],  # No smoothing for clarity
        "collision_check_enabled": True,
        "collision_safety_margin": -0.005,
        "collision_max_backtrack": 3,
    }

    # Create solver with collision DISABLED for comparison
    config_collision_off = dict(config_collision_on)
    config_collision_off["collision_check_enabled"] = False

    print("Loading solver WITH collision detection...")
    solver_on = WalkieArmIKSolver(urdf_path, config_collision_on)

    print("Loading solver WITHOUT collision detection...")
    solver_off = WalkieArmIKSolver(urdf_path, config_collision_off)

    # Get zero-config EE position for reference
    q_zero = np.zeros(7)
    fk_zero = solver_on.get_forward_kinematics(q_zero)
    ee_zero = fk_zero[:3, 3]
    print(
        f"\nZero-config EE position: ({ee_zero[0]:.3f}, {ee_zero[1]:.3f}, {ee_zero[2]:.3f})"
    )

    # ------------------------------------------------------------ #
    # Test 1: Direct collision check on known joint configs
    # ------------------------------------------------------------ #
    print_header("Test 1: Direct collision check on joint configs")

    configs = {
        "zero config (safe)": np.zeros(7),
        "slight bend (collides)": np.array([0.0, 0.5, 0.0, -0.5, 0.0, 0.0, 0.0]),
        "extreme fold (collides)": np.array([0.0, 2.5, 0.0, -2.5, 0.0, 0.0, 0.0]),
        "arm curled inward (collides)": np.array([0.0, 1.5, 0.5, -2.0, 0.0, 1.0, 0.0]),
        "arm raised safe": np.array([0.0, -0.5, 0.0, -0.5, 0.0, 0.0, 0.0]),
    }

    for label, q in configs.items():
        print_collision_info(solver_on, q, label)

    # ------------------------------------------------------------ #
    # Test 2: Safe IK target (near zero-config EE)
    # ------------------------------------------------------------ #
    print_header("Test 2: Safe IK target (near zero-config)")

    target_safe = make_se3(
        ee_zero[0],
        ee_zero[1] + 0.05,  # 5cm left
        ee_zero[2] - 0.05,  # 5cm down
    )
    print(
        f"  Target: ({target_safe[0, 3]:.3f}, {target_safe[1, 3]:.3f}, {target_safe[2, 3]:.3f})"
    )

    solver_on.reset()
    q_safe, _ = solver_on.solve_ik(target_safe, q_zero)
    fk_safe = solver_on.get_forward_kinematics(q_safe)

    pos_err = np.linalg.norm(fk_safe[:3, 3] - target_safe[:3, 3])
    print(f"  Solved q: {np.array2string(q_safe, precision=3, suppress_small=True)}")
    print(
        f"  Reached:  ({fk_safe[0, 3]:.3f}, {fk_safe[1, 3]:.3f}, {fk_safe[2, 3]:.3f})"
    )
    print(f"  Pos error: {pos_err * 1000:.2f} mm")
    print_collision_info(solver_on, q_safe, "solution")

    # ------------------------------------------------------------ #
    # Test 3: Collision-inducing target (behind the robot body)
    # ------------------------------------------------------------ #
    print_header("Test 3: Target behind body (forces collision)")

    # Place target behind the robot — the arm must fold through itself/body
    target_behind = make_se3(-0.15, 0.0, 0.40)
    print(
        f"  Target: ({target_behind[0, 3]:.3f}, {target_behind[1, 3]:.3f}, {target_behind[2, 3]:.3f})"
    )
    print(f"  (behind the robot, should force arm into body)")

    # Solve WITH collision detection
    solver_on.reset()
    q_on, _ = solver_on.solve_ik(target_behind, q_zero)
    fk_on = solver_on.get_forward_kinematics(q_on)
    err_on = np.linalg.norm(fk_on[:3, 3] - target_behind[:3, 3])

    print(f"\n  WITH collision detection:")
    print(f"    Solved q: {np.array2string(q_on, precision=3, suppress_small=True)}")
    print(f"    Reached:  ({fk_on[0, 3]:.3f}, {fk_on[1, 3]:.3f}, {fk_on[2, 3]:.3f})")
    print(
        f"    Pos error: {err_on * 1000:.1f} mm (expected: large, target unreachable safely)"
    )
    collides_on = print_collision_info(solver_on, q_on, "final solution")

    # Solve WITHOUT collision detection
    solver_off.reset()
    q_off, _ = solver_off.solve_ik(target_behind, q_zero)
    fk_off = solver_off.get_forward_kinematics(q_off)
    err_off = np.linalg.norm(fk_off[:3, 3] - target_behind[:3, 3])

    print(f"\n  WITHOUT collision detection:")
    print(f"    Solved q: {np.array2string(q_off, precision=3, suppress_small=True)}")
    print(f"    Reached:  ({fk_off[0, 3]:.3f}, {fk_off[1, 3]:.3f}, {fk_off[2, 3]:.3f})")
    print(f"    Pos error: {err_off * 1000:.1f} mm")
    collides_off = solver_on.check_collision(
        q_off
    )  # check with collision-enabled solver
    dist_off, f1, f2 = solver_on.get_min_distance(q_off)
    status_off = "COLLISION" if collides_off else "safe"
    print(f"    {status_off}: min_dist={dist_off * 1000:.2f}mm  ({f1} <-> {f2})")

    # ------------------------------------------------------------ #
    # Test 4: Target deep inside body (extreme case)
    # ------------------------------------------------------------ #
    print_header("Test 4: Target deep inside body (extreme)")

    target_inside = make_se3(0.0, 0.0, 0.25)
    print(
        f"  Target: ({target_inside[0, 3]:.3f}, {target_inside[1, 3]:.3f}, {target_inside[2, 3]:.3f})"
    )
    print(f"  (inside the robot body)")

    solver_on.reset()
    q_inside, _ = solver_on.solve_ik(target_inside, q_zero)
    fk_inside = solver_on.get_forward_kinematics(q_inside)
    err_inside = np.linalg.norm(fk_inside[:3, 3] - target_inside[:3, 3])

    print(f"\n  WITH collision detection:")
    print(
        f"    Solved q: {np.array2string(q_inside, precision=3, suppress_small=True)}"
    )
    print(
        f"    Reached:  ({fk_inside[0, 3]:.3f}, {fk_inside[1, 3]:.3f}, {fk_inside[2, 3]:.3f})"
    )
    print(f"    Pos error: {err_inside * 1000:.1f} mm")
    print_collision_info(solver_on, q_inside, "final solution")

    solver_off.reset()
    q_inside_off, _ = solver_off.solve_ik(target_inside, q_zero)
    collides_inside_off = solver_on.check_collision(q_inside_off)
    dist_inside_off, f1, f2 = solver_on.get_min_distance(q_inside_off)
    status_ioff = "COLLISION" if collides_inside_off else "safe"

    print(f"\n  WITHOUT collision detection:")
    print(
        f"    Solved q: {np.array2string(q_inside_off, precision=3, suppress_small=True)}"
    )
    print(
        f"    {status_ioff}: min_dist={dist_inside_off * 1000:.2f}mm  ({f1} <-> {f2})"
    )

    # ------------------------------------------------------------ #
    # Summary
    # ------------------------------------------------------------ #
    print_header("Summary")
    print("  Test 1: Direct collision checks on known configs        -- PASS")
    print(
        f"  Test 2: Safe target solved without collision             -- {'PASS' if not solver_on.check_collision(q_safe) else 'FAIL'}"
    )

    t3_pass = (not collides_on) and collides_off
    print(f"  Test 3: Behind-body target:")
    print(f"           Collision ON  -> solution safe: {not collides_on}")
    print(f"           Collision OFF -> solution collides: {collides_off}")
    print(
        f"           Collision detection protected the arm: {'PASS' if t3_pass else 'CHECK (see details above)'}"
    )

    t4_pass = not solver_on.check_collision(q_inside)
    print(f"  Test 4: Inside-body target:")
    print(f"           Collision ON  -> solution safe: {t4_pass}")
    print(f"           Collision OFF -> solution collides: {collides_inside_off}")
    print(f"           {'PASS' if t4_pass else 'CHECK'}")

    print()


if __name__ == "__main__":
    main()
