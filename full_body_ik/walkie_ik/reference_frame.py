"""
Copied from walker_robot/reference_frame.py (Coordinated-Base-Arm-Motion-Control); rotation helpers from rotations.py.

Target reference frames: world (absolute) vs. robot base (relative).

The IK (WalkieIK in full_body_ik.py) always works with world-frame (= map) targets. These helpers convert
end-effector poses between the world frame and the base frame, so targets can be given relative
to the robot and converted to world just before each ik.step().

Base frame: origin at (x, y, 0) on the floor, rotated by `yaw` about the world z axis, i.e. the
mobile base's pose. Lift is not included: it is a joint the targets are reached with, so a
relative target keeps its height above the floor when the lift moves.

Poses are (pos [3] in metres, rpy [3] in radians, ZYX like the IK).
"""
import dataclasses

import numpy as np

from . import rotations

WORLD = "world"
BASE = "base"
FRAMES = (WORLD, BASE)


def base_pose(joint_state: dict):
    """(origin [3], R [3x3]) of the base frame in the world frame."""
    c, s = np.cos(joint_state["yaw"]), np.sin(joint_state["yaw"])
    R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return np.array([joint_state["x"], joint_state["y"], 0.0]), R


def base_to_world(pos, rpy, joint_state: dict):
    p_base, R_base = base_pose(joint_state)
    R = R_base @ rotations.euler_ZYX_to_rotmat(*rpy)
    return p_base + R_base @ np.asarray(pos, float), rotations.rotmat_to_euler_ZYX(R)


def world_to_base(pos, rpy, joint_state: dict):
    p_base, R_base = base_pose(joint_state)
    R = R_base.T @ rotations.euler_ZYX_to_rotmat(*rpy)
    return R_base.T @ (np.asarray(pos, float) - p_base), rotations.rotmat_to_euler_ZYX(R)


def convert_pose(pos, rpy, from_frame: str, to_frame: str, joint_state: dict):
    """Re-express a pose given in `from_frame` in `to_frame` (both in FRAMES)."""
    if from_frame == to_frame:
        return np.asarray(pos, float), np.asarray(rpy, float)
    if to_frame == WORLD:
        return base_to_world(pos, rpy, joint_state)
    return world_to_base(pos, rpy, joint_state)


def targets_to_world(targets: dict, frame: str, joint_state: dict) -> dict:
    """{side: Target} given in `frame` -> the same targets in the world frame, for ik.step()."""
    if frame == WORLD:
        return targets
    out = {}
    for side, t in targets.items():
        rpy = np.zeros(3) if t.rpy is None else t.rpy
        pos, rpy = base_to_world(t.pos, rpy, joint_state)
        out[side] = dataclasses.replace(t, pos=pos, rpy=None if t.rpy is None else rpy)
    return out
