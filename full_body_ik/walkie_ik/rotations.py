"""
Rotation helpers (numpy), copied from the static methods of walker_robot/IK/forward_kinematics.py
(Coordinated-Base-Arm-Motion-Control), which needs sympy (not in the walkie pixi env).

Orientation convention: roll/pitch/yaw, ZYX: R = Rz(yaw) @ Ry(pitch) @ Rx(roll), radians.
"""
import numpy as np


def euler_ZYX_to_rotmat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def rotmat_to_euler_ZYX(R: np.ndarray) -> np.ndarray:
    """Returns [roll, pitch, yaw] in radians."""
    return np.array([
        np.arctan2(R[2, 1], R[2, 2]),
        np.arctan2(-R[2, 0], np.hypot(R[0, 0], R[1, 0])),
        np.arctan2(R[1, 0], R[0, 0]),
    ])


def rotmat_to_quat(R: np.ndarray) -> np.ndarray:
    """Shepperd's method. Returns [qw, qx, qy, qz]."""
    t = R[0, 0] + R[1, 1] + R[2, 2]
    k = int(np.argmax([t, R[0, 0], R[1, 1], R[2, 2]]))  # which term is largest

    if k == 0:
        s = 2.0 * np.sqrt(1.0 + t)
        q = [s / 4, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    elif k == 1:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        q = [(R[2, 1] - R[1, 2]) / s, s / 4, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s]
    elif k == 2:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        q = [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, s / 4, (R[1, 2] + R[2, 1]) / s]
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        q = [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, s / 4]

    q = np.array(q, dtype=float)
    return q / np.linalg.norm(q)


def rotation_error(R_target: np.ndarray, R_current: np.ndarray) -> np.ndarray:
    """
    Rotation vector (axis * angle, world frame) that rotates R_current onto R_target,
    i.e. log(R_target @ R_current^T). Always takes the short way round (angle <= pi).
    """
    q = rotmat_to_quat(R_target @ R_current.T)
    if q[0] < 0.0:  # q and -q are the same rotation; pick the one with angle <= pi
        q = -q
    s = np.linalg.norm(q[1:])
    if s < 1e-9:
        return 2.0 * q[1:]  # small-angle limit
    return (2.0 * np.arctan2(s, q[0]) / s) * q[1:]


def yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    return float(np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)))
