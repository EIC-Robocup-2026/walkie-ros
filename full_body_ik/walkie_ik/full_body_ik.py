"""
Walkie full-body IK: BimanualIK of walker_robot/bimanual_ik.py, on the walkie model
(walkie_model.py) instead of the DH tables.

Conventions (as in bimanual_ik.py):
    - joint_state / joint_command: dict {joint name: value}, names as in JOINT_NAMES,
      radians for revolute joints, metres for prismatic (x, y, lift).
    - Targets are in the map frame. Orientation is roll/pitch/yaw, ZYX, radians.
    - The end-effector frame is the openarm_*_hand_tcp body of the walkie MJCF.
"""
from dataclasses import dataclass

import numpy as np

from . import rotations
from .inverse_kinematics import InverseKinematics
from .walkie_model import ARM_JOINTS, BASE_JOINTS, JOINT_NAMES, WalkieModel

# base joints the IK may move, toggled per group: planar position, heading, torso height
BASE_GROUPS = {"xy": ["x", "y"], "yaw": ["yaw"], "lift": ["lift"]}

# Rest posture of the arms (null-space desired pose), like a chef's: elbows bent 90 deg (joint 4),
# upper arms ~16 deg out from the body so the elbows sit ~7 cm out (joint 2), forearms forward and
# ~11 deg inward (joint 3), wrists neutral. The walkie arms are not exact mirror images (both elbows
# sit 3 cm towards -y at zero), so left and right differ in joints 2 and 3 to give a symmetric pose.
ARM_REST_POSE = {
    "q_l1": 0.0, "q_l2": -0.37, "q_l3": 0.28, "q_l4": np.pi / 2, "q_l5": 0.0, "q_l6": 0.0, "q_l7": 0.0,
    "q_r1": 0.0, "q_r2": 0.14, "q_r3": -0.12, "q_r4": np.pi / 2, "q_r5": 0.0, "q_r6": 0.0, "q_r7": 0.0,
}


@dataclass
class Target:
    """End-effector target in the map frame."""
    pos: np.ndarray                    # [x, y, z] in metres
    rpy: np.ndarray | None = None      # [roll, pitch, yaw] in radians (ZYX). None = position only


class WalkieIK:
    """
    Usage, once per control cycle:
        joint_command = ik.step(joint_state, {"left": Target(...), "right": Target(...)})
    Each call takes ONE damped-least-squares step from the current state towards the targets,
    so call it repeatedly and feed each command back in. An arm left out of `targets` is held still.
    """

    ARM_DAMPING = 0.2           # DLS damping for arm joints (higher = slower but safer near singularities)
    BASE_DAMPING = 0.6          # higher damping => the arms do the work, the base only helps when needed
    ORIENTATION_WEIGHT = 0.5    # metres of position error that 1 rad of orientation error is worth
    MAX_JOINT_STEP = 0.05       # rad (or m) per call; caps speed when a target jumps far away
    DESIRED_GAIN = 0.05         # null-space pull of the base towards its desired pose, per call
    ARM_REST_GAIN = 0.05        # null-space pull of the arms towards ARM_REST_POSE, per call (0 = off)

    def __init__(self, walkie: WalkieModel):
        self.walkie = walkie
        self.joint_limits = walkie.joint_limits
        self.arms = {side: (walkie.chain(side), ARM_JOINTS[side]) for side in ARM_JOINTS}
        self.solver = InverseKinematics()

    def step(self, joint_state: dict, targets: dict, base_groups=(), desired_base=None) -> dict:
        """
        joint_state:  {joint name: value} for every name in JOINT_NAMES.
        targets:      {"left": Target, "right": Target}; either may be missing (that arm is held
                      still and left out of the IK entirely).
        base_groups:  which base joints may move, any subset of BASE_GROUPS ("xy", "yaw", "lift").
        desired_base: {"x", "y", "yaw": value} the base drifts back to (null space) when the arms
                      allow it, e.g. where the robot is now. None = no pull.
        The arms drift towards ARM_REST_POSE the same way, as far as the targets leave them free
        (e.g. the elbow swivel; with position-only targets, also the wrists).
        Returns the next joint command, {joint name: value} for every name in JOINT_NAMES.
        """
        chains = []
        solve_names = [n for group, names in BASE_GROUPS.items() if group in base_groups for n in names]
        for side, target in targets.items():
            fk, arm_names = self.arms[side]
            target_rot = None if target.rpy is None else rotations.euler_ZYX_to_rotmat(*target.rpy)
            chains.append((fk, BASE_JOINTS + arm_names, target.pos, target_rot))
            solve_names += arm_names
        if not chains:
            return dict(joint_state)

        damping = [self.BASE_DAMPING if n in BASE_JOINTS else self.ARM_DAMPING for n in solve_names]
        desired = {**(desired_base or {}), **ARM_REST_POSE}
        gains = [self.DESIRED_GAIN if n in BASE_JOINTS else self.ARM_REST_GAIN for n in solve_names]
        dq = self.solver.solve_ik_stacked(
            current_joint_state=joint_state,
            chains=chains,
            all_symbols=solve_names,
            joint_limits={n: self.joint_limits[n] for n in solve_names},
            damped_factor=damping,
            desired_pos=desired,
            desired_gain=gains,
            orientation_weight=self.ORIENTATION_WEIGHT,
        )

        # cap the step so a far-away target gives a smooth motion instead of a jump
        biggest = np.max(np.abs(dq))
        if biggest > self.MAX_JOINT_STEP:
            dq *= self.MAX_JOINT_STEP / biggest

        command = dict(joint_state)
        for n, d in zip(solve_names, dq):
            command[n] = joint_state[n] + d
        return command

    def forward(self, joint_state: dict) -> dict:
        """End-effector poses: {"left": (pos [3], R [3x3]), "right": (...)} in the map frame."""
        return {side: (fk.pos_np(joint_state), fk.orientation_matrix_np(joint_state))
                for side, (fk, _) in self.arms.items()}

    def errors(self, joint_state: dict, targets: dict) -> dict:
        """{side: (position error [m], orientation error [rad] or None)}."""
        out = {}
        for side, (pos, R) in self.forward(joint_state).items():
            if side not in targets:
                continue
            t = targets[side]
            rot_err = None
            if t.rpy is not None:
                R_t = rotations.euler_ZYX_to_rotmat(*t.rpy)
                rot_err = float(np.linalg.norm(rotations.rotation_error(R_t, R)))
            out[side] = (float(np.linalg.norm(t.pos - pos)), rot_err)
        return out


__all__ = ["ARM_REST_POSE", "BASE_GROUPS", "JOINT_NAMES", "Target", "WalkieIK"]
