"""Copied from walker_robot/sim/drag_targets.py (Coordinated-Base-Arm-Motion-Control)."""
import mujoco
import numpy as np

# an arm is dragged by any body from this one down (wrist link, hand, fingers)
DRAG_ROOT_BODIES = {"left": "openarm_left_link7", "right": "openarm_right_link7"}


class DragTargets:
    """
    Move the end-effector targets by dragging an arm's hand in the MuJoCo viewer, with the
    viewer's own perturbation controls:
        double-click the hand     select it
        Ctrl + left drag          rotate the target's orientation
        Ctrl + right drag         move the target's position (+ Shift: the other plane)

    The drag's motion (viewer.perturb refpos / refquat) is applied to the target as it was when
    the drag started: a translation moves the target position, a rotation turns its orientation
    about the target itself (in the world frame). So the target does not jump to the hand when a
    drag starts, and the IK model's end-effector convention does not matter.
    """

    def __init__(self, model: mujoco.MjModel):
        roots = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name): side
                 for side, name in DRAG_ROOT_BODIES.items()}
        self._side = {}                     # body id -> arm it drags
        for body in range(model.nbody):
            b = body
            while b > 0 and b not in roots:
                b = model.body_parentid[b]
            if b in roots:
                self._side[body] = roots[b]
        self._anchor = None                 # (select, active, ref pos, ref R, target pos, target R)

    def update(self, perturb, current_target):
        """
        Call every cycle (inside `with viewer.lock():`).
        current_target(side) -> (pos [3], R [3x3]): that arm's target in the world frame now;
        only called when a drag starts.
        Returns {side: (pos, R)}, the dragged arm's new world-frame target, or None if no hand
        is being dragged.
        """
        side = self._side.get(int(perturb.select))
        if side is None or not perturb.active:
            self._anchor = None
            return None
        ref_pos = np.array(perturb.refpos, float)
        ref_R = np.zeros(9)
        mujoco.mju_quat2Mat(ref_R, np.array(perturb.refquat, float))
        ref_R = ref_R.reshape(3, 3)
        key = (int(perturb.select), int(perturb.active))
        if self._anchor is None or self._anchor[:2] != key:
            self._anchor = key + (ref_pos, ref_R) + tuple(np.array(v, float) for v in current_target(side))
            return None
        _, _, p0, R0, target_pos, target_R = self._anchor
        return {side: (target_pos + (ref_pos - p0), ref_R @ R0.T @ target_R)}
