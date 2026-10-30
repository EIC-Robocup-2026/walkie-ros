"""
Walkie kinematic model for the IK, read from walkie_description's MJCF (the model the MuJoCo
sim runs), instead of the DH tables of walker_robot/bimanual_ik.py, whose robot has a different
arm mount and link lengths.

The IK keeps its own joint names (x, y, yaw, lift, q_l1..q_l7, q_r1..q_r7), as in
bimanual_ik.py; WALKIE_JOINTS maps them to the walkie joint names (MJCF, ros2_control, MoveIt).
x, y, yaw are the planar pose of base_footprint in the map frame: the MJCF's free joint
floating_base_joint, with the height fixed and no roll/pitch.

ChainFK has the numeric interface InverseKinematics.solve_ik_stacked() uses
(pos_np, jacobian_np, angular_jacobian_np, orientation_matrix_np, rotation_error); the pose
and Jacobian come from MuJoCo (mj_kinematics + mj_jacBody) rather than from sympy.
"""
import mujoco
import numpy as np

from . import rotations

BASE_JOINTS = ["x", "y", "yaw", "lift"]
LEFT_ARM_JOINTS = [f"q_l{i}" for i in range(1, 8)]
RIGHT_ARM_JOINTS = [f"q_r{i}" for i in range(1, 8)]
ARM_JOINTS = {"left": LEFT_ARM_JOINTS, "right": RIGHT_ARM_JOINTS}
JOINT_NAMES = BASE_JOINTS + LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS

# IK joint name -> walkie joint name (x, y, yaw have none: they are the base pose)
WALKIE_JOINTS = {"lift": "lift_joint"}
WALKIE_JOINTS.update({f"q_l{i}": f"openarm_left_joint{i}" for i in range(1, 8)})
WALKIE_JOINTS.update({f"q_r{i}": f"openarm_right_joint{i}" for i in range(1, 8)})

BASE_BODY = "base_footprint"
TCP_BODIES = {"left": "openarm_left_hand_tcp", "right": "openarm_right_hand_tcp"}
HOME_KEYFRAME = "home"

# The base has no joint limits; Nav2 and the map bound it. Lift and arm limits come from the MJCF.
BASE_LIMITS = {"x": (-np.inf, np.inf), "y": (-np.inf, np.inf), "yaw": (-np.inf, np.inf)}


class WalkieModel:
    """The walkie MJCF, and conversion between IK joint states and MuJoCo qpos."""

    def __init__(self, xml_path: str):
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        m = self.model

        base = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, BASE_BODY)
        free = next(j for j in range(m.njnt)
                    if m.jnt_bodyid[j] == base and m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE)
        self.free_qpos = int(m.jnt_qposadr[free])
        self.free_dof = int(m.jnt_dofadr[free])

        self.joint_id, self.qpos_adr, self.dof_adr, self.joint_limits = {}, {}, {}, dict(BASE_LIMITS)
        for name, walkie_name in WALKIE_JOINTS.items():
            jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, walkie_name)
            if jid < 0:
                raise KeyError(f"joint {walkie_name} not in {xml_path}")
            self.joint_id[name] = jid
            self.qpos_adr[name] = int(m.jnt_qposadr[jid])
            self.dof_adr[name] = int(m.jnt_dofadr[jid])
            self.joint_limits[name] = tuple(float(v) for v in m.jnt_range[jid])
        self.tcp_body = {side: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body)
                         for side, body in TCP_BODIES.items()}

        # home: the 'home' keyframe (or the model's qpos0), base at the map origin
        key = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, HOME_KEYFRAME)
        self.home_qpos = (m.key_qpos[key] if key >= 0 else m.qpos0).copy()
        self.base_z = float(self.home_qpos[self.free_qpos + 2])
        self.home_state = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.home_state.update({name: float(self.home_qpos[adr]) for name, adr in self.qpos_adr.items()})

    def new_data(self) -> mujoco.MjData:
        data = mujoco.MjData(self.model)
        data.qpos[:] = self.home_qpos
        return data

    def write_qpos(self, data: mujoco.MjData, state: dict) -> None:
        """Write the joints of `state` (any subset of JOINT_NAMES) into data.qpos."""
        if "x" in state:
            a = self.free_qpos
            yaw = state["yaw"]
            data.qpos[a:a + 3] = state["x"], state["y"], self.base_z
            data.qpos[a + 3:a + 7] = np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)
        for name, adr in self.qpos_adr.items():
            if name in state:
                data.qpos[adr] = state[name]

    def chain(self, side: str) -> "ChainFK":
        return ChainFK(self, side)


class ChainFK:
    """
    Base + one arm, from the map frame to the arm's hand TCP: variables BASE_JOINTS + that arm's
    joints. Same numeric interface as walker_robot's ForwardKinematics, taking {name: value}.
    """

    def __init__(self, walkie: WalkieModel, side: str):
        self.walkie = walkie
        self.side = side
        self.variables = BASE_JOINTS + ARM_JOINTS[side]
        self._data = walkie.new_data()
        self._body = walkie.tcp_body[side]
        # Jacobian columns: free-joint x, y and rotation about z (the base has no roll/pitch, so
        # the body z axis is the world z axis), then the lift and arm joints
        f = walkie.free_dof
        self._cols = [f, f + 1, f + 5] + [walkie.dof_adr[n] for n in self.variables[3:]]
        self._jacp = np.zeros((3, walkie.model.nv))
        self._jacr = np.zeros((3, walkie.model.nv))
        self._key = None

    def _update(self, values: dict) -> None:
        key = tuple(float(values[n]) for n in self.variables)
        if key == self._key:
            return
        m, d = self.walkie.model, self._data
        self.walkie.write_qpos(d, dict(zip(self.variables, key)))
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)          # mj_jacBody needs the com-based quantities
        mujoco.mj_jacBody(m, d, self._jacp, self._jacr, self._body)
        self._pos = d.xpos[self._body].copy()
        self._R = d.xmat[self._body].reshape(3, 3).copy()
        self._Jp = self._jacp[:, self._cols].copy()
        self._Jr = self._jacr[:, self._cols].copy()
        self._key = key

    def pos_np(self, values: dict) -> np.ndarray:                 # (3,)
        self._update(values)
        return self._pos.copy()

    def orientation_matrix_np(self, values: dict) -> np.ndarray:  # (3, 3)
        self._update(values)
        return self._R.copy()

    def jacobian_np(self, values: dict) -> np.ndarray:            # (3, n) linear
        self._update(values)
        return self._Jp.copy()

    def angular_jacobian_np(self, values: dict) -> np.ndarray:    # (3, n) geometric angular
        self._update(values)
        return self._Jr.copy()

    rotation_error = staticmethod(rotations.rotation_error)
