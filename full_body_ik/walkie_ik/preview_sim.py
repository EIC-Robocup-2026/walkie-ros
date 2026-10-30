"""
Preview: a second MuJoCo viewer of the walkie model that shows the IK solution (the expected
outcome) and the targets, next to the real MuJoCo sim. Kinematic only: the joints are set
directly from the IK command, nothing is simulated (no physics, no sensors).
"""
import mujoco
import mujoco.viewer
import numpy as np

from .walkie_model import BASE_BODY, WalkieModel


class PreviewSim:
    """
    walkie: the model to show, e.g. the robot in the main sim's world (world.py). The robot is a
    ghost there: no physics runs, so it passes through walls and furniture and moves nothing; its
    geoms also get no collision classes, so it cannot interact even in principle.
    """

    def __init__(self, walkie: WalkieModel):
        self.walkie = walkie
        self.model = walkie.model
        self.data = walkie.new_data()
        robot_root = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, BASE_BODY)
        robot_geoms = self.model.body_rootid[self.model.geom_bodyid] == robot_root
        self.model.geom_contype[robot_geoms] = 0
        self.model.geom_conaffinity[robot_geoms] = 0
        # the wheel-floor <pair> contacts ignore the collision classes; this preview never drives,
        # so turn contacts off entirely (unlike the real sim, whose wheels need the pairs)
        self.model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT

    def show(self, state: dict) -> None:
        """Put the preview robot at `state` ({IK joint name: value})."""
        self.walkie.write_qpos(self.data, state)
        mujoco.mj_kinematics(self.model, self.data)
        mujoco.mj_camlight(self.model, self.data)

    def joint_frame(self, name: str):
        """(pos [3], R [3x3]) of an IK joint (lift or arm) in the preview: its anchor point,
        with the orientation of the body it moves."""
        jid = self.walkie.joint_id[name]
        body = self.model.jnt_bodyid[jid]
        return self.data.xanchor[jid].copy(), self.data.xmat[body].reshape(3, 3).copy()

    def launch_viewer(self):
        # no physics runs here, so the viewer's actuator sliders would do nothing: hide the panels
        return mujoco.viewer.launch_passive(self.model, self.data, show_left_ui=False, show_right_ui=False)

    @staticmethod
    def draw_frames(viewer, frames) -> None:
        """
        Draw coordinate frames as RGB (= x, y, z) axes into the viewer's user scene.
        frames: list of (pos, R or None, axis_length, alpha). R = None draws only a sphere.
        Call inside `with viewer.lock():`. (Same as walker_robot/sim/mujoco_sim.py.)
        """
        scn = viewer.user_scn
        scn.ngeom = 0
        for pos, R, length, alpha in frames:
            if scn.ngeom >= scn.maxgeom:
                break
            g = scn.geoms[scn.ngeom]
            mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE, np.array([0.12 * length, 0, 0]),
                                np.asarray(pos, float), np.eye(3).flatten(),
                                np.array([1.0, 0.8, 0.0, alpha], np.float32))
            scn.ngeom += 1
            if R is None:
                continue
            for axis in range(3):
                if scn.ngeom >= scn.maxgeom:
                    break
                g = scn.geoms[scn.ngeom]
                rgba = np.zeros(4, np.float32)
                rgba[axis], rgba[3] = 1.0, alpha
                mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3),
                                    np.eye(3).flatten(), rgba)
                mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, 0.06 * length,
                                     np.asarray(pos, float), np.asarray(pos, float) + length * R[:, axis])
                scn.ngeom += 1
