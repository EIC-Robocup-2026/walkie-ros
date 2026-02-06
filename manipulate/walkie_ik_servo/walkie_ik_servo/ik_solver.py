"""Inverse Kinematics solver for Walkie single-arm robot using Pinocchio.

Adapted from the dual-arm reference implementation for single left arm only.
Uses Jacobian-based differential IK with damped least squares for real-time
servo-rate solving. Includes Pinocchio-based self-collision detection with
step rejection to prevent arm-body and arm-arm collisions.
"""

import os
import logging
from typing import Optional, Tuple

import numpy as np
import pinocchio as pin

from walkie_ik_servo.weighted_moving_filter import WeightedMovingFilter

logger = logging.getLogger(__name__)

# Default IK configuration
DEFAULT_IK_CONFIG = {
    # End-effector frame name in the URDF
    "ee_frame": "left_gripper_center",
    # Active arm joint names (the 7 DOF we solve for)
    "arm_joint_names": [
        "left_joint1",
        "left_joint2",
        "left_joint3",
        "left_joint4",
        "left_joint5",
        "left_joint6",
        "left_joint7",
    ],
    # Task-space error weights [translation_weight, rotation_weight]
    # Translation weight is applied to the 3 position error components,
    # rotation weight to the 3 orientation error components.
    "ik_translation_weight": 1.0,
    "ik_rotation_weight": 0.3,
    # Damped least squares damping factor (lambda).
    # Higher = more stable near singularities, but slower convergence.
    # Lower = faster convergence, but can be unstable near singularities.
    "ik_damping": 1e-2,
    # Step size for each IK iteration (0 < step <= 1).
    # 1.0 = full Newton step. Lower values are more conservative.
    "ik_step_size": 1.0,
    # Maximum number of iterations per solve call.
    "ik_max_iterations": 20,
    # Convergence tolerance (norm of 6D error vector)
    "ik_tolerance": 1e-3,
    # Smoothing filter weights (most recent first)
    "filter_weights": [0.4, 0.3, 0.2, 0.1],
    # Self-collision detection
    #   collision_check_enabled: whether to check for self-collisions during IK
    #   collision_safety_margin: minimum allowed distance (meters) between collision
    #     geometries. Negative values allow slight penetration (useful for approximate
    #     collision shapes). E.g., -0.005 allows up to 5mm overlap.
    #   collision_max_backtrack: max number of step-size halvings when a step causes
    #     collision. After this many rejections, the step is skipped entirely.
    "collision_check_enabled": True,
    "collision_safety_margin": -0.005,
    "collision_max_backtrack": 3,
}


class WalkieArmIKSolver:
    """Single-arm IK solver for the Walkie robot.

    Loads the full URDF, locks all non-arm joints to create a reduced model
    with only the 7 arm DOFs, then uses Jacobian-based damped least squares
    (Levenberg-Marquardt style) to solve IK from a target SE3 pose.

    Optionally performs self-collision detection using Pinocchio's collision
    geometry, rejecting IK steps that would cause arm-body or arm-arm collisions.

    The solver is designed for real-time use at servo rates (e.g. 50Hz).
    Each call to solve_ik() runs a few iterations of differential IK,
    so the solution improves over successive calls as the target is tracked.

    Args:
        urdf_path: Path to the URDF file.
        config: IK configuration dict (uses DEFAULT_IK_CONFIG if None).
    """

    def __init__(self, urdf_path: str, config: Optional[dict] = None):
        np.set_printoptions(precision=5, suppress=True, linewidth=200)

        self.config = config or DEFAULT_IK_CONFIG
        self.urdf_path = urdf_path

        # Cache collision config (needed before _load_robot_model)
        self._collision_enabled = bool(self.config.get("collision_check_enabled", True))
        self._collision_margin = float(
            self.config.get("collision_safety_margin", -0.005)
        )
        self._collision_max_bt = int(self.config.get("collision_max_backtrack", 3))

        # Load robot model and build reduced model
        self._load_robot_model()

        # Resolve EE frame ID
        ee_frame_name = self.config["ee_frame"]
        self.ee_id = self.model.getFrameId(ee_frame_name)
        if self.ee_id >= self.model.nframes:
            available = [self.model.frames[i].name for i in range(self.model.nframes)]
            raise ValueError(
                f"EE frame '{ee_frame_name}' not found. Available: {available}"
            )
        logger.info(f"EE frame: '{ee_frame_name}' (ID={self.ee_id})")

        # Cache solver parameters
        self._damping = float(self.config.get("ik_damping", 1e-2))
        self._step_size = float(self.config.get("ik_step_size", 1.0))
        self._max_iter = int(self.config.get("ik_max_iterations", 20))
        self._tolerance = float(self.config.get("ik_tolerance", 1e-3))
        self._w_trans = float(self.config.get("ik_translation_weight", 1.0))
        self._w_rot = float(self.config.get("ik_rotation_weight", 0.3))

        # Build the 6x6 task-space weight matrix
        self._W = np.diag(
            [
                self._w_trans,
                self._w_trans,
                self._w_trans,
                self._w_rot,
                self._w_rot,
                self._w_rot,
            ]
        )

        # Initialize state
        self.num_arm_joints = self.model.nq
        self.init_data = np.zeros(self.num_arm_joints)
        self.smooth_filter = WeightedMovingFilter(
            self.config.get("filter_weights", [0.4, 0.3, 0.2, 0.1]),
            self.num_arm_joints,
        )

        logger.info(
            f"WalkieArmIKSolver initialized: {self.num_arm_joints} DOF, "
            f"damping={self._damping}, max_iter={self._max_iter}, "
            f"tol={self._tolerance}, "
            f"collision={'ON' if self._collision_enabled else 'OFF'}"
        )

    # ------------------------------------------------------------------ #
    # Model loading
    # ------------------------------------------------------------------ #

    def _load_robot_model(self) -> None:
        """Load URDF and build reduced model with only arm joints active.

        If collision checking is enabled, also loads collision geometry from
        the URDF and registers relevant collision pairs (arm-vs-body and
        arm-vs-arm, skipping quasi-adjacent pairs).
        """
        logger.info(f"Loading URDF: {self.urdf_path}")

        # Build full model from URDF
        full_model = pin.buildModelFromUrdf(self.urdf_path)
        logger.info(f"Full model: {full_model.njoints} joints, nq={full_model.nq}")

        # Load collision geometry if enabled
        full_collision_model = None
        if self._collision_enabled:
            full_collision_model = pin.buildGeomFromUrdf(
                full_model, self.urdf_path, pin.GeometryType.COLLISION
            )
            logger.info(
                f"Loaded collision geometry: {full_collision_model.ngeoms} objects"
            )

        # Identify which joints to lock (everything except arm joints)
        arm_joint_names = set(self.config["arm_joint_names"])
        joints_to_lock = []
        for i in range(1, full_model.njoints):  # Skip universe joint (0)
            joint_name = full_model.names[i]
            if joint_name not in arm_joint_names:
                joints_to_lock.append(i)

        logger.info(
            f"Locking {len(joints_to_lock)} joints, "
            f"keeping {len(arm_joint_names)} arm joints"
        )

        # Build reference configuration (needed for continuous joints)
        ref_config = np.zeros(full_model.nq)
        for i in range(1, full_model.njoints):
            j = full_model.joints[i]
            if j.nq == 2:  # Continuous joint uses (cos, sin) representation
                ref_config[j.idx_q] = 1.0  # cos(0) = 1
                ref_config[j.idx_q + 1] = 0.0  # sin(0) = 0

        # Build reduced model
        if self._collision_enabled and full_collision_model is not None:
            self.model, [self.collision_model] = pin.buildReducedModel(
                full_model,
                list_of_geom_models=[full_collision_model],
                list_of_joints_to_lock=joints_to_lock,
                reference_configuration=ref_config,
            )
            self.data = self.model.createData()

            # Register collision pairs and create collision data
            self._setup_collision_pairs()
            self.collision_data = pin.GeometryData(self.collision_model)

            logger.info(
                f"Collision checking enabled: "
                f"{len(self.collision_model.collisionPairs)} pairs, "
                f"margin={self._collision_margin}m"
            )
        else:
            self.model, _ = pin.buildReducedModel(
                full_model,
                list_of_geom_models=[],
                list_of_joints_to_lock=joints_to_lock,
                reference_configuration=ref_config,
            )
            self.data = self.model.createData()
            self.collision_model = None
            self.collision_data = None

        logger.info(
            f"Reduced model: {self.model.njoints} joints, "
            f"nq={self.model.nq}, nv={self.model.nv}"
        )

    def _setup_collision_pairs(self) -> None:
        """Register collision pairs between arm and body/other arm links.

        Strategy:
          - Arm links (parentJoint > 0) vs body/fixed links (parentJoint == 0)
            - Skip quasi-adjacent pairs:
              - Arm joints 1,2 vs left_arm_base (physically connected through mount)
              - Arm joint 1 vs base_link (adjacent through arm base mount)
          - Arm links vs arm links (non-adjacent, joint gap >= 2)
        """
        arm_ids = []
        body_ids = []
        for i, go in enumerate(self.collision_model.geometryObjects):
            if go.parentJoint > 0:
                arm_ids.append(i)
            else:
                body_ids.append(i)

        logger.info(
            f"Collision geometry: {len(arm_ids)} arm, {len(body_ids)} body/fixed"
        )

        # Arm-vs-body pairs (filtered)
        count_avb = 0
        for a in arm_ids:
            ga = self.collision_model.geometryObjects[a]
            for b in body_ids:
                gb = self.collision_model.geometryObjects[b]
                frame_name = self.model.frames[gb.parentFrame].name

                # Skip quasi-adjacent: arm joints 1,2 vs left_arm_base
                if frame_name == "left_arm_base" and ga.parentJoint <= 2:
                    continue
                # Skip quasi-adjacent: arm joint 1 vs base_link
                if frame_name == "base_link" and ga.parentJoint == 1:
                    continue

                self.collision_model.addCollisionPair(pin.CollisionPair(a, b))
                count_avb += 1

        # Arm-vs-arm pairs (non-adjacent, joint gap >= 2)
        count_ava = 0
        for i_idx, a in enumerate(arm_ids):
            for j_idx, b in enumerate(arm_ids):
                if a >= b:
                    continue
                ga = self.collision_model.geometryObjects[a]
                gb = self.collision_model.geometryObjects[b]
                if abs(ga.parentJoint - gb.parentJoint) <= 1:
                    continue
                self.collision_model.addCollisionPair(pin.CollisionPair(a, b))
                count_ava += 1

        logger.info(
            f"Registered collision pairs: "
            f"{count_avb} arm-vs-body + {count_ava} arm-vs-arm = "
            f"{count_avb + count_ava} total"
        )

    # ------------------------------------------------------------------ #
    # Collision checking
    # ------------------------------------------------------------------ #

    def check_collision(self, q: np.ndarray) -> bool:
        """Check if configuration q causes self-collision.

        Uses distance-based checking with a safety margin rather than binary
        collision detection, allowing for slight penetration due to approximate
        collision geometry (cylinders/boxes).

        Args:
            q: Joint configuration array of shape (nq,).

        Returns:
            True if any collision pair distance < safety_margin.
        """
        if not self._collision_enabled or self.collision_model is None:
            return False

        pin.computeDistances(
            self.model,
            self.data,
            self.collision_model,
            self.collision_data,
            q,
        )
        for dr in self.collision_data.distanceResults:
            if dr.min_distance < self._collision_margin:
                return True
        return False

    def get_min_distance(self, q: np.ndarray) -> Tuple[float, str, str]:
        """Get the minimum distance between any registered collision pair.

        Useful for diagnostics and debugging collision detection.

        Args:
            q: Joint configuration array of shape (nq,).

        Returns:
            Tuple of (min_distance, frame_name_1, frame_name_2).
            Returns (inf, "", "") if collision checking is disabled.
        """
        if not self._collision_enabled or self.collision_model is None:
            return (float("inf"), "", "")

        pin.computeDistances(
            self.model,
            self.data,
            self.collision_model,
            self.collision_data,
            q,
        )

        min_dist = float("inf")
        min_f1 = ""
        min_f2 = ""
        for pair, dr in zip(
            self.collision_model.collisionPairs,
            self.collision_data.distanceResults,
        ):
            if dr.min_distance < min_dist:
                min_dist = dr.min_distance
                g1 = self.collision_model.geometryObjects[pair.first]
                g2 = self.collision_model.geometryObjects[pair.second]
                min_f1 = self.model.frames[g1.parentFrame].name
                min_f2 = self.model.frames[g2.parentFrame].name

        return (min_dist, min_f1, min_f2)

    # ------------------------------------------------------------------ #
    # IK solving
    # ------------------------------------------------------------------ #

    def _compute_6d_error(self, q: np.ndarray, target_se3: pin.SE3) -> np.ndarray:
        """Compute the 6D task-space error between current and target EE pose.

        Returns a 6D vector [position_error(3), orientation_error(3)] in the
        world-aligned frame (LOCAL_WORLD_ALIGNED convention).

        Args:
            q: Current joint configuration.
            target_se3: Target end-effector pose as pin.SE3.

        Returns:
            6D error vector (3 translation + 3 rotation).
        """
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        current_se3 = self.data.oMf[self.ee_id]

        # Position error: target - current (in world frame)
        pos_error = target_se3.translation - current_se3.translation

        # Orientation error: log3 of relative rotation (in world frame)
        # R_err = R_target * R_current^T  => we want current to rotate to target
        R_err = target_se3.rotation @ current_se3.rotation.T
        rot_error = pin.log3(R_err)

        return np.concatenate([pos_error, rot_error])

    def solve_ik(
        self,
        target_pose: np.ndarray,
        current_q: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Solve inverse kinematics for the left arm.

        Uses iterative Jacobian-based damped least squares (DLS). Starting from
        current_q (or the previous solution), iterates until convergence or
        max_iterations is reached.

        When collision checking is enabled, each step is validated against
        self-collision. If a step causes collision, the step size is halved and
        retried (up to collision_max_backtrack times). If all retries fail, the
        step is skipped entirely for that iteration.

        Args:
            target_pose: 4x4 SE3 homogeneous transformation matrix for the
                         end-effector target pose.
            current_q: Current joint positions for initialization. If None,
                       uses the previous solution (warm-start).

        Returns:
            Tuple of (joint_positions, joint_torques):
                - joint_positions: Array of shape (nq,) with solved joint angles
                - joint_torques: Array of shape (nv,) with gravity compensation torques
        """
        # Convert target to pin.SE3
        target_se3 = pin.SE3(target_pose[:3, :3], target_pose[:3, 3])

        # Initialize from current state or previous solution
        if current_q is not None:
            q = current_q.copy()
        else:
            q = self.init_data.copy()

        # Cache limits for clamping
        q_lower = self.model.lowerPositionLimit
        q_upper = self.model.upperPositionLimit

        # Damping matrix: lambda^2 * I
        damping_matrix = self._damping**2 * np.eye(6)

        for i in range(self._max_iter):
            # Compute 6D error
            err = self._compute_6d_error(q, target_se3)

            # Check convergence (weighted error norm)
            weighted_err = self._W @ err
            err_norm = np.linalg.norm(weighted_err)
            if err_norm < self._tolerance:
                logger.debug(f"IK converged in {i + 1} iterations (err={err_norm:.6f})")
                break

            # Compute frame Jacobian in LOCAL_WORLD_ALIGNED frame
            # This gives J such that v_ee = J * dq, where v_ee is
            # [linear_vel(3), angular_vel(3)] in world-aligned frame
            pin.computeJointJacobians(self.model, self.data, q)
            J = pin.getFrameJacobian(
                self.model, self.data, self.ee_id, pin.LOCAL_WORLD_ALIGNED
            )

            # Apply task-space weights to Jacobian and error
            Jw = self._W @ J
            ew = weighted_err

            # Damped least squares: dq = Jw^T (Jw Jw^T + lambda^2 I)^{-1} * ew
            # This is the damped pseudo-inverse solution
            JwJwT = Jw @ Jw.T
            dq = Jw.T @ np.linalg.solve(JwJwT + damping_matrix, ew)

            # Apply step size and update, with collision rejection
            if self._collision_enabled:
                step = self._step_size
                q_prev = q.copy()
                accepted = False
                for bt in range(self._collision_max_bt + 1):
                    q_candidate = np.clip(q_prev + step * dq, q_lower, q_upper)
                    if not self.check_collision(q_candidate):
                        q = q_candidate
                        accepted = True
                        break
                    step *= 0.5  # halve step size
                    logger.debug(
                        f"Collision at iter {i}, backtrack {bt + 1}, step={step:.4f}"
                    )
                if not accepted:
                    logger.warning(
                        f"IK iter {i}: all {self._collision_max_bt + 1} backtracks "
                        f"failed, skipping step"
                    )
            else:
                q = q + self._step_size * dq
                q = np.clip(q, q_lower, q_upper)

        else:
            logger.debug(
                f"IK did not converge after {self._max_iter} iterations "
                f"(err={err_norm:.6f})"
            )

        # Apply smoothing filter
        self.smooth_filter.add_data(q)
        sol_q = self.smooth_filter.filtered_data.copy()

        # Clamp again after filtering (filter can push past limits)
        sol_q = np.clip(sol_q, q_lower, q_upper)

        # Final collision check on smoothed result
        if self._collision_enabled and self.check_collision(sol_q):
            logger.warning(
                "Smoothed IK result has collision, using unsmoothed solution"
            )
            sol_q = np.clip(q, q_lower, q_upper)

        # Update state for next call warm-start
        self.init_data = sol_q.copy()

        # Compute gravity compensation torques via RNEA
        v = np.zeros(self.model.nv)
        sol_tauff = pin.rnea(self.model, self.data, sol_q, v, np.zeros(self.model.nv))

        return sol_q, sol_tauff

    def get_forward_kinematics(self, q: np.ndarray) -> np.ndarray:
        """Compute forward kinematics for given joint configuration.

        Args:
            q: Joint configuration array of shape (nq,)

        Returns:
            4x4 SE3 homogeneous transformation matrix of the end-effector.
        """
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.ee_id].homogeneous.copy()

    def reset(self) -> None:
        """Reset solver state to initial (zero) configuration."""
        self.init_data = np.zeros(self.num_arm_joints)
        self.smooth_filter.reset()
