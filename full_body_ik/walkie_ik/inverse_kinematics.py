# Copied from walker_robot/IK/inverse_kinematics.py (Coordinated-Base-Arm-Motion-Control); only this import changed.
# The FK objects passed in are walkie_model.ChainFK (MuJoCo-backed) instead of the sympy ForwardKinematics.
import numpy as np


class InverseKinematics:
    def __init__(self, fk=None):
        self.fk = fk

    @staticmethod
    def _limit_weights(q, dq, lim, margin, gain):
        """
        Per-joint cost multiplier w_i >= 1.

        w_i == 1 when joint i is moving away from its nearest limit, or is
        still outside the margin zone. Inside the zone (the last `margin`
        fraction of the range) and moving TOWARD that limit, w_i grows
        smoothly and blows up as q_i reaches the limit.
        Infinite limits (e.g. a base x/y axis) are never penalised.
        """
        lo, hi = lim[:, 0], lim[:, 1]
        rng = hi - lo
        finite = np.isfinite(rng) & (rng > 0)
        safe_rng = np.where(finite, rng, 1.0)

        # normalised distance (0..1) to the limit this joint is heading toward
        dist = np.where(dq >= 0.0, hi - q, q - lo) / safe_rng
        dist = np.where(finite, dist, np.inf)

        p = np.clip((margin - dist) / margin, 0.0, 1.0)  # 0 = far, 1 = at limit
        return 1.0 + gain * p ** 2 / (1.0 - p + 1e-3)


    def solve_ik_stacked(
        self,
        current_joint_state: dict,
        chains,
        all_symbols,
        joint_limits,
        damped_factor=0.2,
        margin: float = 0.15,
        gain: float = 50.0,
        n_refine: int = 2,
        desired_pos=None,
        desired_gain=0.2,
        null_damping: float = 0.01,
        orientation_weight: float = 0.5,
    ) -> np.ndarray:
        """
        Solve several kinematic chains (e.g. left arm, right arm) that share joints in ONE
        stacked system, with the desired pose as a lower-priority task in the null space.

            primary   : dq1 = (J^T J + diag(lam^2 * w))^-1 J^T e        (J, e stacked over chains;
                        3 position rows per chain, +3 orientation rows if it has a target rotation)
            secondary : dq  = dq1 + (I - J^+ J) * desired_gain * (q_desired - q)

        The projector (I - J^+ J) removes any desired-pose motion that would move an end effector,
        so the desired-pose pull can never fight the targets and leaves no steady-state error.

        Args:
            current_joint_state: dict {symbol: value} containing every symbol in all_symbols.
            chains: list of (fk, symbols, target_pos) or (fk, symbols, target_pos, target_rot).
                `symbols` is that fk's variable list, in the order of its Jacobian columns;
                `target_pos` is its [3] target position; `target_rot` is an optional (3, 3)
                target rotation matrix (base frame). Without it (or None) only position is tracked.
            all_symbols: list of the joint symbols to SOLVE for, defining the order of the output dq.
                Shared joints (e.g. x, y, yaw, lift) appear once here. A chain symbol left out
                of all_symbols is held fixed at its current_joint_state value (e.g. a locked base).
            joint_limits: (n, 2) array in all_symbols order, or dict {symbol: (lo, hi)}.
            damped_factor: scalar lambda, or length-n vector (all_symbols order).
            margin, gain, n_refine: joint-limit damping settings (see _limit_weights).
            desired_pos: dict {symbol: desired value} (symbols left out have none), or a length-n
                array with np.nan where a joint has none. None = no desired-pose pull.
            desired_gain: fraction of the gap to the desired pose closed per iteration (scalar or length-n).
            null_damping: small epsilon that smooths the null-space projector near
                singularities. Larger = smoother but lets a little desired-pose motion leak into the task.
            orientation_weight: metres of position error that 1 rad of orientation error is worth.
                Scales the orientation rows of J and e; lower = orientation gives way first.
        Returns:
            dq of length n in all_symbols order (kept within the joint limits).
        """
        n = len(all_symbols)
        index = {s: i for i, s in enumerate(all_symbols)}
        q = np.array([float(current_joint_state[s]) for s in all_symbols])

        # --- stack the Jacobians and errors; shared joints land in the same column ---
        blocks, errs = [], []
        for fk, symbols, target_pos, *rest in chains:
            target_rot = rest[0] if rest else None
            sub = {s: current_joint_state[s] for s in symbols}

            # position task: 3 rows
            Jc = np.atleast_2d(np.asarray(fk.jacobian_np(sub), dtype=float))
            ec = (np.asarray(target_pos, dtype=float).reshape(-1)
                  - np.asarray(fk.pos_np(sub), dtype=float).reshape(-1))

            # orientation task: 3 more rows. Error is the rotation vector log(R_target R^T) and
            # the Jacobian is the geometric angular one, so both live in the same (base) frame.
            if target_rot is not None:
                Jw = fk.angular_jacobian_np(sub)
                ew = fk.rotation_error(np.asarray(target_rot, dtype=float), fk.orientation_matrix_np(sub))
                Jc = np.vstack([Jc, orientation_weight * Jw])
                ec = np.concatenate([ec, orientation_weight * ew])

            # keep only the columns of joints being solved for (others are held fixed)
            keep = [k for k, s in enumerate(symbols) if s in index]
            blocks.append((Jc[:, keep], [index[symbols[k]] for k in keep]))
            errs.append(ec)
        J = np.zeros((sum(Jc.shape[0] for Jc, _ in blocks), n))
        r0 = 0
        for Jc, cols in blocks:
            J[r0:r0 + Jc.shape[0], cols] = Jc
            r0 += Jc.shape[0]
        e = np.concatenate(errs)

        if isinstance(joint_limits, dict):
            lim = np.array([joint_limits[s] for s in all_symbols], dtype=float)
        else:
            lim = np.asarray(joint_limits, dtype=float)
        if lim.shape != (n, 2):
            raise ValueError(f"joint_limits must have shape ({n}, 2), got {lim.shape}")

        # --- primary task: DLS with joint-limit weights (Selective Damped Least Squares) ---
        lam2 = np.broadcast_to(np.asarray(damped_factor, dtype=float) ** 2, (n,))
        JtJ, Jte = J.T @ J, J.T @ e

        def solve(w):
            return np.linalg.solve(JtJ + np.diag(lam2 * w), Jte)

        dq = solve(np.ones(n))
        for _ in range(n_refine):
            dq = solve(self._limit_weights(q, dq, lim, margin, gain))

        # --- secondary task: pull toward the desired pose, projected into the null space of J ---
        if desired_pos is not None:
            if isinstance(desired_pos, dict):
                q_desired = np.array([desired_pos.get(s, np.nan) for s in all_symbols], dtype=float)
            else:
                q_desired = np.asarray(desired_pos, dtype=float)
            if q_desired.shape != (n,):
                raise ValueError(f"desired_pos must have length {n}, got {q_desired.shape}")
            k = np.broadcast_to(np.asarray(desired_gain, dtype=float), (n,))
            want = np.where(np.isfinite(q_desired), k * (q_desired - q), 0.0)

            _, s, Vt = np.linalg.svd(J, full_matrices=False)
            P = (Vt.T * (s ** 2 / (s ** 2 + null_damping ** 2))) @ Vt   # soft projector onto row space
            dq = dq + (np.eye(n) - P) @ want

        # safety net: never step past a hard limit
        return np.clip(q + dq, lim[:, 0], lim[:, 1]) - q
