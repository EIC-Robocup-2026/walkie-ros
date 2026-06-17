#!/usr/bin/env python3
"""Derive a per-group XRDF from the bimanual XRDF.

cuMotion plans the WHOLE XRDF cspace, so to plan a specific MoveIt group we make
an XRDF whose cspace is exactly that group's joints. Joints NOT in the group stay
LOCKED (pinned at the arms-raised home), and we KEEP both arms' collision spheres
+ self_collision so the active joints still avoid the idle arm and self.

Groups (match the SRDF):
  right_arm       -> right 7
  left_arm        -> left 7
  right_arm_lift  -> lift + right 7   (8)
  left_arm_lift   -> lift + left 7    (8)
  both_arms_lift  -> lift + left 7 + right 7  (15)
(both_arms = the stock openarm_bimanual.xrdf, 14)

  .spherefit_venv/bin/python make_single_arm_xrdf.py both_arms_lift
"""
import sys
import yaml

SRC = "openarm_bimanual.xrdf"
RAISED = {"left": [0.0, -1.4, 0.0, 0.6, 0.0, 0.0, 0.0],
          "right": [0.0, 1.4, 0.0, 0.6, 0.0, 0.0, 0.0]}
LIFT_HOME = 0.7435
# Prismatic lift limits for cuRobo trajopt (URDF vel limit 0.1 m/s; m units).
LIFT_ACCEL, LIFT_JERK = 0.5, 5.0
ARM_ACCEL, ARM_JERK = 1.0, 100.0

L = [f"openarm_left_joint{i}" for i in range(1, 8)]
R = [f"openarm_right_joint{i}" for i in range(1, 8)]
GROUPS = {
    "right_arm":      (["lift_joint"] * 0 + R, False),
    "left_arm":       (L, False),
    "right_arm_lift": (["lift_joint"] + R, True),
    "left_arm_lift":  (["lift_joint"] + L, True),
    "both_arms_lift": (["lift_joint"] + L + R, True),
}


def main():
    group = sys.argv[1] if len(sys.argv) > 1 else "both_arms_lift"
    if group not in GROUPS:
        sys.exit(f"unknown group {group}; choose from {list(GROUPS)}")
    active, _has_lift = GROUPS[group]

    d = yaml.safe_load(open(SRC))

    # cspace = the group's joints, with matching accel/jerk per joint.
    d["cspace"]["joint_names"] = list(active)
    d["cspace"]["acceleration_limits"] = [
        LIFT_ACCEL if j == "lift_joint" else ARM_ACCEL for j in active]
    d["cspace"]["jerk_limits"] = [
        LIFT_JERK if j == "lift_joint" else ARM_JERK for j in active]

    # tool frame(s): the active arm hand(s).
    tools = []
    if any("left" in j for j in active):
        tools.append("openarm_left_hand")
    if any("right" in j for j in active):
        tools.append("openarm_right_hand")
    d["tool_frames"] = tools

    # default_joint_positions: all 14 arms raised + lift home. Active joints use
    # these as the start; non-active (locked) joints are pinned here.
    djp = {f"openarm_left_joint{i}": v for i, v in enumerate(RAISED["left"], 1)}
    djp.update({f"openarm_right_joint{i}": v for i, v in enumerate(RAISED["right"], 1)})
    djp["lift_joint"] = LIFT_HOME
    d["default_joint_positions"] = djp

    # collision / self_collision / geometry / modifiers: keep as-is (both arms).

    out = f"{group}.xrdf"
    with open(out, "w") as f:
        yaml.safe_dump(d, f, sort_keys=False, default_flow_style=None)
    print(f"wrote {out}: cspace={len(active)} joints {active}, tools={tools}")


if __name__ == "__main__":
    main()
