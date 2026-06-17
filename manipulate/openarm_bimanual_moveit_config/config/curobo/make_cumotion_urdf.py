#!/usr/bin/env python3
"""Produce a cuMotion-only URDF where all joints NOT in the planning group are
FIXED (lift's raise baked into its origin).

WHY: cuMotion 4.4 fails (INVALID_INITIAL) when the request start_state contains
joints outside the cspace. RViz always sends the FULL robot state, so it breaks.
The cuMotion planner uses its OWN urdf (cumotion_action_server.urdf_file_path,
separate from move_group's), so if that urdf has the extra joints FIXED, they're
no longer actuated -> the plugin maps only the arm joints from RViz's full state
-> no extra-joint failure. move_group/RViz keep the full (movable) robot.

  .spherefit_venv/bin/python make_cumotion_urdf.py both_arms
      <in>  : /tmp/openarm_bimanual.urdf (flat, package:// meshes)
      <out> : openarm_bimanual_<group>_cumotion.urdf  (file:// meshes)
"""
import sys
import xml.etree.ElementTree as ET

IN = "/tmp/openarm_bimanual.urdf"
REPO = "/home/ryu/walkie_ws/src/walkie-ros"

L = [f"openarm_left_joint{i}" for i in range(1, 8)]
R = [f"openarm_right_joint{i}" for i in range(1, 8)]
GROUP_ACTIVE = {
    "both_arms":      L + R,
    "right_arm":      R,
    "left_arm":       L,
    "both_arms_lift": ["lift_joint"] + L + R,
    "right_arm_lift": ["lift_joint"] + R,
    "left_arm_lift":  ["lift_joint"] + L,
}
# Home values to BAKE when fixing a movable non-active joint (others default 0).
HOME = {"lift_joint": 0.70}


def main():
    group = sys.argv[1] if len(sys.argv) > 1 else "both_arms"
    active = set(GROUP_ACTIVE[group])

    tree = ET.parse(IN)
    root = tree.getroot()
    fixed = []
    for j in root.findall("joint"):
        jtype = j.get("type")
        name = j.get("name")
        if jtype == "fixed" or name in active:
            continue
        # Make this non-active movable joint FIXED, baking its home offset.
        q = HOME.get(name, 0.0)
        if q != 0.0:
            ax = j.find("axis")
            axis = [float(x) for x in (ax.get("xyz") if ax is not None else "0 0 1").split()]
            org = j.find("origin")
            if org is None:
                org = ET.SubElement(j, "origin")
            xyz = [float(x) for x in (org.get("xyz") or "0 0 0").split()]
            if jtype == "prismatic":
                xyz = [xyz[i] + q * axis[i] for i in range(3)]   # bake translation
            org.set("xyz", " ".join(f"{v:.6f}" for v in xyz))
            # (revolute baking would need rpy compose; our only baked joint is the
            #  prismatic lift, so translation is sufficient.)
        j.set("type", "fixed")
        for tag in ("axis", "limit", "dynamics", "mimic"):
            e = j.find(tag)
            if e is not None:
                j.remove(e)
        fixed.append(name)

    out = f"openarm_bimanual_{group}_cumotion.urdf"
    xml = ET.tostring(root, encoding="unicode")
    # package://walkie_description -> file://<repo>/description (host/container both
    # resolve via the staged copy path; we rewrite to the container mount path).
    xml = xml.replace("package://walkie_description/",
                      "file:///workspaces/isaac_ros-dev/openarm_cumotion/desc/")
    with open(out, "w") as f:
        f.write(xml)
    print(f"wrote {out}: fixed {len(fixed)} joints -> {fixed}")
    print(f"active (movable) joints kept: {sorted(active)}")


if __name__ == "__main__":
    main()
