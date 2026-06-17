#!/usr/bin/env python3
"""Measure clearance between the DISTAL arm collision spheres and the base/torso
collision PRIMITIVES (base_link cylinder + 2 torso boxes, lift_link 2 boxes) at
key arm poses. Tells us whether tight base/torso self-collision spheres can be
added without false-flagging valid poses: clearance must exceed (body_sphere_R +
arm_sphere_R + cuMotion padding) at every collision-free pose.

  .spherefit_venv/bin/python body_clearance.py
"""
import numpy as np
import yaml
import curate_cross_arm_ignore as C   # reuse load_joints + fk

U = "/home/ryu/workspaces/isaac_ros-dev/openarm_cumotion/openarm_bimanual.urdf"
XRDF = "openarm_bimanual.xrdf"

DISTAL = ("link4", "link5", "link6", "link7", "hand", "finger", "wristcam")

# base_link primitives (link frame): (kind, dims, origin)
BASE = [
    ("cyl", (0.275, 0.33), (0.0, 0.0, 0.165)),
    ("box", (0.196, 0.18, 1.04), (0.0, 0.07, 0.845)),
    ("box", (0.46, 0.07, 1.04), (0.0, 0.07, 0.845)),
]
LIFT = [
    ("box", (0.346, 0.174, 0.132), (0.0, 0.0, 0.109)),
    ("box", (0.19, 0.19, 0.132), (0.0, -0.172, 0.109)),
]

POSES = {
    "home(j1=.2618)": [0.2618, 0, 0, 0.2618, 0, 0, 0,  -0.2618, 0, 0, 0.2618, 0, 0, 0],
    "home(j1=0)":     [0, 0, 0, 0.2618, 0, 0, 0,        0, 0, 0, 0.2618, 0, 0, 0],
    "raised":         [0, -1.4, 0, 0.6, 0, 0, 0,        0, 1.4, 0, 0.6, 0, 0, 0],
    "forward":        [0, -1.0, 0, 0.9, 0, 0, 0,        0, 1.0, 0, 0.9, 0, 0, 0],
}
L = [f"openarm_left_joint{i}" for i in range(1, 8)]
R = [f"openarm_right_joint{i}" for i in range(1, 8)]


def dist_box(p, dims, o):
    d = np.abs(p - np.array(o)) - np.array(dims) / 2.0
    outside = np.linalg.norm(np.maximum(d, 0.0))
    inside = min(0.0, max(d))
    return outside + inside  # signed: <0 inside


def dist_cyl(p, dims, o):
    r, length = dims
    q = p - np.array(o)
    radial = np.hypot(q[0], q[1]) - r
    axial = abs(q[2]) - length / 2.0
    if radial <= 0 and axial <= 0:
        return max(radial, axial)
    return np.hypot(max(radial, 0.0), max(axial, 0.0))


def main():
    x = yaml.safe_load(open(XRDF))
    spheres = x["geometry"][next(iter(x["geometry"]))]["spheres"]
    joints = C.load_joints(U)
    distal_links = [ln for ln in spheres if any(t in ln for t in DISTAL)]

    for name, pose in POSES.items():
        q = {"lift_joint": 0.7435}
        q.update(dict(zip(L + R, pose)))
        w = C.fk(joints, q)
        Wbase = w("base_link")
        Wlift = w("lift_link")

        worst = {}
        for ln in distal_links:
            for s in spheres[ln]:
                wc = (w(ln) @ np.array(s["center"] + [1.0]))[:3]
                ra = s["radius"]
                # base primitives: point in base_link frame
                pb = (np.linalg.inv(Wbase) @ np.append(wc, 1.0))[:3]
                for kind, dims, o in BASE:
                    d = (dist_cyl if kind == "cyl" else dist_box)(pb, dims, o) - ra
                    key = "base_" + kind + str(o[2])
                    worst[key] = min(worst.get(key, 9), d)
                pl = (np.linalg.inv(Wlift) @ np.append(wc, 1.0))[:3]
                for kind, dims, o in LIFT:
                    d = dist_box(pl, dims, o) - ra
                    worst["lift"] = min(worst.get("lift", 9), d)
        mn = min(worst.values())
        tag = "  <-- arm SURFACE here (tight spheres would false-flag)" if mn < 0.05 else ""
        print(f"{name:16s} min distal-arm clearance to body = {mn*1000:6.1f} mm{tag}")
        for k, v in sorted(worst.items(), key=lambda kv: kv[1])[:3]:
            print(f"      {k:16s} {v*1000:6.1f} mm")


if __name__ == "__main__":
    main()
