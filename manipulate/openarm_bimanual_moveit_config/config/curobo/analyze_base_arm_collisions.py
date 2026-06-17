#!/usr/bin/env python3
"""Measure base<->arm collision rates across the arm workspace.

move_group's CheckStartStateCollision flags base_link<->arm contacts because
those pairs are not in the SRDF disable_collisions. We must NOT blindly disable
them (that would let the planner drive the arm through the base). Instead, like
the MoveIt Setup Assistant, we sample many valid arm configs and measure the
actual contact rate of each (base-obstacle, arm-link) pair using the XRDF arm
spheres vs the base collision primitives (base_link cylinder, lift_link box).

  - rate == 0%  -> safe to add to SRDF disable_collisions
  - rate  > 0%  -> a real collision risk; keep checking + cuMotion needs base
                   spheres to avoid it.

  .spherefit_venv/bin/python analyze_base_arm_collisions.py
"""
import xml.etree.ElementTree as ET
import numpy as np
import yaml
import check_self_collision as csc

URDF = "/tmp/openarm_bimanual.urdf"
XRDF = "openarm_bimanual.xrdf"
N_SAMPLES = 4000
ARM_JOINTS = [f"openarm_left_joint{i}" for i in range(1, 8)] + \
             [f"openarm_right_joint{i}" for i in range(1, 8)]


def build_fk(urdf):
    root = ET.parse(urdf).getroot()
    parent = {}
    for j in root.findall("joint"):
        p = j.find("parent").get("link"); ch = j.find("child").get("link")
        org = j.find("origin")
        xyz = [float(v) for v in ((org.get("xyz") if org is not None else None) or "0 0 0").split()]
        rpy = [float(v) for v in ((org.get("rpy") if org is not None else None) or "0 0 0").split()]
        ax = j.find("axis")
        axis = [float(v) for v in (ax.get("xyz") if ax is not None else "0 0 1").split()] if ax is not None else [0, 0, 1]
        T = np.eye(4); T[:3, :3] = csc.rpy_to_R(*rpy); T[:3, 3] = xyz
        parent[ch] = (p, T, j.get("name"), j.get("type"), np.array(axis, float))
    return parent


def axisR(axis, ang):
    a = axis / np.linalg.norm(axis); x, y, z = a
    ca, sa = np.cos(ang), np.sin(ang); C = 1 - ca
    return np.array([[ca+x*x*C, x*y*C-z*sa, x*z*C+y*sa],
                     [y*x*C+z*sa, ca+y*y*C, y*z*C-x*sa],
                     [z*x*C-y*sa, z*y*C+x*sa, ca+z*z*C]])


def world_fn(parent, q):
    cache = {}
    def world(l):
        if l in cache: return cache[l]
        if l not in parent:
            cache[l] = np.eye(4); return cache[l]
        p, T, name, typ, axis = parent[l]
        M = world(p) @ T
        if name in q:
            if typ == "revolute":
                R = np.eye(4); R[:3, :3] = axisR(axis, q[name]); M = M @ R
            elif typ == "prismatic":
                M = M @ np.block([[np.eye(3), (axis*q[name]).reshape(3,1)], [0,0,0,1]])
        cache[l] = M; return M
    return world


def joint_limits(urdf):
    root = ET.parse(urdf).getroot()
    lim = {}
    for j in root.findall("joint"):
        n = j.get("name")
        if n in ARM_JOINTS:
            L = j.find("limit")
            lim[n] = (float(L.get("lower")), float(L.get("upper")))
    return lim


def dist_to_cylinder(p, center, axis, half_len, radius):
    """Signed distance from point p to a finite cylinder surface (neg = inside)."""
    d = p - center
    ax = axis / np.linalg.norm(axis)
    z = d @ ax
    radial = np.linalg.norm(d - z*ax)
    # outside distances along radial and axial; inside is negative
    dr = radial - radius
    dz = abs(z) - half_len
    if dr <= 0 and dz <= 0:
        return max(dr, dz)          # inside
    return np.hypot(max(dr, 0), max(dz, 0))


def dist_to_box(p, center, R, half):
    """Signed distance from point p to an oriented box (neg = inside)."""
    local = R.T @ (p - center)
    q = np.abs(local) - half
    outside = np.linalg.norm(np.maximum(q, 0))
    inside = min(max(q[0], max(q[1], q[2])), 0)
    return outside + inside


def main():
    parent = build_fk(URDF)
    lim = joint_limits(URDF)
    x = yaml.safe_load(open(XRDF))
    spheres = x["geometry"]["openarm_bimanual_collision_spheres"]["spheres"]

    # Base obstacles in world frame (at q=0 / lift=0; base is fixed).
    w0 = world_fn(parent, {})
    bl = w0("base_link")
    bl_center = (bl @ np.array([0, 0, 0.165, 1]))[:3]
    bl_axis = bl[:3, 2]
    lf = w0("lift_link")
    lf_center = (lf @ np.array([0, 0, 0.109, 1]))[:3]
    lf_R = lf[:3, :3]; lf_half = np.array([0.346, 0.174, 0.132]) / 2.0

    arm_links = list(spheres.keys())
    contact = {("base_link", l): 0 for l in arm_links}
    contact.update({("lift_link", l): 0 for l in arm_links})

    rng = np.random.default_rng(0)
    for _ in range(N_SAMPLES):
        q = {j: rng.uniform(*lim[j]) for j in ARM_JOINTS}
        world = world_fn(parent, q)
        for l in arm_links:
            T = world(l)
            for s in spheres[l]:
                c = (T @ np.array(s["center"] + [1.0]))[:3]
                r = s["radius"]
                if dist_to_cylinder(c, bl_center, bl_axis, 0.165, 0.275) < r:
                    contact[("base_link", l)] += 1; break
            for s in spheres[l]:
                c = (T @ np.array(s["center"] + [1.0]))[:3]
                r = s["radius"]
                if dist_to_box(c, lf_center, lf_R, lf_half) < r:
                    contact[("lift_link", l)] += 1; break

    print(f"base<->arm contact rates over {N_SAMPLES} random valid configs:\n")
    never, sometimes = [], []
    for (base, l), n in sorted(contact.items()):
        rate = 100.0 * n / N_SAMPLES
        (never if n == 0 else sometimes).append((base, l, rate))
    print("NEVER collide (safe to disable in SRDF):")
    for base, l, _ in never:
        print(f"  {base:10s} <-> {l}")
    print(f"\nSOMETIMES collide (KEEP checking; need cuMotion base spheres):")
    for base, l, rate in sorted(sometimes, key=lambda t: -t[2]):
        print(f"  {rate:5.1f}%  {base:10s} <-> {l}")


if __name__ == "__main__":
    main()
