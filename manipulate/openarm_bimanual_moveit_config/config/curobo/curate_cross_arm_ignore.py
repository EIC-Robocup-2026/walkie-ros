#!/usr/bin/env python3
"""Curate the cross-arm (left<->right) self_collision ignores in the bimanual XRDF.

Problem: the arm collision spheres are over-approximate, so some cross-arm sphere
pairs OVERLAP even in poses where the two arms are physically far apart (e.g. the
neutral home). If those pairs are checked, cuMotion rejects valid starts
("self collision detected"); if ALL cross-arm pairs are ignored, cuMotion plans
the arms straight through each other.

Fix: a cross-arm pair is a FALSE POSITIVE (-> keep ignoring) iff its spheres
overlap in a KNOWN collision-free pose. Pairs that never overlap across these
safe poses are kept CHECKED, so cuMotion still detects a genuine arm crossing.

This (a) keeps every same-arm ignore already in the XRDF, (b) adds back only the
false-positive cross-arm pairs, and (c) rewrites just the `ignore:` block in
place (block style), preserving comments and the spheres block.

  .spherefit_venv/bin/python curate_cross_arm_ignore.py \
      --urdf ~/workspaces/isaac_ros-dev/openarm_cumotion/openarm_bimanual.urdf \
      --xrdf openarm_bimanual.xrdf
"""
import argparse
import xml.etree.ElementTree as ET

import numpy as np
import yaml

L = [f"openarm_left_joint{i}" for i in range(1, 8)]
R = [f"openarm_right_joint{i}" for i in range(1, 8)]

# Known collision-free arm poses (left j1..7, right j1..7). lift fixed raised.
SAFE_POSES = [
    [0.2618, 0, 0, 0.2618, 0, 0, 0,  -0.2618, 0, 0, 0.2618, 0, 0, 0],   # home
    [0, -1.4, 0, 0.6, 0, 0, 0,        0, 1.4, 0, 0.6, 0, 0, 0],          # raised
    [0, -1.0, 0, 0.9, 0, 0, 0,        0, 1.0, 0, 0.9, 0, 0, 0],          # forward
    [0, -1.4, 0, 1.2, 0, 0, 0,        0, 1.4, 0, 1.2, 0, 0, 0],          # raised+elbow
    [0, -0.6, 0, 0.6, 0, 0, 0,        0, 0.6, 0, 0.6, 0, 0, 0],          # low spread
]


def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @
            np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @
            np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def axis_motion(axis, q, prismatic):
    T = np.eye(4)
    a = np.array(axis, float)
    n = np.linalg.norm(a)
    a = a / n if n else a
    if prismatic:
        T[:3, 3] = a * q
    else:
        K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
        T[:3, :3] = np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * (K @ K)
    return T


def load_joints(urdf):
    root = ET.parse(urdf).getroot()
    joints = {}
    for j in root.findall("joint"):
        org = j.find("origin")
        xyz = [float(v) for v in ((org.get("xyz") if org is not None else None) or "0 0 0").split()]
        rpy = [float(v) for v in ((org.get("rpy") if org is not None else None) or "0 0 0").split()]
        T = np.eye(4); T[:3, :3] = rpy_to_R(*rpy); T[:3, 3] = xyz
        ax = j.find("axis")
        joints[j.find("child").get("link")] = {
            "parent": j.find("parent").get("link"), "T": T,
            "type": j.get("type"), "name": j.get("name"),
            "axis": [float(v) for v in (ax.get("xyz") if ax is not None else "0 0 1").split()],
        }
    return joints


def fk(joints, q):
    cache = {}

    def world(link):
        if link in cache:
            return cache[link]
        if link not in joints:
            cache[link] = np.eye(4); return cache[link]
        info = joints[link]
        T = info["T"]
        if info["type"] in ("revolute", "continuous", "prismatic") and info["name"] in q:
            T = T @ axis_motion(info["axis"], q[info["name"]], info["type"] == "prismatic")
        W = world(info["parent"]) @ T
        cache[link] = W; return W

    return world


def side(n):
    p = n.split("_"); return p[1] if len(p) > 1 and p[1] in ("left", "right") else ""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--xrdf", default="openarm_bimanual.xrdf")
    args = ap.parse_args()

    x = yaml.safe_load(open(args.xrdf))
    spheres = x["geometry"]["openarm_bimanual_collision_spheres"]["spheres"]
    buf = x["self_collision"]["buffer_distance"]
    ignore = {k: set(v if isinstance(v, list) else [v]) for k, v in
              x["self_collision"].get("ignore", {}).items()}

    joints = load_joints(args.urdf)
    cross_links = [(a, b) for i, a in enumerate(spheres) for b in list(spheres)[i + 1:]
                   if side(a) and side(b) and side(a) != side(b)]

    false_pos = set()   # cross-arm pairs overlapping in a safe pose
    for pose in SAFE_POSES:
        q = {"lift_joint": 0.7435}
        q.update(dict(zip(L + R, pose)))
        world = fk(joints, q)
        wsp = {ln: [((world(ln) @ np.array(s["center"] + [1.0]))[:3], s["radius"])
                    for s in sl] for ln, sl in spheres.items() if ln in joints or ln == "base_footprint"}
        for a, b in cross_links:
            if frozenset((a, b)) in false_pos:
                continue
            m = buf.get(a, 0.0) + buf.get(b, 0.0)
            hit = any(np.linalg.norm(ca - cb) < ra + rb + m
                      for ca, ra in wsp.get(a, []) for cb, rb in wsp.get(b, []))
            if hit:
                false_pos.add(frozenset((a, b)))

    checked = [p for p in cross_links if frozenset(p) not in false_pos]
    print(f"cross-arm pairs: {len(cross_links)}  false-positive(ignore): {len(false_pos)}  "
          f"kept CHECKED: {len(checked)}")
    print("kept-checked cross-arm pairs (these now block arm crossings):")
    for a, b in checked:
        print(f"  {a} <-> {b}")

    # Merge false positives into the ignore map (canonical: key = lexicographically first).
    for pr in false_pos:
        a, b = sorted(pr)
        ignore.setdefault(a, set()).add(b)

    _rewrite_ignore_block(args.xrdf, {k: sorted(v) for k, v in sorted(ignore.items())})
    print(f"updated {args.xrdf}: ignore now {sum(len(v) for v in ignore.values())} pairs "
          f"(added {len(false_pos)} curated cross-arm)")


def _rewrite_ignore_block(path, ignore):
    """Replace the `  ignore:` block (block style) in place, keeping everything else."""
    lines = open(path).read().splitlines()
    out, i, n = [], 0, len(lines)
    while i < n:
        if lines[i].rstrip() == "  ignore:":
            out.append("  ignore:")
            for k in ignore:
                if not ignore[k]:
                    continue
                out.append(f"    {k}:")
                for v in ignore[k]:
                    out.append(f'      - "{v}"')
            i += 1
            # skip old block: lines indented >= 4 spaces (or blank within block)
            while i < n and (lines[i][:4] == "    " or lines[i].strip() == ""):
                if lines[i].strip() == "" and (i + 1 >= n or lines[i + 1][:4] != "    "):
                    break
                i += 1
            continue
        out.append(lines[i]); i += 1
    open(path, "w").write("\n".join(out) + "\n")


if __name__ == "__main__":
    main()
