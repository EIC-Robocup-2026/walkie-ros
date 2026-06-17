#!/usr/bin/env python3
"""Find self-collisions of the XRDF spheres at the zero configuration.

cuMotion rejects the start config if any two spheres on link pairs NOT in the
XRDF self_collision.ignore map overlap (distance < r1 + r2 + 2*buffer). This
reproduces that check on the host (FK at q=0 from the URDF) so we can see which
link pairs collide and decide: shrink spheres or add ignore pairs.

  .spherefit_venv/bin/python check_self_collision.py --urdf /tmp/openarm_bimanual.urdf --xrdf openarm_bimanual.xrdf
"""
import argparse
import xml.etree.ElementTree as ET

import numpy as np
import yaml


def rpy_to_R(r, p, y):
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @
            np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @
            np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def fk_world_transforms(urdf):
    """World transform per link at q=0 (joint motion = identity)."""
    root = ET.parse(urdf).getroot()
    parent_of = {}   # child_link -> (parent_link, T_parent_child)
    for j in root.findall("joint"):
        p = j.find("parent").get("link")
        c = j.find("child").get("link")
        org = j.find("origin")
        xyz_s = (org.get("xyz") if org is not None else None) or "0 0 0"
        rpy_s = (org.get("rpy") if org is not None else None) or "0 0 0"
        xyz = [float(x) for x in xyz_s.split()]
        rpy = [float(x) for x in rpy_s.split()]
        T = np.eye(4)
        T[:3, :3] = rpy_to_R(*rpy)
        T[:3, 3] = xyz
        parent_of[c] = (p, T)

    links = [ln.get("name") for ln in root.findall("link")]
    cache = {}

    def world(link):
        if link in cache:
            return cache[link]
        if link not in parent_of:      # root
            cache[link] = np.eye(4)
            return cache[link]
        p, T = parent_of[link]
        W = world(p) @ T
        cache[link] = W
        return W

    return {ln: world(ln) for ln in links}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", default="/tmp/openarm_bimanual.urdf")
    ap.add_argument("--xrdf", default="openarm_bimanual.xrdf")
    ap.add_argument("--extra", type=float, default=0.0,
                    help="extra global margin (m) to mimic cuRobo activation distance")
    args = ap.parse_args()

    x = yaml.safe_load(open(args.xrdf))
    spheres = x["geometry"]["openarm_bimanual_collision_spheres"]["spheres"]
    buf = x["self_collision"]["buffer_distance"]
    ignore = x["self_collision"].get("ignore", {})

    # Symmetric ignore set.
    ign = set()
    for a, lst in ignore.items():
        for b in lst:
            ign.add(frozenset((a, b)))

    W = fk_world_transforms(args.urdf)

    # World-frame spheres per link.
    world_spheres = {}
    for link, sl in spheres.items():
        if link not in W:
            print(f"  WARN: link {link} not in URDF FK")
            continue
        T = W[link]
        arr = []
        for s in sl:
            c = np.array(s["center"] + [1.0])
            wc = (T @ c)[:3]
            arr.append((wc, s["radius"]))
        world_spheres[link] = arr

    links = list(world_spheres.keys())
    collisions = []
    for i in range(len(links)):
        for j in range(i + 1, len(links)):
            a, b = links[i], links[j]
            if frozenset((a, b)) in ign:
                continue
            ba, bb = buf.get(a, 0.0), buf.get(b, 0.0)
            worst = None
            for (ca, ra) in world_spheres[a]:
                for (cb, rb) in world_spheres[b]:
                    d = np.linalg.norm(ca - cb)
                    pen = (ra + rb + ba + bb + args.extra) - d
                    if pen > 0 and (worst is None or pen > worst):
                        worst = pen
            if worst is not None:
                collisions.append((a, b, worst))

    collisions.sort(key=lambda t: -t[2])
    if not collisions:
        print("NO self-collisions at q=0  ✓")
        return
    print(f"{len(collisions)} colliding link pairs at q=0 (penetration depth, m):")
    for a, b, pen in collisions:
        print(f"  {pen*1000:6.1f} mm   {a}  <->  {b}")


if __name__ == "__main__":
    main()
