#!/usr/bin/env python3
"""Add base_link + lift_link (mobile base / torso / lift carriage) collision
spheres to the XRDF so cuMotion avoids driving the arms into the body.

cuMotion only collision-checks links that have XRDF spheres; base_link and
lift_link had none, so the arms could plan straight through the torso/base.
These links are URDF PRIMITIVES (cylinder + boxes), so we fill each primitive
with a grid of overlapping spheres (in the link's local frame; the collision
<origin> offset is baked into the centers). A prior attempt used one big on-axis
cylinder sphere (r=0.275) which engulfed the base and false-collided with the
arm mounts -> here we use many small spheres tracking the actual shape.

Ignore map: the proximal arm links are mounted on the body and are always near
it, so base_link<->arm link0/1/2/3 and lift_link<->arm link0/1/2 are ignored
(matches the SRDF disable_collisions and the base<->arm sampling analysis); the
DISTAL arm links (link4+ / hand / fingers) stay CHECKED vs the body.

Run AFTER gen_xrdf_spheres.py (arm spheres) and BEFORE make_single_arm_xrdf.py
(which inherits these). Idempotent: replaces any existing base_link/lift_link
sphere + ignore entries.

  .spherefit_venv/bin/python gen_body_spheres.py \
      --urdf ~/workspaces/isaac_ros-dev/openarm_cumotion/openarm_bimanual.urdf \
      --xrdf openarm_bimanual.xrdf
"""
import argparse
import xml.etree.ElementTree as ET

import numpy as np
import yaml

BODY_LINKS = ("base_link", "lift_link")

# base_link's URDF collision is a low mobile-base CYLINDER (r=0.275 @ z 0..0.33)
# plus the torso-column BOXES (z 0.33..1.37). The arms mount high on the torso and
# never reach the mobile base below their workspace, so the lower cylinder layers
# only over-constrain planning (and risk false collisions near the floor/wheels).
# Keep ONLY the cylinder's TOP layer (the base's top deck, the one surface the arm
# could swing down onto) and drop the layers below it -> the torso column plus that
# top deck constrain the arm.
CYL_TOP_LAYER_ONLY = {"base_link"}

R = 0.045         # sphere radius. Spheres are INSET by R so their surface sits
                  # flush with the primitive face (no outward bulge into the arm's
                  # near-field) -> tight, accurate body shell.
STEP = 0.08       # grid spacing (< 2R so spheres overlap -> no gaps)

# Proximal arm links to IGNORE against each body link (mount-adjacent, never a
# real collision). Distal links (link4+/hand/finger/wristcam) stay checked.
IGNORE_ARM = {
    "base_link": [3, 2, 1, 0],   # SRDF disables base_link<->link0..3
    "lift_link": [3, 2, 1, 0],   # SRDF disables lift_link<->link0..2; link3 added
                                 # (upper-arm rides next to the lift carriage)
}


def _grid(lo, hi):
    # INSET by R so a sphere centered here has its surface at the primitive face
    # (flush, no bulge). Thin dimension (< 2R) collapses to one centered layer.
    lo, hi = lo + R, hi - R
    if hi <= lo:
        return [float((lo + hi) / 2.0)]
    if hi - lo <= STEP:
        return [float((lo + hi) / 2.0)]
    n = int(np.ceil((hi - lo) / STEP)) + 1
    return [float(v) for v in np.linspace(lo, hi, n)]


def box_spheres(size, origin):
    sx, sy, sz = size
    ox, oy, oz = origin
    out = []
    for x in _grid(ox - sx / 2, ox + sx / 2):
        for y in _grid(oy - sy / 2, oy + sy / 2):
            for z in _grid(oz - sz / 2, oz + sz / 2):
                out.append({"center": [round(x, 5), round(y, 5), round(z, 5)],
                            "radius": R})
    return out


def cyl_spheres(radius, length, origin):
    ox, oy, oz = origin
    out = []
    for z in _grid(oz - length / 2, oz + length / 2):
        for x in _grid(-radius, radius):
            for y in _grid(-radius, radius):
                if x * x + y * y <= radius * radius + 1e-9:
                    out.append({"center": [round(ox + x, 5), round(oy + y, 5),
                                           round(z, 5)], "radius": R})
    return out


def link_spheres(urdf, link_name):
    root = ET.parse(urdf).getroot()
    link = next(l for l in root.findall("link") if l.get("name") == link_name)
    spheres = []
    for col in link.findall("collision"):
        g = col.find("geometry")
        org = col.find("origin")
        o = [float(v) for v in ((org.get("xyz") if org is not None else None)
                                or "0 0 0").split()]
        box = g.find("box")
        cyl = g.find("cylinder")
        if box is not None:
            spheres += box_spheres([float(v) for v in box.get("size").split()], o)
        elif cyl is not None:
            cs = cyl_spheres(float(cyl.get("radius")),
                             float(cyl.get("length")), o)
            if link_name in CYL_TOP_LAYER_ONLY:
                # keep only the highest z-layer (the cylinder's top deck)
                zmax = max(s["center"][2] for s in cs)
                cs = [s for s in cs if abs(s["center"][2] - zmax) < 1e-6]
            spheres += cs
    return spheres


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--xrdf", default="openarm_bimanual.xrdf")
    args = ap.parse_args()

    x = yaml.safe_load(open(args.xrdf))
    geo_name = next(iter(x["geometry"]))
    spheres = x["geometry"][geo_name]["spheres"]
    ignore = {k: set(v if isinstance(v, list) else [v])
              for k, v in x["self_collision"].get("ignore", {}).items()}
    buf = x["self_collision"]["buffer_distance"]

    for body in BODY_LINKS:
        s = link_spheres(args.urdf, body)
        spheres[body] = s
        buf.setdefault(body, 0.001)
        # ignore body <-> proximal arm links (both sides)
        for side in ("left", "right"):
            for i in IGNORE_ARM[body]:
                ignore.setdefault(body, set()).add(f"openarm_{side}_link{i}")
        print(f"{body}: {len(s)} spheres, ignored vs arm links "
              f"{IGNORE_ARM[body]} (both arms)")
    # base_link and lift_link are ADJACENT (lift carriage mounts on the base), so
    # their spheres always overlap -> must ignore that pair or cuMotion reports a
    # constant self-collision at EVERY pose. (The skeleton generator only ignores
    # openarm<->openarm pairs, so this body<->body pair is never seeded from SRDF.)
    ignore.setdefault("base_link", set()).add("lift_link")

    x["self_collision"]["ignore"] = {k: sorted(v) for k, v in sorted(ignore.items())}
    with open(args.xrdf, "w") as f:
        yaml.safe_dump(x, f, sort_keys=False, default_flow_style=None, width=100)
    total = sum(len(v) for v in spheres.values())
    print(f"wrote {args.xrdf}: {total} total spheres, "
          f"{sum(len(v) for v in x['self_collision']['ignore'].values())} ignore pairs")


if __name__ == "__main__":
    main()
