#!/usr/bin/env python3
"""Add base_link + lift_link collision spheres to the XRDF.

cuMotion only knows the links that have XRDF spheres. Without base geometry it
can sweep a distal arm link THROUGH the mobile base. We add coarse, slightly
over-approximating spheres for the base (base_link cylinder) and torso
(lift_link box), with self_collision.ignore mirroring the SRDF's base<->arm
disable_collisions (link0/1/2/3 never collide; distal links stay checked so
cuMotion avoids them).

Spheres are defined in each link's LOCAL frame (XRDF convention).

  .spherefit_venv/bin/python add_base_spheres.py --xrdf openarm_bimanual.xrdf
"""
import argparse
import yaml

# base_link: cylinder r=0.275, length 0.33, origin z=0.165, axis +z (local frame)
# -> on-axis column of spheres covering z in [0, 0.33].
BASE_LINK_SPHERES = [
    {"center": [0.0, 0.0, z], "radius": 0.275}
    for z in (0.0, 0.11, 0.22, 0.33)
]
# lift_link: box 0.346(x) x 0.174(y) x 0.132(z), origin z=0.109 (local frame)
# -> row of spheres along x covering the y-z cross-section.
LIFT_LINK_SPHERES = [
    {"center": [x, 0.0, 0.109], "radius": 0.11}
    for x in (-0.13, -0.043, 0.043, 0.13)
]

# Mirror the SRDF base<->arm disable_collisions (never-collide pairs).
ARM_SUFFIX = ["link0", "link1", "link2"]
BASE_IGNORE = ["lift_link"] + \
    [f"openarm_left_{s}" for s in ARM_SUFFIX + ["link3"]] + \
    [f"openarm_right_{s}" for s in ARM_SUFFIX + ["link3"]]
LIFT_IGNORE = ["base_link"] + \
    [f"openarm_left_{s}" for s in ARM_SUFFIX] + \
    [f"openarm_right_{s}" for s in ARM_SUFFIX]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--xrdf", default="openarm_bimanual.xrdf")
    args = ap.parse_args()

    d = yaml.safe_load(open(args.xrdf))
    geom = d["geometry"][d["collision"]["geometry"]]["spheres"]
    if "base_link" in geom:
        print("base spheres already present; nothing to do")
        return

    # geometry
    geom["base_link"] = BASE_LINK_SPHERES
    geom["lift_link"] = LIFT_LINK_SPHERES

    # buffer_distance entries
    for blk in ("collision", "self_collision"):
        bd = d[blk]["buffer_distance"]
        val = 0.005 if blk == "collision" else 0.001
        bd["base_link"] = val
        bd["lift_link"] = val

    # self_collision.ignore (mirror SRDF)
    ign = d["self_collision"]["ignore"]
    ign["base_link"] = BASE_IGNORE
    ign["lift_link"] = LIFT_IGNORE

    # Preserve dict ordering roughly; dump.
    with open(args.xrdf, "w") as f:
        yaml.safe_dump(d, f, sort_keys=False, default_flow_style=None)
    n = sum(len(v) for v in geom.values())
    print(f"added base_link ({len(BASE_LINK_SPHERES)}) + lift_link "
          f"({len(LIFT_LINK_SPHERES)}) spheres; total now {n} over {len(geom)} links")
    print("base_link ignore:", BASE_IGNORE)
    print("lift_link ignore:", LIFT_IGNORE)


if __name__ == "__main__":
    main()
