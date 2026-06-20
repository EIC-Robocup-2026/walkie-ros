#!/usr/bin/env python3
"""Scale BOTH arms' collision-sphere radii in an XRDF (openarm_left_* and
openarm_right_* links). base_link/lift_link left untouched. Preserves formatting.

Usage: shrink_arm_spheres.py <in.xrdf> <out.xrdf> <factor>   e.g. 0.75
"""
import re
import sys

src, dst, factor = sys.argv[1], sys.argv[2], float(sys.argv[3])
in_spheres = False
in_arm = False
n = 0
out = []
for line in open(src):
    if re.match(r"^    spheres:\s*$", line):
        in_spheres = True
    if in_spheres:
        m = re.match(r"^      (\S+):\s*$", line)          # link header under spheres:
        if m:
            in_arm = m.group(1).startswith(("openarm_left_", "openarm_right_"))
        rm = re.match(r"^(\s*radius:\s*)([0-9.]+)(.*)$", line)
        if rm and in_arm:
            line = f"{rm.group(1)}{round(float(rm.group(2)) * factor, 5)}{rm.group(3)}\n"
            n += 1
    out.append(line)

open(dst, "w").writelines(out)
print(f"scaled {n} arm sphere radii (both arms) by {factor} -> {dst}")
