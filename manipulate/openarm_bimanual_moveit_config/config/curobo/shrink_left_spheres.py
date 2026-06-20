#!/usr/bin/env python3
"""Scale the LEFT-arm collision-sphere radii in an XRDF by a factor, preserving
formatting (line-based). Right-arm/base/lift spheres untouched.

Usage: shrink_left_spheres.py <in.xrdf> <out.xrdf> <factor>   e.g. 0.75
"""
import re
import sys

src, dst, factor = sys.argv[1], sys.argv[2], float(sys.argv[3])
in_spheres = False          # inside the `spheres:` block
in_left = False             # current link is an openarm_left_* link
n = 0
out = []
for line in open(src):
    if re.match(r"^    spheres:\s*$", line):
        in_spheres = True
    if in_spheres:
        m = re.match(r"^      (\S+):\s*$", line)   # a link header under spheres:
        if m:
            in_left = m.group(1).startswith("openarm_left_")
        rm = re.match(r"^(\s*radius:\s*)([0-9.]+)(.*)$", line)
        if rm and in_left:
            line = f"{rm.group(1)}{round(float(rm.group(2)) * factor, 5)}{rm.group(3)}\n"
            n += 1
    out.append(line)

open(dst, "w").writelines(out)
print(f"scaled {n} left-arm sphere radii by {factor} -> {dst}")
