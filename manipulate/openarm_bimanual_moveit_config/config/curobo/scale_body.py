#!/usr/bin/env python3
"""Scale ONLY the body (base_link + lift_link) collision-sphere radii in an XRDF.
Arm spheres untouched. Handles inline and multi-line formats.

  scale_body.py <in> <out> <factor>   e.g. 0.6  (smaller body -> arm hugs body,
  less wide arcs; but too small -> ValidateSolution mesh rejects again).
"""
import re
import sys

src, dst, f = sys.argv[1], sys.argv[2], float(sys.argv[3])
in_spheres = False
cur = ""
n = 0
out = []
for line in open(src):
    if re.match(r"^    spheres:\s*$", line):
        in_spheres = True
    if in_spheres:
        m = re.match(r"^      (\S+):\s*$", line)
        if m:
            cur = m.group(1)
        rm = re.search(r"radius:\s*([0-9.]+)", line)
        if rm and cur in ("base_link", "lift_link"):
            nr = round(float(rm.group(1)) * f, 5)
            line = line[:rm.start(1)] + str(nr) + line[rm.end(1):]
            n += 1
    out.append(line)
open(dst, "w").writelines(out)
print(f"scaled {n} base/lift sphere radii by {f} -> {dst}")
