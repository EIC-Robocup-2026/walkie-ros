#!/usr/bin/env python3
"""Scale arm collision-sphere radii in an XRDF, with an optional per-link override.

  scale_spheres.py <in> <out> <default_factor> [<link_substr> <link_factor>]

- Arm links (openarm_left_* / openarm_right_*) get <default_factor>.
- Links whose name contains <link_substr> get <link_factor> instead.
- base_link/lift_link untouched. Formatting preserved.

e.g. all arms 0.75 but link4 (both arms) 0.6:
  scale_spheres.py openarm_bimanual.xrdf openarm_bimanual_shrunk.xrdf 0.75 link4 0.6
"""
import re
import sys

src, dst, default_f = sys.argv[1], sys.argv[2], float(sys.argv[3])
sub = sys.argv[4] if len(sys.argv) > 4 else None
sub_f = float(sys.argv[5]) if len(sys.argv) > 5 else default_f

in_spheres = False
cur = ""
counts = {}
out = []
for line in open(src):
    if re.match(r"^    spheres:\s*$", line):
        in_spheres = True
    if in_spheres:
        m = re.match(r"^      (\S+):\s*$", line)
        if m:
            cur = m.group(1)
        # match `radius: <num>` anywhere on the line -> handles both the
        # multi-line (`        radius: 0.04`) and inline
        # (`- {center: [...], radius: 0.04}`) XRDF sphere formats.
        rm = re.search(r"radius:\s*([0-9.]+)", line)
        if rm and cur.startswith(("openarm_left_", "openarm_right_")):
            f = sub_f if (sub and sub in cur) else default_f
            new_r = round(float(rm.group(1)) * f, 5)
            line = line[:rm.start(1)] + str(new_r) + line[rm.end(1):]
            counts[f] = counts.get(f, 0) + 1
    out.append(line)

open(dst, "w").writelines(out)
print(f"scaled radii -> {dst}: " + ", ".join(f"{k}x:{v}" for k, v in sorted(counts.items())))
