#!/usr/bin/env python3
"""Remove cross-arm (left<->right) pairs from an XRDF self_collision.ignore block,
in place, preserving everything else (spheres, comments, formatting).

Cross-arm pairs in the ignore map tell cuMotion NOT to check left-vs-right
collisions, so it plans the two arms through each other. They get seeded from
the SRDF disable_collisions (Setup Assistant marks them "never in collision").
This strips only those pairs; same-arm adjacency ignores are kept.

Usage: strip_cross_arm_ignore.py file1.xrdf [file2.xrdf ...]
"""
import re
import sys


# Distal links can actually reach across the centerline and collide; proximal
# links (shoulder/upper arm) can't, but their over-approximate spheres false-
# overlap at rest. So only UN-ignore (check) cross-arm pairs where BOTH links
# are distal.
DISTAL = ("link4", "link5", "link6", "link7", "hand", "finger", "wristcam")


def arm_side(link):
    p = link.split("_")
    return p[1] if len(p) > 1 and p[1] in ("left", "right") else ""


def is_distal(link):
    return any(t in link for t in DISTAL)


def strip(path):
    lines = open(path).read().splitlines()
    out = []
    in_ignore = False
    key = None          # current "    <name>:" line text
    key_side = ""
    kept_items = []     # kept "      - ..." lines under the current key
    dropped = 0

    def flush_key():
        # Emit the key + its kept items, unless every item was dropped.
        nonlocal kept_items, key
        if key is None:
            return
        if kept_items:
            out.append(key)
            out.extend(kept_items)
        key = None
        kept_items = []

    for line in lines:
        if not in_ignore:
            out.append(line)
            if line.rstrip() == "  ignore:":
                in_ignore = True
            continue

        # inside the ignore block
        m_key = re.match(r"^    (\S[^:]*):\s*$", line)       # 4-space key
        m_item = re.match(r'^      - "?([^"]+)"?\s*$', line)  # 6-space list item
        if m_key:
            flush_key()
            key = line
            key_side = arm_side(m_key.group(1))
        elif m_item:
            other = m_item.group(1)
            os_ = arm_side(other)
            key_name = key.strip().rstrip(":")
            cross = key_side and os_ and key_side != os_
            if cross and is_distal(key_name) and is_distal(other):
                dropped += 1                  # distal cross-arm: drop -> keep checked
            else:
                kept_items.append(line)       # same-arm or proximal cross-arm: keep ignored
        else:
            # line with indent < 4 (or blank past the block) -> end of ignore block
            flush_key()
            in_ignore = False
            out.append(line)

    flush_key()
    open(path, "w").write("\n".join(out) + "\n")
    print(f"{path}: dropped {dropped} cross-arm ignore pairs")


if __name__ == "__main__":
    for f in sys.argv[1:]:
        strip(f)
