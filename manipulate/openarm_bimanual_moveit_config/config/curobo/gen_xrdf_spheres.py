#!/usr/bin/env python3
"""Fit collision spheres to the OpenArm links and write them into the XRDF.

cuRobo / cuMotion needs a set of collision spheres per link (XRDF `geometry`
block). This script auto-fits them from the URDF collision meshes using a
medial-axis greedy cover, so the result is reproducible and tracked in-repo
(no Isaac Sim / GUI editor needed).

Pipeline per link:
  1. Read the link's <collision> mesh, scale and <origin> from the flat URDF.
  2. Load the STL, apply scale, fix winding if the scale mirrors (neg det),
     then transform vertices into the LINK-LOCAL frame (XRDF spheres are
     defined per-link in that frame).
  3. Voxelize the interior; clearance(point) = signed distance to surface.
  4. Greedily place inscribed spheres largest-clearance-first, removing
     covered candidates, until coverage/target met.

The XRDF `collision.buffer_distance` (0.005) and `self_collision.buffer_distance`
(0.02) inflate the effective radius at query time, covering the thin surface
shell the inscribed spheres leave uncovered.

Run (uses the local venv with trimesh):
  .spherefit_venv/bin/python gen_xrdf_spheres.py \
      --urdf /tmp/openarm_bimanual.urdf \
      --xrdf openarm_bimanual.xrdf
"""
import argparse
import os
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

# Repo layout: this file lives in
#   .../openarm_bimanual_moveit_config/config/curobo/
REPO_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..", "..")
)
# walkie_description package -> description/ submodule (meshes under it).
DESC_DIR = os.path.join(REPO_ROOT, "description")

# Only fit spheres for these links (the arm/EE chain in the XRDF). Mobile base,
# wheels, head, etc. are not part of any cuMotion planning group.
LINK_PREFIXES = ("openarm_left_", "openarm_right_")

# Per-link tuning. Long links get a finer pitch / more spheres.
DEFAULT_PITCH = 0.012      # voxel size (m) -> candidate density
DEFAULT_MAX_SPHERES = 18
MIN_RADIUS = 0.010         # drop spheres smaller than this (noise)
COVERAGE = 0.97            # stop once this fraction of interior is covered
# Cluster density: spheres are strung ~(SLICE_FRAC * cross-radius) apart along
# the link's long axis. Smaller -> more, thinner slices -> tighter spheres that
# hug the cross-section instead of bulging (which caused false self-collisions).
SLICE_FRAC = 0.55

# Links that are small / thin: give a finer pitch so they still get spheres.
FINE_LINKS = {
    "left_finger": (0.006, 4),
    "right_finger": (0.006, 4),
    "wristcam_bracket": (0.008, 8),
    "link7": (0.008, 8),
    "hand": (0.010, 12),
}


def resolve_mesh(filename):
    """Map a URDF mesh filename to an absolute path on disk."""
    fn = filename.split("//")[-1]  # strip package:// or file://
    if fn.startswith("walkie_description/"):
        return os.path.join(DESC_DIR, fn[len("walkie_description/"):])
    if os.path.isabs(fn):
        return fn
    return os.path.join(REPO_ROOT, fn)


def rpy_to_matrix(rpy):
    r, p, y = rpy
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def link_local_mesh(mesh_path, scale, xyz, rpy):
    """Load STL, apply scale + origin, return mesh in the link-local frame."""
    m = trimesh.load(mesh_path, force="mesh", process=False)
    S = np.diag([scale[0], scale[1], scale[2], 1.0])
    T = np.eye(4)
    T[:3, :3] = rpy_to_matrix(rpy)
    T[:3, 3] = xyz
    M = T @ S
    m.apply_transform(M)
    # A negative-determinant transform (mirror) flips face winding -> normals
    # point inward, breaking inside/outside tests. Fix it.
    if np.linalg.det(M[:3, :3]) < 0:
        m.invert()
    m.merge_vertices()
    m.fix_normals()
    return m


def fit_spheres(mesh, pitch, max_spheres):
    """Surface-covering sphere decomposition in the mesh's own frame.

    The OpenArm `_symp` collision meshes are thin SHELLS, not solids, so
    inscribed (medial) spheres collapse to the wall thickness. For collision
    avoidance we instead want spheres that ENCLOSE the link. We cluster surface
    points (k-means) and wrap each cluster in a sphere (radius = farthest point
    in the cluster), so the union over-approximates the link volume. The number
    of spheres scales with how elongated the link is, giving a chain of spheres
    along long links and a single ball on compact ones.
    """
    from scipy.cluster.vq import kmeans2

    # Dense surface point cloud: sampled surface + mesh vertices.
    try:
        samp = mesh.sample(3000)
    except Exception:
        samp = mesh.vertices
    pts = np.vstack([np.asarray(samp), np.asarray(mesh.vertices)])
    if len(pts) == 0:
        c = mesh.bounding_sphere.primitive.center
        return [(c.tolist(), float(mesh.bounding_sphere.primitive.radius))]

    # Pick K from elongation: r_target ~ half the median cross-section, then
    # string ~ (longest extent / r_target) spheres along the length.
    ext = np.sort(mesh.extents)            # [min, mid, max]
    r_target = max(ext[1] / 2.0, MIN_RADIUS)
    k = int(np.clip(round(ext[2] / (SLICE_FRAC * r_target)), 1, max_spheres))

    if k == 1:
        c = pts.mean(axis=0)
        r = float(np.linalg.norm(pts - c, axis=1).max())
        return [(c.tolist(), r)]

    centers, labels = kmeans2(pts, k, minit="++", seed=0, missing="warn")
    spheres = []
    for j in range(len(centers)):
        cl = pts[labels == j]
        if len(cl) == 0:
            continue
        c = cl.mean(axis=0)               # recenter on the cluster's points
        r = float(np.linalg.norm(cl - c, axis=1).max())
        spheres.append((c.tolist(), max(r, MIN_RADIUS)))
    if not spheres:
        c = pts.mean(axis=0)
        spheres = [(c.tolist(), float(np.linalg.norm(pts - c, axis=1).max()))]
    return spheres


def parse_links(urdf_path):
    root = ET.parse(urdf_path).getroot()
    out = {}
    for ln in root.findall("link"):
        name = ln.get("name")
        if not name.startswith(LINK_PREFIXES):
            continue
        col = ln.find("collision")
        if col is None:
            continue
        geo = col.find("geometry")
        mesh = geo.find("mesh") if geo is not None else None
        if mesh is None:
            continue
        scale = mesh.get("scale")
        scale = [float(x) for x in scale.split()] if scale else [1.0, 1.0, 1.0]
        org = col.find("origin")
        xyz = [float(x) for x in (org.get("xyz") if org is not None else "0 0 0").split()]
        rpy = [float(x) for x in (org.get("rpy") if org is not None else "0 0 0").split()]
        out[name] = dict(filename=mesh.get("filename"), scale=scale, xyz=xyz, rpy=rpy)
    return out


def tuning_for(link_name):
    for key, (pitch, mx) in FINE_LINKS.items():
        if link_name.endswith(key):
            return pitch, mx
    return DEFAULT_PITCH, DEFAULT_MAX_SPHERES


def render_geometry_block(all_spheres, geom_name):
    lines = ["geometry:", f"  {geom_name}:", "    spheres:"]
    for link, spheres in all_spheres.items():
        lines.append(f"      {link}:")
        for (c, r) in spheres:
            lines.append(
                f'        - center: [{c[0]:.5f}, {c[1]:.5f}, {c[2]:.5f}]'
                f'  # noqa'
            )
            lines[-1] = (
                f"        - {{center: [{c[0]:.5f}, {c[1]:.5f}, {c[2]:.5f}], "
                f"radius: {r:.5f}}}"
            )
    return "\n".join(lines) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", default="/tmp/openarm_bimanual.urdf")
    ap.add_argument("--xrdf", default="openarm_bimanual.xrdf")
    ap.add_argument("--geom-name", default="openarm_bimanual_collision_spheres")
    args = ap.parse_args()

    links = parse_links(args.urdf)
    print(f"links with collision meshes: {len(links)}")

    all_spheres = {}
    total = 0
    for name, info in links.items():
        path = resolve_mesh(info["filename"])
        if not os.path.exists(path):
            print(f"  !! MISSING MESH for {name}: {path}")
            continue
        mesh = link_local_mesh(path, info["scale"], info["xyz"], info["rpy"])
        pitch, mx = tuning_for(name)
        spheres = fit_spheres(mesh, pitch, mx)
        all_spheres[name] = spheres
        total += len(spheres)
        print(f"  {name:34s} {len(spheres):2d} spheres  "
              f"(r {min(r for _, r in spheres):.3f}..{max(r for _, r in spheres):.3f})")
    print(f"total spheres: {total}")

    block = render_geometry_block(all_spheres, args.geom_name)

    # Replace everything from the existing 'geometry:' line to EOF.
    with open(args.xrdf) as f:
        text = f.read()
    idx = text.find("\ngeometry:")
    if idx == -1:
        new_text = text.rstrip() + "\n\n" + block
    else:
        new_text = text[:idx].rstrip() + "\n\n" + block
    with open(args.xrdf, "w") as f:
        f.write(new_text)
    print(f"wrote geometry block -> {args.xrdf}")


if __name__ == "__main__":
    main()
