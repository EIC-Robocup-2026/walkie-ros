#!/usr/bin/env python3
"""
Save a SLAM + OctoMap bundle for the walkie map editor.

Produces, in ONE map directory, everything the editor needs to draw no-go zones
on top of a slam_toolbox map enriched with the 3D structure octomap captured:

  map.pgm / map.yaml             the slam_toolbox 2D map (saved from /map)
  octomap.bt / octomap.ot        the 3D octree (binary + full, for 3D archive)
  octomap_overlay.pgm / .yaml    the octomap's 2D occupancy projection RESAMPLED
                                 onto the slam map's exact grid (same resolution,
                                 origin and pixel size) so it drops straight on
                                 top of map.pgm and reveals the full furniture
                                 footprints the horizontal lidar slam misses.

Run it AFTER you finish mapping, with slam_toolbox + octomap still running and
the robot not moved / SLAM not reset, so the `map` frame is identical for both
maps:

    ros2 launch robot_bringup slam_toolbox.launch.py
    ros2 launch robot_bringup octomap_3d_mapping.launch.py
    # ... drive around, build the map ...
    python3 scripts/save_octomap_for_editor.py \
        --map-dir src/walkie-ros/navigation/walkie-navigation/map/23_<date>_<name>

Source your ROS workspace first (so rclpy, nav2_map_server and octomap_server
are on the path and rmw_zenoh is configured) -- i.e. `source install/setup.bash`.

Why the resample step
---------------------
octomap's /projected_map picks its own bounding box and origin, so its pixel
grid never matches the slam map. We map every slam-map cell to a world point and
max-pool the octomap projection over it (so thin obstacles survive a downsample),
producing a layer that overlays the slam map exactly -- whether the editor
georeferences by the .yaml or naively stacks same-size rasters.
"""
import argparse
import os
import subprocess
import sys
import time

import numpy as np


# OccupancyGrid cell values.
OCC_FREE = 0
OCC_OCCUPIED = 100
OCC_UNKNOWN = -1

# PGM greyscale values (Nav2 map_saver "trinary" convention, negate=0):
#   occupied -> black, free -> white, unknown -> mid grey.
PGM_OCCUPIED = 0
PGM_FREE = 254
PGM_UNKNOWN = 205


def run(cmd, timeout):
    """Run a subprocess, streaming output, return True on success."""
    print("  $ " + " ".join(cmd))
    try:
        proc = subprocess.run(cmd, timeout=timeout)
        return proc.returncode == 0
    except subprocess.TimeoutExpired:
        print(f"  !! timed out after {timeout:.0f}s", file=sys.stderr)
        return False
    except FileNotFoundError:
        print(f"  !! command not found: {cmd[0]} (did you source the workspace?)",
              file=sys.stderr)
        return False


def save_slam_map(path_stem, topic, timeout):
    """Save the live slam map (/map) to <path_stem>.pgm/.yaml via nav2."""
    print(f"[1/4] Saving slam map from {topic} -> {path_stem}.pgm/.yaml")
    ok = run(
        ["ros2", "run", "nav2_map_server", "map_saver_cli",
         "-t", topic, "-f", path_stem,
         "--ros-args", "-p", "map_subscribe_transient_local:=true"],
        timeout=timeout,
    )
    if not ok:
        print("  !! slam map save failed -- is slam_toolbox publishing on "
              f"{topic}?", file=sys.stderr)
    return ok


def save_octree(path_stem, timeout):
    """Save the live octree to <path_stem>.bt (binary) and .ot (full)."""
    print(f"[2/4] Saving 3D octree -> {path_stem}.bt and {path_stem}.ot")
    ok = True
    for ext in (".bt", ".ot"):
        # This jazzy octomap_saver_node reads the output path from the
        # `octomap_path` PARAMETER (extension decides binary vs full), NOT argv.
        if not run(
            ["ros2", "run", "octomap_server", "octomap_saver_node",
             "--ros-args", "-p", f"octomap_path:={path_stem}{ext}"],
            timeout=timeout,
        ):
            print(f"  !! octree {ext} save failed -- is octomap_server running?",
                  file=sys.stderr)
            ok = False
    return ok


def read_pgm_size(path):
    """Return (width, height) of a binary (P5) PGM by parsing its header."""
    with open(path, "rb") as f:
        if f.readline().strip() != b"P5":
            raise ValueError(f"{path} is not a binary P5 PGM")
        vals = []
        while len(vals) < 3:
            line = f.readline()
            if not line:
                raise ValueError(f"{path} has a truncated PGM header")
            if line.lstrip().startswith(b"#"):
                continue
            vals += line.split()
        return int(vals[0]), int(vals[1])


def read_map_yaml(path):
    """Minimal map.yaml reader -> dict with resolution + origin (no PyYAML dep)."""
    res = None
    origin = None
    with open(path) as f:
        for line in f:
            line = line.split("#", 1)[0].strip()
            if line.startswith("resolution:"):
                res = float(line.split(":", 1)[1])
            elif line.startswith("origin:"):
                inside = line.split(":", 1)[1].strip().lstrip("[").rstrip("]")
                origin = [float(x) for x in inside.split(",")]
    if res is None or origin is None:
        raise ValueError(f"{path} is missing resolution/origin")
    return res, origin


def fetch_projected_map(topic, timeout):
    """Spin up a tiny rclpy node and grab one /projected_map OccupancyGrid."""
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                           QoSDurabilityPolicy, QoSHistoryPolicy)

    print(f"[3/4] Fetching {topic} ...")
    rclpy.init()
    node = rclpy.create_node("octomap_editor_saver")
    # Match octomap_server's latched (TRANSIENT_LOCAL) publisher so we get the
    # current map immediately even if no new cloud arrives while we wait.
    qos = QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )
    holder = {}
    node.create_subscription(
        OccupancyGrid, topic, lambda m: holder.setdefault("msg", m), qos)

    deadline = time.time() + timeout
    while rclpy.ok() and "msg" not in holder and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()

    if "msg" not in holder:
        print(f"  !! no message on {topic} within {timeout:.0f}s -- is octomap "
              "running with latch:=true (default)?", file=sys.stderr)
        return None
    return holder["msg"]


def reproject_onto_slam_grid(pm, slam_res, slam_origin, slam_w, slam_h):
    """Resample octomap /projected_map (pm) onto the slam map's pixel grid.

    Returns a (slam_h, slam_w) uint8 PGM image. We scatter every octomap cell to
    its slam pixel; occupied cells win over free (max-pool), unknown cells leave
    the slam pixel unknown. Image row 0 = top = max world-y (PGM convention);
    OccupancyGrid row 0 = bottom = origin-y.
    """
    w_o, h_o, res_o = pm.info.width, pm.info.height, pm.info.resolution
    ox_o, oy_o = pm.info.origin.position.x, pm.info.origin.position.y
    occ = np.asarray(pm.data, dtype=np.int16).reshape(h_o, w_o)

    # The world<->pixel mapping below assumes both grids are axis-aligned (no
    # yaw). octomap's projection and slam_toolbox maps always are; bail loudly
    # rather than silently emit a rotated, misaligned overlay.
    q = pm.info.origin.orientation
    if abs(q.z) > 1e-6 or abs(1.0 - q.w) > 1e-6:
        raise ValueError("projected_map origin is rotated; axis-aligned only")
    slam_yaw = slam_origin[2] if len(slam_origin) > 2 else 0.0
    if abs(slam_yaw) > 1e-6:
        raise ValueError(f"slam map origin yaw={slam_yaw} unsupported; axis-aligned only")

    res_s = slam_res
    ox_s, oy_s = slam_origin[0], slam_origin[1]

    # World centre of every octomap cell.
    cols = np.arange(w_o)
    rows = np.arange(h_o)
    wx = ox_o + (cols + 0.5) * res_o            # (w_o,)
    wy = oy_o + (rows + 0.5) * res_o            # (h_o,)  row 0 = min y
    WX, WY = np.meshgrid(wx, wy)                # (h_o, w_o)

    # Destination slam pixel for each octomap cell.
    ti = np.floor((WX - ox_s) / res_s).astype(np.int64)               # col
    tj = (slam_h - 1 - np.floor((WY - oy_s) / res_s)).astype(np.int64)  # row (top-down)
    valid = (ti >= 0) & (ti < slam_w) & (tj >= 0) & (tj < slam_h)

    flat = (tj * slam_w + ti).reshape(-1)
    occ_flat = occ.reshape(-1)
    vmask = valid.reshape(-1)
    flat = flat[vmask]
    vals = occ_flat[vmask]

    out = np.full(slam_w * slam_h, PGM_UNKNOWN, dtype=np.uint8)
    # Free first, then occupied overwrites (occupancy max-pool).
    np.put(out, flat[vals == OCC_FREE], PGM_FREE)
    np.put(out, flat[vals == OCC_OCCUPIED], PGM_OCCUPIED)
    return out.reshape(slam_h, slam_w)


def write_pgm(path, img):
    with open(path, "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (img.shape[1], img.shape[0]))
        f.write(img.tobytes())


def write_overlay_yaml(path, image_name, res, origin):
    with open(path, "w") as f:
        f.write(
            f"image: {image_name}\n"
            "mode: trinary\n"
            f"resolution: {res}\n"
            f"origin: [{origin[0]}, {origin[1]}, "
            f"{origin[2] if len(origin) > 2 else 0.0}]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.196\n"
        )


def main():
    ap = argparse.ArgumentParser(
        description="Save slam map + octomap + slam-aligned overlay for the "
                    "walkie map editor.")
    ap.add_argument("--map-dir", required=True,
                    help="Output directory for the bundle (created if needed).")
    ap.add_argument("--slam-name", default="map",
                    help="Basename for the slam map (default: map).")
    ap.add_argument("--octomap-name", default="octomap",
                    help="Basename for the 3D octree files (default: octomap).")
    ap.add_argument("--overlay-name", default="octomap_overlay",
                    help="Basename for the 2D overlay (default: octomap_overlay).")
    ap.add_argument("--map-topic", default="/map",
                    help="slam_toolbox occupancy grid topic (default: /map).")
    ap.add_argument("--projected-topic", default="/projected_map",
                    help="octomap 2D projection topic (default: /projected_map).")
    ap.add_argument("--no-slam", action="store_true",
                    help="Do not save the slam map; reuse an existing "
                         "<map-dir>/<slam-name>.pgm/.yaml as the alignment "
                         "reference instead.")
    ap.add_argument("--no-octree", action="store_true",
                    help="Skip saving the 3D .bt/.ot octree files.")
    ap.add_argument("--timeout", type=float, default=15.0,
                    help="Per-step timeout in seconds (default: 15).")
    args = ap.parse_args()

    map_dir = os.path.abspath(args.map_dir)
    os.makedirs(map_dir, exist_ok=True)
    slam_stem = os.path.join(map_dir, args.slam_name)
    octo_stem = os.path.join(map_dir, args.octomap_name)
    overlay_pgm = os.path.join(map_dir, args.overlay_name + ".pgm")
    overlay_yaml = os.path.join(map_dir, args.overlay_name + ".yaml")

    print(f"Saving editor bundle into: {map_dir}\n")

    # 1) slam map (the alignment reference the overlay is resampled onto).
    if not args.no_slam:
        save_slam_map(slam_stem, args.map_topic, args.timeout)
    else:
        print("[1/4] Skipping slam map save (--no-slam)")

    slam_yaml = slam_stem + ".yaml"
    slam_pgm = slam_stem + ".pgm"
    if not (os.path.exists(slam_yaml) and os.path.exists(slam_pgm)):
        print(f"\nERROR: need {slam_yaml} and {slam_pgm} as the alignment "
              "reference. Save the slam map first (drop --no-slam).",
              file=sys.stderr)
        return 1

    # 2) 3D octree.
    if not args.no_octree:
        save_octree(octo_stem, args.timeout)
    else:
        print("[2/4] Skipping octree save (--no-octree)")

    # 3) live 2D projection.
    pm = fetch_projected_map(args.projected_topic, args.timeout)
    if pm is None:
        print("\nERROR: could not get the octomap projection; overlay not "
              "written. The slam map / octree (if requested) were still saved.",
              file=sys.stderr)
        return 1

    # 4) resample onto the slam grid and write the overlay.
    print("[4/4] Resampling octomap projection onto the slam grid -> overlay")
    slam_res, slam_origin = read_map_yaml(slam_yaml)
    slam_w, slam_h = read_pgm_size(slam_pgm)
    img = reproject_onto_slam_grid(pm, slam_res, slam_origin, slam_w, slam_h)
    write_pgm(overlay_pgm, img)
    write_overlay_yaml(overlay_yaml, os.path.basename(overlay_pgm),
                       slam_res, slam_origin)

    occupied = int((img == PGM_OCCUPIED).sum())
    print(f"  overlay {slam_w}x{slam_h} @ {slam_res} m, origin {slam_origin[:2]}, "
          f"{occupied} occupied cells")
    print("\nDone. Bundle for the map editor:")
    for p in (slam_pgm, slam_yaml,
              octo_stem + ".bt", octo_stem + ".ot",
              overlay_pgm, overlay_yaml):
        mark = "  " if os.path.exists(p) else "  (missing) "
        print(mark + p)
    return 0


if __name__ == "__main__":
    sys.exit(main())
