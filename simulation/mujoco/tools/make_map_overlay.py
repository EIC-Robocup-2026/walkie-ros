#!/usr/bin/env python3
"""Draw a world's collision footprints over its navigation map, in ROS metres.

Geometry comes from the exported scene through MuJoCo, so body and geom
rotations, joint references and mesh vertices match the simulation. Static
colliders are filled (blue when they reach 1.5 m, otherwise orange), limited
hinges are outlined at their initial pose (solid) and at their limits (thin),
and the manifest spawns are drawn with their headings. Movable bodies with a
free joint are not drawn.

Optional inputs, which default to the world's authoring/checks.py OVERLAY:
a second occupancy image with the same size and origin as the map (drawn as a
second panel) and a waypoint file ({"elements": [{"type": "waypoint", "label",
"coords": [[x, y]], "heading": radians}]}).

--check-navigation also writes navigation_validation_<variant>.json: a
4-neighbour flood fill for a round robot from the default spawn to every other
spawn and waypoint. It shows geometric connectivity only, not Nav2 behaviour.

    pixi run python simulation/mujoco/tools/make_map_overlay.py chula_pat14_6 --check-navigation

The image is simulation/mujoco/output/<world>/map_alignment_<variant>.png.
"""

from __future__ import annotations

import argparse
from collections import deque
import hashlib
import json
import math
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import world_tools
import yaml

PIXELS_PER_METRE = 160
TALL_M = 1.5
COLORS = {"tall": (34, 110, 179), "low": (217, 130, 30), "hinge": (177, 50, 99),
          "spawn": (23, 78, 66), "waypoint": (90, 60, 140), "grid": (0, 0, 0)}


def geom_points(model, geom) -> np.ndarray | None:
    """Return points on a geom's surface in its local frame."""
    kind, size = model.geom_type[geom], model.geom_size[geom]
    angles = np.linspace(0, 2 * math.pi, 25)[:-1]
    if kind == mujoco.mjtGeom.mjGEOM_BOX:
        return np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]) * size
    if kind == mujoco.mjtGeom.mjGEOM_MESH:
        mesh = model.geom_dataid[geom]
        start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
        return model.mesh_vert[start:start + count].astype(float)
    if kind in (mujoco.mjtGeom.mjGEOM_SPHERE, mujoco.mjtGeom.mjGEOM_ELLIPSOID):
        radii = size if kind == mujoco.mjtGeom.mjGEOM_ELLIPSOID else np.repeat(size[0], 3)
        tilts = np.linspace(-math.pi / 2, math.pi / 2, 13)
        return np.array([[radii[0] * math.cos(a) * math.cos(t), radii[1] * math.sin(a)
                          * math.cos(t), radii[2] * math.sin(t)] for a in angles for t in tilts])
    if kind in (mujoco.mjtGeom.mjGEOM_CYLINDER, mujoco.mjtGeom.mjGEOM_CAPSULE):
        half = size[1] + (size[0] if kind == mujoco.mjtGeom.mjGEOM_CAPSULE else 0)
        return np.array([[size[0] * math.cos(a), size[0] * math.sin(a), z]
                         for a in angles for z in (-half, 0, half)])
    return None


def convex_hull(points) -> np.ndarray:
    """Monotone chain hull of the XY projection."""
    points = sorted(set(map(tuple, np.round(np.asarray(points)[:, :2], 6))))
    if len(points) < 3:
        return np.asarray(points)

    def cross(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    lower, upper = [], []
    for point in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)
    for point in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)
    return np.asarray(lower[:-1] + upper[:-1])


def footprints(model, data, geoms) -> list[dict]:
    """Project the colliding geoms at the current kinematic state."""
    result = []
    for geom in geoms:
        if not (model.geom_contype[geom] or model.geom_conaffinity[geom]):
            continue
        local = geom_points(model, geom)
        if local is None:
            continue
        points = local @ data.geom_xmat[geom].reshape(3, 3).T + data.geom_xpos[geom]
        # Thin ground slabs would hide the obstacles above them.
        if points[:, 2].max() < .04:
            continue
        hull = convex_hull(points)
        if len(hull) >= 3:
            result.append({"name": model.geom(geom).name, "hull": hull,
                           "z_min": float(points[:, 2].min()),
                           "z_max": float(points[:, 2].max())})
    return result


def scene_geometry(model) -> tuple[list[dict], list[dict]]:
    """Return static footprints and, per top-level hinge, footprints at three angles."""
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    static = [g for g in range(model.ngeom) if model.body_weldid[model.geom_bodyid[g]] == 0]
    result = footprints(model, data, static)
    hinges = []
    for joint in range(model.njnt):
        body = int(model.jnt_bodyid[joint])
        if (model.jnt_type[joint] != mujoco.mjtJoint.mjJNT_HINGE
                or model.body_weldid[model.body_parentid[body]] != 0):
            continue
        subtree = {b for b in range(model.nbody) if _in_subtree(model, b, body)}
        geoms = [g for g in range(model.ngeom) if model.geom_bodyid[g] in subtree]
        address = int(model.jnt_qposadr[joint])
        initial = float(model.qpos0[address])
        angles = {"initial": initial}
        if model.jnt_limited[joint]:
            angles.update(lower=float(model.jnt_range[joint][0]),
                          upper=float(model.jnt_range[joint][1]))
        poses = {}
        for label, angle in angles.items():
            data.qpos[:] = model.qpos0
            data.qpos[address] = angle
            mujoco.mj_kinematics(model, data)
            poses[label] = footprints(model, data, geoms)
        hinges.append({"joint": model.joint(joint).name, "poses": poses,
                       "pivot": data.xanchor[joint].tolist()})
    return result, hinges


def _in_subtree(model, body, root) -> bool:
    while body > 0:
        if body == root:
            return True
        body = int(model.body_parentid[body])
    return False


def read_map(path: Path) -> tuple[dict, Image.Image]:
    config = yaml.safe_load(path.read_text())
    if abs(float(config["origin"][2])) > 1e-9:
        raise ValueError(f"{path}: the overlay needs a map origin with zero yaw")
    return config, Image.open(path.parent / config["image"]).convert("L")


def map_extent(config: dict, image: Image.Image) -> tuple[float, float, float, float]:
    x0, y0 = float(config["origin"][0]), float(config["origin"][1])
    resolution = float(config["resolution"])
    return x0, x0 + image.width * resolution, y0, y0 + image.height * resolution


def read_waypoints(path: Path | None) -> list[dict]:
    if path is None:
        return []
    elements = json.loads(path.read_text())["elements"]
    return [{"label": element["label"], "xy": element["coords"][0],
             "heading": float(element.get("heading", 0.))}
            for element in elements if element.get("type") == "waypoint"]


def polygon_distance(x, y, hull):
    """Distance to a convex footprint, zero inside it."""
    distance2 = np.full(np.broadcast_shapes(np.shape(x), np.shape(y)), np.inf)
    inside = np.ones_like(distance2, dtype=bool)
    for a, b in zip(hull, np.roll(hull, -1, axis=0)):
        edge = b - a
        length2 = float(edge @ edge)
        if length2 < 1e-16:
            continue
        t = np.clip(((x - a[0]) * edge[0] + (y - a[1]) * edge[1]) / length2, 0, 1)
        distance2 = np.minimum(distance2, (x - a[0] - t * edge[0]) ** 2
                               + (y - a[1] - t * edge[1]) ** 2)
        inside &= (edge[0] * (y - a[1]) - edge[1] * (x - a[0])) >= -1e-10
    return np.where(inside, 0, np.sqrt(distance2))


def check_navigation(hulls, bounds, start, goals, radius, height, step=.01) -> dict:
    """Flood fill free space for a round robot of the given radius and height."""
    xmin, xmax, ymin, ymax = bounds
    xs, ys = np.arange(xmin, xmax + step / 2, step), np.arange(ymin, ymax + step / 2, step)
    free = np.ones((len(ys), len(xs)), dtype=bool)
    # Surfaces above the robot, such as a lintel, do not block it.
    obstacles = [item["hull"] for item in hulls if item["z_min"] < height]
    limit = radius + .002
    for hull in obstacles:
        lo, hi = hull.min(axis=0) - limit, hull.max(axis=0) + limit
        ix = np.where((xs >= lo[0]) & (xs <= hi[0]))[0]
        iy = np.where((ys >= lo[1]) & (ys <= hi[1]))[0]
        if len(ix) and len(iy):
            distance = polygon_distance(xs[ix][None, :], ys[iy][:, None], hull)
            free[np.ix_(iy, ix)] &= distance >= limit

    def cell(point):
        return round((point[1] - ymin) / step), round((point[0] - xmin) / step)

    def inside(index):
        return 0 <= index[0] < len(ys) and 0 <= index[1] < len(xs)

    origin = cell(start)
    if not inside(origin):
        raise ValueError("The default spawn is outside the navigation check bounds")
    reachable = np.zeros_like(free)
    reachable[origin] = free[origin]
    queue = deque([origin] if free[origin] else [])
    while queue:
        row, col = queue.popleft()
        for d_row, d_col in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nxt = (row + d_row, col + d_col)
            if inside(nxt) and free[nxt] and not reachable[nxt]:
                reachable[nxt] = True
                queue.append(nxt)
    results = {}
    for label, point in goals.items():
        index = cell(point)
        clearance = min((float(polygon_distance(point[0], point[1], h)) for h in obstacles),
                        default=math.inf)
        results[label] = {"xy_m": list(point), "center_clearance_m": round(clearance, 4),
                          "reachable_from_default_spawn": bool(inside(index)
                                                               and reachable[index])}
    return {"status": "PASS" if all(goal["reachable_from_default_spawn"]
                                    for goal in results.values()) else "FAIL",
            "method": "2D projection of static colliders and hinged bodies at their initial "
                      "pose; 4-neighbour flood fill from the default spawn",
            "radius_m": radius, "height_m": height, "grid_resolution_m": step,
            "bounds_xy_m": list(bounds), "start_xy_m": list(start), "goals": results}


class Panel:
    """Map one image panel between ROS metres and pixels."""

    def __init__(self, bounds, scale=PIXELS_PER_METRE):
        self.xmin, self.xmax, self.ymin, self.ymax = bounds
        self.scale = scale
        self.size = (round((self.xmax - self.xmin) * scale),
                     round((self.ymax - self.ymin) * scale))

    def pixel(self, x, y):
        return (x - self.xmin) * self.scale, (self.ymax - y) * self.scale

    def background(self, config, image):
        x0, _, y0, y1 = map_extent(config, image)
        resolution = float(config["resolution"])
        box = ((self.xmin - x0) / resolution, (y1 - self.ymax) / resolution,
               (self.xmax - x0) / resolution, (y1 - self.ymin) / resolution)
        crop = image.transform(self.size, Image.Transform.EXTENT, box,
                               resample=Image.Resampling.NEAREST, fillcolor=205)
        return Image.blend(Image.new("RGB", self.size, "white"), crop.convert("RGB"), .82)


def draw_panel(panel, background, static, hinges, spawns, waypoints, font, title):
    image = background.convert("RGBA")
    layer = Image.new("RGBA", image.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(layer)
    for x in np.arange(math.ceil(panel.xmin * 2) / 2, panel.xmax, .5):
        draw.line([panel.pixel(x, panel.ymin), panel.pixel(x, panel.ymax)],
                  fill=COLORS["grid"] + (40,))
        draw.text((panel.pixel(x, panel.ymin)[0] + 2, image.height - 16), f"{x:g}",
                  fill=(60, 60, 60, 255), font=font)
    for y in np.arange(math.ceil(panel.ymin * 2) / 2, panel.ymax, .5):
        draw.line([panel.pixel(panel.xmin, y), panel.pixel(panel.xmax, y)],
                  fill=COLORS["grid"] + (40,))
        draw.text((2, panel.pixel(panel.xmin, y)[1] - 14), f"{y:g}",
                  fill=(60, 60, 60, 255), font=font)
    for item in static:
        color = COLORS["tall" if item["z_max"] >= TALL_M else "low"]
        points = [panel.pixel(*point) for point in item["hull"]]
        draw.polygon(points, fill=color + (80,), outline=color + (200,))
    for hinge in hinges:
        for label, items in hinge["poses"].items():
            initial = label == "initial"
            for item in items:
                points = [panel.pixel(*point) for point in item["hull"]]
                draw.polygon(points, outline=COLORS["hinge"] + (255 if initial else 140,),
                             width=3 if initial else 1)
        px, py = panel.pixel(*hinge["pivot"][:2])
        draw.ellipse([px - 4, py - 4, px + 4, py + 4], fill=COLORS["hinge"] + (255,))
    for label, (x, y, heading), color in (
            [(name, pose, COLORS["spawn"]) for name, pose in spawns.items()]
            + [(w["label"], (*w["xy"], w["heading"]), COLORS["waypoint"]) for w in waypoints]):
        px, py = panel.pixel(x, y)
        tip = panel.pixel(x + .2 * math.cos(heading), y + .2 * math.sin(heading))
        draw.line([(px, py), tip], fill=color + (255,), width=3)
        draw.ellipse([px - 5, py - 5, px + 5, py + 5], fill=color + (255,))
        box = draw.textbbox((px + 8, py - 8), label, font=font)
        draw.rectangle(box, fill=(255, 255, 255, 190))
        draw.text((px + 8, py - 8), label, fill=color + (255,), font=font)
    image = Image.alpha_composite(image, layer)
    framed = Image.new("RGB", (image.width, image.height + 30), "white")
    framed.paste(image.convert("RGB"), (0, 30))
    ImageDraw.Draw(framed).text((4, 4), title, fill=(0, 0, 0), font=font)
    return framed


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    world_tools.add_world_arguments(parser)
    parser.add_argument("--variant", help="Variant to draw (default: the manifest default)")
    parser.add_argument("--reference-image", type=Path,
                        help="Second occupancy image with the map's size and origin")
    parser.add_argument("--waypoints", type=Path, help="Waypoint element JSON")
    parser.add_argument("--check-navigation", action="store_true",
                        help="Also write navigation_validation.json")
    parser.add_argument("--radius", type=float, default=.30,
                        help="Robot radius for --check-navigation (default: %(default)s m)")
    parser.add_argument("--height", type=float, default=1.90,
                        help="Robot height for --check-navigation (default: %(default)s m)")
    args = parser.parse_args(argv)

    directory, manifest, output = world_tools.open_world(args)
    hooks = world_tools.load_hooks(directory)
    defaults = getattr(hooks, "OVERLAY", {}) if hooks is not None else {}
    authoring = directory / "authoring"
    reference = args.reference_image or (
        authoring / defaults["reference_image"] if "reference_image" in defaults else None)
    waypoint_file = args.waypoints or (
        authoring / defaults["waypoints"] if "waypoints" in defaults else None)
    variant = args.variant or manifest["default_variant"]
    if variant not in manifest["variants"]:
        parser.error(f"Unknown variant {variant!r}; available: {', '.join(manifest['variants'])}")
    scene = directory / manifest["variants"][variant]
    model = mujoco.MjModel.from_xml_path(str(scene))
    static, hinges = scene_geometry(model)
    if not static:
        raise ValueError(f"No collision footprints found in {scene}")
    spawns = {name: (pose[0], pose[1], math.radians(pose[2]))
              for name, pose in manifest["spawns"].items()}
    waypoints = read_waypoints(waypoint_file)

    config, map_image = read_map(directory / manifest["map"])
    hulls = [item["hull"] for item in static]
    hulls += [item["hull"] for hinge in hinges for item in hinge["poses"]["initial"]]
    points = np.vstack(hulls + [np.array([pose[:2] for pose in spawns.values()])]
                       + ([np.array([w["xy"] for w in waypoints])] if waypoints else []))
    x0, x1, y0, y1 = map_extent(config, map_image)
    bounds = (max(x0, points[:, 0].min() - .4), min(x1, points[:, 0].max() + .4),
              max(y0, points[:, 1].min() - .4), min(y1, points[:, 1].max() + .4))

    font = ImageFont.load_default(size=14)
    panel = Panel(bounds)
    panels = [draw_panel(panel, panel.background(config, map_image), static, hinges, spawns,
                         waypoints, font, f"Navigation map ({manifest['map']})")]
    if reference is not None:
        reference_image = Image.open(reference).convert("L")
        if reference_image.size != map_image.size:
            raise ValueError(f"{reference}: size {reference_image.size} differs from the map")
        panels.append(draw_panel(panel, panel.background(config, reference_image), static,
                                 hinges, spawns, waypoints, font,
                                 f"Reference image ({reference.name})"))
    footer = (f"{args.world} / {variant}: blue colliders reach {TALL_M} m, orange are lower; "
              "magenta hinged bodies at the initial pose (thick) and limits (thin); "
              "green spawns, purple waypoints. Grid 0.5 m in the ROS map frame.")
    width = sum(p.width for p in panels) + 10 * (len(panels) - 1)
    sheet = Image.new("RGB", (width, panels[0].height + 26), "white")
    for index, image in enumerate(panels):
        sheet.paste(image, (index * (panels[0].width + 10), 0))
    ImageDraw.Draw(sheet).text((4, panels[0].height + 6), footer, fill=(60, 60, 60), font=font)
    path = output / f"map_alignment_{variant}.png"
    sheet.save(path)
    print(f"Wrote {path} ({len(static)} static footprints, {len(hinges)} hinges)")

    if args.check_navigation:
        default = manifest["default_spawn"]
        goals = {f"spawn:{name}": pose[:2] for name, pose in spawns.items() if name != default}
        goals.update({f"waypoint:{w['label']}": tuple(w["xy"]) for w in waypoints})
        obstacles = static + [item for hinge in hinges for item in hinge["poses"]["initial"]]
        report = check_navigation(obstacles, bounds, spawns[default][:2], goals,
                                  args.radius, args.height)
        report.update(world=args.world, variant=variant,
                      source_revision=world_tools.source_revision(model),
                      geometry_sha256=hashlib.sha256(scene.read_bytes()).hexdigest(),
                      waypoints=str(waypoint_file) if waypoint_file else None)
        world_tools.write_json(output / f"navigation_validation_{variant}.json", report)
        reached = sum(goal["reachable_from_default_spawn"] for goal in report["goals"].values())
        print(f"Navigation {report['status']}: {reached}/{len(goals)} goals reachable")
        if report["status"] != "PASS":
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
