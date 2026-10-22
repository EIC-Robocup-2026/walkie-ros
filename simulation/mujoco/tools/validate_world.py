#!/usr/bin/env python3
"""Validate an exported MuJoCo world before it is used from ROS.

Checks every variant in the world's world.toml: it compiles, has one analytic
floor plane that is the floor body's only collider, references every file in
its asset directories, keeps the physics of the default variant, settles
without warnings, and lets every limited hinge reach both limits under a small
torque. --robot also assembles each variant with Walkie through
robot_bringup.world_loader, as the ROS launch does, and checks the start pose
and the lidar readings. Checks that need world-specific names come from the
world's optional authoring/checks.py.

Run with the pixi Python from the repository root; no colcon build is needed:

    pixi run python simulation/mujoco/tools/validate_world.py chula_pat14_6 --robot

Reports go to simulation/mujoco/output/<world>/. --render needs an OpenGL
context; MUJOCO_GL=egl works on many headless machines.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import logging
from pathlib import Path
import re
import time
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import world_tools
from world_tools import world_loader
import yaml

PARITY_BODY_JOINT_FIELDS = (
    "qpos0", "body_pos", "body_quat", "body_ipos", "body_iquat", "body_mass",
    "body_inertia", "body_gravcomp", "jnt_type", "jnt_bodyid", "jnt_pos",
    "jnt_axis", "jnt_stiffness", "jnt_limited", "jnt_range", "jnt_margin",
    "jnt_solref", "jnt_solimp", "dof_damping", "dof_armature",
    "dof_frictionloss", "dof_solref", "dof_solimp",
)
PARITY_GEOM_FIELDS = (
    "geom_type", "geom_size", "geom_pos", "geom_quat", "geom_contype",
    "geom_conaffinity", "geom_condim", "geom_priority", "geom_solmix",
    "geom_solref", "geom_solimp", "geom_friction", "geom_margin", "geom_gap",
)
PARITY_OPTION_FIELDS = (
    "timestep", "gravity", "integrator", "cone", "solver", "iterations",
    "tolerance", "disableflags", "enableflags",
)


def warnings(data) -> dict:
    return {mujoco.mjtWarning(index).name: int(warning.number)
            for index, warning in enumerate(data.warning) if warning.number}


def fingerprint(path: Path) -> dict:
    return {"path": str(path.resolve()),
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def colliders(model) -> dict[str, int]:
    return {model.geom(index).name: index for index in range(model.ngeom)
            if model.geom_contype[index] or model.geom_conaffinity[index]}


def free_bodies(model) -> list[str]:
    return sorted({model.body(int(model.jnt_bodyid[joint])).name for joint in range(model.njnt)
                   if model.jnt_type[joint] == mujoco.mjtJoint.mjJNT_FREE})


def model_summary(model, data=None) -> dict:
    summary = {"nq": model.nq, "nv": model.nv, "nu": model.nu, "nbody": model.nbody,
               "ngeom": model.ngeom, "nmesh": model.nmesh, "ntex": model.ntex,
               "ncam": model.ncam, "model_bytes": model.nbuffer,
               "texture_bytes": model.ntexdata}
    if data is not None:
        summary.update(data_bytes=data.nbuffer, arena_bytes=data.narena)
    return summary


def floor_checks(model) -> dict:
    floors = [index for index in range(model.ngeom) if model.geom(index).name == "floor"]
    checks = {"one_geom_named_floor": len(floors) == 1}
    if len(floors) == 1:
        floor = floors[0]
        body = model.geom_bodyid[floor]
        others = [name for name, index in colliders(model).items()
                  if model.geom_bodyid[index] == body and index != floor]
        checks["floor_is_a_plane"] = bool(model.geom_type[floor] == mujoco.mjtGeom.mjGEOM_PLANE)
        checks["floor_plane_collides"] = bool(
            model.geom_contype[floor] and model.geom_conaffinity[floor])
        checks["floor_body_has_no_other_colliders"] = not others
    return checks


def asset_checks(directory: Path, manifest: dict) -> dict:
    """Report asset files that no variant references, in the folders variants use."""
    referenced = set()
    for relative in manifest["variants"].values():
        spec = mujoco.MjSpec.from_file(str(directory / relative))
        base = (directory / relative).parent
        for items, folder in ((spec.meshes, spec.meshdir), (spec.hfields, spec.meshdir),
                              (spec.textures, spec.texturedir)):
            for item in items:
                if item.file:
                    referenced.add((base / folder / item.file).resolve())
    folders = {path.parent for path in referenced}
    unreferenced = sorted(
        str(path.relative_to(directory.resolve())) for folder in folders
        for path in folder.iterdir() if path.is_file() and path.resolve() not in referenced
    )
    outside = sorted(str(path) for path in referenced
                     if not path.is_relative_to(directory.resolve()))
    return {"checks": {"every_asset_file_is_referenced": not unreferenced,
                       "assets_stay_inside_the_world": not outside},
            "referenced_files": len(referenced), "unreferenced_files": unreferenced,
            "outside_files": outside}


def map_checks(directory: Path, manifest: dict) -> dict:
    path = directory / manifest["map"]
    config = yaml.safe_load(path.read_text())
    image = path.parent / config.get("image", "")
    return {"checks": {"map_image_exists": image.is_file(),
                       "map_origin_has_zero_yaw": abs(float(config["origin"][2])) < 1e-9},
            "map": str(path), "image": str(image),
            "resolution_m": config.get("resolution"), "origin": config.get("origin")}


def physics_parity(base, variant) -> dict:
    """Compare everything that affects simulation; visuals may differ."""
    errors = []
    for name in ("nq", "nv", "nu", "njnt", "nbody", "neq", "npair", "nexclude"):
        if getattr(base, name) != getattr(variant, name):
            errors.append(f"{name} differs")
    if not errors:
        for kind, count in (("joint", base.njnt), ("body", base.nbody)):
            if [getattr(base, kind)(i).name for i in range(count)] != \
                    [getattr(variant, kind)(i).name for i in range(count)]:
                errors.append(f"{kind} names or order differ")
        for field in PARITY_BODY_JOINT_FIELDS:
            if not np.array_equal(getattr(base, field), getattr(variant, field)):
                errors.append(f"{field} differs")
    for field in PARITY_OPTION_FIELDS:
        if not np.array_equal(getattr(base.opt, field), getattr(variant.opt, field)):
            errors.append(f"option.{field} differs")
    base_geoms, variant_geoms = colliders(base), colliders(variant)
    if base_geoms.keys() != variant_geoms.keys():
        errors.append({"missing_colliders": sorted(base_geoms.keys() - variant_geoms.keys()),
                       "extra_colliders": sorted(variant_geoms.keys() - base_geoms.keys())})
    for name in sorted(base_geoms.keys() & variant_geoms.keys()):
        first, second = base_geoms[name], variant_geoms[name]
        if (base.body(int(base.geom_bodyid[first])).name
                != variant.body(int(variant.geom_bodyid[second])).name):
            errors.append(f"{name}: owning body differs")
        for field in PARITY_GEOM_FIELDS:
            if not np.array_equal(getattr(base, field)[first], getattr(variant, field)[second]):
                errors.append(f"{name}: {field} differs")
    return {"checks": {"physics_matches_default_variant": not errors}, "errors": errors,
            "colliders_compared": len(base_geoms)}


def settling(models: dict, default: str, seconds: float) -> dict:
    """Step every variant from its initial state and compare with the default."""
    datas = {name: mujoco.MjData(model) for name, model in models.items()}
    for name, model in models.items():
        mujoco.mj_forward(model, datas[name])
    base, base_data = models[default], datas[default]
    props = {name: base_data.xpos[base.body(name).id].copy() for name in free_bodies(base)}
    start = time.perf_counter()
    for _ in range(round(seconds / base.opt.timestep)):
        for name, model in models.items():
            mujoco.mj_step(model, datas[name])
    elapsed = time.perf_counter() - start
    deltas = {name: (base_data.xpos[base.body(name).id] - position).tolist()
              for name, position in props.items()}
    report = {"simulated_seconds": seconds, "physics_wall_seconds_all_variants": elapsed,
              "free_body_delta_m": deltas, "variants": {}}
    for name, model in models.items():
        data = datas[name]
        deepest = min((float(contact.dist) for contact in data.contact), default=0.)
        checks = {
            "finite_state": bool(np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()),
            "no_warnings": not warnings(data),
            "no_penetration_over_3mm": deepest > -.003,
        }
        entry = {"deepest_contact_m": deepest, "warnings": warnings(data),
                 "summary": model_summary(model, data)}
        if name == default:
            checks["free_bodies_stay_on_their_supports"] = all(
                np.linalg.norm(delta[:2]) < .02 and abs(delta[2]) < .015
                for delta in deltas.values())
        elif model.nq == base.nq:
            difference = float(np.max(np.abs(data.qpos - base_data.qpos), initial=0.))
            checks["trajectory_matches_default_variant"] = difference < 1e-6
            entry["max_qpos_difference_from_default"] = difference
        entry["checks"] = checks
        report["variants"][name] = entry
    return report


def hinge_tests(model, torque: float, duration: float) -> dict:
    """Push every limited hinge to both limits from the initial state."""
    tests = []
    for joint in range(model.njnt):
        if model.jnt_type[joint] != mujoco.mjtJoint.mjJNT_HINGE or not model.jnt_limited[joint]:
            continue
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        position, dof = int(model.jnt_qposadr[joint]), int(model.jnt_dofadr[joint])
        lower, upper = model.jnt_range[joint]
        for sign, target in ((1., upper), (-1., lower)):
            for _ in range(round(duration / model.opt.timestep)):
                data.qfrc_applied[dof] = sign * torque
                mujoco.mj_step(model, data)
            actual = float(data.qpos[position])
            tests.append({"joint": model.joint(joint).name, "torque_nm": sign * torque,
                          "duration_s": duration, "angle_rad": actual,
                          "expected_rad": float(target),
                          "passed": bool(abs(actual - target) < .01 and not warnings(data))})
    return {"checks": {"hinges_reach_both_limits": all(test["passed"] for test in tests)},
            "tests": tests}


def read_xml(path: Path) -> ET.Element:
    # MuJoCo accepts '--' inside comments; ElementTree does not.
    return ET.fromstring(re.sub(r"<!--.*?-->", "", path.read_text(), flags=re.DOTALL))


def multi_ray(model, data, origin, directions, minimum, maximum, groups=None):
    ids = np.full(len(directions), -1, dtype=np.int32)
    distances = np.zeros(len(directions))
    mujoco.mj_multiRay(model, data, origin, directions.reshape(-1), groups, True, -1,
                       ids, distances, None, len(directions), maximum)
    distances[(distances < minimum) | (distances >= maximum)] = -1.
    return distances, ids


def range_difference(first, second) -> dict:
    common = (first >= 0) & (second >= 0)
    errors = np.abs(first[common] - second[common])
    return {"common_hits": int(common.sum()),
            "hit_presence_disagreements": int(np.count_nonzero((first >= 0) != (second >= 0))),
            "range_difference_p99_m": float(np.percentile(errors, 99)) if len(errors) else None,
            "range_difference_max_m": float(errors.max()) if len(errors) else None}


def lidar_measurements(model, robot_source: ET.Element) -> dict:
    """Compare Walkie's lidar readings at the home pose with geometry raycasts."""
    # A periodic lidar keeps its last update time when MjData is reset, so a
    # fresh MjData gives a scan at time zero.
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    mujoco.mj_forward(model, data)
    instance = robot_source.find("extension/plugin/instance[@name='unitree_l2']")
    config = {entry.get("key"): entry.get("value") for entry in instance}
    azimuth_count, elevation_count = map(int, config["resolution"].split())
    azimuth = np.linspace(*map(float, config["azimuth_range"].split()), azimuth_count)
    elevation = np.linspace(*map(float, config["elevation_range"].split()), elevation_count)
    elevations, azimuths = np.meshgrid(elevation, azimuth, indexing="ij")
    local = np.stack((np.cos(azimuths) * np.cos(elevations),
                      np.sin(azimuths) * np.cos(elevations), np.sin(elevations)),
                     axis=-1).reshape(-1, 3)
    site = int(model.sensor("unitree_l2").objid[0])
    directions = np.ascontiguousarray(local @ data.site_xmat[site].reshape(3, 3).T)
    readings = data.sensor("unitree_l2").data.copy()
    expected, hit_ids = multi_ray(model, data, data.site_xpos[site].copy(), directions,
                                  float(config["min_range"]), float(config["max_range"]))

    builtin, reference, builtin_hits = [], [], []
    for prefix in ("front_lidar", "back_lidar"):
        beam = 0
        while mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, f"{prefix}-{beam}") >= 0:
            sensor = model.sensor(f"{prefix}-{beam}")
            site = int(sensor.objid[0])
            geom = np.full(1, -1, dtype=np.int32)
            distance = mujoco.mj_ray(model, data, data.site_xpos[site],
                                     data.site_xmat[site].reshape(3, 3)[:, 2], None, True,
                                     int(model.site_bodyid[site]), geom)
            reference.append(float(distance))
            builtin.append(float(data.sensordata[int(sensor.adr[0])]))
            builtin_hits.append(model.geom(int(geom[0])).name if geom[0] >= 0 else None)
            beam += 1
    plugin_error = float(np.max(np.abs(readings - expected)))
    builtin_error = float(np.max(np.abs(np.asarray(builtin) - reference))) if builtin else None
    return {
        "checks": {
            "unitree_matches_geometry_rays": plugin_error < 1e-5,
            "builtin_2d_lidars_exist": bool(builtin),
            "builtin_2d_lidars_match_geometry_rays": builtin_error is not None
            and builtin_error < 1e-5,
        },
        "unitree_readings": len(readings),
        "unitree_hits": int(np.count_nonzero(readings > 0)),
        "unitree_max_plugin_geometry_error_m": plugin_error,
        "builtin_2d_beams": len(builtin),
        "builtin_2d_max_sensor_geometry_error_m": builtin_error,
        # Raw arrays for comparisons and world hooks; removed before writing.
        "_ranges": readings, "_hit_geoms": hit_ids, "_builtin_hit_geoms": builtin_hits,
    }


def robot_report(model, robot_path: Path, steps: int) -> dict:
    """Step the assembled world from Walkie's home keyframe at the world spawn."""
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    mujoco.mj_forward(model, data)
    robot_root = model.body("base_footprint").id
    robot_bodies = {index for index in range(model.nbody)
                    if model.body_rootid[index] == robot_root}

    def deep_contacts() -> list[dict]:
        return [{"geoms": [model.geom(int(geom)).name for geom in contact.geom],
                 "penetration_m": float(-contact.dist)}
                for contact in data.contact if contact.dist < -.005]

    initial_base, initial_deep = data.qpos[:7].copy(), deep_contacts()
    robot_geoms = [index for index in range(model.ngeom)
                   if model.geom_bodyid[index] in robot_bodies and model.geom_contype[index]]
    world_geoms = [index for index in colliders(model).values()
                   if model.geom_bodyid[index] not in robot_bodies
                   and model.geom(index).name != "floor"]
    nearest = None
    for robot_geom in robot_geoms:
        for world_geom in world_geoms:
            distance = mujoco.mj_geomDistance(model, data, robot_geom, world_geom, 1., None)
            if distance < 1. and (nearest is None or distance < nearest["distance_m"]):
                nearest = {"distance_m": float(distance),
                           "geoms": [model.geom(robot_geom).name, model.geom(world_geom).name]}
    props = {model.body(index).name: data.xpos[index].copy()
             for index in range(model.nbody) if index not in robot_bodies
             and model.body(index).name in free_bodies(model)}
    start = time.perf_counter()
    for _ in range(steps):
        mujoco.mj_step(model, data)
    elapsed = time.perf_counter() - start
    final_deep = deep_contacts()
    moved = {name: float(np.linalg.norm(data.xpos[model.body(name).id] - position))
             for name, position in props.items()}
    robot = mujoco.MjModel.from_xml_path(str(robot_path))
    source_home = robot.key("home").qpos
    home = model.key("home").id
    wheel_pairs = sum(
        {model.geom(int(model.pair_geom1[pair])).name,
         model.geom(int(model.pair_geom2[pair])).name} == {wheel, "floor"}
        for pair in range(model.npair) for wheel in world_loader.WHEEL_GEOMS)
    translation = float(np.linalg.norm(data.qpos[:3] - initial_base[:3]))
    checks = {
        "finite_state": bool(np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()),
        "finite_sensors": bool(np.isfinite(data.sensordata).all()),
        "no_physics_warnings": not warnings(data),
        "no_contacts_deeper_than_5mm": not initial_deep and not final_deep,
        "robot_spawn_has_no_world_overlap": nearest is None or nearest["distance_m"] >= 0,
        "startup_and_home_base_pose_match": bool(
            np.allclose(model.qpos0[:7], initial_base, atol=1e-9, rtol=0)),
        "upper_body_home_pose_preserved": bool(np.allclose(
            model.key_qpos[home, 7:robot.nq], source_home[7:], atol=1e-9, rtol=0)),
        "robot_translation_below_5cm": translation < .05,
        "free_body_translations_below_10cm": all(value < .1 for value in moved.values()),
        "wheel_floor_pairs_preserved": wheel_pairs == len(world_loader.WHEEL_GEOMS),
    }
    return {"checks": checks, "summary": model_summary(model, data),
            "simulation": {"steps": steps, "duration_s": float(data.time),
                           "wall_time_s": elapsed, "warnings": warnings(data)},
            "robot": {"initial_freejoint_qpos": initial_base.tolist(),
                      "final_freejoint_qpos": data.qpos[:7].tolist(),
                      "translation_m": translation,
                      "nearest_initial_world_clearance_ignoring_masks": nearest},
            "free_body_translation_m": moved,
            "contacts": {"initial_deeper_than_5mm": initial_deep,
                         "final_deeper_than_5mm": final_deep}}


def robot_checks(world, robot_path: Path, pose, steps: int, hooks) -> dict:
    wheel_scene = robot_path.parent / "scene.xml"
    robot_source = read_xml(robot_path)
    reports, lidars = {}, {}
    for variant, relative in world.manifest["variants"].items():
        tree = world_loader.assemble_world(robot_path, world.directory / relative,
                                           wheel_scene, pose)
        model = mujoco.MjModel.from_xml_string(ET.tostring(tree, encoding="unicode"))
        report = robot_report(model, robot_path, steps)
        lidar = lidar_measurements(model, robot_source)
        lidars[variant] = lidar
        if hooks is not None and hasattr(hooks, "robot_checks"):
            report["world_checks"] = hooks.robot_checks(world, variant, model, {
                "unitree_ranges": lidar["_ranges"],
                "unitree_hit_geoms": lidar["_hit_geoms"],
                "builtin_hit_geoms": lidar["_builtin_hit_geoms"],
            })
        report["lidar"] = lidar
        reports[variant] = report
    default = world.manifest["default_variant"]
    for variant, lidar in lidars.items():
        if variant != default:
            comparison = range_difference(lidars[default]["_ranges"], lidar["_ranges"])
            p99 = comparison["range_difference_p99_m"]
            lidar["checks"]["unitree_p99_range_change_from_default_below_3cm"] = (
                p99 is not None and p99 < .03)
            lidar["checks"]["unitree_hit_agreement_with_default_above_99pct"] = (
                comparison["hit_presence_disagreements"] < len(lidar["_ranges"]) * .01)
            lidar["compared_with_default"] = comparison
    for lidar in lidars.values():
        for key in [key for key in lidar if key.startswith("_")]:
            del lidar[key]
    return reports


def render(world, output: Path, hooks) -> list[str]:
    """Render every scene camera and the world's extra views after settling."""
    from PIL import Image

    views = getattr(hooks, "VIEWS", {}) if hooks is not None else {}
    written = []
    for variant in world.manifest["variants"]:
        model = world.models[variant]
        data = mujoco.MjData(model)
        for _ in range(500):
            mujoco.mj_step(model, data)
        destination = output / variant
        destination.mkdir(parents=True, exist_ok=True)
        fovy = model.vis.global_.fovy
        with mujoco.Renderer(model, height=960, width=1280) as renderer:
            jobs = [(model.camera(index).name, None) for index in range(model.ncam)]
            jobs += [(name, view) for name, view in views.items()
                     if variant in view.get("variants", world.manifest["variants"])]
            for name, view in jobs:
                camera = mujoco.MjvCamera()
                if view is None:
                    camera.type = mujoco.mjtCamera.mjCAMERA_FIXED
                    camera.fixedcamid = model.camera(name).id
                else:
                    position = np.asarray(view["position"], dtype=float)
                    offset = position - np.asarray(view["target"], dtype=float)
                    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
                    camera.lookat[:] = view["target"]
                    camera.distance = float(np.linalg.norm(offset))
                    camera.azimuth = float(np.degrees(np.arctan2(offset[1], offset[0])) + 180)
                    camera.elevation = float(-np.degrees(np.arcsin(offset[2] /
                                                                   camera.distance)))
                    # A free camera uses the model's global field of view.
                    model.vis.global_.fovy = float(view.get("fovy", fovy))
                renderer.update_scene(data, camera=camera)
                model.vis.global_.fovy = fovy
                path = destination / f"{name}.png"
                Image.fromarray(renderer.render()).save(path)
                written.append(str(path))
    return written


def collect_checks(report, prefix="") -> dict[str, bool]:
    """Flatten every nested ``checks`` mapping into dotted names."""
    found = {}
    if isinstance(report, dict):
        for key, value in report.items():
            if key == "checks" and isinstance(value, dict):
                found.update({f"{prefix}{name}": bool(result) for name, result in value.items()})
            else:
                found.update(collect_checks(value, f"{prefix}{key}."))
    elif isinstance(report, list):
        for index, value in enumerate(report):
            found.update(collect_checks(value, f"{prefix}{index}."))
    return found


def finish(path: Path, report: dict) -> bool:
    checks = collect_checks(report)
    failed = sorted(name for name, passed in checks.items() if not passed)
    report["status"] = "FAIL" if failed else "PASS"
    report["failed_checks"] = failed
    world_tools.write_json(path, report)
    print(f"{report['status']}: {len(checks) - len(failed)}/{len(checks)} checks passed")
    for name in failed:
        print(f"  failed: {name}")
    return not failed


def main(argv=None) -> int:
    # Show the world settings that world_loader.assemble_world drops (--robot).
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    world_tools.add_world_arguments(parser)
    parser.add_argument("--seconds", type=float, default=10.,
                        help="Settling time per variant (default: %(default)s)")
    parser.add_argument("--hinge-torque", type=float, default=5.,
                        help="Torque in Nm that must move each hinge to its limits")
    parser.add_argument("--robot", action="store_true",
                        help="Also assemble every variant with Walkie and check it")
    parser.add_argument("--robot-mjcf", type=Path, default=world_tools.ROBOT_MJCF,
                        help="Robot MJCF (default: %(default)s)")
    parser.add_argument("--robot-steps", type=int, default=1000)
    spawn = parser.add_mutually_exclusive_group()
    spawn.add_argument("--spawn", help="Named spawn; default: the manifest's default_spawn")
    spawn.add_argument("--spawn-pose", type=float, nargs=3, metavar=("X", "Y", "YAW_DEG"))
    parser.add_argument("--render", action="store_true",
                        help="Render each camera of each variant to PNG")
    args = parser.parse_args(argv)
    if args.seconds <= 0:
        parser.error("--seconds must be positive")

    directory, manifest, output = world_tools.open_world(args)
    hooks = world_tools.load_hooks(directory)
    default = manifest["default_variant"]
    models = {variant: mujoco.MjModel.from_xml_path(str(directory / relative))
              for variant, relative in manifest["variants"].items()}
    world = SimpleNamespace(name=args.world, directory=directory, manifest=manifest,
                            models=models)

    report = {
        "world": args.world,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "mujoco_version": mujoco.__version__,
        "source_revision": world_tools.source_revision(models[default]),
        "sources": {"manifest": fingerprint(directory / world_loader.MANIFEST_NAME),
                    **{variant: fingerprint(directory / relative)
                       for variant, relative in manifest["variants"].items()}},
        "default_variant": default,
        "floor": {variant: {"checks": floor_checks(model)} for variant, model in models.items()},
        "assets": asset_checks(directory, manifest),
        "map": map_checks(directory, manifest),
        "physics_parity": {variant: physics_parity(models[default], model)
                           for variant, model in models.items() if variant != default},
        "settling": settling(models, default, args.seconds),
        "hinges": {variant: hinge_tests(model, args.hinge_torque, 4.)
                   for variant, model in models.items()},
    }
    if hooks is not None and hasattr(hooks, "scene_checks"):
        report["world_checks"] = hooks.scene_checks(world)
    passed = finish(output / "validation_report.json", report)

    if args.robot:
        pose = world_loader.resolve_spawn(
            args.world, manifest, args.spawn,
            " ".join(map(str, args.spawn_pose)) if args.spawn_pose else None)
        robot_path = args.robot_mjcf.resolve()
        integration = {
            "world": args.world,
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "spawn_pose": pose,
            "sources": {"robot": fingerprint(robot_path)},
            "variants": robot_checks(world, robot_path, pose, args.robot_steps, hooks),
        }
        passed = finish(output / "integration_validation.json", integration) and passed
    if args.render:
        for path in render(world, output, hooks):
            print(f"Wrote {path}")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
