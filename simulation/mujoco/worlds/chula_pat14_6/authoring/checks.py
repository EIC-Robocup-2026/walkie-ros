"""World-specific hooks for the tools in simulation/mujoco/tools/.

validate_world.py and make_map_overlay.py import this file when it exists. Every
hook is optional:

- ``scene_checks(world)``: extra checks on the compiled variants. ``world`` has
  ``name``, ``directory``, ``manifest`` and ``models`` (variant -> MjModel).
- ``robot_checks(world, variant, model, lidar)``: extra checks on a variant
  assembled with Walkie. ``lidar`` holds the head lidar ranges and hit geom ids
  and the geom names hit by the 2D lidar beams.
- ``VIEWS``: extra rendered views, name -> position, target and vertical
  field of view in degrees, optionally limited to some variants.
- ``OVERLAY``: default inputs of make_map_overlay.py, relative to authoring/.

Each check hook returns a mapping whose nested ``checks`` entries are booleans.
The names below come from the ChulaPat14_6 export; update them when the
exported scene renames these bodies or geoms.
"""

import mujoco
import numpy as np

TASK_OBJECT_COUNT = 12
# Six shared task-object meshes plus one fitted floor or baked room mesh.
MESH_COUNT = 7
FLOOR_BODY = "chula_floor"
FURNITURE = (
    "chula_sofa", "chula_tv_desk", "chula_coffee_table", "chula_kitchen_table",
    "chula_bar", "chula_shelf", "chula_stool_1", "chula_stool_2", "chula_stool_3",
)
TV_PARTS = {
    "desk_top": "chula_tv_desk_00", "housing": "chula_tv_desk_06",
    "display": "chula_tv_desk_07", "base": "chula_tv_desk_08", "neck": "chula_tv_desk_09",
}
DISH_TEST = {"object": "task_apple_2", "joint": "task_apple_2_free",
             "dish": "task_red_plate_1"}
BAKED_VARIANT = "baked"
BAKED_VISUAL = "chula_baked_static_visual"
# Uncluttered floor points in both rooms, 0.35 m above the floor.
FLOOR_PROBES = {"living": (1.1, -2.3, .35), "kitchen": (.55, -6.1, .35)}

VIEWS = {
    "baked_kitchen_reference": {"position": (2.6, -3.55, 1.95), "target": (1.35, -6.4, .85),
                                "fovy": 80., "variants": (BAKED_VARIANT,)},
    "baked_living_reference": {"position": (1.8, -.55, 1.67), "target": (1.8, -4.8, .95),
                               "fovy": 83., "variants": (BAKED_VARIANT,)},
    "tv_front_detail": {"position": (2.12, -.9, 1.29), "target": (1.06, -1.65, .96),
                        "fovy": 48.},
    "tv_rear_detail": {"position": (.96, -2.30, 1.10), "target": (1.045, -1.65, .96),
                       "fovy": 83.},
}
OVERLAY = {
    "reference_image": "source/map_og.pgm",
    "waypoints": "source/30_16102026_ChulaPat14_6_element.json",
}


def _warnings(data):
    return {mujoco.mjtWarning(i).name: int(w.number)
            for i, w in enumerate(data.warning) if w.number}


def _bounds(model, data, name):
    geom = model.geom(name).id
    rotation = data.geom_xmat[geom].reshape(3, 3)
    center = data.geom_xpos[geom] + rotation @ model.geom_aabb[geom, :3]
    half_extent = np.abs(rotation) @ model.geom_aabb[geom, 3:]
    return center - half_extent, center + half_extent


def tv_checks(model, data):
    """Catch misplaced TV parts on the TV desk."""
    top_lo, top_hi = _bounds(model, data, TV_PARTS["desk_top"])
    base_lo, base_hi = _bounds(model, data, TV_PARTS["base"])
    neck_lo, neck_hi = _bounds(model, data, TV_PARTS["neck"])
    housing_lo, housing_hi = _bounds(model, data, TV_PARTS["housing"])
    display_lo, _ = _bounds(model, data, TV_PARTS["display"])
    return {
        "tv_base_rests_on_desk": bool(abs(base_lo[2] - top_hi[2]) < 1e-6),
        "tv_base_fits_on_desk": bool(np.all(base_lo[:2] >= top_lo[:2])
                                     and np.all(base_hi[:2] <= top_hi[:2])),
        "tv_neck_rests_on_base": bool(abs(neck_lo[2] - base_hi[2]) < 1e-6),
        "tv_neck_connects_to_housing": bool(np.all(np.minimum(neck_hi, housing_hi)
                                                   - np.maximum(neck_lo, housing_lo) > 0)),
        "tv_stand_below_active_screen": bool(max(base_hi[2], neck_hi[2]) <= display_lo[2]),
        "tv_has_visible_neck": bool(housing_lo[2] - base_hi[2] > .05),
    }


def furniture_clearances(model, data, cap=.10):
    """Find furniture embedded in walls or other furniture, visual trim included."""
    static = {model.body(i).name: [g for g in range(model.ngeom) if model.geom_bodyid[g] == i]
              for i in range(1, model.nbody)
              if model.body_weldid[i] == 0 and model.body(i).name != FLOOR_BODY}
    nearest = {}
    for name in FURNITURE:
        others = [g for other, geoms in static.items() if other != name for g in geoms]
        for geom in static[name]:
            for other in others:
                distance = float(mujoco.mj_geomDistance(model, data, geom, other, cap, None))
                if name not in nearest or distance < nearest[name]["distance_m"]:
                    nearest[name] = {"distance_m": distance, "geom": model.geom(geom).name,
                                     "other_geom": model.geom(other).name}
    return nearest


def dish_test(model):
    """Drop the apple into the dish: the dish collider must be open, not its hull."""
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    item = model.body(DISH_TEST["object"]).id
    dish = model.body(DISH_TEST["dish"]).id
    address = int(model.joint(DISH_TEST["joint"]).qposadr[0])
    position = data.xpos[dish].copy()
    position[2] += .15
    data.qpos[address:address + 3] = position
    data.qpos[address + 3:address + 7] = [1, 0, 0, 0]
    for _ in range(1500):
        mujoco.mj_step(model, data)
    relative = (data.xpos[item] - data.xpos[dish]).tolist()
    return relative, {
        "apple_settles_inside_dish": bool(np.linalg.norm(relative[:2]) < .05
                                          and .025 < relative[2] < .042),
        "no_dish_test_warnings": not _warnings(data),
    }


def baked_checks(model):
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    baked = model.geom(BAKED_VISUAL).id
    rays = {}
    for name, origin in FLOOR_PROBES.items():
        geom = np.full(1, -1, dtype=np.int32)
        distance = mujoco.mj_ray(model, data, np.asarray(origin, dtype=float),
                                 np.array([0., 0., -1.]), None, True, -1, geom)
        rays[name] = {"distance_m": float(distance),
                      "geom": model.geom(int(geom[0])).name if geom[0] >= 0 else None}
    return rays, {
        "baked_visual_has_no_contacts": bool(model.geom_contype[baked] == 0
                                             and model.geom_conaffinity[baked] == 0),
        "baked_visual_is_group_1": int(model.geom_group[baked]) == 1,
        "baked_floor_is_ray_visible": all(ray["geom"] == BAKED_VISUAL
                                          and .30 < ray["distance_m"] < .36
                                          for ray in rays.values()),
    }


def scene_checks(world):
    default = world.manifest["default_variant"]
    report = {"variants": {}}
    for variant, model in world.models.items():
        task_objects = [model.body(i).name for i in range(model.nbody)
                        if model.body(i).name.startswith("task_") and model.body_jntnum[i]]
        entry = {"checks": {"twelve_task_objects": len(task_objects) == TASK_OBJECT_COUNT,
                            "task_objects_share_meshes": model.nmesh == MESH_COUNT}}
        if variant == default:
            # The other variants share these colliders (physics parity), but
            # their merged visuals cannot be checked part by part.
            data = mujoco.MjData(model)
            mujoco.mj_forward(model, data)
            entry["tv"] = {"checks": tv_checks(model, data)}
            nearest = furniture_clearances(model, data)
            entry["furniture"] = {
                "checks": {"furniture_has_no_intersections": all(
                    item["distance_m"] >= -.001 for item in nearest.values())},
                "nearest_clearance_m": nearest,
                "note": "Distances at the 0.10 m cap mean at least 0.10 m.",
            }
            relative, checks = dish_test(model)
            entry["dish"] = {"checks": checks, "apple_relative_to_dish_m": relative}
        if variant == BAKED_VARIANT:
            rays, checks = baked_checks(model)
            entry["baked"] = {"checks": checks, "floor_rays": rays}
        report["variants"][variant] = entry
    return report


def robot_checks(world, variant, model, lidar):
    if variant != BAKED_VARIANT:
        return {}
    baked = model.geom(BAKED_VISUAL).id
    head_hits = int(np.count_nonzero((lidar["unitree_hit_geoms"] == baked)
                                     & (lidar["unitree_ranges"] >= 0)))
    planar_hits = sum(name == BAKED_VISUAL for name in lidar["builtin_hit_geoms"])
    return {"checks": {"head_lidar_sees_baked_visual": head_hits > 0,
                       "planar_lidars_see_baked_visual": planar_hits > 0},
            "head_lidar_baked_hits": head_hits, "planar_lidar_baked_hits": planar_hits}
