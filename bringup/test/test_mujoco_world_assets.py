"""Compile every installed MuJoCo world variant with the installed Walkie model.

Run after a build: the worlds come from the walkie_mujoco_worlds share and the
robot from the walkie_description share, both found through ament_index, so
the test covers the installed files and the CMake install rules. MuJoCo comes
from the pixi environment, which also provides the mujoco.plugin.lidar
extension that walkie.xml loads.

    pixi run build-pkg walkie_description walkie_mujoco_worlds robot_bringup
    pixi run test --packages-select robot_bringup --ctest-args " -LE" linter
"""

import math
from pathlib import Path
import re

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import numpy as np
import pytest
from robot_bringup import world_loader
import yaml

mujoco = pytest.importorskip("mujoco")

SETTLE_STEPS = 150          # 0.3 s at the 2 ms timestep
SETTLE_DRIFT_M = 0.01       # base translation allowed while settling at the spawn
HEADER_CLASS = re.compile(r"^\s*(\w+)\s+contype\s+(\d+)\s+conaffinity\s+(\d+)", re.MULTILINE)


def _share(package: str) -> Path | None:
    try:
        return Path(get_package_share_directory(package))
    except PackageNotFoundError:
        return None


WORLDS_SHARE = _share(world_loader.WORLDS_PACKAGE)
DESCRIPTION_SHARE = _share("walkie_description")
BRINGUP_SHARE = _share("robot_bringup")
WORLDS_ROOT = WORLDS_SHARE / "worlds" if WORLDS_SHARE else None
ROBOT_MJCF = DESCRIPTION_SHARE / "mujoco" / "walkie.xml" if DESCRIPTION_SHARE else None
WHEEL_SCENE = DESCRIPTION_SHARE / "mujoco" / "scene.xml" if DESCRIPTION_SHARE else None


def _world_variants() -> list:
    if WORLDS_ROOT is None:
        return []
    params = []
    for world in world_loader.available_worlds(WORLDS_ROOT):
        manifest = world_loader.load_manifest(WORLDS_ROOT / world / world_loader.MANIFEST_NAME)
        params += [pytest.param((world, variant), id=f"{world}-{variant}")
                   for variant in manifest["variants"]]
    return params


def test_worlds_and_robot_are_installed():
    assert WORLDS_SHARE is not None, f"{world_loader.WORLDS_PACKAGE} is not installed"
    assert DESCRIPTION_SHARE is not None, "walkie_description is not installed"
    assert ROBOT_MJCF.is_file() and WHEEL_SCENE.is_file()
    assert _world_variants(), f"no world.toml below {WORLDS_ROOT}"


INSTALLED_SUFFIXES = {".xml", ".obj", ".stl", ".msh", ".png", ".yaml", ".pgm"}


def test_install_holds_only_launch_files_linked_under_symlink_install():
    """Check the install rules of simulation/mujoco/CMakeLists.txt.

    Only the manifest and the files the launch reads are installed; authoring
    data and Blender projects stay in the source tree. When the package was
    built with --symlink-install (its package.xml is a link), every world file
    must be a link too, so that an edit applies without a rebuild.
    """
    assert WORLDS_ROOT is not None, f"{world_loader.WORLDS_PACKAGE} is not installed"
    files = [path for path in WORLDS_ROOT.rglob("*") if not path.is_dir()]
    assert files
    relative = [path.relative_to(WORLDS_ROOT) for path in files]
    unexpected = [str(path) for path in relative
                  if "authoring" in path.parts or "__pycache__" in path.parts
                  or not (path.name == world_loader.MANIFEST_NAME
                          or path.suffix in INSTALLED_SUFFIXES)]
    assert not unexpected
    if (WORLDS_SHARE / "package.xml").is_symlink():
        copies = [str(path.relative_to(WORLDS_ROOT)) for path in files
                  if not path.is_symlink()]
        assert not copies, "world files were copied, not linked"


@pytest.fixture(scope="module")
def robot():
    return mujoco.MjModel.from_xml_path(str(ROBOT_MJCF))


@pytest.fixture(scope="module")
def wheel_scene():
    return mujoco.MjModel.from_xml_path(str(WHEEL_SCENE))


@pytest.fixture(scope="module", params=_world_variants())
def merged(request, tmp_path_factory):
    """Prepare a world variant the way the launch files do and compile it."""
    world, variant = request.param
    directory, manifest = world_loader.load_world(world)
    prepared = world_loader.prepare_world(
        world=world, variant=variant, worlds_dir=None,
        output_dir=tmp_path_factory.mktemp("world_cache"),
        robot_path=ROBOT_MJCF, wheel_scene_path=WHEEL_SCENE,
    )
    model = mujoco.MjModel.from_xml_path(prepared["model_path"])
    scene = directory / manifest["variants"][variant]
    return {"world": world, "variant": variant, "manifest": manifest, "scene": scene,
            "prepared": prepared, "model": model}


def _robot_bodies(model, robot) -> np.ndarray:
    """Body ids of the robot in ``model``: the loader puts the robot's bodies first."""
    bodies = np.arange(1, robot.nbody)
    assert [model.body(int(i)).name for i in bodies] == [
        robot.body(int(i)).name for i in bodies]
    return bodies


def _robot_geoms(model, robot) -> np.ndarray:
    return np.flatnonzero((model.geom_bodyid >= 1) & (model.geom_bodyid < robot.nbody))


def test_loader_collision_masks_match_the_walkie_header():
    """The masks the loader writes into world geoms are the classes walkie.xml documents."""
    header = ROBOT_MJCF.read_text().split("-->", 1)[0]
    classes = {name: (int(contype), int(conaffinity))
               for name, contype, conaffinity in HEADER_CLASS.findall(header)}
    assert classes["FLOOR"] == world_loader.FLOOR_MASK
    assert classes["FURN"] == world_loader.FURNITURE_MASK
    assert classes["OBJ"] == world_loader.OBJECT_MASK
    wheels = re.search(r"Wheel capsules (.*?) have contype = conaffinity = 0", header, re.DOTALL)
    assert wheels is not None
    assert set(re.findall(r"\w+_wheel_contact", wheels[1])) == set(world_loader.WHEEL_GEOMS)

    def collide(first, second):
        return bool(first[0] & second[1] or second[0] & first[1])

    world_classes = (world_loader.FLOOR_MASK, world_loader.FURNITURE_MASK,
                     world_loader.OBJECT_MASK)
    for name in ("BASE", "BODY"):
        assert not any(collide(classes[name], mask) for mask in world_classes), name
    for name in ("ARM", "GRIP"):
        assert collide(classes[name], world_loader.FURNITURE_MASK), name
        assert collide(classes[name], world_loader.OBJECT_MASK), name
        assert not collide(classes[name], world_loader.FLOOR_MASK), name
    assert not collide(world_loader.FURNITURE_MASK, world_loader.FURNITURE_MASK)
    assert not collide(world_loader.FURNITURE_MASK, world_loader.FLOOR_MASK)
    assert collide(world_loader.OBJECT_MASK, world_loader.FLOOR_MASK)


def test_robot_geoms_use_the_documented_classes(robot):
    header = ROBOT_MJCF.read_text().split("-->", 1)[0]
    robot_classes = {(int(contype), int(conaffinity))
                     for name, contype, conaffinity in HEADER_CLASS.findall(header)
                     if name in {"BASE", "BODY", "ARM", "GRIP"}}
    for geom in range(robot.ngeom):
        name = robot.geom(geom).name
        masks = (int(robot.geom_contype[geom]), int(robot.geom_conaffinity[geom]))
        if name in world_loader.WHEEL_GEOMS:
            assert masks == world_loader.NO_COLLISION_MASK, name
            assert robot.geom_type[geom] == mujoco.mjtGeom.mjGEOM_CAPSULE, name
        else:
            assert masks in robot_classes | {world_loader.NO_COLLISION_MASK}, (name, masks)


def test_wheel_floor_pairs_match_the_reference_scene(merged, wheel_scene):
    model = merged["model"]
    floor = model.geom("floor")
    assert floor.type[0] == mujoco.mjtGeom.mjGEOM_PLANE
    assert (int(floor.contype[0]), int(floor.conaffinity[0])) == world_loader.FLOOR_MASK

    def pairs(source):
        found = {}
        for pair in range(source.npair):
            names = {source.geom(int(source.pair_geom1[pair])).name,
                     source.geom(int(source.pair_geom2[pair])).name}
            for wheel in world_loader.WHEEL_GEOMS:
                if names == {wheel, "floor"}:
                    found.setdefault(wheel, []).append(pair)
        return found

    merged_pairs, reference_pairs = pairs(model), pairs(wheel_scene)
    for wheel in world_loader.WHEEL_GEOMS:
        assert len(merged_pairs.get(wheel, [])) == 1, wheel
        assert model.geom(wheel).type[0] == mujoco.mjtGeom.mjGEOM_CAPSULE
        (pair,), (reference,) = merged_pairs[wheel], reference_pairs[wheel]
        for field in ("pair_dim", "pair_friction", "pair_solref", "pair_solreffriction",
                      "pair_solimp", "pair_margin", "pair_gap"):
            np.testing.assert_array_equal(getattr(model, field)[pair],
                                          getattr(wheel_scene, field)[reference], field)


def test_robot_inertia_is_unchanged(merged, robot):
    model = merged["model"]
    bodies = _robot_bodies(model, robot)
    # Massless bodies (base_footprint, sensor frames) report their body pose as
    # inertial frame, and the spawn moves base_footprint.
    massive = bodies[robot.body_mass[bodies] > 0]
    for field, selected in (("body_mass", bodies), ("body_inertia", bodies),
                            ("body_ipos", massive), ("body_iquat", massive)):
        np.testing.assert_allclose(getattr(model, field)[selected],
                                   getattr(robot, field)[selected],
                                   rtol=0, atol=1e-9, err_msg=field)
    assert model.body_subtreemass[1] == pytest.approx(robot.body_subtreemass[1], abs=1e-6)


def test_robot_joints_geoms_and_actuators_are_unchanged(merged, robot):
    model = merged["model"]
    assert [model.joint(i).name for i in range(robot.njnt)] == [
        robot.joint(i).name for i in range(robot.njnt)]
    for field in ("dof_damping", "dof_armature", "dof_frictionloss"):
        np.testing.assert_array_equal(getattr(model, field)[:robot.nv], getattr(robot, field),
                                      field)
    for field in ("jnt_stiffness", "jnt_range", "jnt_limited", "jnt_type"):
        np.testing.assert_array_equal(getattr(model, field)[:robot.njnt],
                                      getattr(robot, field), field)
    # The floating base's spring reference is its start pose, which the spawn sets.
    np.testing.assert_array_equal(model.qpos_spring[7:robot.nq], robot.qpos_spring[7:])
    geoms = _robot_geoms(model, robot)
    reference = np.flatnonzero(robot.geom_bodyid >= 1)
    assert len(geoms) == len(reference)
    for field in ("geom_type", "geom_size", "geom_contype", "geom_conaffinity", "geom_condim",
                  "geom_friction", "geom_solref", "geom_solimp", "geom_solmix",
                  "geom_margin", "geom_gap", "geom_priority", "geom_group"):
        np.testing.assert_array_equal(getattr(model, field)[geoms],
                                      getattr(robot, field)[reference], field)
    # dampratio gains are computed from the joint-space inertia at the start
    # pose; the spawn yaw changes them in the last bits only.
    for field in ("actuator_gainprm", "actuator_biasprm", "actuator_ctrlrange",
                  "actuator_forcerange", "actuator_gear"):
        np.testing.assert_allclose(getattr(model, field)[:robot.nu], getattr(robot, field),
                                   rtol=1e-12, atol=0, err_msg=field)


def test_physics_and_camera_settings_come_from_the_robot(merged, robot, wheel_scene):
    model = merged["model"]
    assert repr(model.opt) == repr(robot.opt)
    assert (model.opt.iterations, model.opt.timestep, model.opt.integrator) == (
        robot.opt.iterations, robot.opt.timestep, robot.opt.integrator)
    assert repr(model.vis.quality) == repr(robot.vis.quality)
    assert (model.vis.global_.offwidth, model.vis.global_.offheight) == (
        robot.vis.global_.offwidth, robot.vis.global_.offheight)
    # Camera near and far planes scale with the extent: keep the reference scene's.
    assert model.stat.extent == pytest.approx(wheel_scene.stat.extent)
    assert model.vis.map.znear == pytest.approx(wheel_scene.vis.map.znear)
    assert model.vis.map.zfar == pytest.approx(wheel_scene.vis.map.zfar)


def test_arena_is_at_least_what_the_world_requests(merged):
    size = world_loader._read_mjcf(merged["scene"]).find("size")
    if size is not None and size.get("memory"):
        assert merged["model"].narena >= world_loader._memory_bytes(size.get("memory"))


def test_home_keyframe_places_the_base_at_the_default_spawn(merged, robot):
    model, manifest = merged["model"], merged["manifest"]
    x, y, yaw_deg = manifest["spawns"][manifest["default_spawn"]]
    assert merged["prepared"]["spawn_pose"] == (x, y, yaw_deg)
    home, robot_home = model.key("home"), robot.key("home")
    qpos = home.qpos
    yaw = math.radians(yaw_deg)
    np.testing.assert_allclose(qpos[:2], [x, y], atol=1e-9)
    assert qpos[2] == pytest.approx(robot_home.qpos[2], abs=1e-12)
    quaternion = np.array([math.cos(yaw / 2), 0., 0., math.sin(yaw / 2)])
    np.testing.assert_allclose(qpos[3:7], quaternion, atol=1e-9)
    np.testing.assert_allclose(qpos[7:robot.nq], robot_home.qpos[7:], atol=1e-12)
    np.testing.assert_allclose(home.ctrl[:robot.nu], robot_home.ctrl, atol=1e-12)
    np.testing.assert_allclose(model.qpos0[:2], [x, y], atol=1e-9)
    np.testing.assert_allclose(model.qpos0[3:7], quaternion, atol=1e-9)
    np.testing.assert_array_equal(qpos[robot.nq:], model.qpos0[robot.nq:])


def test_world_geoms_use_walkie_collision_classes(merged, robot):
    model = merged["model"]
    world_geoms = np.flatnonzero((model.geom_bodyid == 0) | (model.geom_bodyid >= robot.nbody))
    assert len(world_geoms) > 0
    floors = [geom for geom in world_geoms if model.geom(int(geom)).name == "floor"]
    assert len(floors) == 1
    for geom in world_geoms:
        name = model.geom(int(geom)).name or f"geom {geom}"
        masks = (int(model.geom_contype[geom]), int(model.geom_conaffinity[geom]))
        if geom == floors[0]:
            assert masks == world_loader.FLOOR_MASK
        elif model.body_weldid[model.geom_bodyid[geom]] == 0:
            assert masks in {world_loader.FURNITURE_MASK, world_loader.NO_COLLISION_MASK}, name
        else:
            assert masks in {world_loader.OBJECT_MASK, world_loader.NO_COLLISION_MASK}, name


def test_world_cameras_are_disabled_in_the_camera_plugin(merged, robot):
    """CameraPlugin streams every camera of the model unless its entry disables it."""
    assert BRINGUP_SHARE is not None, "robot_bringup is not installed"
    plugins = yaml.safe_load(
        (BRINGUP_SHARE / "config" / "mujoco" / "mujoco_plugins.yaml").read_text())
    cameras = plugins["/**"]["ros__parameters"]["mujoco_plugins"]["mujoco_camera_plugin"]
    robot_cameras = {robot.camera(index).name for index in range(robot.ncam)}
    model = merged["model"]
    streaming = [name for name in (model.camera(index).name for index in range(model.ncam))
                 if name not in robot_cameras
                 and cameras.get(name, {}).get("policy") != "disabled"]
    assert not streaming, (
        "add these world cameras with policy: disabled under mujoco_camera_plugin in "
        f"bringup/config/mujoco/mujoco_plugins.yaml: {streaming}")
    # CameraPlugin 0.1.2 indexes triggers by model camera, so with a disabled camera a
    # polled camera's trigger reaches the wrong camera.
    polled = [name for name, entry in cameras.items()
              if isinstance(entry, dict) and entry.get("policy") == "polled"]
    assert not polled, f"polled cameras next to disabled ones: {polled}"


def test_robot_settles_at_the_spawn(merged):
    model = merged["model"]
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    start = data.qpos[:3].copy()
    for _ in range(SETTLE_STEPS):
        mujoco.mj_step(model, data)
    assert np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
    assert not [mujoco.mjtWarning(index).name for index, warning in enumerate(data.warning)
                if warning.number]
    assert np.linalg.norm(data.qpos[:3] - start) < SETTLE_DRIFT_M
