"""Load exported environments for ROS launches, retaining Walkie contacts.

This module is internal launch support. It has no standalone command interface
and never edits the exported world, robot model, or navigation configuration.

A world is a directory ``<worlds root>/<name>/`` with a ``world.toml``
manifest next to its assets. The worlds root is the ``worlds`` directory in
the share of the ``walkie_mujoco_worlds`` package, or an explicit
``worlds_dir``.

``assemble_world`` merges an expanded world MJCF into the generated
``walkie.xml`` under these rules:

* Model-wide settings come from the robot. MuJoCo applies ``<compiler>``,
  ``<option>``, ``<size>``, ``<visual>`` and ``<statistic>`` to the whole
  model, so a world value would also change Walkie's physics or sensors. The
  world contributes only the presentation settings in ``WORLD_SETTINGS``.
  Other world values are dropped and reported through the module logger:
  ``<compiler>`` values as warnings, because they can change how the world's
  own inertia is computed, and the rest as info. A world ``<size memory>`` is
  kept only when it exceeds an explicit robot value; otherwise MuJoCo sizes
  the arena of the merged model automatically.
* The world's compiler ``angle`` and ``eulerseq`` must equal the robot's.
* Neither model may set attributes in its top-level ``<default>``, which
  MuJoCo applies to every element of the merged model. Named default classes
  are merged and must have unique names.
* World geoms get Walkie's collision classes (``FLOOR_MASK``,
  ``FURNITURE_MASK``, ``OBJECT_MASK``); the world needs exactly one plane
  geom named ``floor``. The wheel-floor ``<pair>`` elements come from the
  reference scene ``description/mujoco/scene.xml``.
* ``<include>`` elements, world keyframes and top-level sections other than
  ``SETTING_SECTIONS`` and ``COMBINED_SECTIONS`` are rejected.
"""

from __future__ import annotations

from copy import deepcopy
import logging
import math
import os
from pathlib import Path
import re
import tempfile
import tomllib
import xml.etree.ElementTree as ET


LOGGER = logging.getLogger(__name__)

WHEEL_GEOMS = (
    "fl_wheel_contact", "fr_wheel_contact", "bl_wheel_contact", "br_wheel_contact"
)
PATH_ATTRIBUTES = {"meshdir", "texturedir", "assetdir"}
ASSET_FILE_ATTRIBUTES = (
    "file", "fileright", "fileleft", "fileup", "filedown", "filefront", "fileback"
)

# Collision classes (contype, conaffinity) of the generated walkie.xml, defined
# by description/scripts/build_mjcf.py and listed in the walkie.xml header.
# test_mujoco_world_assets.py checks them against the installed header.
FLOOR_MASK = (1, 32)
FURNITURE_MASK = (64, 56)
OBJECT_MASK = (32, 121)
NO_COLLISION_MASK = (0, 0)

# Sections with model-wide settings: the merged model takes them from the robot.
SETTING_SECTIONS = ("compiler", "option", "size", "visual", "statistic")
# Sections whose children are concatenated, robot first, so that the robot's
# bodies and degrees of freedom come first in qpos.
COMBINED_SECTIONS = (
    "default", "extension", "custom", "asset", "worldbody", "contact",
    "equality", "tendon", "actuator", "sensor", "keyframe"
)
# World settings that pass into the merged model when the robot does not set
# them: the free-camera view, scene lighting, haze colour and statistic centre.
# Keys are (section, child tag or "" for the section's own attributes).
WORLD_SETTINGS = {
    ("visual", "global"): {"azimuth", "elevation"},
    ("visual", "headlight"): {"ambient", "diffuse", "specular", "active"},
    ("visual", "rgba"): {"haze"},
    ("statistic", ""): {"center"},
}
# Values set in every merged model: the statistic extent and map znear of
# description/mujoco/scene.xml, whose product is the camera near plane (rule 4
# in its header).
LOADER_SETTINGS = {
    ("statistic", "", "extent"): "2.2",
    ("visual", "map", "znear"): "0.025",
}
# Statistic centre of description/mujoco/scene.xml, used when neither model sets one.
DEFAULT_STATISTIC_CENTER = "0 0 0.7"
# MuJoCo 3 defaults of settings that exporters commonly write. A dropped world
# value equal to the value in effect is not reported.
MUJOCO_DEFAULTS = {
    ("compiler", "", "autolimits"): "true",
    ("compiler", "", "inertiafromgeom"): "auto",
    ("compiler", "", "inertiagrouprange"): "0 5",
    ("compiler", "", "boundmass"): "0",
    ("compiler", "", "boundinertia"): "0",
    ("compiler", "", "settotalmass"): "-1",
    ("compiler", "", "balanceinertia"): "false",
    ("compiler", "", "fusestatic"): "false",
    ("compiler", "", "discardvisual"): "false",
    ("compiler", "", "alignfree"): "false",
    ("compiler", "", "fitaabb"): "false",
    ("compiler", "", "saveinertial"): "false",
    ("compiler", "", "usethread"): "true",
    ("option", "", "timestep"): "0.002",
    ("option", "", "gravity"): "0 0 -9.81",
    ("option", "", "wind"): "0 0 0",
    ("option", "", "magnetic"): "0 -0.5 0",
    ("option", "", "density"): "0",
    ("option", "", "viscosity"): "0",
    ("option", "", "impratio"): "1",
    ("option", "", "tolerance"): "1e-8",
    ("option", "", "integrator"): "Euler",
    ("option", "", "cone"): "pyramidal",
    ("option", "", "jacobian"): "auto",
    ("option", "", "solver"): "Newton",
    ("option", "", "iterations"): "100",
    ("option", "", "ls_iterations"): "50",
    ("option", "", "noslip_iterations"): "0",
    ("size", "", "memory"): "-1",
    ("visual", "global", "offwidth"): "640",
    ("visual", "global", "offheight"): "480",
    ("visual", "quality", "shadowsize"): "4096",
    ("visual", "quality", "offsamples"): "4",
    ("visual", "quality", "numslices"): "28",
    ("visual", "quality", "numstacks"): "16",
    ("visual", "quality", "numquads"): "4",
    ("visual", "map", "zfar"): "50",
}
MEMORY_UNITS = {"": 1, "K": 1 << 10, "M": 1 << 20, "G": 1 << 30, "T": 1 << 40,
                "P": 1 << 50, "E": 1 << 60}

WORLDS_PACKAGE = "walkie_mujoco_worlds"
MANIFEST_NAME = "world.toml"
SCHEMA_VERSION = 1
MANIFEST_KEYS = {
    "schema_version", "default_variant", "map", "default_spawn", "variants", "spawns"
}
NAME_PATTERN = re.compile(r"[A-Za-z0-9_-]+")


def _read_mjcf(path: Path) -> ET.Element:
    """Parse an MJCF file; raise ValueError naming the file if it is malformed."""
    # MuJoCo's parser accepts '--' in comments, including command examples in
    # the reference scene header. ElementTree rejects those XML comments, so
    # they are removed first. Each comment keeps its line breaks, so parse
    # errors report the line number of the original file.
    source = re.sub(r"<!--.*?-->", lambda comment: "\n" * comment[0].count("\n"),
                    path.read_text(), flags=re.DOTALL)
    try:
        return ET.fromstring(source)
    except ET.ParseError as error:
        raise ValueError(f"{path}: malformed XML: {error}") from error


def _load_source(path: Path) -> ET.Element:
    root = _read_mjcf(path)
    if root.tag != "mujoco":
        raise ValueError(f"{path}: expected a <mujoco> root")
    if root.find(".//include") is not None:
        raise ValueError(f"{path}: use expanded MJCF without <include> elements")
    unsupported = [child.tag for child in root
                   if child.tag not in (*SETTING_SECTIONS, *COMBINED_SECTIONS)]
    if unsupported:
        raise ValueError(
            f"{path}: unsupported top-level MJCF section "
            f"{', '.join(f'<{tag}>' for tag in dict.fromkeys(unsupported))}; the world "
            f"loader merges only {', '.join(SETTING_SECTIONS + COMBINED_SECTIONS)}"
        )
    return root


def _check_top_level_default(root: ET.Element, path: Path, other: str) -> None:
    """Reject settings in the top-level <default>, which would also apply to ``other``."""
    for default in root.findall("default"):
        settings = dict.fromkeys(child.tag for child in default if child.tag != "default")
        if settings:
            raise ValueError(
                f"{path}: the top-level <default> sets "
                f"{', '.join(f'<{tag}>' for tag in settings)}. MuJoCo applies the "
                "top-level default class to every element of the merged model, "
                f"including the {other}. Move these settings into a named "
                '<default class="..."> and select it with class or childclass'
            )


def _default_class_names(root: ET.Element) -> set[str]:
    return {element.get("class") for top in root.findall("default")
            for element in top.iter("default")
            if element is not top and element.get("class")}


def _absolute_asset_files(root: ET.Element, source_path: Path) -> None:
    compiler = root.find("compiler")
    settings = compiler.attrib if compiler is not None else {}
    assetdir = settings.get("assetdir", "")
    for asset_section in root.findall("asset"):
        for asset in asset_section.iter():
            for attribute in ASSET_FILE_ATTRIBUTES:
                filename = asset.get(attribute)
                if not filename:
                    continue
                if settings.get("strippath", "false") == "true":
                    filename = Path(filename).name
                file_path = Path(filename).expanduser()
                if not file_path.is_absolute():
                    if asset.tag in {"mesh", "hfield"}:
                        directory = settings.get("meshdir", assetdir)
                    elif asset.tag == "texture":
                        directory = settings.get("texturedir", assetdir)
                    else:
                        directory = assetdir
                    file_path = source_path.parent / directory / file_path
                file_path = file_path.resolve()
                if not file_path.is_file():
                    raise ValueError(f"{source_path}: missing asset {file_path}")
                asset.set(attribute, str(file_path))
    if compiler is not None:
        for name in PATH_ATTRIBUTES:
            compiler.attrib.pop(name, None)
        # Absolute paths must survive the merge even if a source stripped paths.
        compiler.set("strippath", "false")


def _same_value(first: str, second: str | None) -> bool:
    """Compare two MJCF attribute values, numerically where both are numbers."""
    if second is None:
        return False
    first_parts, second_parts = str(first).split(), str(second).split()
    if len(first_parts) != len(second_parts):
        return False
    for first_part, second_part in zip(first_parts, second_parts):
        try:
            if float(first_part) != float(second_part):
                return False
        except ValueError:
            if first_part.lower() != second_part.lower():
                return False
    return True


def _memory_bytes(value: str) -> int:
    """Return the size of an MJCF <size memory> value such as '8M'; -1 is automatic."""
    match = re.fullmatch(r"\s*(-?\d+)\s*([KMGTPE]?)\s*", str(value), flags=re.IGNORECASE)
    if match is None:
        raise ValueError(f"Invalid <size memory> value {value!r}")
    return int(match[1]) * MEMORY_UNITS[match[2].upper()]


def _describe_setting(section: str, child: str, key: str, value: str,
                      effective: str | None) -> str:
    element = (f'<{section}><{child} {key}="{value}">' if child
               else f'<{section} {key}="{value}">')
    if effective is None:
        effective = "MuJoCo default"
    elif (section, child, key) == ("size", "", "memory") and _same_value(effective, "-1"):
        effective = "automatic"
    return f"{element} -> {effective}"


def _robot_compiler(robot: ET.Element, environment: ET.Element,
                    ignored: list[str]) -> ET.Element:
    """Return the robot's <compiler>; list differing world values in ``ignored``.

    A different angle or eulerseq changes the meaning of the world's numbers,
    so it is an error. Every other world compiler value (inertiafromgeom,
    inertiagrouprange, boundmass, settotalmass, fusestatic, autolimits, ...)
    would also apply to the robot, so it is dropped. Asset paths were already
    made absolute.
    """
    robot_compiler = robot.find("compiler")
    environment_compiler = environment.find("compiler")
    for attribute, default in (("angle", "degree"), ("eulerseq", "xyz")):
        robot_value = (robot_compiler.get(attribute, default)
                       if robot_compiler is not None else default)
        environment_value = (environment_compiler.get(attribute, default)
                             if environment_compiler is not None else default)
        if robot_value != environment_value:
            raise ValueError(
                f"Compiler {attribute} differs: robot={robot_value}, "
                f"environment={environment_value}; use the same angular convention"
            )
    result = (deepcopy(robot_compiler) if robot_compiler is not None
              else ET.Element("compiler"))
    if environment_compiler is None:
        return result
    for key, value in environment_compiler.attrib.items():
        if key in {"angle", "eulerseq", "strippath"}:
            continue
        effective = result.get(key, MUJOCO_DEFAULTS.get(("compiler", "", key)))
        if not _same_value(value, effective):
            ignored.append(_describe_setting("compiler", "", key, value, effective))
    for child in environment_compiler:
        ignored.append(f"<compiler><{child.tag}>")
    return result


def _robot_settings(name: str, robot: ET.Element, environment: ET.Element,
                    ignored: list[str]) -> ET.Element:
    """Return the robot's <name> section plus the world values in WORLD_SETTINGS.

    Every other world value is listed in ``ignored`` unless it equals the value
    in effect: the robot's, the loader's (LOADER_SETTINGS) or MuJoCo's default.
    """
    section = robot.find(name)
    result = deepcopy(section) if section is not None else ET.Element(name)
    world = environment.find(name)
    if world is None:
        return result
    for child, world_element in [("", world), *((element.tag, element) for element in world)]:
        target = result if not child else result.find(child)
        allowed = WORLD_SETTINGS.get((name, child), set())
        for key, value in world_element.attrib.items():
            setting = (name, child, key)
            current = None if target is None else target.get(key)
            if current is None and key in allowed:
                if target is None:
                    target = ET.SubElement(result, child)
                target.set(key, value)
                continue
            if setting == ("size", "", "memory") and current is not None \
                    and _memory_bytes(current) >= 0:
                # Keep the larger explicit arena: a large world may need more.
                if _memory_bytes(value) > _memory_bytes(current):
                    target.set(key, value)
                continue
            effective = LOADER_SETTINGS.get(
                setting, current if current is not None else MUJOCO_DEFAULTS.get(setting)
            )
            if not _same_value(value, effective):
                ignored.append(_describe_setting(name, child, key, value, effective))
        if child:
            ignored.extend(f"<{name}><{child}><{nested.tag}>" for nested in world_element)
    return result


def _environment_collision_masks(environment: ET.Element) -> None:
    """Apply Walkie contact classes while preserving visual-only defaults.

    Exported worlds need only an analytic plane named ``floor``. Static and
    articulated collision bodies are classified from their joints; no room-
    specific naming or exporter default-class names are required.
    """
    defaults: dict[str, dict[str, str]] = {"": {}}

    def read_defaults(element: ET.Element, inherited: dict[str, str]) -> None:
        attrs = dict(inherited)
        geom = element.find("geom")
        if geom is not None:
            attrs.update(geom.attrib)
        name = element.get("class", "")
        defaults[name] = attrs
        for child in element.findall("default"):
            read_defaults(child, attrs)

    for default in environment.findall("default"):
        read_defaults(default, defaults[""])

    def has_joint(body: ET.Element) -> bool:
        # MuJoCo assigns a joint inside a <frame> to the enclosing body.
        for child in body:
            if child.tag in {"joint", "freejoint"}:
                return True
            if child.tag == "frame" and has_joint(child):
                return True
        return False

    def walk(element: ET.Element, moving: bool = False,
             inherited_class: str = "") -> None:
        if element.tag == "body":
            moving = moving or has_joint(element)
        if element.tag in {"body", "frame"}:
            # A frame's childclass applies to its elements as a body's does.
            inherited_class = element.get("childclass", inherited_class)
        if element.tag == "geom":
            attrs = dict(defaults.get(element.get("class", inherited_class), {}))
            attrs.update(element.attrib)
            if element.get("name") == "floor":
                if attrs.get("type", "sphere") != "plane":
                    raise ValueError("Environment geom 'floor' must be a plane")
                mask = FLOOR_MASK
            elif attrs.get("contype", "1") == "0" and attrs.get("conaffinity", "1") == "0":
                mask = NO_COLLISION_MASK
            elif moving:
                mask = OBJECT_MASK
            else:
                mask = FURNITURE_MASK
            element.set("contype", str(mask[0]))
            element.set("conaffinity", str(mask[1]))
        for child in element:
            walk(child, moving, inherited_class)

    worldbody = environment.find("worldbody")
    if worldbody is None:
        raise ValueError("Environment must have a <worldbody>")
    walk(worldbody)
    if sum(geom.get("name") == "floor" for geom in worldbody.iter("geom")) != 1:
        raise ValueError("Environment must have exactly one plane geom named 'floor'")


def _set_spawn(root: ET.Element, pose: tuple[float, float, float]) -> None:
    """Set the ROS startup pose and reset keyframes in the generated world only."""
    if len(pose) != 3 or not all(math.isfinite(value) for value in pose):
        raise ValueError("Spawn must contain finite X, Y and yaw degrees")
    base = root.find("./worldbody/body[@name='base_footprint']")
    if base is None or base.find("joint[@name='floating_base_joint'][@type='free']") is None:
        raise ValueError("Expected Walkie's root floating_base_joint on base_footprint")
    if root.find("worldbody")[0] is not base:
        raise ValueError("Walkie's floating base must be the first body/qpos entries")
    position = [float(value) for value in base.get("pos", "0 0 0").split()]
    yaw = math.radians(pose[2])
    quaternion = [math.cos(yaw / 2), 0., 0., math.sin(yaw / 2)]
    position[:2] = pose[:2]
    base.set("pos", " ".join(format(value, ".12g") for value in position))
    base.set("quat", " ".join(format(value, ".12g") for value in quaternion))
    for key in root.findall("keyframe/key"):
        qpos = key.get("qpos", "").split()
        if len(qpos) < 7:
            raise ValueError("Robot reset keyframe must explicitly contain the floating base pose")
        # Preserve key-specific Z, upper-body joints, velocities and controls.
        qpos[:2] = [format(value, ".12g") for value in position[:2]]
        qpos[3:7] = [format(value, ".12g") for value in quaternion]
        key.set("qpos", " ".join(qpos))


def _spawn_xyyaw(root: ET.Element) -> tuple[float, float, float]:
    base = root.find("./worldbody/body[@name='base_footprint']")
    x, y, _ = map(float, base.get("pos", "0 0 0").split())
    w, qx, qy, qz = map(float, base.get("quat", "1 0 0 0").split())
    yaw = math.atan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return x, y, yaw


def _write_atomic(path: Path, data: bytes) -> None:
    """Replace ``path`` with ``data`` so that readers never see a partial file."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        dir=path.parent, prefix=f".{path.name}.", delete=False
    ) as temporary:
        temporary.write(data)
        temporary_path = Path(temporary.name)
    try:
        # NamedTemporaryFile creates 0600 files; match a normal write instead.
        os.chmod(temporary_path, 0o644)
        os.replace(temporary_path, path)
    finally:
        temporary_path.unlink(missing_ok=True)


def write_nav2_params(source: Path, output: Path, root: ET.Element) -> None:
    """Copy the current Nav2 configuration and match AMCL to the world spawn."""
    import yaml

    try:
        config = yaml.safe_load(source.read_text())
        amcl = config["amcl"]["ros__parameters"]
        if not isinstance(amcl, dict):
            raise TypeError("AMCL parameters must be a mapping")
    except (yaml.YAMLError, KeyError, TypeError) as error:
        raise ValueError(
            f"{source}: expected a full Nav2 YAML with amcl.ros__parameters"
        ) from error
    x, y, yaw = _spawn_xyyaw(root)
    amcl["use_sim_time"] = True
    amcl["set_initial_pose"] = True
    amcl["initial_pose"] = {"x": x, "y": y, "z": 0., "yaw": yaw}
    header = "# Generated spawn-matched Nav2 parameters; source configuration is unchanged.\n"
    _write_atomic(output, (header + yaml.safe_dump(config, sort_keys=False)).encode())


def assemble_world(robot_path: Path, scene_path: Path,
                   wheel_scene_path: Path,
                   spawn_pose: tuple[float, float, float] | None = None) -> ET.Element:
    """Return a merged MJCF tree; never mutate a source file.

    See the module docstring for the merge rules. World settings that the
    merge drops are logged and listed in a comment of the returned tree.
    """
    robot_path = robot_path.resolve()
    scene_path = scene_path.resolve()
    robot = _load_source(robot_path)
    environment = _load_source(scene_path)
    _check_top_level_default(robot, robot_path, "world")
    _check_top_level_default(environment, scene_path, "robot")
    shared_classes = _default_class_names(robot) & _default_class_names(environment)
    if shared_classes:
        raise ValueError(
            f"{scene_path}: default classes {', '.join(sorted(shared_classes))} are also "
            f"defined by the robot model {robot_path}; rename them in the world"
        )
    _absolute_asset_files(robot, robot_path)
    _absolute_asset_files(environment, scene_path)

    if environment.find("keyframe/key") is not None:
        raise ValueError(
            "Standalone environment keyframes cannot be copied before robot qpos; "
            "set initial body poses and joint references instead"
        )
    _environment_collision_masks(environment)

    ignored_compiler: list[str] = []
    ignored: list[str] = []
    settings = {"compiler": _robot_compiler(robot, environment, ignored_compiler)}
    for name in SETTING_SECTIONS[1:]:
        settings[name] = _robot_settings(name, robot, environment, ignored)
    for (name, child, key), value in LOADER_SETTINGS.items():
        target = settings[name] if not child else settings[name].find(child)
        if target is None:
            target = ET.SubElement(settings[name], child)
        target.set(key, value)
    settings["statistic"].attrib.setdefault("center", DEFAULT_STATISTIC_CENTER)

    # The messages are formatted here, not passed as logging arguments: the
    # launch screen and log file handlers print the record's message template
    # ("{msg}") without its arguments.
    if ignored_compiler:
        LOGGER.warning(
            f"{scene_path}: ignored world <compiler> settings (world value -> value in "
            f"effect): {'; '.join(ignored_compiler)}. The merged model compiles with the "
            "robot's compiler settings, so the world's inertia can differ from a "
            "standalone load of the world"
        )
    if ignored:
        LOGGER.info(
            f"{scene_path}: ignored world settings that would also apply to the robot "
            f"(world value -> value in effect): {'; '.join(ignored)}"
        )

    result = ET.Element("mujoco", {"model": "walkie_world"})
    result.append(ET.Comment(
        " Generated by robot_bringup.world_loader. Robot source is unchanged. "
        "ROS launch regenerates this cache from the exported world and robot. "
    ))
    if ignored_compiler or ignored:
        listing = "; ".join(ignored_compiler + ignored).replace("--", "- -")
        result.append(ET.Comment(
            f" Ignored world settings (world value -> value in effect): {listing} "
        ))
    for name in SETTING_SECTIONS:
        result.append(settings[name])

    # Keep all robot bodies before environment bodies: home qpos starts with
    # the robot's 31 entries. MuJoCo fills added joint entries from qpos0.
    for name in COMBINED_SECTIONS:
        sections = [*robot.findall(name), *environment.findall(name)]
        if not sections:
            continue
        target = ET.SubElement(result, name)
        for section in sections:
            for key, value in section.attrib.items():
                target.attrib.setdefault(key, value)
            for child in section:
                target.append(deepcopy(child))

    if result.find("keyframe/key[@name='home']") is None:
        raise ValueError("Robot must provide the 'home' keyframe")
    if spawn_pose is not None:
        _set_spawn(result, spawn_pose)

    wheel_scene = _read_mjcf(wheel_scene_path)
    contact = result.find("contact")
    if contact is None:
        contact = ET.SubElement(result, "contact")
    for wheel in WHEEL_GEOMS:
        candidates = [
            pair for pair in wheel_scene.findall("contact/pair")
            if {pair.get("geom1"), pair.get("geom2")} == {wheel, "floor"}
        ]
        if len(candidates) != 1:
            raise ValueError(
                f"{wheel_scene_path}: expected one {wheel}/floor contact pair"
            )
        for old in list(contact.findall("pair")):
            if {old.get("geom1"), old.get("geom2")} == {wheel, "floor"}:
                contact.remove(old)
        contact.append(deepcopy(candidates[0]))

    # Recalculate nkey in case a future robot adds reset poses.
    size = result.find("size")
    if size is not None:
        size.set("nkey", str(len(result.findall("keyframe/key"))))
    return result


def worlds_root(worlds_dir=None) -> Path:
    """Return the directory that holds one subdirectory per world.

    An empty ``worlds_dir`` selects the installed ``walkie_mujoco_worlds``
    share. Under ``--symlink-install`` its files are links into the source
    tree. The installed path is kept unresolved, so the returned paths stay in
    the share and map_server reads the map image next to the installed YAML.
    """
    if worlds_dir is not None and str(worlds_dir).strip():
        root = Path(str(worlds_dir).strip()).expanduser().resolve()
        if not root.is_dir():
            raise ValueError(f"worlds_dir is not a directory: {root}")
        return root
    from ament_index_python.packages import (
        get_package_share_directory, PackageNotFoundError,
    )

    try:
        share = Path(get_package_share_directory(WORLDS_PACKAGE))
    except PackageNotFoundError as error:
        raise ValueError(
            f"Package {WORLDS_PACKAGE} is not installed. Build it with "
            f"'pixi run build-pkg {WORLDS_PACKAGE}', or pass worlds_dir:=DIR "
            "where DIR contains one directory per world."
        ) from error
    return share / "worlds"


def available_worlds(root: Path) -> list[str]:
    """Return the names of the worlds below ``root`` that have a manifest.

    A dangling link is skipped: a world renamed or deleted after a
    --symlink-install build leaves its installed links behind.
    """
    return sorted(path.parent.name for path in Path(root).glob(f"*/{MANIFEST_NAME}")
                  if path.is_file())


def load_manifest(path: Path) -> dict:
    """Read and validate a ``world.toml`` manifest.

    Raise ValueError naming the manifest and the first problem found. Paths in
    the manifest are relative to its directory and may not leave it.
    """
    path = Path(path)
    try:
        config = tomllib.loads(path.read_text())
    except (OSError, UnicodeDecodeError, tomllib.TOMLDecodeError) as error:
        raise ValueError(f"Invalid world manifest {path}: {error}") from error

    def invalid(reason: str) -> ValueError:
        return ValueError(f"Invalid world manifest {path}: {reason}")

    def relative_file(value, label: str) -> None:
        if not isinstance(value, str) or not value.strip():
            raise invalid(f"{label} must be a non-empty relative path")
        relative = Path(value)
        if relative.is_absolute() or ".." in relative.parts:
            raise invalid(f"{label} {value!r} must stay inside the world directory")

    version = config.get("schema_version")
    if type(version) is not int or version != SCHEMA_VERSION:
        raise invalid(
            f"schema_version is {version!r}; this loader reads "
            f"schema_version = {SCHEMA_VERSION}"
        )
    unknown = sorted(set(config) - MANIFEST_KEYS)
    if unknown:
        raise invalid(
            f"unknown keys {', '.join(unknown)}; allowed: {', '.join(sorted(MANIFEST_KEYS))}"
        )
    missing = sorted(MANIFEST_KEYS - set(config))
    if missing:
        raise invalid(f"missing keys {', '.join(missing)}")

    variants = config["variants"]
    if not isinstance(variants, dict) or not variants:
        raise invalid("[variants] must map at least one variant name to a scene file")
    for name, scene in variants.items():
        if not NAME_PATTERN.fullmatch(name):
            raise invalid(f"variant name {name!r} may contain only letters, digits, '_' or '-'")
        relative_file(scene, f"variant {name!r}")
    if not isinstance(config["default_variant"], str):
        raise invalid(f"default_variant must be a string, got {config['default_variant']!r}")
    if config["default_variant"] not in variants:
        raise invalid(
            f"default_variant {config['default_variant']!r} is not one of "
            f"{', '.join(variants)}"
        )
    relative_file(config["map"], "map")

    spawns = config["spawns"]
    if not isinstance(spawns, dict) or not spawns:
        raise invalid("[spawns] must map at least one spawn name to [X, Y, YAW_DEG]")
    for name, pose in spawns.items():
        if not (isinstance(pose, list) and len(pose) == 3 and all(
                type(value) in (int, float) and math.isfinite(value) for value in pose)):
            raise invalid(f"spawn {name!r} must be [X, Y, YAW_DEG] with finite numbers")
    if not isinstance(config["default_spawn"], str):
        raise invalid(f"default_spawn must be a string, got {config['default_spawn']!r}")
    if config["default_spawn"] not in spawns:
        raise invalid(
            f"default_spawn {config['default_spawn']!r} is not one of {', '.join(spawns)}"
        )
    return config


def load_world(world: str, worlds_dir=None) -> tuple[Path, dict]:
    """Return the directory and validated manifest of the world named ``world``."""
    if not isinstance(world, str) or not NAME_PATTERN.fullmatch(world):
        raise ValueError("World name must contain only letters, digits, '_' or '-'")
    root = worlds_root(worlds_dir)
    world_dir = root / world
    manifest = world_dir / MANIFEST_NAME
    if not manifest.is_file():
        available = ", ".join(available_worlds(root)) or "none"
        raise ValueError(f"Unknown world {world!r} in {root}; available: {available}")
    return world_dir, load_manifest(manifest)


def resolve_spawn(world: str, config: dict, spawn=None,
                  spawn_pose=None) -> tuple[float, float, float]:
    """Return (X, Y, YAW_DEG) for a named spawn or an 'X Y YAW_DEG' pose.

    Without either, the manifest's ``default_spawn`` is used.
    """
    if spawn_pose and spawn:
        raise ValueError("Use either spawn or spawn_pose, not both")
    if spawn_pose:
        values = spawn_pose.split() if isinstance(spawn_pose, str) else spawn_pose
        try:
            pose = tuple(float(value) for value in values)
        except (TypeError, ValueError) as error:
            raise ValueError(f"spawn_pose must be 'X Y YAW_DEG', got {spawn_pose!r}") from error
    else:
        name = spawn or config["default_spawn"]
        if name not in config["spawns"]:
            raise ValueError(
                f"Unknown spawn {name!r} for world {world!r}; "
                f"available: {', '.join(config['spawns'])}"
            )
        pose = tuple(float(value) for value in config["spawns"][name])
    if len(pose) != 3 or not all(math.isfinite(value) for value in pose):
        raise ValueError("Spawn must contain finite X, Y and yaw degrees")
    return pose


def prepare_world(world: str, variant: str | None, worlds_dir, output_dir,
                  robot_path, wheel_scene_path, nav2_params_source=None,
                  spawn: str | None = None, spawn_pose=None) -> dict:
    """Resolve a world for a ROS launch and cache its robot and AMCL files.

    The world is ``<worlds root>/<world>/world.toml`` (see ``worlds_root``).
    An empty ``variant`` selects the manifest's ``default_variant``. Without
    ``nav2_params_source`` no Nav2 parameters are generated and the returned
    ``params_path`` is None. The exported scene, robot model and Nav2 source
    stay unchanged.

    The cache path depends only on the world and variant. Two launches of the
    same world and variant started together overwrite each other's files.
    """
    world_dir, config = load_world(world, worlds_dir)
    manifest = world_dir / MANIFEST_NAME

    variant = variant or config["default_variant"]
    if variant not in config["variants"]:
        raise ValueError(
            f"Unknown variant {variant!r} for world {world!r}; "
            f"available: {', '.join(config['variants'])}"
        )

    def world_file(relative: str) -> Path:
        # load_manifest has rejected absolute and '..' paths. Keep the
        # unresolved path: map_server reads the map image next to this YAML.
        path = world_dir / relative
        if not path.is_file():
            raise ValueError(f"World {world!r} is missing {relative} in {world_dir}")
        return path

    scene = world_file(config["variants"][variant])
    map_path = world_file(config["map"])
    pose = resolve_spawn(world, config, spawn, spawn_pose)

    merged = assemble_world(Path(robot_path), scene, Path(wheel_scene_path), pose)
    cache = Path(output_dir).expanduser().resolve() / world / variant
    model_path, params_path = cache / "world.xml", cache / "nav2.yaml"
    inputs = {Path(robot_path).resolve(), Path(wheel_scene_path).resolve(),
              scene.resolve(), map_path.resolve(), manifest.resolve()}
    if nav2_params_source is not None:
        inputs.add(Path(nav2_params_source).resolve())
    if model_path in inputs or params_path in inputs:
        raise ValueError("The ROS world cache must differ from all source files")
    ET.indent(merged, space="  ")
    # Each launch regenerates the cache, so an edit to a linked (symlink
    # install) or worlds_dir source applies at the next launch.
    _write_atomic(
        model_path, ET.tostring(merged, encoding="utf-8", xml_declaration=True) + b"\n"
    )
    if nav2_params_source is None:
        params_path = None
    else:
        write_nav2_params(Path(nav2_params_source).resolve(), params_path, merged)
    return {"model_path": str(model_path), "map_path": str(map_path),
            "params_path": None if params_path is None else str(params_path),
            "variant": variant, "spawn_pose": pose}
