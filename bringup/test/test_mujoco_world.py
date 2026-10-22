"""Checks for world discovery, manifests and MJCF assembly.

These tests use synthetic files and need no build and no Blender. Only
test_mujoco_defaults_table_matches_mujoco and
test_frame_classes_match_a_mujoco_compile need MuJoCo; they are skipped
without it. test_mujoco_world_assets.py compiles the installed worlds.
"""

import logging
import math
from pathlib import Path
from unittest.mock import patch
import xml.etree.ElementTree as ET

import pytest
from robot_bringup import world_loader


MANIFEST = """\
schema_version = 1
default_variant = "lowpoly"
map = "map/map.yaml"
default_spawn = "living-room"

[variants]
lowpoly = "scene.xml"
baked = "baked/scene.xml"

[spawns]
origin = [0.0, 0.0, 0.0]
living-room = [1.76, -1.48, -90.0]
"""
WORLD_FILES = ("scene.xml", "baked/scene.xml", "map/map.yaml")
MERGED = ('<mujoco><worldbody><body name="base_footprint" pos="1.76 -1.48 .12" '
          'quat=".7071067811865476 0 0 -.7071067811865475"/></worldbody></mujoco>')


def _write_world(root: Path, name: str = "test_world", manifest: str = MANIFEST) -> Path:
    world = root / name
    for relative in WORLD_FILES:
        (world / relative).parent.mkdir(parents=True, exist_ok=True)
        (world / relative).write_text("exported asset")
    (world / "world.toml").write_text(manifest)
    return world


def _prepare(root: Path, **overrides):
    arguments = {
        "world": "test_world", "variant": None, "worlds_dir": root / "worlds",
        "output_dir": root / "cache", "robot_path": root / "robot.xml",
        "wheel_scene_path": root / "wheel.xml",
    }
    arguments.update(overrides)
    return world_loader.prepare_world(**arguments)


ROBOT = """<mujoco model="robot">
  <compiler angle="radian"/>
  <option integrator="implicitfast" cone="elliptic"/>
  <size nkey="1" {robot_size}/>
  <visual><global offwidth="1280" offheight="720"/></visual>
  {robot_extra}
  <worldbody>
    <body name="base_footprint" pos="0 0 0.1">
      <joint name="floating_base_joint" type="free"/>
      <geom name="chassis" type="box" size=".2 .2 .05" contype="2" conaffinity="24"/>
      <body name="arm" pos="0 0 .2">
        <joint name="arm_joint" axis="0 0 1"/>
        <geom name="arm_link" type="capsule" size=".03 .1" contype="8" conaffinity="126"/>
      </body>
    </body>
  </worldbody>
  <keyframe><key name="home" qpos="0 0 0.1 1 0 0 0 0"/></keyframe>
</mujoco>"""
WHEEL_SCENE = "<mujoco><contact>{}</contact></mujoco>".format("".join(
    f'<pair geom1="{wheel}" geom2="floor" condim="3"/>' for wheel in world_loader.WHEEL_GEOMS
))
WORLD = """<mujoco model="room">
  <compiler {compiler}/>
  {settings}
  <default>{defaults}
    <default class="room_static"><geom friction=".6 .004 .0002"/></default>
  </default>
  <worldbody>
    <geom name="floor" type="plane" size="0 0 .05"/>
    <body name="crate" pos="1 0 .1">
      <freejoint/>
      <geom class="room_static" type="box" size=".1 .1 .1"/>
    </body>
  </worldbody>
</mujoco>"""


def _assemble(directory: Path, compiler='angle="radian"', settings="", defaults="",
              robot_size="", robot_extra="") -> ET.Element:
    robot = directory / "robot.xml"
    wheel_scene = directory / "wheel_scene.xml"
    world = directory / "world.xml"
    robot.write_text(ROBOT.format(robot_size=robot_size, robot_extra=robot_extra))
    wheel_scene.write_text(WHEEL_SCENE)
    world.write_text(WORLD.format(compiler=compiler, settings=settings, defaults=defaults))
    return world_loader.assemble_world(robot, world, wheel_scene)


def _comments(root: ET.Element) -> str:
    return "\n".join(node.text for node in root if node.tag is ET.Comment)


class _Records(logging.Handler):

    # The launch screen and log file handlers format records with "{msg}"
    # (launch.logging: '[{levelname}] [{name}]: {msg}'), so a message that
    # relies on logging arguments shows its unformatted template there.
    LAUNCH_FORMAT = logging.Formatter("{msg}", style="{")

    def __init__(self):
        super().__init__(logging.DEBUG)
        self.records = []

    def emit(self, record):
        self.records.append(record)

    def messages(self, level: int) -> list[str]:
        """Return the messages at ``level`` as a ros2 launch prints them."""
        return [self.LAUNCH_FORMAT.format(record) for record in self.records
                if record.levelno == level]


@pytest.fixture
def loader_log():
    """Capture world_loader messages; a launch import may disable propagation."""
    handler = _Records()
    logger = world_loader.LOGGER
    level = logger.level
    logger.addHandler(handler)
    logger.setLevel(logging.DEBUG)
    yield handler
    logger.removeHandler(handler)
    logger.setLevel(level)


@pytest.fixture
def assemble():
    with patch.object(world_loader, "assemble_world",
                      side_effect=lambda *args: ET.fromstring(MERGED)) as mock:
        yield mock


def test_empty_worlds_dir_uses_installed_worlds_package(tmp_path):
    share = tmp_path / "share" / "walkie_mujoco_worlds"
    with patch("ament_index_python.packages.get_package_share_directory",
               return_value=str(share)) as resolve:
        for override in (None, "", "  "):
            assert world_loader.worlds_root(override) == share / "worlds"
    assert resolve.call_count == 3
    resolve.assert_called_with("walkie_mujoco_worlds")


def test_missing_worlds_package_explains_build_or_override():
    from ament_index_python.packages import PackageNotFoundError

    with patch("ament_index_python.packages.get_package_share_directory",
               side_effect=PackageNotFoundError("walkie_mujoco_worlds")):
        with pytest.raises(ValueError, match="build-pkg walkie_mujoco_worlds.*worlds_dir"):
            world_loader.worlds_root()


def test_worlds_dir_override_skips_the_package_index(tmp_path):
    with patch("ament_index_python.packages.get_package_share_directory") as resolve:
        assert world_loader.worlds_root(tmp_path) == tmp_path.resolve()
        with pytest.raises(ValueError, match="not a directory"):
            world_loader.worlds_root(tmp_path / "missing")
    resolve.assert_not_called()


def test_worlds_are_discovered_by_their_manifest(tmp_path, assemble):
    root = tmp_path / "worlds"
    _write_world(root, "room_a")
    _write_world(root, "room_b")
    (root / "not_a_world").mkdir()
    assert world_loader.available_worlds(root) == ["room_a", "room_b"]
    with pytest.raises(ValueError, match=r"Unknown world 'room_c' in .*available: room_a, room_b"):
        _prepare(tmp_path, world="room_c")
    assemble.assert_not_called()


def test_a_dangling_manifest_link_is_not_a_world(tmp_path, assemble):
    """A world renamed under --symlink-install leaves a link to a deleted manifest."""
    root = tmp_path / "worlds"
    _write_world(root, "room_a")
    (root / "old_room").mkdir()
    (root / "old_room" / world_loader.MANIFEST_NAME).symlink_to(tmp_path / "deleted.toml")
    assert world_loader.available_worlds(root) == ["room_a"]
    with pytest.raises(ValueError, match=r"Unknown world 'old_room' in .*available: room_a$"):
        _prepare(tmp_path, world="old_room")
    assemble.assert_not_called()


def test_world_name_cannot_select_another_directory(tmp_path, assemble):
    _write_world(tmp_path / "worlds")
    for name in ("../worlds", "a/b", ""):
        with pytest.raises(ValueError, match="World name"):
            _prepare(tmp_path, world=name)


def test_default_variant_spawn_and_no_nav2_params(tmp_path, assemble):
    world = _write_world(tmp_path / "worlds")
    prepared = _prepare(tmp_path)
    assert prepared["variant"] == "lowpoly"
    assert prepared["spawn_pose"] == (1.76, -1.48, -90.)
    assert prepared["params_path"] is None
    assert Path(prepared["map_path"]) == world / "map" / "map.yaml"
    model = Path(prepared["model_path"])
    assert model == (tmp_path / "cache" / "test_world" / "lowpoly" / "world.xml").resolve()
    assert ET.parse(model).getroot().find("worldbody/body").get("name") == "base_footprint"
    assert not (model.parent / "nav2.yaml").exists()
    assemble.assert_called_once_with(
        tmp_path / "robot.xml", world / "scene.xml", tmp_path / "wheel.xml",
        (1.76, -1.48, -90.),
    )


def test_selected_variant_and_spawn_matched_nav2_params(tmp_path, assemble):
    import yaml

    world = _write_world(tmp_path / "worlds")
    source = tmp_path / "nav2.yaml"
    source.write_text("amcl:\n  ros__parameters: {}\n")
    prepared = _prepare(tmp_path, variant="baked", spawn="origin",
                        nav2_params_source=source)
    assert prepared["variant"] == "baked"
    assert assemble.call_args.args[1] == world / "baked" / "scene.xml"
    assert assemble.call_args.args[3] == (0., 0., 0.)
    params = Path(prepared["params_path"])
    assert params.parent == (tmp_path / "cache" / "test_world" / "baked").resolve()
    amcl = yaml.safe_load(params.read_text())["amcl"]["ros__parameters"]
    assert amcl["set_initial_pose"] is True
    assert source.read_text() == "amcl:\n  ros__parameters: {}\n"


def test_unknown_variant_lists_the_manifest_variants(tmp_path, assemble):
    _write_world(tmp_path / "worlds")
    with pytest.raises(ValueError) as error:
        _prepare(tmp_path, variant="realistic")
    assert str(error.value) == (
        "Unknown variant 'realistic' for world 'test_world'; available: lowpoly, baked"
    )
    assemble.assert_not_called()


@pytest.mark.parametrize("spawn, spawn_pose, message", [
    ("kitchen", None, "Unknown spawn 'kitchen' for world 'test_world'; available: origin"),
    ("origin", "1 2 3", "either spawn or spawn_pose"),
    (None, "1 2", "finite X, Y and yaw"),
    (None, "1 2 north", "spawn_pose must be 'X Y YAW_DEG'"),
    (None, "1 nan 0", "finite X, Y and yaw"),
])
def test_spawn_errors(tmp_path, assemble, spawn, spawn_pose, message):
    _write_world(tmp_path / "worlds")
    with pytest.raises(ValueError, match=message):
        _prepare(tmp_path, spawn=spawn, spawn_pose=spawn_pose)
    assemble.assert_not_called()


def test_custom_spawn_pose(tmp_path, assemble):
    _write_world(tmp_path / "worlds")
    assert _prepare(tmp_path, spawn_pose="0.5 -1 45")["spawn_pose"] == (0.5, -1., 45.)


@pytest.mark.parametrize("change, message", [
    (("schema_version = 1", "schema_version = 2"), "schema_version is 2"),
    (("schema_version = 1", 'schema_version = "1"'), "schema_version is '1'"),
    (("schema_version = 1", 'schema_version = 1\npackage = "robot_simulation"'),
     "unknown keys package"),
    (('map = "map/map.yaml"\n', ""), "missing keys map"),
    (('default_variant = "lowpoly"', 'default_variant = "hd"'),
     "default_variant 'hd' is not one of lowpoly, baked"),
    (('baked = "baked/scene.xml"', 'baked = "../scene.xml"'), "must stay inside"),
    (('baked = "baked/scene.xml"', 'baked = "/tmp/scene.xml"'), "must stay inside"),
    (('baked = "baked/scene.xml"', '"bad name" = "baked/scene.xml"'), "variant name"),
    (('origin = [0.0, 0.0, 0.0]', 'origin = [0.0, 0.0]'), "spawn 'origin'"),
    (('default_spawn = "living-room"', 'default_spawn = "hall"'), "default_spawn 'hall'"),
    (('default_variant = "lowpoly"', 'default_variant = ["lowpoly"]'),
     "default_variant must be a string"),
    (('default_variant = "lowpoly"', "default_variant = {a = 1}"),
     "default_variant must be a string"),
    (('default_spawn = "living-room"', 'default_spawn = ["living-room"]'),
     "default_spawn must be a string"),
    (('default_spawn = "living-room"', "default_spawn = {a = 1}"),
     "default_spawn must be a string"),
    (("[variants]", "[variants"), "Invalid world manifest"),
])
def test_invalid_manifests_name_the_file_and_problem(tmp_path, assemble, change, message):
    world = _write_world(tmp_path / "worlds", manifest=MANIFEST.replace(*change))
    with pytest.raises(ValueError, match=message) as error:
        _prepare(tmp_path)
    assert str(world / "world.toml") in str(error.value)
    assemble.assert_not_called()


def test_missing_world_file_is_reported(tmp_path, assemble):
    world = _write_world(tmp_path / "worlds")
    (world / "baked" / "scene.xml").unlink()
    with pytest.raises(ValueError, match="missing baked/scene.xml"):
        _prepare(tmp_path, variant="baked")


def test_symlink_installed_files_are_accepted_but_cache_must_differ(tmp_path, assemble):
    """Under --symlink-install each installed world file links to its source."""
    source = _write_world(tmp_path / "source")
    installed = tmp_path / "worlds" / "test_world"
    for relative in (*WORLD_FILES, "world.toml"):
        (installed / relative).parent.mkdir(parents=True, exist_ok=True)
        (installed / relative).symlink_to(source / relative)
    prepared = _prepare(tmp_path)
    # The map stays at its installed path so map_server resolves its image there.
    assert Path(prepared["map_path"]) == installed / "map" / "map.yaml"
    (installed / "scene.xml").unlink()
    (installed / "scene.xml").symlink_to(prepared["model_path"])
    with pytest.raises(ValueError, match="cache must differ"):
        _prepare(tmp_path)


def test_atomic_write_replaces_the_file_without_leftovers(tmp_path):
    target = tmp_path / "out" / "world.xml"
    world_loader._write_atomic(target, b"first")
    world_loader._write_atomic(target, b"second")
    assert target.read_bytes() == b"second"
    assert sorted(path.name for path in target.parent.iterdir()) == ["world.xml"]
    assert target.stat().st_mode & 0o777 == 0o644


def test_visual_defaults_and_articulated_compound_colliders():
    scene = ET.fromstring("""<mujoco>
      <default>
        <default class="display"><geom contype="0" conaffinity="0"/></default>
      </default>
      <worldbody>
        <geom name="floor" type="plane"/>
        <geom name="wall" type="box"/>
        <body childclass="display"><geom name="visible_floor" type="mesh"/></body>
        <body name="door"><joint type="hinge"/>
          <geom name="door_panel" type="box"/>
          <body><geom name="door_trim" type="box"/></body>
          <geom name="door_visual" class="display" type="mesh"/>
        </body>
      </worldbody>
    </mujoco>""")
    world_loader._environment_collision_masks(scene)
    expected = {"floor": world_loader.FLOOR_MASK, "wall": world_loader.FURNITURE_MASK,
                "visible_floor": world_loader.NO_COLLISION_MASK,
                "door_panel": world_loader.OBJECT_MASK,
                "door_trim": world_loader.OBJECT_MASK,
                "door_visual": world_loader.NO_COLLISION_MASK}
    for name, masks in expected.items():
        geom = scene.find(f".//geom[@name='{name}']")
        assert tuple(int(geom.get(key)) for key in ("contype", "conaffinity")) == masks


def test_frames_pass_on_their_childclass_and_joints():
    """A frame passes on its childclass, and its joint moves the enclosing body."""
    scene = ET.fromstring("""<mujoco>
      <default>
        <default class="display"><geom contype="0" conaffinity="0"/></default>
      </default>
      <worldbody>
        <geom name="floor" type="plane"/>
        <frame childclass="display"><geom name="framed_visual" type="mesh"/></frame>
        <body name="shelf">
          <frame childclass="display"><geom name="shelf_visual" type="mesh"/></frame>
          <frame><geom name="shelf_board" type="box"/></frame>
        </body>
        <body name="drawer">
          <frame pos="0 0 .1"><frame><joint type="slide"/></frame></frame>
          <geom name="drawer_box" type="box"/>
          <body><geom name="drawer_handle" type="box"/></body>
        </body>
      </worldbody>
    </mujoco>""")
    world_loader._environment_collision_masks(scene)
    expected = {"framed_visual": world_loader.NO_COLLISION_MASK,
                "shelf_visual": world_loader.NO_COLLISION_MASK,
                "shelf_board": world_loader.FURNITURE_MASK,
                "drawer_box": world_loader.OBJECT_MASK,
                "drawer_handle": world_loader.OBJECT_MASK}
    for name, masks in expected.items():
        geom = scene.find(f".//geom[@name='{name}']")
        assert tuple(int(geom.get(key)) for key in ("contype", "conaffinity")) == masks, name


def test_frame_classes_match_a_mujoco_compile():
    """The loader keeps a frame's visual-only geoms at the masks MuJoCo compiles."""
    mujoco = pytest.importorskip("mujoco")
    xml = """<mujoco>
      <default><default class="display"><geom contype="0" conaffinity="0"/></default></default>
      <worldbody>
        <geom name="floor" type="plane" size="1 1 .1"/>
        <frame childclass="display"><geom name="framed" type="box" size=".1 .1 .1"/></frame>
      </worldbody>
    </mujoco>"""
    model = mujoco.MjModel.from_xml_string(xml)
    compiled = (int(model.geom("framed").contype[0]), int(model.geom("framed").conaffinity[0]))
    scene = ET.fromstring(xml)
    world_loader._environment_collision_masks(scene)
    geom = scene.find(".//geom[@name='framed']")
    assert (int(geom.get("contype")), int(geom.get("conaffinity"))) == compiled == (0, 0)


def test_malformed_xml_names_the_file_and_original_line(tmp_path):
    scene = tmp_path / "scene.xml"
    scene.write_text('<mujoco>\n<!-- a comment\n   over three lines -->\n'
                     '<worldbody>\n<geom name="floor" type="plane"\n</worldbody>\n</mujoco>\n')
    with pytest.raises(ValueError, match=r"scene\.xml: malformed XML: .*line 6") as error:
        world_loader._load_source(scene)
    assert str(scene) in str(error.value)


def test_missing_asset_is_a_value_error_naming_both_files(tmp_path):
    with pytest.raises(ValueError, match=r"world\.xml: missing asset .*absent\.png"):
        _assemble(tmp_path, settings='<asset><texture name="t" type="2d" file="absent.png"/>'
                                     '</asset>')


def test_floor_requires_analytic_plane():
    scene = ET.fromstring(
        '<mujoco><worldbody><geom name="floor" type="mesh"/></worldbody></mujoco>'
    )
    with pytest.raises(ValueError, match="must be a plane"):
        world_loader._environment_collision_masks(scene)


def test_spawn_preserves_upper_body_controls_and_keyframe_heights():
    root = ET.fromstring("""<mujoco><worldbody>
      <body name="base_footprint" pos="0 0 0.12">
        <joint name="floating_base_joint" type="free"/>
      </body>
      </worldbody><keyframe>
        <key name="home" qpos="0 0 0.13 1 0 0 0 .4 -.3" ctrl="2 3"/>
        <key name="other" qpos="0 0 0.14 1 0 0 0 .7 -.6" ctrl="4 5"/>
      </keyframe></mujoco>""")
    world_loader._set_spawn(root, (1.76, -1.48, -90.))
    assert world_loader._spawn_xyyaw(root)[2] == pytest.approx(-math.pi / 2)
    for name, height, joints, controls in (("home", .13, [.4, -.3], "2 3"),
                                           ("other", .14, [.7, -.6], "4 5")):
        key = root.find(f"keyframe/key[@name='{name}']")
        qpos = list(map(float, key.get("qpos").split()))
        assert qpos[:3] == [1.76, -1.48, height]
        assert qpos[7:] == joints
        assert key.get("ctrl") == controls
    assert root.find("worldbody/body").get("pos") == "1.76 -1.48 0.12"


def test_spawn_matching_amcl_keeps_other_navigation_settings(tmp_path):
    import yaml

    root = ET.fromstring(MERGED)
    source = tmp_path / "source.yaml"
    target = tmp_path / "generated.yaml"
    original = (
        "amcl:\n  ros__parameters:\n    robot_model_type: nav2_amcl::OmniMotionModel\n"
        "controller_server:\n  ros__parameters:\n    controller_frequency: 25.0\n"
    )
    source.write_text(original)
    world_loader.write_nav2_params(source, target, root)
    generated = yaml.safe_load(target.read_text())
    assert source.read_text() == original
    assert generated["controller_server"]["ros__parameters"]["controller_frequency"] == 25.
    amcl = generated["amcl"]["ros__parameters"]
    assert amcl["robot_model_type"] == "nav2_amcl::OmniMotionModel"
    assert amcl["set_initial_pose"] is True
    assert (amcl["initial_pose"]["x"], amcl["initial_pose"]["y"]) == (1.76, -1.48)
    assert amcl["initial_pose"]["yaw"] == pytest.approx(-math.pi / 2)


def test_world_top_level_defaults_are_rejected(tmp_path):
    with pytest.raises(ValueError, match=r"top-level <default> sets <joint>, <geom>\. "
                                         r"MuJoCo .* including the robot"):
        _assemble(tmp_path, defaults='<joint damping="7"/><geom friction=".1 .001 .0001"/>')


def test_robot_top_level_defaults_are_rejected(tmp_path):
    with pytest.raises(ValueError, match="including the world"):
        _assemble(tmp_path, robot_extra='<default><geom friction="1 .01 .001"/></default>')


def test_world_default_classes_keep_working(tmp_path, loader_log):
    merged = _assemble(tmp_path)
    (default,) = merged.findall("default")
    assert not default.attrib
    assert [(child.tag, child.get("class")) for child in default] == [("default", "room_static")]
    crate = merged.find(".//body[@name='crate']/geom")
    assert crate.get("class") == "room_static"
    assert (int(crate.get("contype")), int(crate.get("conaffinity"))) == world_loader.OBJECT_MASK
    arm_joint = merged.find(".//joint[@name='arm_joint']")
    assert arm_joint.attrib == {"name": "arm_joint", "axis": "0 0 1"}
    assert not loader_log.records


def test_default_class_names_must_differ_from_the_robot(tmp_path):
    robot_class = '<default><default class="room_static"><geom condim="1"/></default></default>'
    with pytest.raises(ValueError, match="default classes room_static are also defined"):
        _assemble(tmp_path, robot_extra=robot_class)


@pytest.mark.parametrize("compiler", ['angle="degree"', ""])
def test_world_compiler_angle_must_match_the_robot(tmp_path, compiler):
    with pytest.raises(ValueError,
                       match="Compiler angle differs: robot=radian, environment=degree"):
        _assemble(tmp_path, compiler=compiler)


def test_world_inertia_compiler_settings_are_dropped_with_a_warning(tmp_path, loader_log):
    merged = _assemble(tmp_path, compiler=(
        'angle="radian" autolimits="true" inertiafromgeom="true" inertiagrouprange="3 3" '
        'settotalmass="161" fusestatic="true"'
    ))
    assert merged.find("compiler").attrib == {"angle": "radian", "strippath": "false"}
    (warning,) = loader_log.messages(logging.WARNING)
    assert warning.startswith(f"{tmp_path / 'world.xml'}: ignored world <compiler> settings")
    for setting in ('<compiler inertiafromgeom="true"> -> auto',
                    '<compiler inertiagrouprange="3 3"> -> 0 5',
                    '<compiler settotalmass="161"> -> -1',
                    '<compiler fusestatic="true"> -> false'):
        assert setting in warning
        assert setting in _comments(merged)
    assert "autolimits" not in warning


def test_world_physics_and_camera_settings_come_from_the_robot(tmp_path, loader_log):
    merged = _assemble(tmp_path, settings="""
      <option timestep="0.002" iterations="40" integrator="Euler">
        <flag contact="disable"/>
      </option>
      <size memory="8M"/>
      <statistic center="1 2 0.7" extent="6.5" meaninertia="3"/>
      <visual>
        <global azimuth="125" elevation="-65" offwidth="1280" offheight="960"/>
        <quality offsamples="2"/>
        <map znear="0.003" zfar="8"/>
        <headlight diffuse=".3 .3 .3"/>
        <rgba haze=".6 .7 .7 1" contactpoint="1 0 0 1"/>
      </visual>""")
    option = merged.find("option")
    assert option.attrib == {"integrator": "implicitfast", "cone": "elliptic"}
    assert not list(option)
    assert merged.find("size").attrib == {"nkey": "1"}
    assert merged.find("statistic").attrib == {"center": "1 2 0.7", "extent": "2.2"}
    visual = merged.find("visual")
    assert visual.find("global").attrib == {
        "offwidth": "1280", "offheight": "720", "azimuth": "125", "elevation": "-65"}
    assert visual.find("quality") is None
    assert visual.find("map").attrib == {"znear": "0.025"}
    assert visual.find("headlight").attrib == {"diffuse": ".3 .3 .3"}
    assert visual.find("rgba").attrib == {"haze": ".6 .7 .7 1"}
    assert not loader_log.messages(logging.WARNING)
    (info,) = loader_log.messages(logging.INFO)
    assert info.startswith(f"{tmp_path / 'world.xml'}: ignored world settings")
    for setting in ('<option iterations="40"> -> 100',
                    '<option integrator="Euler"> -> implicitfast',
                    '<option><flag contact="disable"> -> MuJoCo default',
                    '<size memory="8M"> -> automatic',
                    '<statistic extent="6.5"> -> 2.2',
                    '<statistic meaninertia="3"> -> MuJoCo default',
                    '<visual><global offheight="960"> -> 720',
                    '<visual><quality offsamples="2"> -> 4',
                    '<visual><map znear="0.003"> -> 0.025',
                    '<visual><map zfar="8"> -> 50',
                    '<visual><rgba contactpoint="1 0 0 1"> -> MuJoCo default'):
        assert setting in info
        assert setting in _comments(merged)
    for unchanged in ("timestep", "offwidth", "azimuth", "center", "headlight", "haze"):
        assert unchanged not in info


@pytest.mark.parametrize("robot_memory, world_memory, merged_memory", [
    ("16M", "64M", "64M"),
    ("16M", "8M", "16M"),
])
def test_world_memory_keeps_the_larger_explicit_arena(tmp_path, robot_memory, world_memory,
                                                      merged_memory):
    merged = _assemble(tmp_path, settings=f'<size memory="{world_memory}"/>',
                       robot_size=f'memory="{robot_memory}"')
    assert merged.find("size").attrib == {"nkey": "1", "memory": merged_memory}


def test_world_without_settings_gets_the_reference_camera_rule(tmp_path, loader_log):
    merged = _assemble(tmp_path)
    assert merged.find("statistic").attrib == {"extent": "2.2", "center": "0 0 0.7"}
    assert merged.find("visual/map").attrib == {"znear": "0.025"}
    assert "Ignored world settings" not in _comments(merged)


def test_unsupported_world_sections_are_rejected(tmp_path):
    with pytest.raises(ValueError, match="unsupported top-level MJCF section <deformable>"):
        _assemble(tmp_path, settings="<deformable/>")


def test_cube_texture_faces_resolve_to_absolute_paths(tmp_path):
    faces = ("right", "left", "up", "down", "front", "back")
    (tmp_path / "textures").mkdir()
    for face in faces:
        (tmp_path / "textures" / f"{face}.png").write_bytes(b"")
    texture = " ".join(f'file{face}="{face}.png"' for face in faces)
    merged = _assemble(tmp_path, compiler='angle="radian" texturedir="textures"',
                       settings=f'<asset><texture name="sky" type="skybox" {texture}/></asset>')
    sky = merged.find("asset/texture[@name='sky']")
    for face in faces:
        assert sky.get(f"file{face}") == str((tmp_path / "textures" / f"{face}.png").resolve())
    assert "texturedir" not in merged.find("compiler").attrib


def _compiler_state(compiler) -> dict:
    # 'authored' records which attributes the XML set, not their values.
    return {name: repr(getattr(compiler, name)) for name in dir(compiler)
            if not name.startswith("_") and name not in {"authored", "eulerseq"}}


@pytest.mark.parametrize("setting, value", [
    pytest.param(setting, value, id="/".join(filter(None, setting)))
    for setting, value in sorted(world_loader.MUJOCO_DEFAULTS.items())
])
def test_mujoco_defaults_table_matches_mujoco(setting, value):
    """Each MUJOCO_DEFAULTS entry, written explicitly, must compile like an empty model."""
    mujoco = pytest.importorskip("mujoco")
    section, child, key = setting
    xml = (f'<mujoco><{section}><{child} {key}="{value}"/></{section}></mujoco>' if child
           else f'<mujoco><{section} {key}="{value}"/></mujoco>')
    if section == "compiler":
        assert (_compiler_state(mujoco.MjSpec.from_string(xml).compiler)
                == _compiler_state(mujoco.MjSpec.from_string("<mujoco/>").compiler))
    else:
        model = mujoco.MjModel.from_xml_string(xml)
        empty = mujoco.MjModel.from_xml_string("<mujoco/>")
        assert (repr(model.opt), repr(model.vis), model.narena) == (
            repr(empty.opt), repr(empty.vis), empty.narena)
