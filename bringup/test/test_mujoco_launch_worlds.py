"""Resolve launch actions without starting simulators, controllers, or ROS nodes."""
import importlib.util
from pathlib import Path
import sys

import ament_index_python.packages
from launch import LaunchContext
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction,
    SetLaunchConfiguration, UnsetEnvironmentVariable,
)
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
import pytest
from robot_bringup import mujoco_launch, world_loader
import yaml


LAUNCH_DIR = Path(__file__).resolve().parents[1] / "launch"
PREPARED = {
    "model_path": "/tmp/walkie-test-cache/world.xml",
    "map_path": "/tmp/walkie-test-assets/map.yaml",
    "params_path": "/tmp/walkie-test-cache/nav2.yaml",
    "variant": "lowpoly",
    "spawn_pose": (0., 0., 0.),
}


def _load(name):
    spec = importlib.util.spec_from_file_location(name, LAUNCH_DIR / f"{name}.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _visit(entities, context, actions):
    for entity in entities:
        if isinstance(entity, (DeclareLaunchArgument, SetLaunchConfiguration)):
            entity.execute(context)
        elif isinstance(entity, OpaqueFunction):
            _visit(entity.execute(context) or [], context, actions)
        else:
            actions.append(entity)


def _resolve(module, **arguments):
    """Run argument declarations and OpaqueFunctions; return the other actions."""
    context = LaunchContext()
    context.launch_configurations.update(arguments)
    actions = []
    _visit(module.generate_launch_description().entities, context, actions)
    return context, actions


def _flatten(actions):
    for action in actions:
        yield action
        if isinstance(action, GroupAction):
            yield from _flatten(action.get_sub_entities())


def _includes(actions):
    return [action for action in _flatten(actions)
            if isinstance(action, IncludeLaunchDescription)]


def _active_nodes(actions, context):
    return [action for action in _flatten(actions) if isinstance(action, Node)
            and (action.condition is None or action.condition.evaluate(context))]


def _arguments(include, context):
    return {
        key: perform_substitutions(context, normalize_to_list_of_substitutions(value))
        for key, value in include.launch_arguments
    }


@pytest.fixture
def prepared_world(monkeypatch):
    calls = []

    def prepare_world(**kwargs):
        calls.append(kwargs)
        return dict(PREPARED, params_path=(
            None if kwargs["nav2_params_source"] is None else PREPARED["params_path"]
        ))

    monkeypatch.setattr(world_loader, "prepare_world", prepare_world)
    return calls


def test_room_uses_amcl_and_prepares_only_once(prepared_world):
    nav = _load("nav_mujoco_omnibot")
    context, actions = _resolve(
        nav, world="chula_pat14_6", variant="baked", spawn="living-room"
    )
    assert len(prepared_world) == 1
    call = prepared_world[0]
    assert call["variant"] == "baked"
    assert call["spawn"] == "living-room"
    assert call["spawn_pose"] is None
    assert call["worlds_dir"] is None
    assert str(call["nav2_params_source"]).endswith("nav2_mppi_walkie_sim_params.yaml")
    assert _active_nodes(actions, context) == []
    simulator, navigation = _includes(actions)
    nav_args = _arguments(navigation, context)
    assert nav_args["use_localization"] == "True"
    assert nav_args["map"] == PREPARED["map_path"]
    assert nav_args["params_file"] == PREPARED["params_path"]
    assert nav_args["use_keepout_zones"] == "False"
    assert nav_args["use_speed_zones"] == "False"
    assert nav_args["use_composition"] == "False"

    # Resolve the nested simulator launch as well. It receives the generated
    # model, with world/spawn cleared, and must not call preparation again.
    sim_args = _arguments(simulator, context)
    assert (sim_args["world"], sim_args["spawn"], sim_args["spawn_pose"]) == ("", "", "")
    sim_context, _ = _resolve(_load("mujoco_omnibot"), **sim_args)
    assert len(prepared_world) == 1
    assert sim_context.launch_configurations["mujoco_model"] == PREPARED["model_path"]


def test_navigation_rviz_config_sets_the_initial_pose_and_goals(prepared_world):
    # AMCL in a named world can drift off the true pose; the default RViz config
    # must offer 2D Pose Estimate on the topic AMCL subscribes to.
    context, _ = _resolve(_load("nav_mujoco_omnibot"), world="chula_pat14_6")
    config = yaml.safe_load(Path(context.launch_configurations["rviz_config"]).read_text())
    topics = {tool["Class"]: tool.get("Topic", {}).get("Value")
              for tool in config["Visualization Manager"]["Tools"]}
    assert topics["rviz_default_plugins/SetInitialPose"] == "/initialpose"
    assert topics["rviz_default_plugins/SetGoal"] == "/goal_pose"


def test_empty_floor_retains_static_map_odom(prepared_world):
    context, actions = _resolve(_load("nav_mujoco_omnibot"))
    assert prepared_world == []
    nodes = _active_nodes(actions, context)
    assert [node.node_executable for node in nodes] == ["static_transform_publisher"]
    simulator, navigation = _includes(actions)
    assert _arguments(simulator, context)["mujoco_model"].endswith("/mujoco/scene.xml")
    nav_args = _arguments(navigation, context)
    assert nav_args["use_localization"] == "False"
    assert nav_args["map"].endswith("/config/map/empty_floor.yaml")
    assert nav_args["params_file"].endswith("nav2_mppi_walkie_sim_params.yaml")


def test_explicit_paths_still_work_without_world(prepared_world):
    context, actions = _resolve(
        _load("nav_mujoco_omnibot"), mujoco_model="/tmp/custom.xml",
        map="/tmp/custom.yaml", params_file="/tmp/custom-nav2.yaml",
    )
    simulator, navigation = _includes(actions)
    assert prepared_world == []
    assert _arguments(simulator, context)["mujoco_model"] == "/tmp/custom.xml"
    assert _arguments(navigation, context)["map"] == "/tmp/custom.yaml"
    assert _arguments(navigation, context)["params_file"] == "/tmp/custom-nav2.yaml"


def test_world_mode_matches_amcl_in_the_given_params_file(prepared_world):
    _resolve(_load("nav_mujoco_omnibot"), world="chula_pat14_6",
             params_file="/tmp/custom-nav2.yaml")
    assert str(prepared_world[0]["nav2_params_source"]) == "/tmp/custom-nav2.yaml"


def test_simulator_only_needs_no_navigation_package(prepared_world, monkeypatch):
    find = ament_index_python.packages.get_package_share_directory

    def without_navigation(package, *args, **kwargs):
        if package == "robot_navigation":
            raise ament_index_python.packages.PackageNotFoundError(package)
        return find(package, *args, **kwargs)

    monkeypatch.setattr(
        ament_index_python.packages, "get_package_share_directory", without_navigation
    )
    context, _ = _resolve(
        _load("mujoco_omnibot"), world="chula_pat14_6", spawn_pose="1.76 -1.48 -90",
    )
    assert len(prepared_world) == 1
    call = prepared_world[0]
    assert call["nav2_params_source"] is None
    assert call["spawn_pose"] == "1.76 -1.48 -90"
    assert call["spawn"] is None
    assert call["worlds_dir"] is None
    assert Path(call["robot_path"]).name == "walkie.xml"
    assert context.launch_configurations["mujoco_model"] == PREPARED["model_path"]


@pytest.mark.parametrize("launch_name", ["nav_mujoco_omnibot", "mujoco_omnibot"])
def test_empty_variant_selects_the_manifest_default(launch_name, prepared_world):
    _resolve(_load(launch_name), world="chula_pat14_6")
    assert prepared_world[0]["variant"] is None


@pytest.mark.parametrize("launch_name, nav2", [
    ("nav_mujoco_omnibot", True), ("mujoco_omnibot", False),
])
def test_world_arguments_are_declared_without_fixed_choices(launch_name, nav2):
    declared = {
        action.name: action
        for action in _load(launch_name).generate_launch_description().entities
        if isinstance(action, DeclareLaunchArgument)
    }
    assert set(mujoco_launch.WORLD_ARGUMENTS) <= declared.keys()
    assert declared["variant"].choices is None
    # Only the Nav2 launch writes Nav2 parameters into the cache.
    assert ("Nav2" in declared["world_output_dir"].description) is nav2


@pytest.mark.parametrize("launch_name", ["nav_mujoco_omnibot", "mujoco_omnibot"])
def test_world_asset_root_can_be_overridden(launch_name, prepared_world):
    _resolve(_load(launch_name), world="chula_pat14_6", worlds_dir="/tmp/custom-worlds")
    assert len(prepared_world) == 1
    assert prepared_world[0]["worlds_dir"] == "/tmp/custom-worlds"


@pytest.mark.parametrize("launch_name, argument", [
    ("nav_mujoco_omnibot", "mujoco_model"),
    ("nav_mujoco_omnibot", "map"),
    ("mujoco_omnibot", "mujoco_model"),
])
def test_world_and_explicit_model_or_map_are_rejected(launch_name, argument, prepared_world):
    with pytest.raises(ValueError, match=f"supplies {argument}"):
        _resolve(_load(launch_name), world="chula_pat14_6", **{argument: "/tmp/custom"})
    assert prepared_world == []


@pytest.mark.parametrize("launch_name", ["nav_mujoco_omnibot", "mujoco_omnibot"])
def test_spawn_requires_a_world(launch_name, prepared_world):
    with pytest.raises(ValueError, match="require world"):
        _resolve(_load(launch_name), spawn="living-room")
    assert prepared_world == []


@pytest.mark.parametrize("environ, platform, expected", [
    ({"WAYLAND_DISPLAY": "wayland-0", "DISPLAY": ":0"}, "linux", True),
    ({"WAYLAND_DISPLAY": "wayland-0"}, "linux", False),
    ({"WAYLAND_DISPLAY": "wayland-0", "DISPLAY": ""}, "linux", False),
    ({"DISPLAY": ":0"}, "linux", False),
    ({}, "linux", False),
    ({"WAYLAND_DISPLAY": "wayland-0", "DISPLAY": ":0"}, "darwin", False),
])
def test_x11_override_requires_wayland_and_an_x11_display(environ, platform, expected):
    assert mujoco_launch.x11_override_needed(environ, platform) is expected
    action = object()
    wrapped = mujoco_launch.with_viewer_backend(action, environ, platform)
    if expected:
        # A scoped GroupAction also restores the environment after its actions.
        assert wrapped in _x11_groups([wrapped])
        assert action in wrapped.get_sub_entities()
    else:
        assert wrapped is action


def _simulator_node(actions):
    return next(action for action in _flatten(actions) if isinstance(action, Node)
                and action.node_package == "mujoco_ros2_control")


def _x11_groups(actions):
    return [action for action in _flatten(actions) if isinstance(action, GroupAction)
            and any(isinstance(child, UnsetEnvironmentVariable)
                    for child in action.get_sub_entities())]


@pytest.mark.skipif(not sys.platform.startswith("linux"), reason="Linux-only override")
def test_simulator_launch_scopes_x11_only_under_wayland(monkeypatch):
    monkeypatch.setenv("WAYLAND_DISPLAY", "wayland-0")
    monkeypatch.setenv("DISPLAY", ":0")
    _, actions = _resolve(_load("mujoco_omnibot"))
    (group,) = _x11_groups(actions)
    assert _simulator_node(actions) in group.get_sub_entities()

    monkeypatch.delenv("DISPLAY")
    _, actions = _resolve(_load("mujoco_omnibot"))
    assert _x11_groups(actions) == []
    assert _simulator_node(actions) in actions
