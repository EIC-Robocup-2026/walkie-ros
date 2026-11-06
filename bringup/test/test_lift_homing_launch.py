"""
Tests of the venv path that lift_homing.launch.py gives to the real lift node.

The real node imports TMotorCANControl from bringup/.venv, so the launch file must name the
lib/pythonX.Y folder of the Python that runs it. The mock node does not use the venv.

The launch file is loaded from the source tree and nothing is started, so the tests need no
build and no hardware, e.g.

    pixi run python -m pytest bringup/test/test_lift_homing_launch.py
"""

import importlib.util
from pathlib import Path
import sys

from launch import LaunchContext
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
import pytest

BRINGUP_DIR = Path(__file__).resolve().parent.parent
LAUNCH_FILE = BRINGUP_DIR / 'launch' / 'lift_homing.launch.py'
REAL_NODE = 'lift_homing_node.py'
MOCK_NODE = 'lift_homing_mock_node.py'
OVERRIDE = 'ROBOT_BRINGUP_VENV_SITE'


def expected_venv_site():
    python_dir = f'python{sys.version_info.major}.{sys.version_info.minor}'
    return str(BRINGUP_DIR / '.venv' / 'lib' / python_dir / 'site-packages')


def load_launch_file(path):
    spec = importlib.util.spec_from_file_location('lift_homing_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def additional_env(description, executable):
    node = next(
        entity for entity in description.entities
        if isinstance(entity, Node) and entity.node_executable == executable
    )
    context = LaunchContext()
    return {
        perform_substitutions(context, key): perform_substitutions(context, value)
        for key, value in node.process_description.additional_env or []
    }


@pytest.fixture(autouse=True)
def clean_environment(monkeypatch):
    monkeypatch.delenv('PYTHONPATH', raising=False)
    monkeypatch.delenv(OVERRIDE, raising=False)


def test_real_node_gets_the_venv_of_the_running_python():
    env = additional_env(load_launch_file(LAUNCH_FILE), REAL_NODE)
    assert env['PYTHONPATH'] == expected_venv_site()


def test_venv_is_found_through_the_symlink_that_symlink_install_creates(tmp_path):
    link = tmp_path / LAUNCH_FILE.name
    link.symlink_to(LAUNCH_FILE)
    env = additional_env(load_launch_file(link), REAL_NODE)
    assert env['PYTHONPATH'] == expected_venv_site()


def test_existing_pythonpath_is_kept_behind_the_venv(monkeypatch):
    monkeypatch.setenv('PYTHONPATH', '/existing/path')
    env = additional_env(load_launch_file(LAUNCH_FILE), REAL_NODE)
    assert env['PYTHONPATH'] == f'{expected_venv_site()}:/existing/path'


def test_override_replaces_the_venv_path(monkeypatch):
    monkeypatch.setenv(OVERRIDE, '/custom/site-packages')
    env = additional_env(load_launch_file(LAUNCH_FILE), REAL_NODE)
    assert env['PYTHONPATH'] == '/custom/site-packages'


def test_mock_node_does_not_use_the_venv():
    env = additional_env(load_launch_file(LAUNCH_FILE), MOCK_NODE)
    assert 'PYTHONPATH' not in env
