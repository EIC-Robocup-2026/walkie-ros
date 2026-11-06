"""
Tests of the interpreter header of the scripts that must not use `python3` from PATH.

Under pixi the scripts must run on the Python in $CONDA_PREFIX. Without pixi they must run on the
system Python, because `python3` on PATH can be a venv without rclpy. No Python of a real
environment is started and no node runs, so the tests need no build and no hardware, e.g.

    pixi run python -m pytest bringup/test/test_script_interpreter.py
"""

import ast
import os
from pathlib import Path
import subprocess

import pytest

SCRIPTS_DIR = Path(__file__).resolve().parent.parent / 'scripts'
SCRIPTS = ['zed_watchdog.py', 'head_touch_beep_node.py']
EXEC_WORD = '"exec"'


def environment(**variables):
    env = {key: value for key, value in os.environ.items() if key != 'CONDA_PREFIX'}
    env.update(variables)
    return env


@pytest.mark.parametrize('script', SCRIPTS)
def test_header_is_valid_python(script):
    ast.parse((SCRIPTS_DIR / script).read_text())


@pytest.mark.parametrize('script', SCRIPTS)
def test_script_runs_on_the_python_of_the_environment(script, tmp_path):
    python = tmp_path / 'bin' / 'python3'
    python.parent.mkdir()
    python.write_text('#!/bin/sh\necho "stub $*"\n')
    python.chmod(0o755)
    path = SCRIPTS_DIR / script

    result = subprocess.run(
        [str(path), '--ros-args', '-r', '__node:=test'],
        env=environment(CONDA_PREFIX=str(tmp_path)),
        capture_output=True,
        text=True,
        check=True,
    )

    assert result.stdout.strip() == f'stub {path} --ros-args -r __node:=test'


@pytest.mark.parametrize('script', SCRIPTS)
def test_script_falls_back_to_the_system_python(script):
    path = SCRIPTS_DIR / script
    header = path.read_text().splitlines()[1]
    assert header.startswith(EXEC_WORD)

    result = subprocess.run(
        ['sh', '-c', header.replace(EXEC_WORD, 'echo', 1), str(path), '--flag'],
        env=environment(),
        capture_output=True,
        text=True,
        check=True,
    )

    assert result.stdout.strip() == f'/usr/bin/python3 {path} --flag'
