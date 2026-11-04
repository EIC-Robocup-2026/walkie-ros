"""Tests of the floor handling in cloud_floor_gate.py.

A floor return must reach the octomap updater as a ray that clears space and marks
nothing, which is what a point past the updater's max_range does.
"""

import importlib.util
import os

import numpy as np
import pytest

SCRIPT_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..',
    'scripts',
    'cloud_floor_gate.py',
)
spec = importlib.util.spec_from_file_location('cloud_floor_gate', SCRIPT_PATH)
cloud_floor_gate = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cloud_floor_gate)

CAMERA_HEIGHT = 1.5
FLOOR_HEIGHT = 0.05
FREE_RAY_RANGE = 100.0
# A camera whose axes match the floor frame.
LEVEL = np.eye(3)


def gate(xyz, rotation=LEVEL, camera_height=CAMERA_HEIGHT):
    return cloud_floor_gate.floor_to_free_rays(
        np.array(xyz, dtype=float), rotation, camera_height, FLOOR_HEIGHT, FREE_RAY_RANGE)


def test_floor_return_keeps_its_direction_and_moves_past_max_range():
    point = np.array([1.0, 0.5, -CAMERA_HEIGHT])
    moved = gate([point])[0]
    assert np.linalg.norm(moved) == pytest.approx(FREE_RAY_RANGE)
    assert moved / np.linalg.norm(moved) == pytest.approx(point / np.linalg.norm(point))


def test_return_above_the_floor_is_untouched():
    point = [1.0, 0.5, -1.0]
    assert gate([point])[0] == pytest.approx(point)


def test_floor_height_separates_the_two_cases():
    just_below = [1.0, 0.0, FLOOR_HEIGHT - CAMERA_HEIGHT - 0.001]
    just_above = [1.0, 0.0, FLOOR_HEIGHT - CAMERA_HEIGHT + 0.001]
    moved = gate([just_below, just_above])
    assert np.linalg.norm(moved[0]) == pytest.approx(FREE_RAY_RANGE)
    assert moved[1] == pytest.approx(just_above)


def test_tilted_camera_uses_the_height_in_the_floor_frame():
    # A camera pitched so that its x axis points straight down.
    down = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]])
    below = gate([[CAMERA_HEIGHT, 0.0, 0.0]], rotation=down)[0]
    beside = gate([[0.5, 0.0, 2.0]], rotation=down)[0]
    assert np.linalg.norm(below) == pytest.approx(FREE_RAY_RANGE)
    assert beside == pytest.approx([0.5, 0.0, 2.0])


def test_point_at_the_camera_origin_does_not_divide_by_zero():
    moved = gate([[0.0, 0.0, 0.0]], camera_height=0.0)
    assert np.isfinite(moved).all()
    assert moved[0] == pytest.approx([0.0, 0.0, 0.0])


def test_input_array_is_not_modified():
    points = np.array([[1.0, 0.0, -CAMERA_HEIGHT]])
    before = points.copy()
    cloud_floor_gate.floor_to_free_rays(points, LEVEL, CAMERA_HEIGHT, FLOOR_HEIGHT, FREE_RAY_RANGE)
    assert (points == before).all()
