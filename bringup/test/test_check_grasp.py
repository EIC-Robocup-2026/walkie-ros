"""Pass criteria of the offline grasp check (simulation/mujoco/tools/check_grasp.py)."""
import importlib.util
from pathlib import Path
import sys

import pytest

pytest.importorskip("mujoco")

TOOLS_DIR = Path(__file__).resolve().parents[2] / "simulation" / "mujoco" / "tools"
# Measured by check_grasp.py chula_pat14_6 with the default arguments.
PASSING = {"gap_range": (0.0119, 0.0128), "hand_rise": 0.1000, "object_rise": 0.0989,
           "slip": 0.0056}
LIMITS = {"lift": 0.10, "lift_tolerance": 0.005, "max_slip": 0.01}


@pytest.fixture(scope="module")
def check_grasp():
    path = TOOLS_DIR / "check_grasp.py"
    if not path.is_file():
        pytest.skip("check_grasp.py is not in this source tree")
    # The tool imports world_tools from its own directory.
    sys.path.insert(0, str(TOOLS_DIR))
    try:
        spec = importlib.util.spec_from_file_location("check_grasp", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    finally:
        sys.path.remove(str(TOOLS_DIR))
    return module


def _checks(check_grasp, **changes):
    return check_grasp.grasp_checks(**{**PASSING, **LIMITS, **changes})


def test_a_held_lift_passes_every_check(check_grasp):
    assert all(_checks(check_grasp).values())


@pytest.mark.parametrize("gap_range", [(0.003, 0.0128), (0.0119, 0.046)])
def test_a_gap_that_leaves_the_band_between_samples_fails(check_grasp, gap_range):
    # The gap after the lift and after the carry can both be in the band while it left
    # the band in between.
    checks = _checks(check_grasp, gap_range=gap_range)
    assert checks["gap_in_grasp_band"] is False
    assert sum(not passed for passed in checks.values()) == 1


def test_a_short_lift_fails_even_if_the_object_follows(check_grasp):
    checks = _checks(check_grasp, hand_rise=0.01, object_rise=0.0099)
    assert checks["object_rose_with_hand"] is True
    assert checks["hand_rose_commanded_lift"] is False


@pytest.mark.parametrize("hand_rise, expected", [(0.0951, True), (0.1049, True),
                                                 (0.0949, False), (0.1051, False)])
def test_the_hand_must_rise_within_the_tolerance(check_grasp, hand_rise, expected):
    checks = _checks(check_grasp, hand_rise=hand_rise, object_rise=hand_rise)
    assert checks["hand_rose_commanded_lift"] is expected


def test_slip_over_the_limit_fails(check_grasp):
    assert _checks(check_grasp, slip=0.02)["slip_below_limit"] is False
