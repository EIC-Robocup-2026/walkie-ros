"""Decisions of the scripted bottle pick, without ROS (robot_bringup.sim_pick)."""
import pytest
from robot_bringup import sim_pick
from robot_bringup.sim_pick import (
    FAILED, INCONCLUSIVE, NAV_RETRY, NAV_STOP, NAV_SUCCEEDED, PASSED, PASSED_FINGER_ONLY,
    STATUS_ABORTED, STATUS_CANCELED, STATUS_SUCCEEDED,
)


def test_status_values_match_action_msgs():
    goal_status = pytest.importorskip("action_msgs.msg").GoalStatus
    for value, name in sim_pick.STATUS_NAMES.items():
        assert getattr(goal_status, f"STATUS_{name}") == value


def test_transient_navigation_errors_match_nav2_msgs():
    actions = pytest.importorskip("nav2_msgs.action")
    for code, name in sim_pick.NAV_TRANSIENT_ERRORS.items():
        action, constant = name.split()
        assert getattr(getattr(actions, action).Result, constant) == code


@pytest.mark.parametrize("times, expected", [
    ([1, 2, 3], None),
    ([1, 2, 3, 3, 4], None),
    ([], "0 messages"),
    ([1, 2], "2 messages"),
    ([5, 5, 5], "1 distinct times"),
    ([1, 2, 2], "2 distinct times"),
    ([3, 4, 2, 5], "went back"),
])
def test_clock_needs_three_advancing_samples(times, expected):
    problem = sim_pick.clock_problem(times)
    if expected is None:
        assert problem is None
    else:
        assert expected in problem and problem.startswith("/clock")


def test_gate_passes_only_the_chula_simulation():
    sim_time = {"/bimanual_commander": True, "/bt_navigator": True}
    assert sim_pick.gate_failures(None, sim_time, "chula_pat14_6") == []


def test_gate_names_every_failed_prerequisite():
    failures = sim_pick.gate_failures(
        "/clock: 0 messages", {"/bimanual_commander": False, "/bt_navigator": None}, "")
    assert failures[0] == "/clock: 0 messages"
    assert any(failure.startswith("/bimanual_commander: use_sim_time is False")
               for failure in failures)
    assert "/bt_navigator: use_sim_time could not be read" in failures
    assert any(failure.startswith("mujoco_world: ''") for failure in failures)
    assert len(failures) == 4


@pytest.mark.parametrize("world", [None, "empty_floor", "Chula_PAT14_6"])
def test_gate_rejects_a_missing_or_other_world(world):
    failures = sim_pick.gate_failures(None, {"/bt_navigator": True}, world)
    assert len(failures) == 1 and failures[0].startswith("mujoco_world")


def test_gate_rejects_a_string_true_for_use_sim_time():
    assert sim_pick.gate_failures(None, {"/bt_navigator": "true"}, "chula_pat14_6")


def test_one_server_per_action_passes():
    servers = {"/navigate_to_pose": ["/bt_navigator"], "/go_to_pose": ["/bimanual_commander"]}
    assert sim_pick.action_server_failures(servers) == []


def test_a_second_or_missing_action_server_fails():
    failures = sim_pick.action_server_failures({
        "/navigate_to_pose": ["/bt_navigator", "/bt_navigator"],
        "/go_to_pose": ["/bimanual_commander"],
        "/control_gripper": [],
    })
    assert failures == [
        "/navigate_to_pose: 2 action servers (/bt_navigator, /bt_navigator), need exactly one",
        "/control_gripper: 0 action servers (none), need exactly one",
    ]


def test_commander_defaults_let_the_pick_start():
    values = {"allow_gripper_vs_octomap": False, "grasp_scene_action": "none"}
    assert values == sim_pick.COMMANDER_DEFAULTS
    assert sim_pick.commander_problems(values, "/bimanual_commander") == []


@pytest.mark.parametrize("name, value, command", [
    ("allow_gripper_vs_octomap", True,
     "ros2 param set /bimanual_commander allow_gripper_vs_octomap false"),
    # 0 == False in Python, but the commander's parameter is a bool.
    ("allow_gripper_vs_octomap", 0,
     "ros2 param set /bimanual_commander allow_gripper_vs_octomap false"),
    ("allow_gripper_vs_octomap", None,
     "ros2 param set /bimanual_commander allow_gripper_vs_octomap false"),
    ("grasp_scene_action", "grasp", "ros2 param set /bimanual_commander grasp_scene_action none"),
    ("grasp_scene_action", "place", "ros2 param set /bimanual_commander grasp_scene_action none"),
    ("grasp_scene_action", "None", "ros2 param set /bimanual_commander grasp_scene_action none"),
])
def test_commander_values_other_than_the_defaults_stop_the_pick(name, value, command):
    values = {**sim_pick.COMMANDER_DEFAULTS, name: value}
    problems = sim_pick.commander_problems(values, "/bimanual_commander")
    assert len(problems) == 1
    assert problems[0].startswith(f"/bimanual_commander: {name} is {value!r}")
    assert problems[0].endswith(command)


def test_commander_that_does_not_answer_stops_the_pick():
    problems = sim_pick.commander_problems(None, "/bimanual_commander")
    assert problems == ["/bimanual_commander: allow_gripper_vs_octomap, grasp_scene_action "
                        "could not be read"]


@pytest.mark.parametrize("status, code, verdict", [
    (STATUS_SUCCEEDED, 0, NAV_SUCCEEDED),
    # A canceled goal's result carries the default error_code 0; it is not a success.
    (STATUS_CANCELED, 0, NAV_STOP),
    (STATUS_SUCCEEDED, 9000, NAV_STOP),
    (STATUS_ABORTED, 207, NAV_RETRY),
    (STATUS_ABORTED, 107, NAV_RETRY),
    (STATUS_ABORTED, 105, NAV_RETRY),
    (STATUS_ABORTED, 103, NAV_RETRY),
    (STATUS_ABORTED, 104, NAV_STOP),
    (STATUS_ABORTED, 208, NAV_STOP),
    (STATUS_ABORTED, 9003, NAV_STOP),
    (STATUS_ABORTED, 0, NAV_STOP),
    (STATUS_ABORTED, None, NAV_STOP),
    (sim_pick.GOAL_REJECTED, None, NAV_STOP),
    (sim_pick.GOAL_TIMED_OUT, 207, NAV_STOP),
    (sim_pick.STATUS_UNKNOWN, 0, NAV_STOP),
])
def test_navigation_retries_only_transient_aborts(status, code, verdict):
    assert sim_pick.classify_navigation(status, code)[0] == verdict


def test_navigation_reason_names_the_end():
    assert "CANCELED" in sim_pick.classify_navigation(STATUS_CANCELED, 0)[1]
    assert "TIMEOUT" in sim_pick.classify_navigation(STATUS_ABORTED, 207)[1]


@pytest.mark.parametrize("stamp, received, expected", [
    (9_800_000_000, 99.5, None),
    # Stamps from the simulator can be a little ahead of this node's clock.
    (10_300_000_000, 99.5, None),
    (9_400_000_000, 99.5, "stamp"),
    (10_600_000_000, 99.5, "stamp"),
    (9_900_000_000, 98.5, "received"),
    (None, None, "no reading"),
])
def test_reading_age(stamp, received, expected):
    problem = sim_pick.reading_age_problem(stamp, 10_000_000_000, received, 100.0, 0.5, 1.0)
    if expected is None:
        assert problem is None
    else:
        assert problem.startswith(expected)


@pytest.mark.parametrize("value, expected", [
    (0.0134, True), (0.005, True), (0.044, True), (0.004, False), (0.05, False), (None, False),
])
def test_in_band(value, expected):
    assert sim_pick.in_band(value, (0.005, 0.044)) is expected


def test_in_band_needs_a_known_band():
    assert sim_pick.in_band(0.01, (None, 0.044)) is False


@pytest.mark.parametrize("rise, expected", [
    (0.0972, None), (0.081, None), (0.119, None), (0.079, "hand rose"), (0.005, "hand rose"),
    (0.121, "hand rose"), (None, "hand rise unknown"),
])
def test_lift_needs_the_commanded_rise(rise, expected):
    problem = sim_pick.lift_problem(rise, 0.10, 0.02)
    if expected is None:
        assert problem is None
    else:
        assert problem.startswith(expected)


# Counts from a passing run in the Chula world.
LIFTED = {"old_place_lower_before": 1433, "old_place_lower_after": 0,
          "above_old_top_after": 303, "table_near_old_place_after": 388}


def test_camera_confirms_a_lifted_bottle():
    assert sim_pick.camera_verdict(LIFTED)[0] is True


@pytest.mark.parametrize("change", [
    {"old_place_lower_after": 500},
    {"above_old_top_after": 3},
])
def test_camera_sees_a_bottle_left_down(change):
    assert sim_pick.camera_verdict({**LIFTED, **change})[0] is False


@pytest.mark.parametrize("change, reason", [
    ({"old_place_lower_before": None}, "no depth points"),
    ({"old_place_lower_after": None, "above_old_top_after": None}, "no depth points"),
    ({"old_place_lower_before": 19}, "19 points of the bottle"),
    ({"table_near_old_place_after": 5}, "5 points of the table"),
])
def test_camera_without_evidence_does_not_decide(change, reason):
    lifted, why = sim_pick.camera_verdict({**LIFTED, **change})
    assert lifted is None
    assert reason in why


@pytest.mark.parametrize("finger, hand, camera, finger_only, expected", [
    (True, True, True, False, PASSED),
    (True, True, True, True, PASSED),
    (True, True, None, False, INCONCLUSIVE),
    (True, True, None, True, PASSED_FINGER_ONLY),
    (True, True, False, True, FAILED),
    (False, True, True, False, FAILED),
    (True, False, True, False, FAILED),
    (False, True, None, True, FAILED),
    (None, True, True, False, INCONCLUSIVE),
    (None, True, None, True, INCONCLUSIVE),
    (True, None, True, False, INCONCLUSIVE),
    (None, False, True, False, FAILED),
])
def test_hold_verdict(finger, hand, camera, finger_only, expected):
    assert sim_pick.hold_verdict(finger_in_band=finger, hand_at_target=hand, camera=camera,
                                 accept_finger_only=finger_only) == expected


@pytest.mark.parametrize("result, cleanup_ok, resolved, interrupted, code", [
    (PASSED, True, True, False, 0),
    (PASSED_FINGER_ONLY, True, True, False, 0),
    (FAILED, True, True, False, 1),
    (sim_pick.ERROR, True, True, False, 1),
    (sim_pick.NOT_RUN, True, True, False, 2),
    (INCONCLUSIVE, True, True, False, 3),
    (sim_pick.INTERRUPTED, True, True, True, 130),
    (PASSED, False, True, False, 4),
    (FAILED, True, False, False, 4),
    (sim_pick.INTERRUPTED, False, True, True, 4),
    (sim_pick.INTERRUPTED, True, False, True, 4),
])
def test_exit_code(result, cleanup_ok, resolved, interrupted, code):
    assert sim_pick.exit_code(result, cleanup_ok=cleanup_ok, goals_resolved=resolved,
                              interrupted=interrupted) == code
