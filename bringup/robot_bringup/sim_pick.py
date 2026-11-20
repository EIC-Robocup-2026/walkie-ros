"""Decisions of the scripted MuJoCo bottle pick, without ROS.

bringup/scripts/sim_bottle_pick.py sends the goals and reads the sensors; the
functions here decide what those readings mean, so they can be tested offline:
whether the graph is the Chula simulation, how a navigation attempt ended,
whether a reading is fresh, whether the lift and the hold prove the pick, and
the exit code.
"""
from __future__ import annotations

from collections.abc import Mapping, Sequence

# The exported world whose bottle pose and table height the pick script uses.
WORLD = "chula_pat14_6"

# action_msgs/msg/GoalStatus.
STATUS_UNKNOWN = 0
STATUS_ACCEPTED = 1
STATUS_EXECUTING = 2
STATUS_CANCELING = 3
STATUS_SUCCEEDED = 4
STATUS_CANCELED = 5
STATUS_ABORTED = 6
STATUS_NAMES = {
    STATUS_UNKNOWN: "UNKNOWN",
    STATUS_ACCEPTED: "ACCEPTED",
    STATUS_EXECUTING: "EXECUTING",
    STATUS_CANCELING: "CANCELING",
    STATUS_SUCCEEDED: "SUCCEEDED",
    STATUS_CANCELED: "CANCELED",
    STATUS_ABORTED: "ABORTED",
}
TERMINAL_STATUSES = frozenset({STATUS_SUCCEEDED, STATUS_CANCELED, STATUS_ABORTED})

# How a goal ended when the server never reported a terminal status for it.
GOAL_REJECTED = "rejected"
GOAL_TIMED_OUT = "timed_out"

EXIT_PASSED = 0
EXIT_FAILED = 1
EXIT_NOT_RUN = 2
EXIT_INCONCLUSIVE = 3
EXIT_UNSAFE = 4
EXIT_INTERRUPTED = 130

PASSED = "passed"
PASSED_FINGER_ONLY = "passed_finger_only"
FAILED = "failed"
INCONCLUSIVE = "inconclusive"
NOT_RUN = "not_run"
ERROR = "error"
INTERRUPTED = "interrupted"
RESULT_EXIT_CODES = {
    PASSED: EXIT_PASSED,
    PASSED_FINGER_ONLY: EXIT_PASSED,
    FAILED: EXIT_FAILED,
    ERROR: EXIT_FAILED,
    NOT_RUN: EXIT_NOT_RUN,
    INCONCLUSIVE: EXIT_INCONCLUSIVE,
}

# NavigateToPose error codes after which the same goal is sent again. bt_navigator
# reports the error code of the sub-action that failed; the values are those of
# nav2_msgs/action/ComputePathToPose and FollowPath. On a loaded machine the behavior
# tree can time out waiting for the planner (207) or the controller (107) to accept a
# goal; the controller then reports 105 when the base did not move while it waited.
# Every other code, a cancel, a rejection or an unknown end stops the pick.
NAV_TRANSIENT_ERRORS = {
    105: "FollowPath FAILED_TO_MAKE_PROGRESS",
    107: "FollowPath CONTROLLER_TIMED_OUT",
    207: "ComputePathToPose TIMEOUT",
}
NAV_SUCCEEDED = "succeeded"
NAV_RETRY = "retry"
NAV_STOP = "stop"

# The camera proof needs this many depth points of the bottle at its old place
# before the pick and of the table top around that place after it.
MIN_BOTTLE_POINTS = 20
MIN_TABLE_POINTS = 20
# After the lift, at most this fraction of the bottle's lower part may remain at its
# old place, and at least this many points must be above the bottle's old top.
MAX_REMAINING_FRACTION = 0.1
MIN_LIFTED_POINTS = 10


def status_name(status) -> str:
    """Name of a GoalStatus value or of a non-terminal end such as GOAL_REJECTED."""
    if isinstance(status, str):
        return status
    return STATUS_NAMES.get(status, f"status {status}")


def clock_problem(times_ns: Sequence[int], needed: int = 3) -> str | None:
    """Return why /clock samples do not show simulated time running, or None.

    times_ns are the /clock values in the order they arrived. Time must never go
    back, and at least ``needed`` distinct values must have arrived.
    """
    if len(times_ns) < needed:
        return f"/clock: {len(times_ns)} messages, need {needed} with advancing time"
    for earlier, later in zip(times_ns, times_ns[1:]):
        if later < earlier:
            return "/clock: time went back"
    distinct = len(set(times_ns))
    if distinct < needed:
        return f"/clock: {distinct} distinct times in {len(times_ns)} messages, need {needed}"
    return None


def gate_failures(clock: str | None, use_sim_time: Mapping[str, object],
                  world: object, expected_world: str = WORLD) -> list[str]:
    """Return the failed simulation prerequisites; empty when the pick may run.

    clock is clock_problem's verdict. use_sim_time maps each node that must run on
    simulated time to the value its parameter service returned (None when it could
    not be read). world is the simulator's mujoco_world parameter (None when unread).
    """
    failures = [] if clock is None else [clock]
    for node, value in use_sim_time.items():
        if value is None:
            failures.append(f"{node}: use_sim_time could not be read")
        elif value is not True:
            failures.append(f"{node}: use_sim_time is {value!r}, not true")
    if world is None:
        failures.append("mujoco_world: not set or could not be read")
    elif world != expected_world:
        failures.append(f"mujoco_world: {world!r}, not {expected_world!r}")
    return failures


def classify_navigation(status, error_code: int | None) -> tuple[str, str]:
    """Return (verdict, why) for one NavigateToPose goal.

    verdict is NAV_SUCCEEDED, NAV_RETRY or NAV_STOP.

    status is the terminal GoalStatus, GOAL_REJECTED or GOAL_TIMED_OUT. error_code is
    the result's error_code, or None without a result.
    """
    if status == STATUS_SUCCEEDED and error_code == 0:
        return NAV_SUCCEEDED, "reached the goal"
    if status == STATUS_ABORTED and error_code in NAV_TRANSIENT_ERRORS:
        return NAV_RETRY, f"aborted with {error_code} {NAV_TRANSIENT_ERRORS[error_code]}"
    if status == STATUS_ABORTED:
        return NAV_STOP, f"aborted with error_code {error_code}"
    if status == STATUS_SUCCEEDED:
        return NAV_STOP, f"succeeded with error_code {error_code}"
    return NAV_STOP, f"ended {status_name(status)}"


def reading_age_problem(stamp_ns: int | None, now_ns: int, received: float | None,
                        now_wall: float, max_age: float, max_wall_age: float) -> str | None:
    """Return why a sensor reading is not fresh, or None.

    stamp_ns is the reading's header stamp and now_ns the node clock, both simulated
    time. received and now_wall are wall times (time.monotonic) of the reading's
    arrival and of now. A stamp more than max_age from now in either direction, or a
    reading that arrived more than max_wall_age ago, is stale.
    """
    if stamp_ns is None or received is None:
        return "no reading"
    age = (now_ns - stamp_ns) / 1e9
    if abs(age) > max_age:
        return f"stamp {age:.3f} s from the node clock, limit {max_age} s"
    wall_age = now_wall - received
    if wall_age > max_wall_age:
        return f"received {wall_age:.3f} s ago, limit {max_wall_age} s"
    return None


def in_band(value: float | None, band: Sequence[float | None]) -> bool:
    """Whether value lies in the closed band [low, high]; False if anything is unknown."""
    return value is not None and None not in band and band[0] <= value <= band[1]


def lift_problem(rise: float | None, commanded: float, tolerance: float) -> str | None:
    """Return why a measured hand rise does not match the commanded lift, or None."""
    if rise is None:
        return "hand rise unknown"
    if abs(rise - commanded) > tolerance:
        return f"hand rose {rise:.4f} m of the commanded {commanded:.4f} m"
    return None


def camera_verdict(counts: Mapping[str, int | None]) -> tuple[bool | None, str]:
    """Return (lifted, why) from the head camera's depth point counts.

    lifted is True when the bottle left its old place and is seen above its old top,
    False when the camera sees that it did not, and None when the images do not
    decide: a count is missing, the bottle was not seen at its old place before the
    pick, or the table around that place is not seen after it.
    """
    keys = ("old_place_lower_before", "old_place_lower_after", "above_old_top_after",
            "table_near_old_place_after")
    missing = [key for key in keys if counts.get(key) is None]
    if missing:
        return None, f"no depth points for {', '.join(missing)}"
    seen = counts["old_place_lower_before"]
    if seen < MIN_BOTTLE_POINTS:
        return None, (f"{seen} points of the bottle at its old place before the pick, "
                      f"need {MIN_BOTTLE_POINTS}")
    table = counts["table_near_old_place_after"]
    if table < MIN_TABLE_POINTS:
        return None, (f"{table} points of the table around the old place after the lift, "
                      f"need {MIN_TABLE_POINTS}")
    remaining = counts["old_place_lower_after"]
    lifted = counts["above_old_top_after"]
    if remaining > MAX_REMAINING_FRACTION * seen:
        return False, f"{remaining} of {seen} bottle points remain at the old place"
    if lifted < MIN_LIFTED_POINTS:
        return False, f"{lifted} points above the old top, need {MIN_LIFTED_POINTS}"
    return True, f"{remaining} of {seen} bottle points remain, {lifted} above the old top"


def hold_verdict(*, finger_in_band: bool | None, hand_at_target: bool | None,
                 camera: bool | None, accept_finger_only: bool) -> str:
    """Return the result of the hold check.

    Each input is True or False, or None when its reading is missing or stale: the
    finger gap in the grasp band, the hand at the lifted target, and camera_verdict's
    lifted. Any False fails the pick. Missing evidence makes it inconclusive, except
    that with accept_finger_only a camera that does not decide leaves the fingers and
    the hand to pass it as PASSED_FINGER_ONLY.
    """
    readings = (finger_in_band, hand_at_target, camera)
    if any(reading is False for reading in readings):
        return FAILED
    if finger_in_band is None or hand_at_target is None:
        return INCONCLUSIVE
    if camera is True:
        return PASSED
    return PASSED_FINGER_ONLY if accept_finger_only else INCONCLUSIVE


def exit_code(result: str, *, cleanup_ok: bool, goals_resolved: bool,
              interrupted: bool) -> int:
    """Return the process exit code.

    A failed cleanup or a goal whose end is unknown takes precedence, since the robot
    or the commander may still be in a state the pick left; then an interrupt; then
    the result.
    """
    if not cleanup_ok or not goals_resolved:
        return EXIT_UNSAFE
    if interrupted:
        return EXIT_INTERRUPTED
    return RESULT_EXIT_CODES.get(result, EXIT_FAILED)
