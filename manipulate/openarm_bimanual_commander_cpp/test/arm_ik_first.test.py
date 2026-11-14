"""IK first for single-arm pose goals, and the planning-time parameters that go with it.

The commander's /compute_ik is remapped to a service of this test (IkService), which passes
requests on to move_group, fails them, or never answers them. That makes the fallback, an
unanswered call and a cancel during IK happen on purpose instead of by chance.

Needs a running Zenoh router, as every launch test on this workspace does. Exit codes
are not asserted: move_group and the commander abort in libzenohc during shutdown.
"""
import os
import sys
import threading
import time
import unittest

from action_msgs.msg import GoalStatus
from launch import LaunchDescription
import launch_testing
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
import pytest
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import whole_body_harness as harness  # noqa: E402

STUB_SERVICE = '/arm_ik_first_test/compute_ik'
PLANNED_BY_IK = '[left_arm_lift] Planned to an IK solution'
FELL_BACK = '[left_arm_lift] IK first:'
DEFAULT_JOINT_GOAL_PLANNING_TIME = 1.0
# The commander logs this at startup, which tells its output from that of the others.
COMMANDER_STARTED = b'Arm groups using planner'
# Setting planner_id to the value it already has changes nothing, and the commander logs
# MARKER on the same stream as its goals, after all it logged before.
MARKER_VALUE = 'RRTConnect'
MARKER = b"planner_id -> 'RRTConnect' (live)"
# The commander logs this right before a named goal to left_arm_lift waits for the group's
# planning lock.
NAMED_GOAL_STARTED = b'Executing GoToHome (home) for left_arm_lift'


def commander_log(proc_output):
    """The commander's stderr so far, joined in order.

    RCLCPP logs to stderr, and launch hands the output over in chunks that can split a line.
    """
    streams = {}
    for event in proc_output:
        if event.from_stderr:
            streams.setdefault(event.pid, []).append(event.text)
    for chunks in streams.values():
        text = b''.join(chunks)
        if COMMANDER_STARTED in text:
            return text
    return b''


class IkService:
    """Answers the commander's remapped /compute_ik.

    'pass' sends a request on to move_group's /compute_ik and returns its answer, 'fail'
    answers NO_IK_SOLUTION at once, and 'hang' holds a request until the mode changes again,
    long after the commander has stopped waiting for it.
    """

    def __init__(self):
        self.node = rclpy.create_node('arm_ik_first_stub')
        group = ReentrantCallbackGroup()
        self.upstream = self.node.create_client(GetPositionIK, '/compute_ik', callback_group=group)
        self.node.create_service(GetPositionIK, STUB_SERVICE, self.on_request, callback_group=group)
        self.lock = threading.Lock()
        self.mode = 'pass'
        self.requests = 0
        self.release = threading.Event()
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        threading.Thread(target=self.executor.spin, daemon=True).start()

    def close(self):
        self.set_mode('pass')
        self.executor.shutdown()
        self.node.destroy_node()

    def set_mode(self, mode):
        with self.lock:
            self.mode = mode
            # Lets the held requests answer; the next ones wait for a new event.
            self.release.set()
            self.release = threading.Event()

    def request_count(self):
        with self.lock:
            return self.requests

    def wait_for_requests(self, count, timeout=30.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.request_count() >= count:
                return True
            time.sleep(0.01)
        return False

    def on_request(self, request, response):
        with self.lock:
            self.requests += 1
            mode = self.mode
            release = self.release
        if mode == 'hang':
            release.wait()
        elif mode == 'pass' and self.upstream.wait_for_service(timeout_sec=5.0):
            future = self.upstream.call_async(request)
            answered = threading.Event()
            future.add_done_callback(lambda _: answered.set())
            if answered.wait(10.0):
                return future.result()
        response.error_code.val = MoveItErrorCodes.NO_IK_SOLUTION
        return response


@pytest.mark.launch_test
def generate_test_description():
    return LaunchDescription(
        harness.bringup('fixed', 'fixed', commander_remappings=[('/compute_ik', STUB_SERVICE)])
        + [launch_testing.actions.ReadyToTest()])


class TestArmIkFirst(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        # Before the commander starts, so its remapped client finds the service.
        cls.ik = IkService()
        cls.client = harness.CommanderClient('arm_ik_first_test')
        cls.client.wait_ready()

    @classmethod
    def tearDownClass(cls):
        cls.client.close()
        cls.ik.close()
        rclpy.try_shutdown()

    def setUp(self):
        self.client.reset_world()
        self.ik.set_mode('pass')
        self.addCleanup(self.ik.set_mode, 'pass')
        self.addCleanup(self.client.set_parameter, 'ik_first_enable', True)
        self.addCleanup(
            self.client.set_parameter, 'joint_goal_planning_time', DEFAULT_JOINT_GOAL_PLANNING_TIME)

    def tuck_right_arm(self):
        # With the right arm in home, the four IK calls almost never find a solution with
        # clearance for the near target, and the goal takes the pose-goal fallback. See
        # "Planning time and attempts" in docs/guides/manipulation.md.
        self.assertTrue(self.client.named('right_arm', 'standby').success)
        self.client.settle()

    def near_arm_goal(self):
        result = self.client.pose(
            'left_arm_lift', harness.NEAR_ARM_TARGET, frame='base_footprint')
        self.assertTrue(result.success, result.status)
        self.assertEqual(self.client.base_goals(), [])

    def wait_for_log(self, proc_output, needle, start, timeout=30.0):
        """Wait until the commander logs `needle` after offset `start`, and return its offset."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            found = commander_log(proc_output).find(needle, start)
            if found >= 0:
                return found
            time.sleep(0.05)
        self.fail(f'The commander did not log {needle!r}')

    def logged_during(self, proc_output, action):
        """Run `action` and return what the commander logged meanwhile."""
        start = len(commander_log(proc_output))
        action()
        self.client.set_parameter('planner_id', MARKER_VALUE)
        end = self.wait_for_log(proc_output, MARKER, start)
        return commander_log(proc_output)[start:end].decode(errors='replace')

    def end_goal(self, handle):
        """Cancel a goal and wait for its result. A goal that has ended stays as it was.

        Registered as a cleanup, so a failed check cannot leave a goal running into the
        next test.
        """
        self.client.wait(handle.cancel_goal_async(), 10.0)
        return self.client.wait(handle.get_result_async(), 30.0)

    def test_a_pose_goal_plans_to_an_ik_solution(self, proc_output):
        self.tuck_right_arm()
        before = self.ik.request_count()
        log = self.logged_during(proc_output, self.near_arm_goal)
        self.assertIn(PLANNED_BY_IK, log)
        self.assertNotIn(FELL_BACK, log)
        self.assertGreater(self.ik.request_count(), before)
        self.assertIn(harness.LEFT_ARM, self.client.controllers())

    def test_with_ik_first_off_a_pose_goal_plans_directly(self, proc_output):
        self.tuck_right_arm()
        self.client.set_parameter('ik_first_enable', False)
        before = self.ik.request_count()
        log = self.logged_during(proc_output, self.near_arm_goal)
        # The commander waits for every IK answer before its goal ends, so a call would
        # already be counted here.
        self.assertEqual(self.ik.request_count(), before)
        self.assertNotIn(PLANNED_BY_IK, log)
        self.assertNotIn(FELL_BACK, log)

    def test_without_an_ik_solution_the_pose_goal_is_planned(self, proc_output):
        self.tuck_right_arm()
        self.ik.set_mode('fail')
        before = self.ik.request_count()
        log = self.logged_during(proc_output, self.near_arm_goal)
        self.assertEqual(self.ik.request_count() - before, 4)
        self.assertIn(FELL_BACK + ' 0 of 4 IK calls solved', log)
        self.assertNotIn(PLANNED_BY_IK, log)

    def test_an_unanswered_ik_call_falls_back_to_the_pose_goal(self, proc_output):
        self.tuck_right_arm()
        self.ik.set_mode('hang')
        before = self.ik.request_count()
        log = self.logged_during(proc_output, self.near_arm_goal)
        # The commander stops at the first call without an answer instead of trying the
        # other seeds.
        self.assertEqual(self.ik.request_count() - before, 1)
        self.assertIn(FELL_BACK + ' no answer from /compute_ik in time', log)

    def test_a_cancel_during_ik_ends_the_goal_without_motion(self):
        self.tuck_right_arm()
        self.client.clear_events()
        self.ik.set_mode('hang')
        before = self.ik.request_count()
        handle = self.client.wait(
            self.client.send_pose('left_arm_lift', harness.NEAR_ARM_TARGET, frame='base_footprint'),
            30.0)
        self.assertTrue(handle.accepted)
        self.addCleanup(self.end_goal, handle)
        self.assertTrue(self.ik.wait_for_requests(before + 1), 'The goal made no IK call')
        canceled_at = time.monotonic()
        self.client.wait(handle.cancel_goal_async(), 10.0)
        response = self.client.wait(handle.get_result_async(), 10.0)
        waited = time.monotonic() - canceled_at
        self.assertEqual(response.status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(response.result.status, 'Canceled')
        # Without the cancel, the commander would first wait out the 2 s it gives IK.
        self.assertLess(waited, 1.0)
        self.assertEqual(self.ik.request_count() - before, 1)
        self.client.settle()
        self.assertNotIn(harness.LEFT_ARM, self.client.controllers())

    def test_a_cancel_while_waiting_for_the_group_ends_the_goal(self, proc_output):
        self.tuck_right_arm()
        self.client.clear_events()
        self.ik.set_mode('hang')
        before = self.ik.request_count()
        # The first goal holds the planning lock of left_arm_lift: up to 2 s while its IK
        # call hangs, then 5 s for its pose-goal fallback.
        first = self.client.wait(
            self.client.send_pose('left_arm_lift', harness.NEAR_ARM_TARGET, frame='base_footprint'),
            30.0)
        self.assertTrue(first.accepted)
        self.addCleanup(self.end_goal, first)
        self.assertTrue(self.ik.wait_for_requests(before + 1), 'The first goal made no IK call')
        # A named goal on the same group then waits for that lock.
        start = len(commander_log(proc_output))
        second = self.client.wait(self.client.send_named('left_arm_lift', 'home'), 30.0)
        self.assertTrue(second.accepted)
        self.addCleanup(self.end_goal, second)
        self.wait_for_log(proc_output, NAMED_GOAL_STARTED, start)
        canceled_at = time.monotonic()
        self.client.wait(second.cancel_goal_async(), 10.0)
        response = self.client.wait(second.get_result_async(), 10.0)
        waited = time.monotonic() - canceled_at
        self.assertEqual(response.status, GoalStatus.STATUS_CANCELED)
        # The wait checks for a cancel every 20 ms; the rest is the round trip. With a wait
        # that ignored the cancel, this took 7.7 s: the first goal's IK and its fallback.
        self.assertLess(waited, 1.0)
        # Canceled during its IK calls or, on a slow machine, after its fallback plan, the
        # first goal ends canceled either way and sends no second IK call.
        response = self.end_goal(first)
        self.assertEqual(response.status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(self.ik.request_count() - before, 1)
        self.client.settle()
        self.assertNotIn(harness.LEFT_ARM, self.client.controllers())

    def test_a_planning_time_at_or_below_zero_is_rejected(self, proc_output):
        for value in (0.0, -1.0):
            with self.subTest(value=value):
                result = self.client.try_set_parameter('joint_goal_planning_time', value)
                self.assertFalse(result.successful)
                self.assertIn('joint_goal_planning_time', result.reason)
        # The default still holds: with no time to plan, the goal would take the fallback.
        self.tuck_right_arm()
        log = self.logged_during(proc_output, self.near_arm_goal)
        self.assertIn(PLANNED_BY_IK, log)

    def test_goals_succeed_when_the_joint_goal_planning_time_is_too_short(self):
        # A smoke test: whether a 0.01 s plan finds a path is up to chance. Either way, the
        # named state gets a second plan with arm_planning_time and the pose goal gets its
        # fallback, so both goals succeed.
        self.client.set_parameter('joint_goal_planning_time', 0.01)
        self.tuck_right_arm()
        self.near_arm_goal()
        self.client.settle()
        self.assertTrue(self.client.named('left_arm_lift', 'home').success)
