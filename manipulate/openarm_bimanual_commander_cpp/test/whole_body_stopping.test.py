"""Every way a whole-body motion has to stop, against stub controllers.

The stub world freezes its controllers, so a goal stays running until something ends it.
Needs a running Zenoh router, as every launch test on this workspace does. Exit codes are
not asserted: move_group and the commander abort in libzenohc during shutdown.
"""
import os
import sys
import threading
import time
import unittest

from action_msgs.msg import GoalStatus, GoalStatusArray
from control_msgs.action import FollowJointTrajectory
from launch import LaunchDescription
import launch_testing
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import whole_body_harness as harness  # noqa: E402

# Well below the deadline of a far goal, which is at least its whole-body margin of 15 s.
PROMPT_STOP = 10.0
# Covers the gate, the base sampling, the plan and the start of execution.
CANCEL_DELAYS = (0.3, 0.8, 1.3, 1.8, 2.3, 2.8, 3.3, 3.8, 4.3, 4.8, 5.3, 5.8)
# Seconds after a goal is accepted. Covers the first gate, planning and the second gate.
ARRIVAL_DELAYS = (0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0)


@pytest.mark.launch_test
def generate_test_description():
    return LaunchDescription(
        harness.bringup('whole_body', 'whole_body') + [launch_testing.actions.ReadyToTest()])


class TestWholeBodyStopping(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.client = harness.CommanderClient('whole_body_stopping_test')
        cls.client.wait_ready()
        latched = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cls.nav2_status = cls.client.node.create_publisher(
            GoalStatusArray, '/navigate_to_pose/_action/status', latched)
        # The commander's status subscription can take a moment to match.
        time.sleep(2.0)

    @classmethod
    def tearDownClass(cls):
        cls.client.close()
        rclpy.try_shutdown()

    def setUp(self):
        self.set_nav2_active(False)
        self.client.reset_world()

    def tearDown(self):
        self.set_nav2_active(False)
        self.client.command(freeze=False)
        self.client.settle()

    def set_nav2_active(self, active):
        statuses = [GoalStatus(status=GoalStatus.STATUS_EXECUTING)] if active else []
        self.nav2_status.publish(GoalStatusArray(status_list=statuses))

    def start_frozen_far_goal(self):
        """Return the goal handle of a far goal whose controllers hold until told otherwise."""
        self.client.command(freeze=True)
        handle = self.client.wait(
            self.client.send_pose('whole_body_left', harness.FAR_TARGET), 30.0)
        self.assertTrue(handle.accepted)
        self.assertTrue(self.client.wait_for_goal(), 'No goal reached the stub controllers')
        # The execution manager sends the controllers their goals one after another.
        time.sleep(0.5)
        self.assertIn(harness.BASE_CONTROLLER, self.client.controllers())
        self.assertIn(harness.LEFT_ARM, self.client.controllers())
        return handle

    def assert_nothing_left_running(self):
        # The execution manager cancels its controllers one after another, and the stub
        # notices a cancel on its next tick.
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and \
                not self.client.controllers() <= self.client.canceled():
            time.sleep(0.05)
        self.assertLessEqual(self.client.controllers(), self.client.canceled())

    def test_cancel_stops_every_controller(self):
        handle = self.start_frozen_far_goal()
        self.client.wait(handle.cancel_goal_async(), 30.0)
        outcome = self.client.wait(handle.get_result_async(), PROMPT_STOP)
        self.assertEqual(outcome.status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(outcome.result.status, 'Canceled')
        self.assert_nothing_left_running()

    def test_cancel_at_any_moment_leaves_nothing_running(self):
        for delay in CANCEL_DELAYS:
            with self.subTest(delay=delay):
                self.client.reset_world()
                self.client.command(freeze=True)
                handle = self.client.wait(
                    self.client.send_pose('whole_body_left', harness.FAR_TARGET), 30.0)
                self.assertTrue(handle.accepted)
                time.sleep(delay)
                self.client.wait(handle.cancel_goal_async(), 30.0)
                outcome = self.client.wait(handle.get_result_async(), 60.0)
                self.assertEqual(outcome.status, GoalStatus.STATUS_CANCELED, outcome.result.status)
                # A request that was on its way to move_group shows up a little later.
                time.sleep(3.0)
                self.assert_nothing_left_running()

    def test_expired_deadline_stops_every_controller(self):
        self.client.set_parameter('whole_body_execution_margin', 1.0)
        try:
            self.client.command(freeze=True)
            result = self.client.pose('whole_body_left', harness.FAR_TARGET)
        finally:
            self.client.set_parameter('whole_body_execution_margin', 15.0)
        self.assertFalse(result.success)
        self.assertIn('Execution deadline', result.status)
        self.assertIn(harness.BASE_CONTROLLER, self.client.controllers())
        self.assert_nothing_left_running()

    def test_arm_abort_stops_the_base_at_once(self):
        handle = self.start_frozen_far_goal()
        self.client.command(abort=harness.LEFT_ARM)
        outcome = self.client.wait(handle.get_result_async(), PROMPT_STOP)
        self.assertFalse(outcome.result.success)
        self.assertIn(harness.LEFT_ARM, outcome.result.status)
        self.assertIn(harness.BASE_CONTROLLER, self.client.canceled())

    def test_base_abort_stops_the_arm_at_once(self):
        handle = self.start_frozen_far_goal()
        self.client.command(abort=harness.BASE_CONTROLLER)
        outcome = self.client.wait(handle.get_result_async(), PROMPT_STOP)
        self.assertFalse(outcome.result.success)
        self.assertIn(harness.BASE_CONTROLLER, outcome.result.status)
        self.assertIn(harness.LEFT_ARM, self.client.canceled())

    def test_an_old_abort_does_not_stop_the_next_goal(self):
        handle = self.start_frozen_far_goal()
        self.client.command(abort=harness.LEFT_ARM)
        self.client.wait(handle.get_result_async(), PROMPT_STOP)
        self.client.reset_world()
        result = self.client.pose('whole_body_left', harness.FAR_TARGET)
        self.assertTrue(result.success, result.status)

    def test_nav2_goal_during_execution_stops_the_motion(self):
        handle = self.start_frozen_far_goal()
        self.set_nav2_active(True)
        outcome = self.client.wait(handle.get_result_async(), PROMPT_STOP)
        self.assertFalse(outcome.result.success)
        self.assertIn('Nav2', outcome.result.status)
        self.assertIn(harness.BASE_CONTROLLER, self.client.canceled())

    def test_nav2_goal_at_any_moment_ends_the_motion(self):
        for delay in ARRIVAL_DELAYS:
            with self.subTest(delay=delay):
                self.set_nav2_active(False)
                self.client.reset_world()
                self.client.command(freeze=True)
                handle = self.client.wait(
                    self.client.send_pose('whole_body_left', harness.FAR_TARGET), 30.0)
                self.assertTrue(handle.accepted)
                time.sleep(delay)
                self.set_nav2_active(True)
                outcome = self.client.wait(handle.get_result_async(), 60.0)
                self.assertFalse(outcome.result.success)
                self.assertIn('Nav2', outcome.result.status)
                self.assert_nothing_left_running()

    def test_base_moved_during_planning_refuses_the_motion(self):
        # In the stub world a plan takes well under a second, and nothing marks the moment
        # between the commander's first look at the base and its last, so this sweeps.
        statuses = []
        for delay in ARRIVAL_DELAYS:
            self.client.reset_world()
            handle = self.client.wait(
                self.client.send_pose('whole_body_left', harness.FAR_TARGET), 30.0)
            self.assertTrue(handle.accepted)
            time.sleep(delay)
            self.client.command(base=[0.2, 0.0, 0.0])
            outcome = self.client.wait(handle.get_result_async(), 60.0)
            statuses.append(outcome.result.status)
            if 'Base moved' in outcome.result.status:
                self.assertFalse(outcome.result.success)
                self.assertEqual(self.client.goals(), [])
                return
        self.fail(f'No delay landed between the two looks at the base: {statuses}')

    def test_abort_on_a_controller_outside_the_plan_does_not_stop_the_motion(self):
        handle = self.start_frozen_far_goal()
        # Another client's goal on the right arm, which a whole_body_left plan does not use.
        right_arm = ActionClient(
            self.client.node, FollowJointTrajectory,
            f'/{harness.RIGHT_ARM}/follow_joint_trajectory')
        try:
            self.assertTrue(right_arm.wait_for_server(timeout_sec=10.0))
            other = self.client.wait(right_arm.send_goal_async(FollowJointTrajectory.Goal()), 10.0)
            self.assertTrue(other.accepted)
            self.client.command(abort=harness.RIGHT_ARM)
            self.client.wait(other.get_result_async(), 10.0)
            result = handle.get_result_async()
            time.sleep(2.0)
            if result.done():
                self.fail(f'The motion stopped: {result.result().result.status}')
            self.client.command(freeze=False)
            outcome = self.client.wait(result, 60.0)
            self.assertTrue(outcome.result.success, outcome.result.status)
        finally:
            right_arm.destroy()

    def test_commander_reports_that_it_is_alive_and_executing(self):
        beats = []
        lock = threading.Lock()

        def on_beat(message):
            with lock:
                beats.append(message.data)

        subscription = self.client.node.create_subscription(
            Bool, '/whole_body/commander_alive', on_beat, 10)
        try:
            time.sleep(3.0)
            with lock:
                self.assertGreaterEqual(len(beats), 5)
                self.assertFalse(any(beats), 'The commander reported a motion while idle')
            handle = self.start_frozen_far_goal()
            time.sleep(1.0)
            with lock:
                self.assertTrue(beats[-1], 'The commander did not report its motion')
            self.client.wait(handle.cancel_goal_async(), 30.0)
            self.client.wait(handle.get_result_async(), PROMPT_STOP)
            time.sleep(1.0)
            with lock:
                self.assertFalse(beats[-1], 'The commander still reports a motion that ended')
        finally:
            self.client.node.destroy_subscription(subscription)

    def test_near_goal_after_an_arm_goal_keeps_the_base_still(self):
        # The sequence of the MuJoCo demo. In one run the arm goal left the arm in a state
        # where the whole-body goal after it moved the base by 3 cm.
        for attempt in range(8):
            with self.subTest(attempt=attempt):
                self.client.reset_world()
                arm = self.client.pose(
                    'left_arm_lift', harness.NEAR_ARM_TARGET, frame='base_footprint')
                self.assertTrue(arm.success, arm.status)
                self.client.settle()
                result = self.client.pose(
                    'whole_body_left', harness.NEAR_TARGET, frame='base_footprint')
                self.assertTrue(result.success, result.status)
                self.assertIn('base fixed', result.status)
                self.assertEqual(self.client.base_goals(), [])
