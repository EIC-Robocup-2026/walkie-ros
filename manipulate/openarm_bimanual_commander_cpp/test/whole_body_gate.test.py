"""Bring-ups where a whole-body goal has to be refused, and arm goals keep working.

Needs a running Zenoh router, as every launch test on this workspace does. Exit codes
are not asserted: move_group and the commander abort in libzenohc during shutdown.
"""
import os
import sys
import unittest

from launch import LaunchDescription
import launch_testing
import pytest
import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import whole_body_harness as harness  # noqa: E402

# move_group's base_mode, the commander's base_mode, whether a base server runs, the
# reason a whole-body goal is refused with, and whether arm goals have to work. Arm goals
# are not promised when the two models disagree.
BRINGUPS = [
    ('whole_body', 'whole_body', False, 'No base server', True),
    ('map', 'map', True, 'this bring-up runs base_mode:=map', True),
    ('fixed', 'fixed', True, 'this bring-up runs base_mode:=fixed', True),
    ('fixed', 'whole_body', True, "move_group's robot model has no base joint", False),
    ('whole_body', 'fixed', True, 'this bring-up runs base_mode:=fixed', False),
]


@pytest.mark.launch_test
@launch_testing.parametrize(
    'move_group_mode,commander_mode,with_base_server,reason,arm_goals_work', BRINGUPS)
def generate_test_description(
        move_group_mode, commander_mode, with_base_server, reason, arm_goals_work):
    return LaunchDescription(
        harness.bringup(move_group_mode, commander_mode, with_base_server)
        + [launch_testing.actions.ReadyToTest()])


class TestWholeBodyGate(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.client = harness.CommanderClient('whole_body_gate_test')
        cls.client.wait_ready()

    @classmethod
    def tearDownClass(cls):
        cls.client.close()
        rclpy.try_shutdown()

    def setUp(self):
        self.client.reset_world()

    def test_whole_body_goal_is_refused_with_a_reason(self, reason):
        for _ in range(2):
            # The second goal shows that the commander survived the first refusal.
            result = self.client.pose('whole_body_left', harness.FAR_TARGET)
            self.assertFalse(result.success)
            self.assertIn(reason, result.status)
            self.assertEqual(self.client.goals(), [])

    def test_arm_groups_still_work_and_never_command_the_base(self, arm_goals_work):
        if not arm_goals_work:
            self.skipTest('The models disagree in this bring-up')
        for group, controllers in harness.ARM_GROUPS.items():
            with self.subTest(group=group):
                self.client.clear_events()
                self.assertTrue(self.client.named(group, 'standby').success)
                self.client.settle()
                self.assertTrue(self.client.named(group, 'home').success)
                self.client.settle()
                self.assertLessEqual(controllers, self.client.controllers())
                self.assertEqual(self.client.base_goals(), [])
