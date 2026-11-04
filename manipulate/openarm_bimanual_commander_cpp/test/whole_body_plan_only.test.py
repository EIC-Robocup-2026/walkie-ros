"""A stored whole-body plan runs only while the base is where it was planned from.

Needs a running Zenoh router, as every launch test on this workspace does. Exit codes
are not asserted: move_group and the commander abort in libzenohc during shutdown.
plan_only is read once at startup, so this needs its own bring-up.
"""
import os
import sys
import unittest

from launch import LaunchDescription
import launch_testing
import pytest
import rclpy
from std_srvs.srv import Trigger

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import whole_body_harness as harness  # noqa: E402


@pytest.mark.launch_test
def generate_test_description():
    return LaunchDescription(
        harness.bringup('whole_body', 'whole_body', plan_only=True)
        + [launch_testing.actions.ReadyToTest()])


class TestWholeBodyPlanOnly(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.client = harness.CommanderClient('whole_body_plan_only_test')
        cls.client.wait_ready()

    @classmethod
    def tearDownClass(cls):
        cls.client.close()
        rclpy.try_shutdown()

    def test_stored_plan_refuses_a_moved_base_and_runs_after_it_returns(self):
        service = self.client.node.create_client(Trigger, '/execute_stored_plan')
        self.assertTrue(service.wait_for_service(timeout_sec=30.0))
        self.client.set_parameter('execute_plan_group', 'whole_body_left')

        planned = self.client.pose('whole_body_left', harness.FAR_TARGET)
        self.assertTrue(planned.success, planned.status)
        self.assertIn('Plan Only', planned.status)
        self.assertEqual(self.client.goals(), [])

        self.client.command(base=[0.2, 0.0, 0.0])
        self.client.settle()
        stale = self.client.wait(service.call_async(Trigger.Request()), 30.0)
        self.assertFalse(stale.success)
        self.assertIn('plan again', stale.message)
        self.assertIn('plan still stored', stale.message)
        self.assertEqual(self.client.goals(), [])

        self.client.command(base=[0.0, 0.0, 0.0])
        self.client.settle()
        executed = self.client.wait(service.call_async(Trigger.Request()), 120.0)
        self.assertTrue(executed.success, executed.message)
        self.assertEqual(len(self.client.base_goals()), 1)

        consumed = self.client.wait(service.call_async(Trigger.Request()), 30.0)
        self.assertFalse(consumed.success)
        self.assertIn('No stored plan', consumed.message)
