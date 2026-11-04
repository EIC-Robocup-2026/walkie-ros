"""The whole-body gate on simulation time, with a clock that runs slowly and in bursts.

A loaded machine ran MuJoCo at 0.16 of real time, and the gate refused goals because /clock
did not change during its 250 ms sleep. Needs a running Zenoh router, as every launch test on
this workspace does. Exit codes are not asserted: move_group and the commander abort in
libzenohc during shutdown. use_sim_time needs its own bring-up, and plan_only keeps the
stub controllers, which run on wall time, out of it.
"""
import os
import sys
import threading
import time
import unittest

from builtin_interfaces.msg import Time
from launch import LaunchDescription
import launch_testing
import pytest
import rclpy
from rosgraph_msgs.msg import Clock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import whole_body_harness as harness  # noqa: E402

# One step per wall period: 0.1 of real time, with stalls four times as long as the old sleep.
CLOCK_PERIOD = 1.0
CLOCK_STEP = 0.1


@pytest.mark.launch_test
def generate_test_description():
    return LaunchDescription(
        harness.bringup('whole_body', 'whole_body', plan_only=True, commander_sim_time=True)
        + [launch_testing.actions.ReadyToTest()])


class TestWholeBodyClock(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.client = harness.CommanderClient('whole_body_clock_test')
        cls.publisher = cls.client.node.create_publisher(Clock, '/clock', 10)
        cls.running = threading.Event()
        cls.running.set()
        cls.done = threading.Event()
        cls.thread = threading.Thread(target=cls.publish_clock, daemon=True)
        cls.thread.start()
        cls.client.wait_ready()

    @classmethod
    def tearDownClass(cls):
        cls.done.set()
        cls.thread.join()
        cls.client.close()
        rclpy.try_shutdown()

    @classmethod
    def publish_clock(cls):
        now = 1.0
        while not cls.done.wait(CLOCK_PERIOD):
            if cls.running.is_set():
                now += CLOCK_STEP
                seconds = int(now)
                stamp = Time(sec=seconds, nanosec=int((now - seconds) * 1e9))
                cls.publisher.publish(Clock(clock=stamp))

    def test_goal_passes_the_gate_on_a_slow_clock(self):
        # A short sleep can still catch a clock step, so one goal would not show the bug.
        for attempt in range(3):
            with self.subTest(attempt=attempt):
                result = self.client.pose('whole_body_left', harness.FAR_TARGET)
                self.assertNotIn('clock', result.status)
                self.assertTrue(result.success, result.status)
                self.assertIn('Plan Only', result.status)

    def test_goal_is_refused_when_the_clock_stops(self):
        self.running.clear()
        # The clock thread may be in the middle of its last step.
        time.sleep(2 * CLOCK_PERIOD)
        try:
            result = self.client.pose('whole_body_left', harness.FAR_TARGET)
        finally:
            self.running.set()
        self.assertFalse(result.success)
        self.assertIn('clock is not advancing', result.status)
