"""The goal handling of the RViz hand target (whole_body_marker) against a fake commander.

The fake commander is a go_to_pose_quat action server in its own process, so a test can
make it slow to accept a goal, kill it in the middle of one, or run two of them. Its goals
run until they are canceled. Plays the part of RViz with harness.MarkerDriver. The marker runs
on simulation time and /clock stands still at 1000 s, as in a paused simulation, so its
lost-goal check must run on steady time. The clock has to start at all: until it does,
the interactive marker server ignores every client. Needs a running Zenoh router.
"""
import os
import subprocess
import sys
import threading
import time
import unittest

from action_msgs.msg import GoalStatusArray
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import launch_testing.asserts
from my_robot_interfaces.action import GoToPoseQuaternion
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rosgraph_msgs.msg import Clock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import whole_body_harness as harness  # noqa: E402

STATUS_TOPIC = '/go_to_pose_quat/_action/status'

# argv[1]: seconds the fake waits before it accepts a goal.
FAKE_COMMANDER = """
import sys
import time

from my_robot_interfaces.action import GoToPoseQuaternion
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor

accept_delay = float(sys.argv[1])


def on_goal(goal):
    time.sleep(accept_delay)
    return GoalResponse.ACCEPT


def execute(handle):
    while not handle.is_cancel_requested:
        time.sleep(0.05)
    handle.canceled()
    return GoToPoseQuaternion.Result(success=False, status='canceled')


rclpy.init()
node = rclpy.create_node('fake_commander')
server = ActionServer(
    node, GoToPoseQuaternion, 'go_to_pose_quat', execute, goal_callback=on_goal,
    cancel_callback=lambda request: CancelResponse.ACCEPT)
executor = MultiThreadedExecutor()
executor.add_node(node)
executor.spin()
"""


@pytest.mark.launch_test
def generate_test_description():
    marker = Node(
        package='openarm_bimanual_commander_cpp', executable='whole_body_marker', output='screen',
        parameters=[{'use_sim_time': True}])
    return LaunchDescription([marker, launch_testing.actions.ReadyToTest()]), {'marker': marker}


@launch_testing.post_shutdown_test()
class TestWholeBodyMarkerShutdown(unittest.TestCase):

    def test_marker_exits_cleanly(self, proc_info, marker):
        # Launch stops it with SIGINT, as Ctrl-C does.
        launch_testing.asserts.assertExitCodes(proc_info, process=marker)


class TestWholeBodyMarkerClient(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('whole_body_marker_client_test')
        cls.commander = ActionClient(cls.node, GoToPoseQuaternion, 'go_to_pose_quat')
        clock = cls.node.create_publisher(Clock, '/clock', 10)
        paused = Clock()
        paused.clock.sec = 1000
        cls.node.create_timer(0.1, lambda: clock.publish(paused))
        cls.executor = MultiThreadedExecutor(num_threads=2)
        cls.executor.add_node(cls.node)
        threading.Thread(target=cls.executor.spin, daemon=True).start()
        cls.rviz = harness.MarkerDriver(cls.node, '/whole_body_marker_client_test')

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.try_shutdown()

    def start_commander(self, accept_delay):
        process = subprocess.Popen([sys.executable, '-c', FAKE_COMMANDER, str(accept_delay)])
        self.addCleanup(self.stop_commander, process)
        self.assertTrue(self.commander.wait_for_server(timeout_sec=30.0), 'The fake commander did not start')
        # The marker discovers the server on its own.
        time.sleep(1.0)
        return process

    def stop_commander(self, process):
        process.kill()
        process.wait()
        deadline = time.monotonic() + 15.0
        while self.commander.server_is_ready() and time.monotonic() < deadline:
            time.sleep(0.1)

    def wait_for_status_publishers(self, count):
        deadline = time.monotonic() + 15.0
        while len(self.node.get_publishers_info_by_topic(STATUS_TOPIC)) != count:
            self.assertLess(time.monotonic(), deadline, f'{STATUS_TOPIC} never had {count} publishers')
            time.sleep(0.1)
        # The marker's view of the graph follows the test's.
        time.sleep(0.5)

    def cancel_and_expect_canceled(self):
        self.rviz.pick('Cancel goal')
        status = self.rviz.wait_for_status('whole_body_left: failed', 20.0)
        self.assertEqual(status, 'whole_body_left: failed: canceled')

    def test_cancel_before_the_commander_accepts(self):
        self.start_commander(accept_delay=2.0)
        self.rviz.pick('Reach with left arm')
        self.rviz.wait_for_status('whole_body_left: sent', 10.0)
        self.rviz.pick('Cancel goal')
        self.rviz.wait_for_status('Cancel requested, waiting', 10.0)
        # Without the pending cancel, the goal would run until the timeout.
        status = self.rviz.wait_for_status('whole_body_left: failed', 20.0)
        self.assertEqual(status, 'whole_body_left: failed: canceled')

    def test_commander_lost_in_the_middle_of_a_goal(self):
        process = self.start_commander(accept_delay=0.0)
        self.rviz.pick('Reach with left arm')
        self.rviz.wait_for_status('whole_body_left: accepted, running', 10.0)
        process.kill()
        status = self.rviz.wait_for_status("The commander's go_to_pose_quat action went away", 20.0)
        self.assertTrue(status.endswith('outcome unknown'), status)
        # The marker takes a new goal instead of waiting for the lost one.
        self.rviz.pick('Reach with left arm')
        self.rviz.wait_for_status("The commander's go_to_pose_quat action is not available", 10.0)

    def test_commander_restarted_in_the_middle_of_a_goal(self):
        first = self.start_commander(accept_delay=0.0)
        self.rviz.pick('Reach with left arm')
        self.rviz.wait_for_status('whole_body_left: accepted, running', 10.0)
        # The new commander serves the action before the old one exits, so a check that
        # only asks whether the action is served never sees it go away. The new one does
        # not know the goal.
        self.start_commander(accept_delay=0.0)
        self.wait_for_status_publishers(2)
        first.kill()
        first.wait()
        status = self.rviz.wait_for_status("The commander's go_to_pose_quat action went away", 20.0)
        self.assertTrue(status.endswith('outcome unknown'), status)
        # The new commander takes the next goal, and the marker keeps that goal past the
        # 3 s after which it would count a lost one.
        self.rviz.pick('Reach with left arm')
        self.rviz.wait_for_status('whole_body_left: accepted, running', 10.0)
        time.sleep(5.0)
        self.assertEqual(self.rviz.marker().description, 'whole_body_left: accepted, running')
        self.cancel_and_expect_canceled()

    def test_two_status_publishers_refuse_a_goal(self):
        # With two, such as two commanders, the marker could not tell which one has the
        # goal, so it could not notice that one goes away.
        self.start_commander(accept_delay=0.0)
        extra = self.node.create_publisher(GoalStatusArray, STATUS_TOPIC, 10)
        try:
            self.wait_for_status_publishers(2)
            self.rviz.pick('Reach with left arm')
            self.rviz.wait_for_status('Two or more commanders serve go_to_pose_quat', 10.0)
        finally:
            self.node.destroy_publisher(extra)
        self.wait_for_status_publishers(1)
        self.rviz.pick('Reach with left arm')
        self.rviz.wait_for_status('whole_body_left: accepted, running', 10.0)
        self.cancel_and_expect_canceled()
