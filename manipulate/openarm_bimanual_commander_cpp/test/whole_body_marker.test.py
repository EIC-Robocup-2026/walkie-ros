"""The RViz hand target (whole_body_marker) against the real commander and stub controllers.

Plays the part of RViz with harness.MarkerDriver. Needs a running Zenoh router.
"""
import math
import os
import sys
import unittest

from geometry_msgs.msg import Pose
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import pytest
import rclpy
from visualization_msgs.msg import InteractiveMarkerFeedback

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import whole_body_harness as harness  # noqa: E402

# The marker's position before it finds the left hand (DEFAULT_POSITION in the node).
DEFAULT_POSITION = (0.45, 0.25, 1.0)


@pytest.mark.launch_test
def generate_test_description():
    marker = Node(package='openarm_bimanual_commander_cpp', executable='whole_body_marker', output='screen')
    return LaunchDescription(
        harness.bringup('whole_body', 'whole_body') + [marker, launch_testing.actions.ReadyToTest()])


class TestWholeBodyMarker(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.client = harness.CommanderClient('whole_body_marker_test')
        cls.client.wait_ready()
        cls.rviz = harness.MarkerDriver(cls.client.node, '/whole_body_marker_test')

    @classmethod
    def tearDownClass(cls):
        cls.client.close()
        rclpy.try_shutdown()

    def setUp(self):
        self.client.reset_world()

    def wait_for_first_snap(self):
        return self.rviz.wait_for_marker(lambda m: abs(m.pose.position.x - DEFAULT_POSITION[0]) > 1e-6, 30.0)

    def test_marker_starts_on_the_left_hand(self):
        marker = self.wait_for_first_snap()
        self.assertGreater(marker.pose.position.y, 0.0)

    def test_snap_to_right_hand(self):
        self.rviz.pick('Snap to right hand')
        marker = self.rviz.wait_for_marker(lambda m: m.pose.position.y < 0.0, 10.0)
        self.assertLess(marker.pose.position.y, 0.0)

    def test_reach_with_left_arm_sends_a_whole_body_goal(self):
        # The first snap to the hand would otherwise move the target back.
        self.wait_for_first_snap()
        target = Pose()
        target.position.x, target.position.y, target.position.z = harness.FAR_TARGET
        # The gripper pointing forward, as in the other whole-body tests.
        target.orientation.y = math.sin(harness.FORWARD_PITCH / 2)
        target.orientation.w = math.cos(harness.FORWARD_PITCH / 2)
        self.rviz.send(InteractiveMarkerFeedback.POSE_UPDATE, pose=target)
        self.rviz.wait_for_marker(lambda m: abs(m.pose.position.x - harness.FAR_TARGET[0]) < 1e-6, 10.0)
        self.rviz.pick('Reach with left arm')
        status = self.rviz.wait_for_status(('whole_body_left: done', 'whole_body_left: failed'), 180.0)
        self.assertTrue(status.startswith('whole_body_left: done'), status)
        self.assertEqual(len(self.client.base_goals()), 1)
        self.assertIn(harness.LEFT_ARM, self.client.controllers())
