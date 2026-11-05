"""The whole-body option against stub controllers, on a whole_body bring-up.

Needs a running Zenoh router, as every launch test on this workspace does. Exit codes
are not asserted: move_group and the commander abort in libzenohc during shutdown.
"""
import math
import os
import sys
import time
import unittest

from action_msgs.msg import GoalStatus, GoalStatusArray
from launch import LaunchDescription
import launch_testing
from my_robot_interfaces.srv import GetEEPose
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import whole_body_harness as harness  # noqa: E402

# The QoS of an action status topic.
LATCHED = QoSProfile(
    depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

@pytest.mark.launch_test
def generate_test_description():
    return LaunchDescription(
        harness.bringup('whole_body', 'whole_body') + [launch_testing.actions.ReadyToTest()])


class TestWholeBodyOption(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.client = harness.CommanderClient('whole_body_option_test')
        cls.client.wait_ready()

    @classmethod
    def tearDownClass(cls):
        cls.client.close()
        rclpy.try_shutdown()

    def setUp(self):
        self.client.reset_world()

    def test_arm_groups_never_command_the_base(self):
        for group, controllers in harness.ARM_GROUPS.items():
            with self.subTest(group=group):
                self.client.clear_events()
                self.assertTrue(self.client.named(group, 'standby').success)
                self.client.settle()
                self.assertTrue(self.client.named(group, 'home').success)
                self.client.settle()
                self.assertLessEqual(controllers, self.client.controllers())
                self.assertEqual(self.client.base_goals(), [])

    def test_far_goal_on_an_arm_group_fails_without_motion(self):
        result = self.client.pose('left_arm_lift', harness.FAR_TARGET)
        self.assertFalse(result.success)
        self.assertEqual(self.client.goals(), [])

    def test_far_whole_body_goal_moves_the_base(self):
        cases = (
            ('whole_body_left', harness.FAR_TARGET, harness.LEFT_ARM),
            ('whole_body_right', harness.FAR_TARGET_RIGHT, harness.RIGHT_ARM),
        )
        for group, target, arm in cases:
            with self.subTest(group=group):
                self.client.reset_world()
                result = self.client.pose(group, target)
                self.assertTrue(result.success, result.status)
                self.assertIn('base travel', result.status)
                self.assertEqual(len(self.client.base_goals()), 1)
                self.assertIn(arm, self.client.controllers())

    def test_far_goal_keeps_the_base_near_the_straight_line(self):
        result = self.client.pose('whole_body_left', harness.FAR_TARGET)
        self.assertTrue(result.success, result.status)
        goals = self.client.base_goals()
        self.assertEqual(len(goals), 1)
        # The first planning box reaches 0.3 m past the start and the end of the move.
        end_x, end_y = goals[0]['base_end'][:2]
        min_x, min_y, max_x, max_y = goals[0]['base_bounds']
        margin = 0.31
        self.assertGreaterEqual(min_x, min(0.0, end_x) - margin)
        self.assertGreaterEqual(min_y, min(0.0, end_y) - margin)
        self.assertLessEqual(max_x, max(0.0, end_x) + margin)
        self.assertLessEqual(max_y, max(0.0, end_y) + margin)

    def test_near_whole_body_goal_keeps_the_base_still(self):
        result = self.client.pose('whole_body_left', harness.NEAR_TARGET, frame='base_footprint')
        self.assertTrue(result.success, result.status)
        self.assertIn('base fixed', result.status)
        self.assertIn(harness.LEFT_ARM, self.client.controllers())
        self.assertEqual(self.client.base_goals(), [])

    def test_no_goal_spins_the_base_near_the_heading_seam(self):
        for heading in (-3.0, -1.5, 1.5, 3.0):
            with self.subTest(heading=heading):
                self.client.reset_world()
                self.client.command(base=[0.0, 0.0, heading])
                self.client.settle()
                result = self.client.pose('whole_body_left', harness.FAR_TARGET, frame='base_footprint')
                self.assertTrue(result.success, result.status)
                goals = self.client.base_goals()
                self.assertEqual(len(goals), 1)
                self.assertAlmostEqual(goals[0]['heading_start'], heading, delta=0.05)
                self.assertLessEqual(goals[0]['turned'], goals[0]['heading_change'] + math.pi)
                if heading == -3.0:
                    # Nearly every sampled heading for this goal lies past -pi, so a
                    # refusal here would mean the base cannot turn across the seam.
                    self.assertGreater(goals[0]['base_end'][2], 0.0, 'The goal did not cross the seam')

    def test_refused_while_nav2_has_a_goal(self):
        publisher = self.client.node.create_publisher(
            GoalStatusArray, '/navigate_to_pose/_action/status', LATCHED)
        active = GoalStatusArray(status_list=[GoalStatus(status=GoalStatus.STATUS_EXECUTING)])
        # The commander's status subscription can take a moment to match.
        status = ''
        for _ in range(10):
            publisher.publish(active)
            time.sleep(0.5)
            result = self.client.pose('whole_body_left', harness.FAR_TARGET)
            status = result.status
            if 'Nav2 has an active goal' in status:
                break
            self.client.reset_world()
        publisher.publish(GoalStatusArray())
        time.sleep(1.0)
        self.client.node.destroy_publisher(publisher)
        self.assertIn('Nav2 has an active goal', status)

    def refuse_with_a_nav2_server_that_dies(self, topic):
        """Return the refusal a Nav2 server with a running goal causes, after it died."""
        publisher = self.client.node.create_publisher(GoalStatusArray, topic, LATCHED)
        active = GoalStatusArray(status_list=[GoalStatus(status=GoalStatus.STATUS_EXECUTING)])
        status = ''
        for _ in range(10):
            publisher.publish(active)
            time.sleep(0.5)
            status = self.client.pose(
                'whole_body_left', harness.NEAR_TARGET, frame='base_footprint').status
            if 'Nav2 has an active goal' in status:
                break
            self.client.reset_world()
        self.client.node.destroy_publisher(publisher)
        return status

    def clear_nav2_status(self, topic):
        # An EXECUTING status left in the commander would refuse every later test.
        cleanup = self.client.node.create_publisher(GoalStatusArray, topic, LATCHED)
        time.sleep(1.0)
        cleanup.publish(GoalStatusArray())
        time.sleep(1.0)
        self.client.node.destroy_publisher(cleanup)

    def test_status_left_by_a_dead_nav2_server_does_not_refuse(self):
        # A server that dies during a goal never reports its end, so its last status
        # stays EXECUTING in every subscriber.
        topic = '/navigate_through_poses/_action/status'
        status = self.refuse_with_a_nav2_server_that_dies(topic)
        try:
            self.assertIn('Nav2 has an active goal', status)
            # The commander sees the server leave the graph.
            time.sleep(2.0)
            self.client.reset_world()
            result = self.client.pose('whole_body_left', harness.NEAR_TARGET, frame='base_footprint')
            self.assertTrue(result.success, result.status)
        finally:
            self.clear_nav2_status(topic)

    def test_status_left_before_a_nav2_restart_does_not_refuse(self):
        # A restarted server publishes no status before its first goal, so the old one's
        # EXECUTING is all a subscriber has, while the topic has a publisher again.
        topic = '/follow_path/_action/status'
        status = self.refuse_with_a_nav2_server_that_dies(topic)
        restarted = self.client.node.create_publisher(GoalStatusArray, topic, LATCHED)
        try:
            self.assertIn('Nav2 has an active goal', status)
            time.sleep(2.0)
            self.client.reset_world()
            result = self.client.pose('whole_body_left', harness.NEAR_TARGET, frame='base_footprint')
            self.assertTrue(result.success, result.status)
        finally:
            self.client.node.destroy_publisher(restarted)
            self.clear_nav2_status(topic)

    def test_start_repair_only_changes_joints_of_the_group(self):
        # In the home pose the right arm's fingers reach the base below a lift position of
        # about 0.495 m, so this state is in contact with the robot itself.
        self.client.command(joints={'lift_joint': 0.488})
        self.client.settle()
        # left_arm cannot move the lift. A start with the lift raised would describe a
        # robot that is not there, so MoveIt has to see the contact and refuse.
        self.assertFalse(self.client.named('left_arm', 'standby').success)
        self.assertEqual(self.client.goals(), [])
        # left_arm_lift moves the lift, so it plans from the raised lift and the controller
        # closes the difference.
        self.assertTrue(self.client.named('left_arm_lift', 'standby').success)
        self.assertIn(harness.LEFT_ARM, self.client.controllers())

    def test_refused_while_the_base_is_moving(self):
        self.client.command(base_velocity=0.2)
        self.client.settle()
        result = self.client.pose('whole_body_left', harness.FAR_TARGET)
        self.assertFalse(result.success)
        self.assertIn('The base is moving', result.status)
        self.assertEqual(self.client.goals(), [])

    def test_refused_with_collision_checking_off(self):
        services = (
            ('/toggle_all_collision_checking', 'Collision checking is off'),
            ('/toggle_octomap_collision', 'Octomap collision checking is off'),
        )
        for name, reason in services:
            with self.subTest(service=name):
                toggle = self.client.node.create_client(SetBool, name)
                self.assertTrue(toggle.wait_for_service(timeout_sec=30.0))
                self.client.wait(toggle.call_async(SetBool.Request(data=False)), 30.0)
                try:
                    result = self.client.pose('whole_body_left', harness.FAR_TARGET)
                finally:
                    self.client.wait(toggle.call_async(SetBool.Request(data=True)), 30.0)
                self.assertFalse(result.success)
                self.assertIn(reason, result.status)
                self.assertEqual(self.client.goals(), [])

    def test_cartesian_goal_is_refused_with_a_reason(self):
        result = self.client.pose('whole_body_left', harness.FAR_TARGET, cartesian=True)
        self.assertFalse(result.success)
        self.assertIn('Cartesian paths are not supported', result.status)
        self.assertEqual(self.client.goals(), [])

    def test_one_motion_at_a_time_and_cancel_stops_it(self):
        self.client.command(freeze=True)
        first = self.client.wait(self.client.send_pose('whole_body_left', harness.FAR_TARGET), 30.0)
        self.assertTrue(first.accepted)
        self.assertTrue(self.client.wait_for_goal(), 'No goal reached the stub controllers')

        second = self.client.pose('whole_body_left', harness.NEAR_TARGET, frame='base_footprint')
        self.assertFalse(second.success)
        self.assertIn('another motion is in progress', second.status)
        arm = self.client.pose('left_arm_lift', harness.NEAR_TARGET, frame='base_footprint')
        self.assertFalse(arm.success)
        self.assertIn('a whole-body motion is in progress', arm.status)

        self.client.wait(first.cancel_goal_async(), 30.0)
        outcome = self.client.wait(first.get_result_async(), 60.0)
        self.assertEqual(outcome.status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(outcome.result.status, 'Canceled')
        self.assertIn(harness.BASE_CONTROLLER, self.client.canceled())

        # The cancel released the motion slot.
        self.client.reset_world()
        result = self.client.pose('left_arm_lift', harness.NEAR_TARGET, frame='base_footprint')
        self.assertTrue(result.success, result.status)

    def test_expired_deadline_stops_a_frozen_controller(self):
        self.client.set_parameter('whole_body_execution_margin', 1.0)
        try:
            self.client.command(freeze=True)
            result = self.client.pose('whole_body_left', harness.NEAR_TARGET, frame='base_footprint')
        finally:
            self.client.set_parameter('whole_body_execution_margin', 15.0)
        self.assertFalse(result.success)
        self.assertIn('Execution deadline', result.status)
        self.assertIn(harness.LEFT_ARM, self.client.canceled())

    def test_end_effector_query_answers_on_a_whole_body_name(self):
        service = self.client.node.create_client(GetEEPose, '/get_ee_pose')
        self.assertTrue(service.wait_for_service(timeout_sec=30.0))
        request = GetEEPose.Request(group_name='whole_body_left', frame_id='map')
        response = self.client.wait(service.call_async(request), 30.0)
        self.assertTrue(response.success, response.status)
