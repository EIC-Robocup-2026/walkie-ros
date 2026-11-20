"""The commander's collision toggles against the real move_group.

/toggle_all_collision_checking, /toggle_octomap_collision and /toggle_gripper_collision each
turn checking of some names off and back on. The checks read move_group's
AllowedCollisionMatrix the way MoveIt reads it (allows() below): the listed value when both
names are listed, otherwise the default entries, where one that forbids wins. A toggle has to
change what move_group allows, not only the default entries, and turning it back on has to
leave every pair as it was.

Needs a running Zenoh router, as every launch test on this workspace does. Exit codes are not
asserted: move_group and the commander abort in libzenohc during shutdown.
"""
import os
import sys
import time
import unittest

from launch import LaunchDescription
import launch_testing
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from my_robot_interfaces.srv import ToggleGripperCollision
import pytest
import rclpy
from std_srvs.srv import SetBool

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import whole_body_harness as harness  # noqa: E402

OCTOMAP = '<octomap>'
# The touch links of each gripper in commander_template.cpp, gripperAttachInfo(): the
# links /toggle_gripper_collision and the allowance edit.
GRIPPER_LINKS = {
    side: {
        f'openarm_{side}_{link}'
        for link in ('hand', 'left_finger', 'right_finger', 'link7', 'wristcam_bracket')
    }
    for side in ('left', 'right')
}
LEFT_GRIPPER = GRIPPER_LINKS['left']


def default_entry(acm, name):
    if name in acm.default_entry_names:
        return acm.default_entry_values[acm.default_entry_names.index(name)]
    return None


def allows(acm, a, b):
    """Whether move_group lets a and b touch, read as MoveIt reads the message."""
    if a in acm.entry_names and b in acm.entry_names:
        return acm.entry_values[acm.entry_names.index(a)].enabled[acm.entry_names.index(b)]
    defaults = [d for d in (default_entry(acm, a), default_entry(acm, b)) if d is not None]
    return bool(defaults) and all(defaults)


def names(acm):
    """The robot's links and the octomap: every name the matrix lists or has a default for."""
    return sorted(set(acm.entry_names) | set(acm.default_entry_names) | {OCTOMAP})


def pairs(acm, among):
    return {(a, b): allows(acm, a, b) for i, a in enumerate(among) for b in among[i + 1:]}


@pytest.mark.launch_test
def generate_test_description():
    return LaunchDescription(
        harness.bringup('fixed', 'fixed') + [launch_testing.actions.ReadyToTest()])


class TestCollisionToggles(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.client = harness.CommanderClient('collision_toggles_test')
        node = cls.client.node
        cls.get_scene = node.create_client(GetPlanningScene, '/get_planning_scene')
        cls.toggle_all = node.create_client(SetBool, '/toggle_all_collision_checking')
        cls.toggle_octomap = node.create_client(SetBool, '/toggle_octomap_collision')
        cls.toggle_gripper = node.create_client(ToggleGripperCollision, '/toggle_gripper_collision')
        cls.client.wait_ready()
        # The allowance lists the octomap, and a toggle has to reach a listed name too.
        # The commander applies it after the parameter call returns.
        cls.client.set_parameter('allow_gripper_vs_octomap', True)
        grippers = GRIPPER_LINKS['left'] | GRIPPER_LINKS['right']
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            acm = cls.read_acm()
            if OCTOMAP in acm.entry_names and all(allows(acm, g, OCTOMAP) for g in grippers):
                break
            time.sleep(0.1)
        else:
            raise AssertionError('The allowance never reached the scene')

    @classmethod
    def tearDownClass(cls):
        cls.client.close()
        rclpy.try_shutdown()

    @classmethod
    def read_acm(cls):
        assert cls.get_scene.wait_for_service(timeout_sec=30.0)
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        return cls.client.wait(cls.get_scene.call_async(request), 30.0).scene.allowed_collision_matrix

    def acm(self):
        return self.read_acm()

    def call(self, client, request):
        self.assertTrue(client.wait_for_service(timeout_sec=30.0))
        response = self.client.wait(client.call_async(request), 30.0)
        self.assertTrue(response.success, response)

    def set_all(self, check):
        self.call(self.toggle_all, SetBool.Request(data=check))

    def set_octomap(self, use):
        self.call(self.toggle_octomap, SetBool.Request(data=use))

    def set_left_gripper(self, enable):
        self.call(self.toggle_gripper,
                  ToggleGripperCollision.Request(group_name='left_gripper', enable=enable))

    def assert_restored(self, before, among):
        after = pairs(self.acm(), among)
        changed = sorted(pair for pair in before if before[pair] != after[pair])
        self.assertEqual(changed, [], f'{len(changed)} pairs changed')

    def test_turning_all_checking_off_allows_every_pair_and_on_restores_them(self):
        acm = self.acm()
        among = names(acm)
        before = pairs(acm, among)
        self.addCleanup(self.set_all, True)
        self.set_all(False)
        checked = sorted(pair for pair, allowed in pairs(self.acm(), among).items() if not allowed)
        self.assertEqual(checked, [], f'{len(checked)} pairs still checked')
        self.set_all(True)
        self.assert_restored(before, among)

    def test_the_octomap_toggle_reaches_every_link_and_back(self):
        acm = self.acm()
        self.assertIn(OCTOMAP, acm.entry_names)
        among = names(acm)
        before = pairs(acm, among)
        self.addCleanup(self.set_octomap, True)
        self.set_octomap(False)
        acm = self.acm()
        checked = sorted(n for n in among if n != OCTOMAP and not allows(acm, n, OCTOMAP))
        self.assertEqual(checked, [], 'links still checked against the octomap')
        self.set_octomap(True)
        self.assert_restored(before, among)

    def test_a_gripper_toggle_reaches_the_gripper_links_and_back(self):
        acm = self.acm()
        among = names(acm)
        before = pairs(acm, among)
        self.addCleanup(self.set_left_gripper, True)
        self.set_left_gripper(False)
        acm = self.acm()
        checked = sorted((g, n) for g in LEFT_GRIPPER for n in among
                         if n != g and not allows(acm, g, n))
        self.assertEqual(checked, [], 'gripper pairs still checked')
        self.assertFalse(allows(acm, 'base_link', OCTOMAP))
        self.set_left_gripper(True)
        self.assert_restored(before, among)

    def test_the_octomap_toggle_works_after_all_checking_was_off_and_on(self):
        acm = self.acm()
        among = names(acm)
        self.addCleanup(self.set_octomap, True)
        self.addCleanup(self.set_all, True)
        self.set_all(False)
        self.set_all(True)
        self.set_octomap(False)
        acm = self.acm()
        checked = sorted(n for n in among if n != OCTOMAP and not allows(acm, n, OCTOMAP))
        self.assertEqual(checked, [], 'links still checked against the octomap')
