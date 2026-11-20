"""The gripper <-> octomap allowance and the ACM toggles, when their writes overlap.

Each of them reads the whole AllowedCollisionMatrix from /get_planning_scene, changes its
own entries and writes the whole matrix back through /apply_planning_scene. The
commander's /get_planning_scene is remapped to a service of this test (SceneService),
which passes every request on to move_group at once, but can hold one answer back. The
commander then works on a matrix read before a later change landed, as when move_group
answers late, and the test checks that the later change survives.

The checks look only at the gripper links' pairs with the octomap and at the octomap's
default entry, which are what the allowance and /toggle_octomap_collision set.

Needs a running Zenoh router, as every launch test on this workspace does. Exit codes
are not asserted: move_group and the commander abort in libzenohc during shutdown.
"""
import os
import sys
import threading
import time
import unittest

from launch import LaunchDescription
import launch_testing
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
import pytest
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import SetBool

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import whole_body_harness as harness  # noqa: E402

STUB_SERVICE = '/gripper_octomap_allowance_test/get_planning_scene'
OCTOMAP = '<octomap>'
PARAMETER = 'allow_gripper_vs_octomap'
# The touch links of both grippers in commander_template.cpp, gripperAttachInfo(): the
# links the allowance lets touch the octomap.
GRIPPER_LINKS = {
    f'openarm_{side}_{link}'
    for side in ('left', 'right')
    for link in ('hand', 'left_finger', 'right_finger', 'link7', 'wristcam_bracket')
}
# The commander logs this at startup, which tells its output from that of the others.
COMMANDER_STARTED = b'Arm groups using planner'
# The commander logs one of these at the end of each apply of the allowance.
APPLIED = b'Gripper <-> octomap collision '
ALLOWED = b'Gripper <-> octomap collision ALLOWED'
ENFORCED = b'Gripper <-> octomap collision ENFORCED'
# The commander waits at most 5 s for /get_planning_scene, so a held answer is released
# well before that.
HOLD_S = 3.0
# rclcpp's MultiThreadedExecutor starts std::thread::hardware_concurrency() threads, at
# least 2. Both that and os.cpu_count() count the CPUs online.
EXECUTOR_THREADS = max(os.cpu_count() or 2, 2)


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


def wait_until(predicate, timeout):
    """Return True once predicate() is true, or False after `timeout` seconds."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def grippers_allowed(acm):
    """The gripper links that may touch the octomap."""
    if OCTOMAP not in acm.entry_names:
        return set()
    column = acm.entry_names.index(OCTOMAP)
    return {
        name for row, name in enumerate(acm.entry_names)
        if name in GRIPPER_LINKS and acm.entry_values[row].enabled[column]
    }


def octomap_default(acm):
    """The octomap's default entry: True when planning ignores the octomap."""
    if OCTOMAP not in acm.default_entry_names:
        return False
    return acm.default_entry_values[acm.default_entry_names.index(OCTOMAP)]


class SceneService:
    """Answers the commander's remapped /get_planning_scene with move_group's answer.

    Each request goes on to move_group when it arrives. After hold_next(), the answer to
    the next request waits until release(), so the commander gets a matrix that was read
    before whatever changed the scene meanwhile.
    """

    def __init__(self):
        self.node = rclpy.create_node('gripper_octomap_allowance_stub')
        group = ReentrantCallbackGroup()
        self.upstream = self.node.create_client(
            GetPlanningScene, '/get_planning_scene', callback_group=group)
        self.node.create_service(
            GetPlanningScene, STUB_SERVICE, self.on_request, callback_group=group)
        self.lock = threading.Lock()
        self.armed = False
        self.gate = threading.Event()
        self.held = 0
        self.requests = 0
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        threading.Thread(target=self.executor.spin, daemon=True).start()

    def close(self):
        self.release()
        self.executor.shutdown()
        self.node.destroy_node()

    def hold_next(self):
        with self.lock:
            self.armed = True
            self.gate = threading.Event()
            self.held = 0

    def release(self):
        with self.lock:
            self.armed = False
            self.gate.set()

    def counts(self):
        with self.lock:
            return self.held, self.requests

    def wait_held(self, timeout=10.0):
        """Wait until move_group has answered the request after hold_next(), and the
        answer is held."""
        return wait_until(lambda: self.counts()[0] > 0, timeout)

    def on_request(self, request, response):
        with self.lock:
            self.requests += 1
            gate = None
            if self.armed:
                self.armed = False
                gate = self.gate
        answer = self.forward(request, response)
        if gate is not None:
            # Counted only once move_group has answered, so the held matrix was read
            # before anything the test does after wait_held().
            with self.lock:
                self.held += 1
            gate.wait(10.0)
        return answer

    def forward(self, request, response):
        if self.upstream.wait_for_service(timeout_sec=5.0):
            future = self.upstream.call_async(request)
            answered = threading.Event()
            future.add_done_callback(lambda _: answered.set())
            if answered.wait(10.0):
                return future.result()
        return response


@pytest.mark.launch_test
def generate_test_description():
    return LaunchDescription(
        harness.bringup(
            'fixed', 'fixed', commander_remappings=[('/get_planning_scene', STUB_SERVICE)])
        + [launch_testing.actions.ReadyToTest()])


class TestGripperOctomapAllowance(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        # Before the commander starts, so its remapped client finds the service.
        cls.scene = SceneService()
        cls.client = harness.CommanderClient('gripper_octomap_allowance_test')
        node = cls.client.node
        cls.get_scene = node.create_client(GetPlanningScene, '/get_planning_scene')
        cls.toggle_octomap = node.create_client(SetBool, '/toggle_octomap_collision')
        cls.client.wait_ready()

    @classmethod
    def tearDownClass(cls):
        cls.client.close()
        cls.scene.close()
        rclpy.try_shutdown()

    def setUp(self):
        self.scene.release()
        self.addCleanup(self.scene.release)

    def acm(self):
        """move_group's matrix, read directly rather than through the stub."""
        self.assertTrue(self.get_scene.wait_for_service(timeout_sec=30.0))
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        return self.client.wait(self.get_scene.call_async(request), 30.0).scene.allowed_collision_matrix

    def applied_since(self, proc_output, start):
        return commander_log(proc_output)[start:].count(APPLIED)

    def wait_applied(self, proc_output, start, count, timeout=30.0):
        """Wait until the commander has logged `count` applies of the allowance after `start`."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.applied_since(proc_output, start) >= count:
                return commander_log(proc_output)[start:]
            time.sleep(0.05)
        self.fail(f'The commander logged {self.applied_since(proc_output, start)} of '
                  f'{count} applies of the allowance')

    def set_allowance(self, proc_output, allow):
        """Set the parameter and wait for its apply, with no answer held."""
        start = len(commander_log(proc_output))
        self.client.set_parameter(PARAMETER, allow)
        self.wait_applied(proc_output, start, 1)

    def toggle(self, use_octomap):
        """Return the future of a /toggle_octomap_collision call."""
        self.assertTrue(self.toggle_octomap.wait_for_service(timeout_sec=30.0))
        return self.toggle_octomap.call_async(SetBool.Request(data=use_octomap))

    def test_the_allowance_reaches_the_scene(self, proc_output):
        self.addCleanup(self.set_allowance, proc_output, False)
        self.set_allowance(proc_output, True)
        self.assertEqual(grippers_allowed(self.acm()), GRIPPER_LINKS)
        self.set_allowance(proc_output, False)
        self.assertEqual(grippers_allowed(self.acm()), set())

    def test_a_late_enable_does_not_undo_a_later_disable(self, proc_output):
        self.addCleanup(self.set_allowance, proc_output, False)
        start = len(commander_log(proc_output))
        self.scene.hold_next()
        self.client.set_parameter(PARAMETER, True)
        self.assertTrue(self.scene.wait_held(), 'The enable never read the scene')
        self.client.set_parameter(PARAMETER, False)
        # An apply of the disable that does not wait for the enable finishes while the
        # enable's answer is held; wait for that, so the enable's write lands after it.
        # An apply that waits never logs here, and the answer is released after HOLD_S.
        wait_until(lambda: ENFORCED in commander_log(proc_output)[start:], HOLD_S)
        self.scene.release()
        log = self.wait_applied(proc_output, start, 2)
        self.assertLess(log.find(ALLOWED), log.find(ENFORCED), log.decode(errors='replace'))
        self.assertEqual(grippers_allowed(self.acm()), set(),
                         'The scene still lets the fingers touch the octomap')

    def test_a_toggle_during_an_apply_of_the_allowance_survives_it(self, proc_output):
        self.addCleanup(self.set_allowance, proc_output, False)
        self.addCleanup(lambda: self.client.wait(self.toggle(True), 30.0))
        # Start with the octomap ignored and the fingers allowed, then disallow the
        # fingers and turn the octomap back on while the disallow's read is held.
        self.assertTrue(self.client.wait(self.toggle(False), 30.0).success)
        self.set_allowance(proc_output, True)
        self.assertTrue(octomap_default(self.acm()))
        start = len(commander_log(proc_output))
        self.scene.hold_next()
        self.client.set_parameter(PARAMETER, False)
        self.assertTrue(self.scene.wait_held(), 'The disallow never read the scene')
        toggled = self.toggle(True)
        # As in the test above: a toggle that does not wait finishes during the hold.
        wait_until(toggled.done, HOLD_S)
        self.scene.release()
        self.wait_applied(proc_output, start, 1)
        self.assertTrue(self.client.wait(toggled, 30.0).success)
        acm = self.acm()
        self.assertFalse(octomap_default(acm), 'The allowance undid the toggle')
        self.assertEqual(grippers_allowed(acm), set(), 'The toggle undid the allowance')

    def test_an_apply_finishes_while_more_toggles_than_threads_wait_for_it(self, proc_output):
        # Each toggle waits for the apply in one of the commander's executor threads.
        # With more toggles than threads, the apply's answer reaches it only if no
        # executor thread is needed to take that answer in.
        self.addCleanup(self.set_allowance, proc_output, False)
        start = len(commander_log(proc_output))
        self.scene.hold_next()
        self.client.set_parameter(PARAMETER, True)
        self.assertTrue(self.scene.wait_held(), 'The enable never read the scene')
        # Turning the octomap on while it is on changes nothing.
        toggles = [self.toggle(True) for _ in range(EXECUTOR_THREADS + 2)]
        # Time for the commander to take the toggles. None can finish during the hold,
        # since each waits for the apply.
        time.sleep(1.0)
        self.scene.release()
        log = self.wait_applied(proc_output, start, 1)
        self.assertIn(ALLOWED, log)
        for toggled in toggles:
            self.assertTrue(self.client.wait(toggled, 60.0).success)
        self.assertEqual(grippers_allowed(self.acm()), GRIPPER_LINKS)
