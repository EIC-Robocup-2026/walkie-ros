"""The real base follower against a simulated base, without move_group.

The test plays everything around the follower: the base, which integrates the commands and
publishes map -> base_footprint, the commander's heartbeat, the collision monitor's state
and MoveIt's goals. Needs a running Zenoh router, as every launch test on this workspace
does. Exit codes are not asserted: libzenohc can abort during shutdown.
"""
import math
import threading
import time
import unittest

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Transform, TransformStamped, Twist, TwistStamped
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
from nav2_msgs.msg import CollisionMonitorState
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint

COMMANDER_TIMEOUT = 1.0
START = (0.5, -0.2, 1.0)
SPEED = 0.12
# The follower answers within a few control ticks, and the machine may be loaded.
PROMPT = 1.5


@pytest.mark.launch_test
def generate_test_description():
    follower = Node(
        package='openarm_bimanual_commander_cpp',
        executable='whole_body_base_follower',
        output='screen',
        parameters=[{'commander_timeout': COMMANDER_TIMEOUT}],
    )
    return LaunchDescription([follower, launch_testing.actions.ReadyToTest()])


def line_goal(start, duration, speed=SPEED):
    """Return a goal along map x from the start pose, at constant speed and heading."""
    goal = FollowJointTrajectory.Goal()
    goal.multi_dof_trajectory.joint_names = ['virtual_joint']
    x, y, yaw = start
    steps = max(1, int(duration / 0.1))
    for step in range(steps + 1):
        time_from_start = duration * step / steps
        transform = Transform()
        transform.translation.x = x + speed * time_from_start
        transform.translation.y = y
        transform.rotation.z = math.sin(yaw / 2.0)
        transform.rotation.w = math.cos(yaw / 2.0)
        velocity = Twist()
        velocity.linear.x = speed if step < steps else 0.0
        point = MultiDOFJointTrajectoryPoint(transforms=[transform], velocities=[velocity])
        point.time_from_start.sec = int(time_from_start)
        point.time_from_start.nanosec = int((time_from_start % 1.0) * 1e9)
        goal.multi_dof_trajectory.points.append(point)
    return goal


class World:
    """Everything around the follower, on its own executor."""

    def __init__(self):
        self.node = rclpy.create_node('whole_body_follower_test')
        self.lock = threading.Lock()
        self.reset()
        self.transforms = TransformBroadcaster(self.node)
        self.heartbeat = self.node.create_publisher(Bool, '/whole_body/commander_alive', 10)
        self.events = self.node.create_publisher(String, '/trajectory_execution_event', 10)
        self.monitor = self.node.create_publisher(
            CollisionMonitorState, '/collision_monitor_state', 10)
        self.node.create_subscription(TwistStamped, '/cmd_vel_smoothed', self.on_command, 50)
        self.node.create_subscription(String, '/trajectory_execution_event', self.on_event, 10)
        self.action = ActionClient(
            self.node, FollowJointTrajectory, '/base_controller/follow_joint_trajectory')
        self.last_tick = time.monotonic()
        self.node.create_timer(0.02, self.tick)
        self.node.create_timer(0.1, self.beat)
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        threading.Thread(target=self.executor.spin, daemon=True).start()

    def reset(self):
        with self.lock:
            self.pose = list(START)
            self.twist = (0.0, 0.0, 0.0)
            self.commands = []
            self.stops = 0
            self.commander_alive = True
            # Whether the heartbeat reports a running whole-body motion.
            self.commander_executing = False
            # The fraction of each command the base carries out.
            self.command_scale = 1.0
            self.publish_tf = True
            self.follow_commands = True
            # What the real monitor does with a zero command when nothing is in the polygon.
            self.monitor_clears_on_command = False

    def close(self):
        self.executor.shutdown()
        self.node.destroy_node()

    def on_command(self, message):
        twist = message.twist
        command = (twist.linear.x, twist.linear.y, twist.angular.z)
        clear = False
        with self.lock:
            self.commands.append(command)
            if self.follow_commands:
                self.twist = tuple(self.command_scale * value for value in command)
            clear = self.monitor_clears_on_command
        if clear:
            self.set_monitor(CollisionMonitorState.DO_NOTHING, '')

    def on_event(self, message):
        if message.data == 'stop':
            with self.lock:
                self.stops += 1

    def tick(self):
        now = time.monotonic()
        elapsed = now - self.last_tick
        self.last_tick = now
        with self.lock:
            vx, vy, vyaw = self.twist
            yaw = self.pose[2]
            self.pose[0] += (math.cos(yaw) * vx - math.sin(yaw) * vy) * elapsed
            self.pose[1] += (math.sin(yaw) * vx + math.cos(yaw) * vy) * elapsed
            self.pose[2] += vyaw * elapsed
            x, y, yaw = self.pose
            publish = self.publish_tf
        if not publish:
            return
        transform = TransformStamped()
        transform.header.stamp = self.node.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.z = math.sin(yaw / 2.0)
        transform.transform.rotation.w = math.cos(yaw / 2.0)
        self.transforms.sendTransform(transform)

    def beat(self):
        with self.lock:
            alive = self.commander_alive
            executing = self.commander_executing
        if alive:
            self.heartbeat.publish(Bool(data=executing))

    def set_monitor(self, action_type, polygon):
        self.monitor.publish(CollisionMonitorState(action_type=action_type, polygon_name=polygon))

    def set(self, **values):
        with self.lock:
            for name, value in values.items():
                setattr(self, name, value)

    def get_commands(self):
        with self.lock:
            return list(self.commands)

    def get_pose(self):
        with self.lock:
            return list(self.pose)

    def get_stops(self):
        with self.lock:
            return self.stops

    def wait_for_commands(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if any(any(value != 0.0 for value in command) for command in self.get_commands()):
                return True
            time.sleep(0.01)
        return False

    @staticmethod
    def wait(future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        assert future.done(), f'No answer in {timeout} s'
        return future.result()


class TestWholeBodyFollower(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.world = World()
        assert cls.world.action.wait_for_server(timeout_sec=60.0), 'The follower did not start'
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline and \
                cls.world.monitor.get_subscription_count() == 0:
            time.sleep(0.1)
        assert cls.world.monitor.get_subscription_count() > 0
        time.sleep(1.0)

    @classmethod
    def tearDownClass(cls):
        cls.world.close()
        rclpy.try_shutdown()

    def setUp(self):
        self.world.reset()
        self.world.set_monitor(CollisionMonitorState.DO_NOTHING, '')
        # The follower needs a fresh transform and a heartbeat again, and the previous
        # test's last commands have to clear the subscription.
        time.sleep(0.5)
        self.world.set(commands=[], stops=0, twist=(0.0, 0.0, 0.0))
        self.handles = []

    def tearDown(self):
        # A failed test can leave a goal running, and the follower takes one at a time.
        for handle in self.handles:
            self.world.wait(handle.cancel_goal_async(), 5.0)
        time.sleep(0.1)

    def send(self, goal):
        handle = self.world.wait(self.world.action.send_goal_async(goal), 10.0)
        if handle.accepted:
            self.handles.append(handle)
        return handle

    def start(self, duration=6.0):
        """Return the handle of a running goal, once the base is really moving."""
        handle = self.send(line_goal(START, duration))
        self.assertTrue(handle.accepted)
        self.assertTrue(self.world.wait_for_commands(), 'The follower sent no command')
        return handle

    def assert_base_halted(self):
        commands = self.world.get_commands()
        self.assertEqual(commands[-1], (0.0, 0.0, 0.0))
        before = self.world.get_pose()
        time.sleep(0.3)
        self.assertLess(math.dist(before[:2], self.world.get_pose()[:2]), 0.002)

    def assert_aborted_with_stop(self, handle, reason, timeout=PROMPT):
        outcome = self.world.wait(handle.get_result_async(), timeout)
        self.assertEqual(outcome.status, GoalStatus.STATUS_ABORTED)
        self.assertIn(reason, outcome.result.error_string)
        self.assert_base_halted()
        self.assertGreaterEqual(self.world.get_stops(), 1)

    def test_tracks_the_trajectory_in_the_map_frame(self):
        duration = 3.0
        handle = self.start(duration)
        outcome = self.world.wait(handle.get_result_async(), duration + 5.0)
        self.assertEqual(outcome.status, GoalStatus.STATUS_SUCCEEDED, outcome.result.error_string)
        self.assertEqual(outcome.result.error_code, FollowJointTrajectory.Result.SUCCESSFUL)
        pose = self.world.get_pose()
        self.assertAlmostEqual(pose[0], START[0] + SPEED * duration, delta=0.03)
        self.assertAlmostEqual(pose[1], START[1], delta=0.03)
        self.assertAlmostEqual(pose[2], START[2], delta=0.03)
        self.assert_base_halted()
        self.assertEqual(self.world.get_stops(), 0)

    def test_refuses_a_goal_without_a_commander(self):
        self.world.set(commander_alive=False)
        time.sleep(COMMANDER_TIMEOUT + 0.3)
        handle = self.send(line_goal(START, 2.0))
        self.assertFalse(handle.accepted)
        self.assertEqual(self.world.get_commands(), [])
        # The commander was idle, so there was nothing to stop.
        self.assertEqual(self.world.get_stops(), 0)

    def test_stops_when_the_commander_goes_silent(self):
        handle = self.start()
        self.world.set(commander_alive=False)
        self.assert_aborted_with_stop(handle, 'commander', COMMANDER_TIMEOUT + PROMPT)

    def test_silent_commander_during_a_motion_sends_one_stop(self):
        # A whole-body motion that keeps the base still sends no goal here, but move_group
        # would run it to the end for a commander that has died.
        self.world.set(commander_executing=True)
        time.sleep(0.5)
        self.world.set(commander_alive=False)
        deadline = time.monotonic() + COMMANDER_TIMEOUT + PROMPT
        while time.monotonic() < deadline and self.world.get_stops() == 0:
            time.sleep(0.05)
        self.assertEqual(self.world.get_stops(), 1)
        time.sleep(1.0)
        self.assertEqual(self.world.get_stops(), 1)

    def test_refuses_a_goal_during_a_monitor_stop(self):
        self.world.set_monitor(CollisionMonitorState.STOP, 'PolygonStop')
        time.sleep(0.3)
        self.assertFalse(self.send(line_goal(START, 2.0)).accepted)
        # The zero command that made the monitor look again.
        self.assertTrue(self.world.get_commands())
        self.assertFalse(self.world.wait_for_commands(0.2))
        self.world.set_monitor(CollisionMonitorState.DO_NOTHING, '')
        time.sleep(0.3)
        handle = self.send(line_goal(START, 0.5))
        self.assertTrue(handle.accepted)
        self.world.wait(handle.get_result_async(), 10.0)

    def test_accepts_a_goal_after_an_old_monitor_stop(self):
        # The monitor reports a change only, and only looks when a command arrives, so a
        # STOP on record may be long over.
        self.world.set_monitor(CollisionMonitorState.STOP, 'PolygonStop')
        time.sleep(0.3)
        self.world.set(monitor_clears_on_command=True)
        handle = self.send(line_goal(START, 0.5))
        self.assertTrue(handle.accepted)
        outcome = self.world.wait(handle.get_result_async(), 10.0)
        self.assertEqual(outcome.status, GoalStatus.STATUS_SUCCEEDED, outcome.result.error_string)

    def test_aborts_on_a_monitor_stop_during_the_goal(self):
        handle = self.start()
        self.world.set_monitor(CollisionMonitorState.STOP, 'PolygonStop')
        self.assert_aborted_with_stop(handle, 'PolygonStop')

    def test_a_slowdown_does_not_abort(self):
        handle = self.start(2.0)
        self.world.set_monitor(CollisionMonitorState.SLOWDOWN, 'PolygonSlow')
        outcome = self.world.wait(handle.get_result_async(), 7.0)
        self.assertEqual(outcome.status, GoalStatus.STATUS_SUCCEEDED, outcome.result.error_string)

    def test_stop_event_ends_the_goal(self):
        handle = self.start()
        self.world.events.publish(String(data='stop'))
        self.assert_aborted_with_stop(handle, 'stop')
        # MoveIt heard the same event, so the follower adds none of its own. Each one it
        # sends could reach a later goal, because an event carries no time.
        time.sleep(0.5)
        self.assertEqual(self.world.get_stops(), 1)

    def test_old_stop_event_does_not_end_the_next_goal(self):
        self.world.events.publish(String(data='stop'))
        time.sleep(0.3)
        handle = self.send(line_goal(START, 0.5))
        self.assertTrue(handle.accepted)
        outcome = self.world.wait(handle.get_result_async(), 10.0)
        self.assertEqual(outcome.status, GoalStatus.STATUS_SUCCEEDED, outcome.result.error_string)

    def test_cancel_ends_with_zero_commands(self):
        handle = self.start()
        self.world.wait(handle.cancel_goal_async(), 5.0)
        outcome = self.world.wait(handle.get_result_async(), PROMPT)
        self.assertEqual(outcome.status, GoalStatus.STATUS_CANCELED)
        self.assert_base_halted()

    def test_stale_transform_aborts(self):
        handle = self.start()
        self.world.set(publish_tf=False)
        self.assert_aborted_with_stop(handle, 'transform')

    def test_refuses_a_trajectory_that_starts_elsewhere(self):
        far_start = (START[0] + 1.0, START[1], START[2])
        self.assertFalse(self.send(line_goal(far_start, 2.0)).accepted)
        self.assertEqual(self.world.get_commands(), [])

    def test_aborts_at_once_when_the_base_does_not_move(self):
        # What a collision monitor stop looks like when its state message never arrived.
        self.world.set(follow_commands=False)
        handle = self.start()
        # One stall window to fall 5 cm behind, and one more if the first ends just short.
        self.assert_aborted_with_stop(handle, 'not moving', 1.0 + PROMPT)

    def test_aborts_when_a_slow_base_falls_behind(self):
        self.world.set(command_scale=0.3)
        handle = self.start()
        # At most 0.075 m/s against 0.12 m/s reaches 0.25 m of error after about 4.5 s.
        self.assert_aborted_with_stop(handle, 'tracking error', 5.5 + PROMPT)

    def test_a_slowed_base_is_not_stalled(self):
        # A slowdown polygon at ratio 0.5. The base lags but moves.
        self.world.set(command_scale=0.5)
        handle = self.start(2.0)
        outcome = self.world.wait(handle.get_result_async(), 8.0)
        self.assertEqual(outcome.status, GoalStatus.STATUS_SUCCEEDED, outcome.result.error_string)
