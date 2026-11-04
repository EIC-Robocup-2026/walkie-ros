#!/usr/bin/env python3
"""Stands in for the robot in the whole-body commander tests.

Publishes joint states and map -> base_footprint, and serves every controller
move_group knows. A goal moves the state straight to the end of its trajectory, so
the tests run without a simulator and in much less time than the trajectory lasts.

Every goal and cancel goes out on /stub_world/events as JSON. Commands arrive as JSON
on /stub_world/command, and each one is answered with an event that carries its id:

    {"id": 1, "reset": true}             initial state, no freeze, no base velocity
    {"id": 2, "freeze": true}            hold goals until a cancel or an unfreeze
    {"id": 3, "base": [x, y, yaw]}       move the base at once
    {"id": 4, "base_velocity": 0.2}      drive the base along map x, in m/s
    {"id": 5, "abort": "lift_controller"}  abort the running goal of that controller
"""
import json
import math
import threading
import time
import xml.etree.ElementTree as ElementTree

from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

TRAJECTORY_CONTROLLERS = (
    'left_joint_trajectory_controller', 'right_joint_trajectory_controller', 'lift_controller')
GRIPPER_CONTROLLERS = ('left_gripper_controller', 'right_gripper_controller')
BASE_CONTROLLER = 'base_controller'
INITIAL_STATE_NAME = 'home'
INITIAL_STATE_GROUP = 'both_arms_lift'


def active_joints(urdf):
    """Return the joints a joint state message has to list, from the URDF text."""
    names = []
    for joint in ElementTree.fromstring(urdf).findall('joint'):
        if joint.get('type') != 'fixed' and joint.find('mimic') is None:
            names.append(joint.get('name'))
    return names


def named_state(srdf, state_name, group):
    """Return the joint values of a named SRDF state."""
    for state in ElementTree.fromstring(srdf).findall('group_state'):
        if state.get('name') == state_name and state.get('group') == group:
            return {joint.get('name'): float(joint.get('value')) for joint in state.findall('joint')}
    raise ValueError(f'The SRDF has no state {state_name} for group {group}')


def wrap(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_of(rotation):
    """Return the heading of a planar quaternion, in [-pi, pi]."""
    return wrap(2.0 * math.atan2(rotation.z, rotation.w))


class StubWorld(Node):

    def __init__(self):
        super().__init__('whole_body_stub_world')
        urdf = self.declare_parameter('robot_description', '').value
        srdf = self.declare_parameter('robot_description_semantic', '').value
        with_base_server = self.declare_parameter('with_base_server', True).value
        self.initial_joints = dict.fromkeys(active_joints(urdf), 0.0)
        self.initial_joints.update(named_state(srdf, INITIAL_STATE_NAME, INITIAL_STATE_GROUP))

        self.lock = threading.Lock()
        self.reset()

        latched = QoSProfile(
            depth=1000, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.events = self.create_publisher(String, '/stub_world/events', latched)
        self.joint_states = self.create_publisher(JointState, '/joint_states', 10)
        self.transforms = TransformBroadcaster(self)

        group = ReentrantCallbackGroup()
        self.create_subscription(
            String, '/stub_world/command', self.on_command,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE), callback_group=group)

        self.servers = []
        controllers = list(TRAJECTORY_CONTROLLERS)
        if with_base_server:
            controllers.append(BASE_CONTROLLER)
        for controller in controllers:
            self.servers.append(self.make_server(
                FollowJointTrajectory, f'/{controller}/follow_joint_trajectory', controller, group))
        for controller in GRIPPER_CONTROLLERS:
            self.servers.append(self.make_server(
                GripperCommand, f'/{controller}/gripper_cmd', controller, group))

        self.last_tick = time.monotonic()
        self.create_timer(0.02, self.tick, callback_group=group)

    def reset(self):
        with self.lock:
            self.joints = dict(self.initial_joints)
            self.base = [0.0, 0.0, 0.0]
            self.base_velocity = 0.0
            self.frozen = False
            self.aborting = set()

    def make_server(self, action_type, name, controller, group):
        return ActionServer(
            self, action_type, name,
            execute_callback=lambda handle: self.execute(handle, controller, action_type),
            goal_callback=lambda _goal: GoalResponse.ACCEPT,
            cancel_callback=lambda _handle: CancelResponse.ACCEPT,
            callback_group=group)

    def emit(self, **event):
        self.events.publish(String(data=json.dumps(event)))

    def on_command(self, message):
        command = json.loads(message.data)
        if command.get('reset'):
            self.reset()
        with self.lock:
            if 'freeze' in command:
                self.frozen = command['freeze']
            if 'base' in command:
                self.base = list(command['base'])
            if 'base_velocity' in command:
                self.base_velocity = command['base_velocity']
            if 'abort' in command:
                self.aborting.add(command['abort'])
        self.emit(ack=command['id'])

    def tick(self):
        now = time.monotonic()
        elapsed = now - self.last_tick
        self.last_tick = now
        with self.lock:
            self.base[0] += self.base_velocity * elapsed
            names = list(self.joints)
            positions = [self.joints[name] for name in names]
            x, y, yaw = self.base

        stamp = self.get_clock().now().to_msg()
        state = JointState(name=names, position=positions)
        state.header.stamp = stamp
        self.joint_states.publish(state)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.z = math.sin(yaw / 2.0)
        transform.transform.rotation.w = math.cos(yaw / 2.0)
        self.transforms.sendTransform(transform)

    def describe(self, goal, controller, action_type):
        """Return the goal event, and how the goal changes the state once it finishes."""
        event = {'goal': controller}
        joints = {}
        base = None
        if action_type is GripperCommand:
            return event, joints, base
        trajectory = goal.trajectory
        if trajectory.points:
            event['joints'] = list(trajectory.joint_names)
            joints = dict(zip(trajectory.joint_names, trajectory.points[-1].positions))
        points = goal.multi_dof_trajectory.points
        if points:
            headings = [yaw_of(point.transforms[0].rotation) for point in points]
            last = points[-1].transforms[0]
            base = [last.translation.x, last.translation.y, headings[-1]]
            event['base_end'] = base
            event['heading_start'] = headings[0]
            event['heading_change'] = abs(wrap(headings[-1] - headings[0]))
            event['turned'] = sum(
                abs(wrap(later - earlier)) for earlier, later in zip(headings, headings[1:]))
            xs = [point.transforms[0].translation.x for point in points]
            ys = [point.transforms[0].translation.y for point in points]
            event['base_bounds'] = [min(xs), min(ys), max(xs), max(ys)]
        return event, joints, base

    def execute(self, handle, controller, action_type):
        event, joints, base = self.describe(handle.request, controller, action_type)
        self.emit(**event)
        while True:
            if handle.is_cancel_requested:
                self.emit(canceled=controller)
                handle.canceled()
                return action_type.Result()
            with self.lock:
                if controller in self.aborting:
                    self.aborting.discard(controller)
                    self.emit(aborted=controller)
                    handle.abort()
                    return action_type.Result()
                if not self.frozen:
                    self.joints.update(joints)
                    if base is not None:
                        self.base = base
                    break
            time.sleep(0.02)
        handle.succeed()
        result = action_type.Result()
        if action_type is GripperCommand:
            result.reached_goal = True
        return result


def main():
    rclpy.init()
    node = StubWorld()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
