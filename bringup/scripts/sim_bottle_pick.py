#!/usr/bin/env python3
"""Scripted pick of a water bottle in the MuJoCo Chula PAT14-6 world.

Needs robot_bringup whole_body_demo.launch.py world:=chula_pat14_6 running. Before it
changes a parameter or sends a goal, the script checks that it talks to that
simulation, and exits 2 naming the failed check otherwise:

  - /clock delivers at least 3 messages with advancing time within 10 s of wall time,
  - the commander and bt_navigator report use_sim_time true,
  - the simulator's controller_manager reports mujoco_world chula_pat14_6, which
    mujoco_omnibot.launch.py sets from world:=NAME.

From the spawn, the script then:

  1. opens the right gripper,
  2. drives with Nav2 to a stand-off pose on the approach side of the bottle, heading
     along the approach, with the bottle ahead and to the right, and sends the left
     arm to its standby pose, clear of the base when the lift goes down,
  3. sends whole_body_right to a pre-grasp pose: the hand at the bottle's centre height,
     pregrasp m back from the grasp pose along the line from the base to the bottle,
     pointing at the bottle, with the wrist camera up. The commander moves the base
     only if the arm cannot reach that pose from where the base stands,
  4. moves the hand to the grasp pose in a straight line on right_arm_lift (whole-body
     groups do not take Cartesian paths), with the gripper allowed to touch the octomap,
  5. closes the gripper and reads the commander's grasp verdict (grasped, gripper_gap),
  6. lifts the hand lift m in a straight line on right_arm_lift and holds it,
  7. checks the hold: the hand is still at the lifted target, a fresh finger reading is
     in the commander's grasp band, and the head camera's depth image shows the bottle
     above its old place and nothing left of it on the table.

A move passes when the commander reports success and the hand frame ends within
0.05 m of its target; the lift also needs the hand to rise within 0.02 m of the
commanded distance. A pass needs the camera to confirm the lift. Without camera
evidence (no image, no camera transform at the image stamp, or too few points of the
bottle before or of the table after) the result is inconclusive, unless
--accept-finger-only accepts the fingers and the hand alone as passed_finger_only.

The commander's grasp_scene_action and allow_gripper_vs_octomap are read before the
script changes them. On every exit it can handle (success, a failed step, an
exception, Ctrl-C or SIGTERM, but not SIGKILL), it first cancels a goal that is still
running and waits for its end, then restores both parameters and reads them back.

All goals are in the map frame. The bottle's pose is the one the world was exported
with, as if it came from memory. Prints one JSON report on every exit; rmw_zenoh logs
to stdout, so use --report to get a file that parses as JSON. Exit codes: 0 passed
(or passed_finger_only), 1 a step failed or an exception, 2 a simulation check or a
prerequisite failed and nothing was changed, 3 the proof is inconclusive, 4 the
cleanup failed or a goal's end is unknown (takes precedence), 130 interrupted with a
clean cleanup.
"""
import argparse
from collections import deque
import json
import math
import signal
import sys
import threading
import time
import traceback

from geometry_msgs.msg import PoseStamped
from my_robot_interfaces.action import ControlGripper, GoToHome, GoToPose
from nav2_msgs.action import NavigateToPose
import numpy as np
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from robot_bringup import sim_pick
from robot_bringup.sim_pick import (
    FAILED, GOAL_REJECTED, GOAL_TIMED_OUT, INCONCLUSIVE, INTERRUPTED, NOT_RUN, PASSED,
    STATUS_ABORTED, status_name, STATUS_SUCCEEDED,
)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_ros import Buffer, TransformException, TransformListener

COMMANDER = '/bimanual_commander'
NAVIGATOR = '/bt_navigator'
# mujoco_ros2_control's controller manager, in the simulator's process.
SIMULATOR = '/controller_manager'
# Commander parameters the pick changes, restored to their values from before it.
SNAPSHOT_PARAMETERS = ('allow_gripper_vs_octomap', 'grasp_scene_action')
FINGER_JOINT = 'openarm_right_finger_joint1'
HAND_LINK = 'openarm_right_hand'
# task_water_bottle_1 on the kitchen table, centre of its collider, in map. The world
# loader puts the spawn's map X and Y at the MuJoCo origin, so these are world
# coordinates too.
BOTTLE = (0.94, -5.57, 0.862)
# The table's near edge is 0.22 m from the bottle on this side (-y), and the nearest
# table leg is 0.24 m to the side. From the west the bottle is 0.32 m in, with a leg
# 0.15 m to the side.
APPROACH_YAW = math.pi / 2
# Nav2 goal, along and to the left of the approach line through the bottle. From there
# the right arm reaches the pre-grasp pose without the base: the bottle is 0.65 m ahead
# and 0.20 m to the right, and the base front is 0.13 m from the table edge. From 0.90 m
# straight behind the bottle, the whole-body goal moved and turned the base by up to
# 0.3 m and 0.7 rad, and once the collision monitor stopped it during the turn.
STANDOFF = 0.65
STANDOFF_LATERAL = 0.20
GRIPPER_OPEN = 0.044
GRIPPER_CLOSED = 0.0
BOTTLE_RADIUS = 0.034
BOTTLE_HALF_HEIGHT = 0.112
TABLE_TOP = 0.75
# Where the held bottle's centre is expected, along the hand's approach axis, and how
# far from it the camera check looks for the bottle.
HELD_DEPTH = 0.045
HELD_RADIUS = 0.04
# Largest distance of the hand frame from its target after a move. The hand ends 0.01 to
# 0.02 m off in a normal run.
HAND_TOLERANCE = 0.05
# Largest difference between the measured hand rise and the commanded lift.
LIFT_TOLERANCE = 0.02
DEPTH_TOPIC = '/zed_head/zed_node/depth/depth_registered'
INFO_TOPIC = '/zed_head/zed_node/depth/camera_info'
COLOR_TOPIC = '/zed_head/zed_node/rgb/color/rect/image'

# Wall-time bounds, in seconds.
CLOCK_WINDOW = 10.0
CLOCK_SAMPLES = 3
READY_TIMEOUT = 240.0
SERVICE_TIMEOUT = 10.0
GOAL_RESPONSE_TIMEOUT = 30.0
CANCEL_RESPONSE_TIMEOUT = 10.0
CANCEL_END_TIMEOUT = 30.0
TF_TIMEOUT = 3.0
IMAGE_TIMEOUT = 30.0
FINGER_TIMEOUT = 2.0
# Freshness. joint_states arrive at 60 Hz of simulated time, which at a real-time
# factor of 0.3 is every 0.06 s of wall time. A finger reading counts when its stamp is
# within FINGER_MAX_AGE of simulated time of the node clock and it arrived less than
# FINGER_MAX_WALL_AGE ago. A hand transform counts when its stamp is within
# TRANSFORM_MAX_AGE of the node clock and not older than the end of the move it checks.
FINGER_MAX_AGE = 0.5
FINGER_MAX_WALL_AGE = 1.0
TRANSFORM_MAX_AGE = 1.0


class Interrupted(BaseException):
    """SIGINT or SIGTERM arrived while the pick ran."""

    def __init__(self, name):
        super().__init__(name)
        self.name = name


class Stop(Exception):
    """The pick ends early with a result."""

    def __init__(self, result, message):
        super().__init__(message)
        self.result = result
        self.message = message


class UnresolvedGoal(Stop):
    """A goal's end could not be established; nothing else may be sent after it."""

    def __init__(self, message):
        super().__init__(FAILED, message)


class UsageError(Exception):
    pass


class SignalGuard:
    """Raise Interrupted on the first SIGINT or SIGTERM while armed; record the others."""

    def __init__(self):
        self.armed = True
        self.received = None
        self.ignored = []

    def __call__(self, signum, frame):
        name = signal.Signals(signum).name
        if not self.armed or self.received is not None:
            self.ignored.append(name)
            return
        self.received = name
        raise Interrupted(name)


def rotation_matrix(q):
    """Rotation matrix of a geometry_msgs Quaternion."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def hand_rpy(yaw, camera_up):
    """Roll, pitch, yaw of the hand frame for a side grasp along heading yaw.

    The gripper approaches along the hand's +z. With pitch +pi/2 the hand's +x, where
    the wrist camera bracket sits, points down and reaches 0.11 m below the hand, which
    at a bottle's centre height is the table top. Pitch -pi/2 with the yaw turned by pi
    points the same approach with the bracket up, which is also the arm's natural wrist
    configuration for a hand at table height.
    """
    if camera_up:
        return 0.0, -math.pi / 2, math.remainder(yaw + math.pi, 2.0 * math.pi)
    return 0.0, math.pi / 2, yaw


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def wait_done(future, timeout):
    """Wait up to timeout wall seconds for future; return whether it is done."""
    deadline = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() >= deadline:
            return False
        time.sleep(0.02)
    return True


def parameter_value(value):
    """Python value of an rcl_interfaces ParameterValue, None when not set."""
    return {ParameterType.PARAMETER_BOOL: value.bool_value,
            ParameterType.PARAMETER_INTEGER: value.integer_value,
            ParameterType.PARAMETER_DOUBLE: value.double_value,
            ParameterType.PARAMETER_STRING: value.string_value}.get(value.type)


def describe_cancel(response):
    """Describe an action_msgs CancelGoal response as the server gave it."""
    if response is None:
        return f'cancel not answered within {CANCEL_RESPONSE_TIMEOUT:.0f} s'
    if response.return_code == 0:
        return 'cancel accepted' if response.goals_canceling else 'cancel answered, no goal'
    return {1: 'cancel rejected', 2: 'cancel: unknown goal',
            3: 'cancel: goal already ended'}.get(
        response.return_code, f'cancel return code {response.return_code}')


class GoalRecord:
    """One action goal, kept from its request to its terminal status."""

    def __init__(self, label):
        self.label = label
        self.lock = threading.Lock()
        self.send_future = None
        self.handle = None
        self.result_future = None
        self.cancel_future = None
        self.abandoned = False

    def attach(self, handle):
        """Keep the goal handle, request the result, and cancel an abandoned goal.

        Runs from the goal response's done callback and from the waiting thread; the
        caller holds self.lock.
        """
        self.handle = handle
        if not handle.accepted:
            return
        if self.result_future is None:
            self.result_future = handle.get_result_async()
        if self.abandoned and self.cancel_future is None:
            self.cancel_future = handle.cancel_goal_async()

    def cancel(self):
        """Mark the goal abandoned and cancel it once it is accepted."""
        with self.lock:
            self.abandoned = True
            if self.handle is not None:
                self.attach(self.handle)

    def ended(self):
        """Whether the goal is known to be over: rejected or with a terminal status."""
        with self.lock:
            if self.handle is not None and not self.handle.accepted:
                return True
            return self.result_future is not None and self.result_future.done()


class Outcome:
    """How one goal ended: a GoalStatus, GOAL_REJECTED or GOAL_TIMED_OUT."""

    def __init__(self, end, result=None, note='', end_ns=None):
        self.end = end
        self.result = result
        self.note = note
        self.end_ns = end_ns

    def describe(self):
        return status_name(self.end) + (f' ({self.note})' if self.note else '')


class PickClient(Node):
    """Sends the pick's goals and watches the clock, the fingers and the head camera."""

    def __init__(self):
        super().__init__('sim_bottle_pick')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.navigate = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.go_to_pose = ActionClient(self, GoToPose, '/go_to_pose')
        self.gripper = ActionClient(self, ControlGripper, '/control_gripper')
        self.go_to_home = ActionClient(self, GoToHome, '/go_to_home')
        self.parameter_clients = {}
        for node in (COMMANDER, NAVIGATOR, SIMULATOR):
            self.parameter_clients[node, 'get'] = self.create_client(
                GetParameters, f'{node}/get_parameters')
        self.parameter_clients[COMMANDER, 'set'] = self.create_client(
            SetParameters, f'{COMMANDER}/set_parameters')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.lock = threading.Lock()
        self.goals = []
        # Set before the first SetParameters request to the commander.
        self.changed_commander = False
        self.clock_samples = deque(maxlen=200)
        self.finger = (None, None, None)
        self.depth = None
        self.info = None
        self.color = None
        self.create_subscription(
            Clock, '/clock', self.on_clock,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(JointState, '/joint_states', self.on_joints, 50)
        self.create_subscription(Image, DEPTH_TOPIC, self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, INFO_TOPIC, self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, COLOR_TOPIC, self.on_color, qos_profile_sensor_data)

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    def on_clock(self, msg):
        with self.lock:
            self.clock_samples.append((time.monotonic(), stamp_ns(msg.clock)))

    def on_joints(self, msg):
        if FINGER_JOINT in msg.name:
            index = msg.name.index(FINGER_JOINT)
            if index < len(msg.position):
                with self.lock:
                    self.finger = (msg.position[index], stamp_ns(msg.header.stamp),
                                   time.monotonic())

    def on_depth(self, msg):
        with self.lock:
            self.depth = msg

    def on_info(self, msg):
        with self.lock:
            self.info = msg

    def on_color(self, msg):
        with self.lock:
            self.color = msg

    def clock_problem(self, window):
        """Why /clock does not advance within window wall seconds from now, or None."""
        start = time.monotonic()
        while True:
            with self.lock:
                times = [ns for wall, ns in self.clock_samples if wall >= start]
            problem = sim_pick.clock_problem(times, CLOCK_SAMPLES)
            if problem is None:
                return None
            if time.monotonic() - start >= window:
                return f'{problem} within {window:.0f} s of wall time'
            time.sleep(0.1)

    def missing_prerequisites(self, timeout):
        """Wait for the servers, the finger reading and TF; return what is missing."""
        deadline = time.monotonic() + timeout
        while True:
            checks = {
                '/navigate_to_pose action server': self.navigate.server_is_ready(),
                '/go_to_pose action server': self.go_to_pose.server_is_ready(),
                '/control_gripper action server': self.gripper.server_is_ready(),
                '/go_to_home action server': self.go_to_home.server_is_ready(),
                f'{FINGER_JOINT} in /joint_states': self.finger[0] is not None,
                'map -> base_footprint transform':
                    self.buffer.can_transform('map', 'base_footprint', Time()),
            }
            for (node, kind), client in self.parameter_clients.items():
                checks[f'{node}/{kind}_parameters service'] = client.service_is_ready()
            missing = [name for name, ready in checks.items() if not ready]
            if not missing or time.monotonic() >= deadline:
                return missing
            time.sleep(0.5)

    def call(self, client, request):
        future = client.call_async(request)
        if not wait_done(future, SERVICE_TIMEOUT):
            client.remove_pending_request(future)
            return None
        return future.result()

    def get_parameters(self, node, names):
        """{name: value or None when not set} from node, or None if it did not answer."""
        response = self.call(self.parameter_clients[node, 'get'],
                             GetParameters.Request(names=list(names)))
        if response is None or len(response.values) != len(names):
            return None
        return {name: parameter_value(value) for name, value in zip(names, response.values)}

    def set_commander(self, **values):
        """Set commander parameters. Returns (all accepted, why not)."""
        parameters = []
        for name, value in values.items():
            if isinstance(value, bool):
                parameter_value_msg = ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                                     bool_value=value)
            else:
                parameter_value_msg = ParameterValue(type=ParameterType.PARAMETER_STRING,
                                                     string_value=value)
            parameters.append(ParameterMsg(name=name, value=parameter_value_msg))
        self.changed_commander = True
        response = self.call(self.parameter_clients[COMMANDER, 'set'],
                             SetParameters.Request(parameters=parameters))
        if response is None:
            return False, f'no response within {SERVICE_TIMEOUT:.0f} s'
        refused = [f'{name}: {result.reason}' for name, result in zip(values, response.results)
                   if not result.successful]
        if refused or len(response.results) != len(values):
            return False, 'refused ' + '; '.join(refused)
        return True, None

    def run_goal(self, client, goal, label, timeout):
        """Send a goal and wait for its end. Returns an Outcome.

        A goal without a result within timeout is canceled, and the call waits for the
        server's answer and the goal's terminal status. Raises UnresolvedGoal when the
        goal's end cannot be established, so nothing else is sent after it.
        """
        record = GoalRecord(label)
        self.goals.append(record)
        record.send_future = client.send_goal_async(goal)

        def on_response(future):
            # A goal accepted after the wait below gave up is canceled at once.
            try:
                handle = future.result()
            except Exception:
                return
            with record.lock:
                record.attach(handle)

        record.send_future.add_done_callback(on_response)
        if not wait_done(record.send_future, GOAL_RESPONSE_TIMEOUT):
            record.cancel()
            raise UnresolvedGoal(f'{label}: no goal response within '
                                 f'{GOAL_RESPONSE_TIMEOUT:.0f} s; canceled if accepted later')
        try:
            handle = record.send_future.result()
        except Exception as error:
            raise UnresolvedGoal(f'{label}: goal request failed ({error})') from error
        with record.lock:
            record.attach(handle)
        if not handle.accepted:
            return Outcome(GOAL_REJECTED, end_ns=self.now_ns())
        if wait_done(record.result_future, timeout):
            response = record.result_future.result()
            return Outcome(response.status, response.result, end_ns=self.now_ns())
        cancel, response = self.stop_goal(record)
        if response is None:
            raise UnresolvedGoal(f'{label}: no result within {timeout:.0f} s; {cancel}; no '
                                 f'terminal status within {CANCEL_END_TIMEOUT:.0f} s')
        return Outcome(GOAL_TIMED_OUT, response.result,
                       f'no result within {timeout:.0f} s; {cancel}; ended '
                       f'{status_name(response.status)}', self.now_ns())

    def stop_goal(self, record):
        """Cancel an accepted goal and wait for its terminal status.

        Returns (the server's answer to the cancel, the result response or None).
        """
        record.cancel()
        cancel = None
        if wait_done(record.cancel_future, CANCEL_RESPONSE_TIMEOUT):
            cancel = record.cancel_future.result()
        response = None
        if wait_done(record.result_future, CANCEL_END_TIMEOUT):
            response = record.result_future.result()
        return describe_cancel(cancel), response

    def resolve(self, record):
        """Bring a goal that has not ended to a known end. Returns a report entry or None."""
        if record.send_future is None or record.ended():
            return None
        entry = {'goal': record.label}
        if not wait_done(record.send_future, CANCEL_RESPONSE_TIMEOUT):
            record.cancel()
            entry.update(resolved=False, end='unknown: no goal response')
            return entry
        try:
            handle = record.send_future.result()
        except Exception as error:
            entry.update(resolved=False, end=f'unknown: goal request failed ({error})')
            return entry
        with record.lock:
            record.attach(handle)
        if not handle.accepted:
            entry.update(resolved=True, end=GOAL_REJECTED)
            return entry
        cancel, response = self.stop_goal(record)
        entry['cancel'] = cancel
        if response is None:
            entry.update(resolved=False, end='unknown: no terminal status within '
                                             f'{CANCEL_END_TIMEOUT:.0f} s')
        else:
            entry.update(resolved=True, end=status_name(response.status))
        return entry

    def fresh_finger(self):
        """(finger position, None) from a fresh joint state, or (None, why not)."""
        deadline = time.monotonic() + FINGER_TIMEOUT
        while True:
            with self.lock:
                value, stamp, received = self.finger
            problem = sim_pick.reading_age_problem(
                stamp, self.now_ns(), received, time.monotonic(), FINGER_MAX_AGE,
                FINGER_MAX_WALL_AGE)
            if problem is None:
                return value, None
            if time.monotonic() >= deadline:
                return None, f'finger reading: {problem}'
            time.sleep(0.02)

    def latest_transform(self, frame, not_before_ns=None):
        """Latest fresh transform of frame in map, or None within TF_TIMEOUT."""
        deadline = time.monotonic() + TF_TIMEOUT
        while True:
            try:
                transform = self.buffer.lookup_transform('map', frame, Time())
                stamp = stamp_ns(transform.header.stamp)
                if abs(self.now_ns() - stamp) <= TRANSFORM_MAX_AGE * 1e9 and \
                        (not_before_ns is None or stamp >= not_before_ns):
                    return transform.transform
            except TransformException:
                pass
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.05)

    def transform_at(self, frame, stamp):
        """Transform of frame in map at stamp, or None within TF_TIMEOUT."""
        deadline = time.monotonic() + TF_TIMEOUT
        while True:
            try:
                return self.buffer.lookup_transform('map', frame, stamp).transform
            except TransformException:
                if time.monotonic() >= deadline:
                    return None
            time.sleep(0.05)

    def pose_in_map(self, frame, not_before_ns=None):
        """(x, y, z, yaw) of frame in map, or None."""
        transform = self.latest_transform(frame, not_before_ns)
        if transform is None:
            return None
        t = transform.translation
        return (t.x, t.y, t.z, yaw_of(transform.rotation))

    def hand_in_map(self, not_before_ns=None):
        """(position, approach axis) of the hand in map, or None."""
        transform = self.latest_transform(HAND_LINK, not_before_ns)
        if transform is None:
            return None
        t = transform.translation
        return np.array([t.x, t.y, t.z]), rotation_matrix(transform.rotation)[:, 2]

    def navigate_to(self, x, y, yaw, timeout):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        return self.run_goal(self.navigate, goal, 'navigate', timeout)

    def move_hand(self, label, group, position, rpy, cartesian, timeout):
        goal = GoToPose.Goal(
            group_name=group, x=float(position[0]), y=float(position[1]),
            z=float(position[2]), roll=float(rpy[0]), pitch=float(rpy[1]), yaw=float(rpy[2]),
            cartesian_path=cartesian, frame_id='map')
        return self.run_goal(self.go_to_pose, goal, label, timeout)

    def move_to_named(self, label, group, pose_name, timeout):
        goal = GoToHome.Goal(group_name=group, pose_name=pose_name)
        return self.run_goal(self.go_to_home, goal, label, timeout)

    def control_gripper(self, label, position, timeout):
        goal = ControlGripper.Goal(group_name='right_gripper', position=float(position))
        return self.run_goal(self.gripper, goal, label, timeout)

    def depth_points(self):
        """Return the points of the next head depth image in map, or why there are none.

        The image must be stamped at or after the node clock at the call, and its points
        are transformed with the camera pose at the image stamp. Returns (N x 3 array,
        None) or (None, why not).
        """
        since = self.now_ns()
        deadline = time.monotonic() + IMAGE_TIMEOUT
        while True:
            with self.lock:
                image, info = self.depth, self.info
            if image is not None and info is not None and stamp_ns(image.header.stamp) >= since:
                break
            if time.monotonic() >= deadline:
                return None, (f'no depth image stamped after {since / 1e9:.3f} s within '
                              f'{IMAGE_TIMEOUT:.0f} s')
            time.sleep(0.05)
        if image.encoding == '32FC1':
            depth = np.frombuffer(image.data, dtype=np.float32)
        elif image.encoding == '16UC1':
            depth = np.frombuffer(image.data, dtype=np.uint16).astype(np.float32) / 1000.0
        else:
            return None, f'depth encoding {image.encoding} is not handled'
        depth = depth.reshape(image.height, image.step // (4 if image.encoding == '32FC1'
                                                           else 2))[:, :image.width]
        transform = self.transform_at(image.header.frame_id, Time.from_msg(image.header.stamp))
        if transform is None:
            return None, (f'no transform of {image.header.frame_id} in map at the image stamp '
                          f'{stamp_ns(image.header.stamp) / 1e9:.3f} s')
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        rows, columns = np.nonzero(np.isfinite(depth) & (depth > 0.05))
        z = depth[rows, columns]
        camera = np.stack([(columns - cx) * z / fx, (rows - cy) * z / fy, z], axis=1)
        t = transform.translation
        return camera @ rotation_matrix(transform.rotation).T + np.array([t.x, t.y, t.z]), None

    def save_color(self, path):
        """Write the latest head color image to path. Returns why not, or None."""
        with self.lock:
            image = self.color
        if image is None:
            return 'no color image'
        channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}.get(image.encoding)
        if channels is None:
            return f'color encoding {image.encoding} is not handled'
        try:
            import cv2
            pixels = np.frombuffer(image.data, dtype=np.uint8).reshape(
                image.height, image.step)[:, :image.width * channels].reshape(
                image.height, image.width, channels)
            conversion = {'rgb8': cv2.COLOR_RGB2BGR, 'rgba8': cv2.COLOR_RGBA2BGR,
                          'bgra8': cv2.COLOR_BGRA2BGR}.get(image.encoding)
            written = cv2.imwrite(path, cv2.cvtColor(pixels, conversion)
                                  if conversion is not None else pixels)
        except Exception as error:
            return f'could not write {path}: {error}'
        return None if written else f'could not write {path}'


def count_in_cylinder(points, centre_xy, radius, z_low, z_high):
    if points is None:
        return None
    inside = (np.hypot(points[:, 0] - centre_xy[0], points[:, 1] - centre_xy[1]) <= radius) \
        & (points[:, 2] >= z_low) & (points[:, 2] <= z_high)
    return int(np.count_nonzero(inside))


def move_status(outcome):
    """(success, status) of a GoToPose or GoToHome goal."""
    result = outcome.result
    ok = outcome.end == STATUS_SUCCEEDED and result is not None and result.success
    detail = getattr(result, 'status', None)
    if detail is None and result is not None:
        detail = 'reached' if result.success else 'failed'
    return ok, outcome.describe() + (f': {detail}' if detail else '')


class PickRun:
    """One pick: the simulation check, the sequence, the cleanup and the report."""

    def __init__(self, args, guard):
        self.args = args
        self.guard = guard
        self.node = None
        self.snapshot = None
        self.started = time.monotonic()
        self.result = None
        self.cleanup_ok = True
        self.goals_resolved = True
        self.report = {'result': None, 'passed': False, 'bottle': list(args.bottle),
                       'steps': []}

    def execute(self):
        """Run the pick and the cleanup. Returns the exit code; never raises."""
        try:
            try:
                self.result = self.pick()
            except Stop as stop:
                self.result = stop.result
                self.report['error'] = stop.message
            except Exception as error:
                self.result = sim_pick.ERROR
                self.report['error'] = f'{type(error).__name__}: {error}'
                self.report['traceback'] = traceback.format_exc()
        except Interrupted as interrupt:
            self.interrupted(interrupt)
        finally:
            while True:
                try:
                    # A signal can still arrive before the guard is disarmed; the guard
                    # raises only once, so the second pass runs the whole cleanup.
                    self.guard.armed = False
                    self.cleanup()
                    break
                except Interrupted as interrupt:
                    self.interrupted(interrupt)
        code = sim_pick.exit_code(self.result, cleanup_ok=self.cleanup_ok,
                                  goals_resolved=self.goals_resolved,
                                  interrupted=self.guard.received is not None)
        self.report['result'] = self.result
        self.report['passed'] = self.result == PASSED and code == sim_pick.EXIT_PASSED
        self.report['exit_code'] = code
        if self.guard.ignored:
            self.report['signals_ignored'] = list(self.guard.ignored)
        return code

    def interrupted(self, interrupt):
        self.result = INTERRUPTED
        self.report['signal'] = interrupt.name
        self.report['error'] = f'interrupted by {interrupt.name}'

    def step(self, name, ok, status, failure=FAILED, **extra):
        """Record a step; if it did not succeed, stop the pick with result failure."""
        entry = {'step': name, 'success': bool(ok), 'status': status,
                 'wall_seconds': round(time.monotonic() - self.started, 1), **extra}
        self.node.get_logger().info(json.dumps(entry, default=str))
        self.report['steps'].append(entry)
        if not ok:
            raise Stop(failure, f'{name}: {status}')

    def hand_step(self, name, outcome, target, hand, problem=None, **extra):
        """Record a move that passes only if the hand ends within HAND_TOLERANCE of target.

        The commander reports success when the joint trajectory finishes. If the arm
        caught on the table on the way, the hand ends far from the target anyway.
        """
        ok, status = move_status(outcome)
        error = None if hand is None else float(np.linalg.norm(hand[0] - target))
        problems = [problem] if problem else []
        if error is None:
            problems.insert(0, 'the hand pose is unknown')
        elif error > HAND_TOLERANCE:
            problems.insert(0, 'the hand is not at the target')
        if ok and problems:
            ok = False
            status = f'{status}, but ' + '; '.join(problems)
        self.step(name, ok, status, **extra,
                  hand=None if hand is None else hand[0].round(4).tolist(),
                  hand_error_m=None if error is None else round(error, 4))

    def start_node(self):
        rclpy.init(args=sys.argv, signal_handler_options=SignalHandlerOptions.NO)
        self.node = PickClient()
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(self.node)
        threading.Thread(target=executor.spin, daemon=True).start()

    def check_simulation(self):
        """Fail closed unless the graph is the Chula MuJoCo simulation. Changes nothing."""
        node = self.node
        gate = self.report['gate'] = {}
        # First, so that a graph without a simulator fails within CLOCK_WINDOW.
        clock = node.clock_problem(CLOCK_WINDOW)
        if clock is not None:
            gate['failures'] = [clock]
            raise Stop(NOT_RUN, f'simulation check: {clock}')
        missing = node.missing_prerequisites(READY_TIMEOUT)
        if missing:
            raise Stop(NOT_RUN, 'not ready: ' + ', '.join(missing))
        clock = node.clock_problem(CLOCK_WINDOW)
        use_sim_time = {}
        for name in (COMMANDER, NAVIGATOR):
            values = node.get_parameters(name, ['use_sim_time'])
            use_sim_time[name] = None if values is None else values['use_sim_time']
        values = node.get_parameters(SIMULATOR, ['mujoco_world'])
        world = None if values is None else values['mujoco_world']
        failures = sim_pick.gate_failures(clock, use_sim_time, world)
        gate.update(clock='advancing' if clock is None else clock,
                    use_sim_time=use_sim_time, mujoco_world=world, failures=failures)
        if failures:
            raise Stop(NOT_RUN, 'simulation check: ' + '; '.join(failures))

    def take_snapshot(self):
        values = self.node.get_parameters(COMMANDER, SNAPSHOT_PARAMETERS)
        if values is None or not isinstance(values['allow_gripper_vs_octomap'], bool) or \
                not isinstance(values['grasp_scene_action'], str):
            raise Stop(NOT_RUN, f'could not read {", ".join(SNAPSHOT_PARAMETERS)} from '
                                f'{COMMANDER}: {values}')
        self.snapshot = values
        self.report['snapshot'] = dict(values)

    def set_commander(self, name, **values):
        ok, why = self.node.set_commander(**values)
        self.step(name, ok, 'set ' + ', '.join(f'{key} {value}' for key, value in values.items())
                  + ('' if ok else f': {why}'))

    def pick(self):
        args = self.args
        self.start_node()
        node = self.node
        self.check_simulation()
        self.take_snapshot()
        names = ('grasp_detect_min', 'grasp_detect_max')
        values = node.get_parameters(COMMANDER, names) or {}
        band = self.report['grasp_band'] = [values.get(name) for name in names]
        self.started = time.monotonic()
        bottle = np.array(args.bottle, dtype=float)

        # The grasp box the commander can attach would collide with the table top in the
        # octomap at the start of the lift, so the planning scene is left alone.
        self.set_commander('commander_params', grasp_scene_action='none',
                           allow_gripper_vs_octomap=False)

        outcome = node.control_gripper('open', GRIPPER_OPEN, 120.0)
        result = outcome.result
        self.step('open', outcome.end == STATUS_SUCCEEDED and result is not None
                  and result.success,
                  outcome.describe() + ('' if result is None else f': {result.status}'),
                  finger=node.finger[0])

        if not args.skip_nav:
            self.navigate(bottle)

        # Seen from the stand-off, before the arm moves in front of the camera.
        time.sleep(2.0)
        before, before_problem = node.depth_points()
        images = self.report['images'] = {'before': before_problem}
        if args.image_prefix:
            images['before_saved'] = node.save_color(f'{args.image_prefix}_before.png')

        # The straight-line moves of right_arm_lift can lower the lift by over 0.1 m. The
        # left arm hangs from the same column, and at home its fingers then reach the base.
        ok, status = move_status(node.move_to_named('tuck_left', 'left_arm', 'standby', 300.0))
        self.step('tuck_left', ok, status)

        base = node.pose_in_map('base_footprint')
        if base is None:
            raise Stop(FAILED, 'plan: no fresh map -> base_footprint transform')
        line = bottle[:2] - np.array(base[:2])
        yaw = math.atan2(line[1], line[0])
        approach = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        grasp = bottle + np.array([0.0, 0.0, args.height_offset]) - args.depth * approach
        pregrasp = grasp - args.pregrasp * approach
        rpy = hand_rpy(yaw, not args.camera_down)
        self.report['grasp'] = {
            'approach_yaw': round(yaw, 4), 'rpy': [round(v, 4) for v in rpy],
            'pregrasp': pregrasp.round(4).tolist(), 'grasp': grasp.round(4).tolist(),
            'base_at_plan': [round(v, 4) for v in base]}

        outcome = node.move_hand('pregrasp', 'whole_body_right', pregrasp, rpy, False, 900.0)
        self.hand_step('pregrasp', outcome, pregrasp, node.hand_in_map(outcome.end_ns),
                       base=node.pose_in_map('base_footprint', outcome.end_ns))

        # The fingers have to enter the bottle's and the table's voxels.
        self.set_commander('allow_octomap', allow_gripper_vs_octomap=True)
        time.sleep(1.0)
        outcome = node.move_hand('approach', 'right_arm_lift', grasp, rpy, True, 600.0)
        self.hand_step('approach', outcome, grasp, node.hand_in_map(outcome.end_ns))

        # The commander computes grasped from the settled gap, apart from the execution
        # result: a close that stalls on the bottle ends ABORTED with grasped true.
        outcome = node.control_gripper('close', GRIPPER_CLOSED, 120.0)
        result = outcome.result
        grasped = outcome.end in (STATUS_SUCCEEDED, STATUS_ABORTED) and result is not None \
            and result.grasped
        self.step('close', grasped,
                  outcome.describe() + ('' if result is None else f': {result.status}'),
                  grasped=grasped,
                  gripper_gap=None if result is None else round(result.gripper_gap, 4))

        hand_before_lift = node.hand_in_map(outcome.end_ns)
        lifted = grasp + np.array([0.0, 0.0, args.lift])
        outcome = node.move_hand('lift', 'right_arm_lift', lifted, rpy, True, 600.0)
        hand = node.hand_in_map(outcome.end_ns)
        rise = None if hand is None or hand_before_lift is None \
            else float(hand[0][2] - hand_before_lift[0][2])
        self.hand_step('lift', outcome, lifted, hand,
                       sim_pick.lift_problem(rise, args.lift, LIFT_TOLERANCE),
                       hand_rise_m=None if rise is None else round(rise, 4))

        time.sleep(args.hold)
        return self.check_hold(bottle, before, band, lifted)

    def navigate(self, bottle):
        args = self.args
        heading = args.approach_yaw
        goal = (bottle[0] - args.standoff * math.cos(heading)
                - args.standoff_lateral * math.sin(heading),
                bottle[1] - args.standoff * math.sin(heading)
                + args.standoff_lateral * math.cos(heading))
        attempts = []
        for attempt in range(args.nav_attempts):
            outcome = self.node.navigate_to(goal[0], goal[1], heading, 900.0)
            result = outcome.result
            error_code = None if result is None else result.error_code
            verdict, why = sim_pick.classify_navigation(outcome.end, error_code)
            attempts.append({'end': outcome.describe(), 'error_code': error_code,
                             'error_msg': None if result is None else result.error_msg,
                             'verdict': verdict})
            if verdict != sim_pick.NAV_RETRY:
                break
            if attempt + 1 < args.nav_attempts:
                self.node.get_logger().warn(f'navigation {why}, sending the goal again')
        else:
            why = f'{why}; gave up after {args.nav_attempts} attempts'
        self.step('navigate', verdict == sim_pick.NAV_SUCCEEDED, why,
                  goal=[round(v, 3) for v in goal],
                  base=self.node.pose_in_map('base_footprint', outcome.end_ns),
                  attempts=attempts)

    def check_hold(self, bottle, before, band, lifted):
        args, node = self.args, self.node
        finger, finger_problem = node.fresh_finger()
        after, after_problem = node.depth_points()
        self.report['images']['after'] = after_problem
        if args.image_prefix:
            self.report['images']['after_saved'] = node.save_color(
                f'{args.image_prefix}_after.png')
        hand = node.hand_in_map()
        hand_error = None if hand is None else float(np.linalg.norm(hand[0] - lifted))
        # The tapered fingers push the bottle toward the palm, so its centre ends up about
        # 0.045 m in front of the hand frame. The camera sees the side of the bottle that
        # faces it, BOTTLE_RADIUS from that centre. Up to 0.04 m from it, and above the
        # bottle's old top, nothing of the hand reaches: the palm stays below, and the wrist
        # camera bracket is behind the hand frame.
        held_xy = bottle[:2] if hand is None else (hand[0] + HELD_DEPTH * hand[1])[:2]
        top = bottle[2] + BOTTLE_HALF_HEIGHT
        camera = {
            # The bottle's lower part at its old place on the table.
            'old_place_lower_before': count_in_cylinder(
                before, bottle[:2], BOTTLE_RADIUS + 0.01, TABLE_TOP + 0.02, bottle[2] - 0.02),
            'old_place_lower_after': count_in_cylinder(
                after, bottle[:2], BOTTLE_RADIUS + 0.01, TABLE_TOP + 0.02, bottle[2] - 0.02),
            # Above the bottle's old top, where only a lifted bottle can be.
            'above_old_top_before': count_in_cylinder(
                before, held_xy, HELD_RADIUS, top + 0.015, top + args.lift + 0.01),
            'above_old_top_after': count_in_cylinder(
                after, held_xy, HELD_RADIUS, top + 0.015, top + args.lift + 0.01),
            # If the whole-body goal turned the base, the bottle's old place can leave the
            # head camera's view. The table top around it tells whether the camera still
            # sees it.
            'table_near_old_place_after': count_in_cylinder(
                after, bottle[:2], 0.15, TABLE_TOP - 0.01, TABLE_TOP + 0.01),
        }
        lifted_seen, camera_why = sim_pick.camera_verdict(camera)
        camera['bottle_left_table'] = lifted_seen
        finger_in_band = None if finger_problem else sim_pick.in_band(finger, band)
        hand_at_target = None if hand_error is None else hand_error <= HAND_TOLERANCE
        result = sim_pick.hold_verdict(
            finger_in_band=finger_in_band, hand_at_target=hand_at_target,
            camera=lifted_seen, accept_finger_only=args.accept_finger_only)
        notes = [f'camera: {camera_why}']
        if finger_problem:
            notes.append(finger_problem)
        elif not finger_in_band:
            notes.append('finger gap out of the grasp band')
        if hand_error is None:
            notes.append('no fresh hand transform')
        elif not hand_at_target:
            notes.append('the hand is not at the lifted target')
        self.step('hold', result not in (FAILED, INCONCLUSIVE),
                  f'{result}: ' + '; '.join(notes), failure=result,
                  finger=None if finger is None else round(finger, 4),
                  hand_error_m=None if hand_error is None else round(hand_error, 4),
                  camera=camera)
        return result

    def cleanup(self):
        """Cancel open goals, then restore the commander parameters and read them back."""
        entry = self.report['cleanup'] = {'goals': [], 'restored': None, 'verified': None,
                                          'errors': []}
        if self.node is None:
            return
        for record in self.node.goals:
            try:
                state = self.node.resolve(record)
            except Exception as error:
                state = {'goal': record.label, 'resolved': False,
                         'end': f'unknown: {type(error).__name__}: {error}'}
            if state is not None:
                entry['goals'].append(state)
                if not state['resolved']:
                    self.goals_resolved = False
                    entry['errors'].append(f'{state["goal"]}: {state["end"]}')
        # Restoring turns the gripper's octomap exemption off again, the conservative
        # direction, so it runs even if a goal's end is unknown.
        if not self.node.changed_commander or self.snapshot is None:
            return
        try:
            ok, why = self.node.set_commander(**self.snapshot)
            entry['restored'] = ok
            if not ok:
                self.cleanup_ok = False
                entry['errors'].append(f'restore: {why}')
            values = self.node.get_parameters(COMMANDER, SNAPSHOT_PARAMETERS)
            entry['read_back'] = values
            entry['verified'] = values == self.snapshot
            if not entry['verified']:
                self.cleanup_ok = False
                entry['errors'].append(f'read back {values}, expected {self.snapshot}')
        except Exception as error:
            self.cleanup_ok = False
            entry['errors'].append(f'restore: {type(error).__name__}: {error}')


class ReportingParser(argparse.ArgumentParser):
    def error(self, message):
        raise UsageError(message)


def parse_arguments(argv):
    parser = ReportingParser(description=__doc__,
                             formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--bottle', type=float, nargs=3, default=BOTTLE, metavar=('X', 'Y', 'Z'),
                        help='bottle centre in map (default: task_water_bottle_1)')
    parser.add_argument('--approach-yaw', type=float, default=APPROACH_YAW,
                        help='heading of the approach to the bottle in map, rad')
    parser.add_argument('--standoff', type=float, default=STANDOFF,
                        help='distance of the Nav2 goal behind the bottle along the '
                             'approach heading, m')
    parser.add_argument('--standoff-lateral', type=float, default=STANDOFF_LATERAL,
                        help='offset of the Nav2 goal to the left of the approach line, m')
    parser.add_argument('--pregrasp', type=float, default=0.10,
                        help='distance of the pre-grasp pose behind the grasp pose, m')
    parser.add_argument('--depth', type=float, default=0.06,
                        help='distance of the hand frame from the bottle centre at the '
                             'grasp, along the approach, m')
    parser.add_argument('--height-offset', type=float, default=0.0,
                        help='grasp height above the bottle centre, m')
    parser.add_argument('--camera-down', action='store_true',
                        help='point the wrist camera down instead of up')
    parser.add_argument('--lift', type=float, default=0.10, help='lift height, m')
    parser.add_argument('--hold', type=float, default=5.0,
                        help='wall seconds to hold the bottle after the lift before the check')
    parser.add_argument('--skip-nav', action='store_true',
                        help='start from where the robot stands')
    parser.add_argument('--nav-attempts', type=int, default=3,
                        help='Nav2 goals to send before the drive counts as failed; only '
                             'error codes 105, 107 and 207 send the goal again')
    parser.add_argument('--accept-finger-only', action='store_true',
                        help='pass as passed_finger_only when the head camera cannot decide '
                             'but the fingers and the hand show the bottle held')
    parser.add_argument('--image-prefix',
                        help='save head camera images to PREFIX_before.png and '
                             'PREFIX_after.png')
    parser.add_argument('--report', help='also write the JSON report to this file')
    args = parser.parse_args(argv)
    if args.hold < 0.0:
        parser.error('--hold must not be negative')
    if args.lift <= 0.0:
        parser.error('--lift must be positive')
    if args.nav_attempts < 1:
        parser.error('--nav-attempts must be at least 1')
    return args


def write_report(report, path, code):
    text = json.dumps(report, indent=2, default=str)
    print(text, flush=True)
    if path:
        try:
            with open(path, 'w') as output:
                output.write(text + '\n')
        except OSError as error:
            print(f'could not write the report to {path}: {error}', file=sys.stderr)
    rclpy.try_shutdown()
    sys.exit(code)


def main():
    argv = remove_ros_args(sys.argv)[1:]
    try:
        args = parse_arguments(argv)
    except UsageError as error:
        print(f'usage error: {error}', file=sys.stderr)
        report_path = None
        if '--report' in argv[:-1]:
            report_path = argv[argv.index('--report') + 1]
        write_report({'result': NOT_RUN, 'passed': False, 'error': f'usage: {error}',
                      'exit_code': sim_pick.EXIT_NOT_RUN}, report_path, sim_pick.EXIT_NOT_RUN)
    guard = SignalGuard()
    signal.signal(signal.SIGINT, guard)
    signal.signal(signal.SIGTERM, guard)
    run = PickRun(args, guard)
    code = run.execute()
    write_report(run.report, args.report, code)


if __name__ == '__main__':
    main()
