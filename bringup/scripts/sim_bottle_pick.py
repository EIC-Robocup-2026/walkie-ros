#!/usr/bin/env python3
"""Scripted pick of a water bottle in the MuJoCo Chula PAT14-6 world.

Needs robot_bringup whole_body_demo.launch.py world:=chula_pat14_6 running. From the
spawn, the script:

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
  7. checks the hold: the finger gap is still in the commander's grasp band, and the
     head camera's depth image shows the bottle above its old place and nothing left
     of it on the table.

All goals are in the map frame. The bottle's pose is the one the world was exported
with, as if it came from memory. Prints one JSON report and exits non-zero if the pick
fails. rmw_zenoh logs to stdout, so use --report to get a file that parses as JSON.
"""
import argparse
import json
import math
import sys
import threading
import time

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
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_ros import Buffer, TransformException, TransformListener

COMMANDER = '/bimanual_commander'
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
DEPTH_TOPIC = '/zed_head/zed_node/depth/depth_registered'
INFO_TOPIC = '/zed_head/zed_node/depth/camera_info'
COLOR_TOPIC = '/zed_head/zed_node/rgb/color/rect/image'


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


class PickClient(Node):
    """Sends the pick's goals and watches the fingers and the head camera."""

    def __init__(self):
        super().__init__('sim_bottle_pick')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.navigate = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.go_to_pose = ActionClient(self, GoToPose, '/go_to_pose')
        self.gripper = ActionClient(self, ControlGripper, '/control_gripper')
        self.go_to_home = ActionClient(self, GoToHome, '/go_to_home')
        self.set_client = self.create_client(SetParameters, f'{COMMANDER}/set_parameters')
        self.get_client = self.create_client(GetParameters, f'{COMMANDER}/get_parameters')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.lock = threading.Lock()
        self.finger = None
        self.depth = None
        self.depth_count = 0
        self.info = None
        self.color = None
        self.create_subscription(JointState, '/joint_states', self.on_joints, 50)
        self.create_subscription(Image, DEPTH_TOPIC, self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, INFO_TOPIC, self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, COLOR_TOPIC, self.on_color, qos_profile_sensor_data)

    def on_joints(self, msg):
        if FINGER_JOINT in msg.name:
            index = msg.name.index(FINGER_JOINT)
            if index < len(msg.position):
                self.finger = msg.position[index]

    def on_depth(self, msg):
        with self.lock:
            self.depth = msg
            self.depth_count += 1

    def on_info(self, msg):
        self.info = msg

    def on_color(self, msg):
        with self.lock:
            self.color = msg

    def wait_until_ready(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.finger is not None and self.navigate.server_is_ready() and \
                    self.go_to_pose.server_is_ready() and self.gripper.server_is_ready() and \
                    self.go_to_home.server_is_ready() and \
                    self.set_client.service_is_ready() and \
                    self.buffer.can_transform('map', 'base_footprint', Time()):
                return True
            time.sleep(0.5)
        return False

    @staticmethod
    def wait_future(future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        return future.result() if future.done() else None

    def send_goal(self, client, goal, timeout):
        """Return (result, None), or (None, why there is no result).

        A goal without a result within the timeout is canceled, so it does not go on
        moving the robot after the script has given up on it.
        """
        accepted = self.wait_future(client.send_goal_async(goal), 30.0)
        if accepted is None:
            return None, 'no goal response within 30 s'
        if not accepted.accepted:
            return None, 'goal not accepted'
        result_future = accepted.get_result_async()
        response = self.wait_future(result_future, timeout)
        if response is None:
            self.wait_future(accepted.cancel_goal_async(), 10.0)
            self.wait_future(result_future, 60.0)
            return None, f'no result within {timeout:.0f} s, canceled'
        return response.result, None

    def set_commander(self, **values):
        """Set commander parameters. Returns True if every one was accepted."""
        parameters = []
        for name, value in values.items():
            if isinstance(value, bool):
                parameter_value = ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                                 bool_value=value)
            else:
                parameter_value = ParameterValue(type=ParameterType.PARAMETER_STRING,
                                                 string_value=value)
            parameters.append(ParameterMsg(name=name, value=parameter_value))
        response = self.wait_future(
            self.set_client.call_async(SetParameters.Request(parameters=parameters)), 10.0)
        return response is not None and all(result.successful for result in response.results)

    def commander_value(self, name):
        if not self.get_client.wait_for_service(timeout_sec=5.0):
            return None
        response = self.wait_future(
            self.get_client.call_async(GetParameters.Request(names=[name])), 10.0)
        if response is None or not response.values:
            return None
        value = response.values[0]
        return {ParameterType.PARAMETER_BOOL: value.bool_value,
                ParameterType.PARAMETER_DOUBLE: value.double_value,
                ParameterType.PARAMETER_STRING: value.string_value}.get(value.type)

    def pose_in_map(self, frame):
        """(x, y, z, yaw) of frame in map, or None."""
        try:
            transform = self.buffer.lookup_transform('map', frame, Time()).transform
        except TransformException:
            return None
        t = transform.translation
        return (t.x, t.y, t.z, yaw_of(transform.rotation))

    def hand_in_map(self):
        """(position, approach axis) of the hand in map, or None."""
        try:
            transform = self.buffer.lookup_transform('map', HAND_LINK, Time()).transform
        except TransformException:
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
        result, status = self.send_goal(self.navigate, goal, timeout)
        if result is None:
            return False, status
        # nav2_msgs NavigateToPose: error_code 0 is NONE.
        return result.error_code == 0, f'error_code {result.error_code} {result.error_msg}'

    def move_hand(self, group, position, rpy, cartesian, timeout):
        goal = GoToPose.Goal(
            group_name=group, x=float(position[0]), y=float(position[1]),
            z=float(position[2]), roll=float(rpy[0]), pitch=float(rpy[1]), yaw=float(rpy[2]),
            cartesian_path=cartesian, frame_id='map')
        result, status = self.send_goal(self.go_to_pose, goal, timeout)
        if result is None:
            return False, status
        return result.success, result.status

    def move_to_named(self, group, pose_name, timeout):
        goal = GoToHome.Goal(group_name=group, pose_name=pose_name)
        result, status = self.send_goal(self.go_to_home, goal, timeout)
        if result is None:
            return False, status
        return result.success, 'reached' if result.success else 'failed'

    def control_gripper(self, position, timeout):
        goal = ControlGripper.Goal(group_name='right_gripper', position=float(position))
        result, status = self.send_goal(self.gripper, goal, timeout)
        if result is None:
            return None, status
        return result, result.status

    def depth_points(self, timeout=30.0):
        """Points of the next head depth image, in map, as an N x 3 array, or None."""
        with self.lock:
            seen = self.depth_count
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                if self.depth_count > seen + 1 and self.info is not None:
                    image, info = self.depth, self.info
                    break
            time.sleep(0.1)
        else:
            return None
        if image.encoding == '32FC1':
            depth = np.frombuffer(image.data, dtype=np.float32)
        elif image.encoding == '16UC1':
            depth = np.frombuffer(image.data, dtype=np.uint16).astype(np.float32) / 1000.0
        else:
            self.get_logger().warning(f'depth encoding {image.encoding} is not handled')
            return None
        depth = depth.reshape(image.height, image.step // (4 if image.encoding == '32FC1'
                                                           else 2))[:, :image.width]
        try:
            transform = self.buffer.lookup_transform(
                'map', image.header.frame_id, Time()).transform
        except TransformException:
            return None
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        rows, columns = np.nonzero(np.isfinite(depth) & (depth > 0.05))
        z = depth[rows, columns]
        camera = np.stack([(columns - cx) * z / fx, (rows - cy) * z / fy, z], axis=1)
        t = transform.translation
        return camera @ rotation_matrix(transform.rotation).T + np.array([t.x, t.y, t.z])

    def save_color(self, path):
        with self.lock:
            image = self.color
        if image is None:
            return False
        import cv2
        channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}.get(image.encoding)
        if channels is None:
            return False
        pixels = np.frombuffer(image.data, dtype=np.uint8).reshape(
            image.height, image.step)[:, :image.width * channels].reshape(
            image.height, image.width, channels)
        conversion = {'rgb8': cv2.COLOR_RGB2BGR, 'rgba8': cv2.COLOR_RGBA2BGR,
                      'bgra8': cv2.COLOR_BGRA2BGR}.get(image.encoding)
        return bool(cv2.imwrite(path, cv2.cvtColor(pixels, conversion)
                                if conversion is not None else pixels))


def count_in_cylinder(points, centre_xy, radius, z_low, z_high):
    if points is None:
        return None
    inside = (np.hypot(points[:, 0] - centre_xy[0], points[:, 1] - centre_xy[1]) <= radius) \
        & (points[:, 2] >= z_low) & (points[:, 2] <= z_high)
    return int(np.count_nonzero(inside))


def main():
    parser = argparse.ArgumentParser(description=__doc__,
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
                        help='Nav2 goals to send before the drive counts as failed')
    parser.add_argument('--image-prefix',
                        help='save head camera images to PREFIX_before.png and '
                             'PREFIX_after.png')
    parser.add_argument('--report', help='also write the JSON report to this file')
    args, ros_args = parser.parse_known_args()

    def finish(report, code):
        text = json.dumps(report, indent=2)
        print(text)
        if args.report:
            with open(args.report, 'w') as output:
                output.write(text + '\n')
        rclpy.try_shutdown()
        sys.exit(code)

    rclpy.init(args=ros_args)
    node = PickClient()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    bottle = np.array(args.bottle, dtype=float)
    report = {'bottle': bottle.tolist(), 'steps': [], 'passed': False}
    if not node.wait_until_ready(240.0):
        report['error'] = 'Nav2, the commander, the joint states or TF did not come up'
        finish(report, 2)
    report['grasp_band'] = [node.commander_value('grasp_detect_min'),
                            node.commander_value('grasp_detect_max')]
    started = time.monotonic()

    def step(name, ok, status, **extra):
        entry = {'step': name, 'success': bool(ok), 'status': status,
                 'wall_seconds': round(time.monotonic() - started, 1), **extra}
        node.get_logger().info(json.dumps(entry))
        report['steps'].append(entry)
        if not ok:
            node.set_commander(allow_gripper_vs_octomap=False)
            report['error'] = f'{name}: {status}'
            report['finger'] = node.finger
            finish(report, 1)

    # The grasp box the commander can attach would collide with the table top in the
    # octomap at the start of the lift, so the planning scene is left alone.
    step('commander_params',
         node.set_commander(grasp_scene_action='none', allow_gripper_vs_octomap=False),
         'set grasp_scene_action none, allow_gripper_vs_octomap false')

    result, status = node.control_gripper(GRIPPER_OPEN, 120.0)
    step('open', result is not None and result.success, status, finger=node.finger)

    if not args.skip_nav:
        heading = args.approach_yaw
        goal = (bottle[0] - args.standoff * math.cos(heading)
                - args.standoff_lateral * math.sin(heading),
                bottle[1] - args.standoff * math.sin(heading)
                + args.standoff_lateral * math.cos(heading))
        # On a loaded machine the behavior tree can time out waiting for the planner or
        # the controller to accept a goal and abort the navigation (error codes 207 and
        # 107). Sending the goal again from where the base stopped recovers from that.
        failures = []
        for _ in range(max(1, args.nav_attempts)):
            ok, status = node.navigate_to(goal[0], goal[1], heading, 900.0)
            if ok:
                break
            failures.append(status)
            node.get_logger().warn(f'navigation failed ({status}), sending the goal again')
        step('navigate', ok, status, goal=[round(v, 3) for v in goal],
             base=node.pose_in_map('base_footprint'), failed_attempts=failures)

    # Seen from the stand-off, before the arm moves in front of the camera.
    time.sleep(2.0)
    before = node.depth_points()
    if args.image_prefix:
        node.save_color(f'{args.image_prefix}_before.png')

    # The straight-line moves of right_arm_lift can lower the lift by over 0.1 m. The left
    # arm hangs from the same column, and at home its fingers then reach the base.
    ok, status = node.move_to_named('left_arm', 'standby', 300.0)
    step('tuck_left', ok, status)

    base = node.pose_in_map('base_footprint')
    line = bottle[:2] - np.array(base[:2])
    yaw = math.atan2(line[1], line[0])
    approach = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    grasp = bottle + np.array([0.0, 0.0, args.height_offset]) - args.depth * approach
    pregrasp = grasp - args.pregrasp * approach
    rpy = hand_rpy(yaw, not args.camera_down)
    report['grasp'] = {'approach_yaw': round(yaw, 4), 'rpy': [round(v, 4) for v in rpy],
                       'pregrasp': pregrasp.round(4).tolist(), 'grasp': grasp.round(4).tolist(),
                       'base_at_plan': [round(v, 4) for v in base]}

    def hand_step(name, ok, status, target, **extra):
        # The commander reports success when the joint trajectory finishes. If the arm
        # caught on the table on the way, the hand ends far from the target anyway.
        hand = node.hand_in_map()
        error = None if hand is None else float(np.linalg.norm(hand[0] - target))
        if ok and (error is None or error > HAND_TOLERANCE):
            ok = False
            status = f'{status}, but the hand is not at the target'
        step(name, ok, status, **extra,
             hand=None if hand is None else hand[0].round(4).tolist(),
             hand_error_m=None if error is None else round(error, 4))

    ok, status = node.move_hand('whole_body_right', pregrasp, rpy, False, 900.0)
    hand_step('pregrasp', ok, status, pregrasp, base=node.pose_in_map('base_footprint'))

    # The fingers have to enter the bottle's and the table's voxels.
    step('allow_octomap', node.set_commander(allow_gripper_vs_octomap=True),
         'set allow_gripper_vs_octomap true')
    time.sleep(1.0)
    ok, status = node.move_hand('right_arm_lift', grasp, rpy, True, 600.0)
    hand_step('approach', ok, status, grasp)

    result, status = node.control_gripper(GRIPPER_CLOSED, 120.0)
    grasped = result is not None and result.grasped
    step('close', grasped, status, grasped=grasped,
         gripper_gap=None if result is None else round(result.gripper_gap, 4))

    hand_before_lift = node.hand_in_map()
    lifted = grasp + np.array([0.0, 0.0, args.lift])
    ok, status = node.move_hand('right_arm_lift', lifted, rpy, True, 600.0)
    hand = node.hand_in_map()
    step('lift', ok, status,
         hand=None if hand is None else hand[0].round(4).tolist(),
         hand_rise_m=None if hand is None or hand_before_lift is None
         else round(float(hand[0][2] - hand_before_lift[0][2]), 4))

    time.sleep(args.hold)
    band = report['grasp_band']
    finger = node.finger
    held = finger is not None and None not in band and band[0] <= finger <= band[1]
    after = node.depth_points()
    if args.image_prefix:
        node.save_color(f'{args.image_prefix}_after.png')
    hand = node.hand_in_map()
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
    }
    # If the whole-body goal turned the base, the bottle's old place can leave the head
    # camera's view. The table top around it tells whether the camera still sees it.
    camera['table_near_old_place_after'] = count_in_cylinder(
        after, bottle[:2], 0.15, TABLE_TOP - 0.01, TABLE_TOP + 0.01)
    seen = camera['old_place_lower_before'] or 0
    in_view = seen >= 20 and (camera['table_near_old_place_after'] or 0) >= 20
    camera['bottle_left_table'] = None if not in_view else bool(
        camera['old_place_lower_after'] <= 0.1 * seen
        and camera['above_old_top_after'] >= 10)
    problems = [] if held else ['finger gap out of the grasp band']
    if camera['bottle_left_table'] is False:
        problems.append('head camera does not see the bottle lifted')
    if problems:
        status = '; '.join(problems)
    elif in_view:
        status = 'held above the table'
    else:
        status = "held; the head camera does not see the bottle's old place"
    step('hold', not problems, status,
         finger=None if finger is None else round(finger, 4), camera=camera)

    node.set_commander(allow_gripper_vs_octomap=False)
    report['passed'] = True
    finish(report, 0)


if __name__ == '__main__':
    main()
