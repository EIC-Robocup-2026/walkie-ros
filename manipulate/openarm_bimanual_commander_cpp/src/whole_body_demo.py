#!/usr/bin/env python3
"""Client of the whole-body demo. Needs robot_bringup whole_body_demo.launch.py running.

Sends the same target through both options of the commander and measures each against
MuJoCo's ground truth:

  far_arm     left_arm_lift to a target on the table, out of reach. It has to fail, with
              no base command at all.
  near_arm    left_arm_lift to a reachable target. The arm moves and the base stays.
  near_body   whole_body_left to a target the arm reaches on its own. The base stays.
  far_body    whole_body_left to the first target. Base, lift and arm move together.
  tuck_right  go_to_home sends right_arm to standby, out of the way of the table. A
              whole-body goal of the left arm never moves the right arm. The base stays.
  deep_body   whole_body_left to a target near the far edge of the table, which needs the
              column close to the table.
  seam_turn   whole_body_left with the hand pointing along -x, which turns the body to a
              heading near +pi.
  seam_cross  whole_body_left to a hand direction whose body headings all lie past the
              seam. The base has to turn through +-pi the short way.
  home_right  go_to_home sends right_arm_lift back to home, away from the table, which
              also raises the lift. The base stays.

With --cycles N, whole_body_left drives back to a target near the start and out to the
first target again, N times, between far_body and tuck_right.

Prints one JSON report and exits non-zero if any step misses its expectation. rmw_zenoh
logs to stdout, so use --report to get a file that parses as JSON.
"""
import argparse
import json
import math
import sys
import threading
import time

from geometry_msgs.msg import TwistStamped
from my_robot_interfaces.action import GoToHome, GoToPose
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

# The gripper points along +x of the frame the goal is in.
FORWARD_PITCH = math.pi / 2
# Above the box. The hand and its wrist camera bracket need about 0.2 m over the box top.
FAR_TARGET_MAP = (1.75, 0.15, 1.05)
NEAR_TARGET_BASE = (0.45, 0.25, 1.0)
HOME_TARGET_MAP = (0.45, 0.25, 1.0)
# Exact IK solves this one from the start with the base where it is. The whole-body group
# only keeps the base still for such a goal: its first attempt needs an exact IK solution,
# while an arm group's pose goal also accepts a solution within the goal tolerances.
NEAR_BODY_TARGET_BASE = (0.50, 0.20, 0.90)
# Hand along -x, where the left arm's body headings lie between about 2.1 and pi.
SEAM_TURN_TARGET_MAP = (0.6, 0.1, 1.0)
SEAM_TURN_HAND_YAW = math.pi
# From there, this hand direction has body headings between about -3 and -2, so the short
# way goes through +pi. The target is about 1.5 m from where seam_turn stops the base: the
# arm reached a closer one alone, and then the base stayed and never crossed the seam.
SEAM_CROSS_TARGET_MAP = (0.7, -1.4, 1.0)
SEAM_CROSS_HAND_YAW = -2.0
# 0.10 m short of the table's far edge. With the gripper pointing forward, its wrist camera
# bracket reaches 0.111 m below the hand frame, so this is about 0.10 m over the table top
# at 0.74 m. At 0.90 m the octomap's table top (up to 0.04 m high) and the commander's
# 0.01 m lift margin left no room. On the table's centre line, so the base stays clear of
# the table legs: at y 0.30 the collision monitor stopped the base near the near left leg.
# The right arm's home pose hangs below the table top and keeps the column about 0.2 m
# from the table, too far for this target on the octomap.
DEEP_TARGET_MAP = (2.15, 0.0, 0.95)
# Front edge of the base footprint, front face of the column (above the base, so it is the
# part that meets the table edge once the base is under the table top), and the table's
# near edge in scene_whole_body.xml.
BASE_FRONT = 0.30
COLUMN_FRONT = 0.02
TABLE_EDGE_X = 1.55
# A base standing still reads below this much ground truth travel.
STILL_TRAVEL = 0.01


class DemoClient(Node):
    """Sends the goals and collects ground truth along the way."""

    def __init__(self):
        super().__init__('whole_body_demo')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.action = ActionClient(self, GoToPose, '/go_to_pose')
        self.home_action = ActionClient(self, GoToHome, '/go_to_home')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.truth = None
        self.travel = 0.0
        self.heading = None
        self.turned = 0.0
        self.seam_crossings = 0
        self.max_x = -math.inf
        self.base_commands = 0
        self.monitor_events = []
        self.timed_out = False
        self.create_subscription(Odometry, '/mujoco_ground_truth/odom', self.on_truth, 50)
        self.create_subscription(TwistStamped, '/cmd_vel_smoothed', self.on_command, 50)
        self.create_subscription(
            CollisionMonitorState, '/collision_monitor_state', self.on_monitor, 10)

    def on_truth(self, msg):
        position = msg.pose.pose.position
        if self.truth is not None:
            self.travel += math.hypot(position.x - self.truth[0], position.y - self.truth[1])
        self.truth = (position.x, position.y)
        self.max_x = max(self.max_x, position.x)
        rotation = msg.pose.pose.orientation
        heading = 2.0 * math.atan2(rotation.z, rotation.w)
        heading = math.remainder(heading, 2.0 * math.pi)
        if self.heading is not None:
            self.turned += abs(math.remainder(heading - self.heading, 2.0 * math.pi))
            if abs(heading - self.heading) > math.pi:
                self.seam_crossings += 1
        self.heading = heading

    def on_command(self, msg):
        twist = msg.twist
        if abs(twist.linear.x) + abs(twist.linear.y) + abs(twist.angular.z) > 0.0:
            self.base_commands += 1

    def on_monitor(self, msg):
        self.monitor_events.append({'action_type': msg.action_type, 'polygon': msg.polygon_name})

    def wait_until_ready(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.truth is not None and self.action.server_is_ready() and \
                    self.home_action.server_is_ready() and \
                    self.buffer.can_transform('map', 'base_footprint', Time()):
                return True
            time.sleep(0.5)
        return False

    def commander_plans_only(self, timeout=10.0):
        """The commander's plan_only parameter, or None if it could not be read."""
        client = self.create_client(GetParameters, '/bimanual_commander/get_parameters')
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        response = self.wait_future(
            client.call_async(GetParameters.Request(names=['plan_only'])), timeout)
        if response is None or not response.values:
            return None
        return response.values[0].bool_value

    def tip_in_map(self, link='openarm_left_hand'):
        try:
            transform = self.buffer.lookup_transform('map', link, Time()).transform.translation
        except TransformException:
            return None
        return (transform.x, transform.y, transform.z)

    # Wall time, and only a last resort: the commander enforces its own deadline on sim
    # time. A loaded machine can run the simulation at a fifth of real time.
    def run_step(self, name, group, frame, target, expect_success, base_may_move,
                 timeout=900.0, hand_yaw=0.0):
        travel_before = self.travel
        position_before = self.truth
        heading_before = self.heading
        turned_before = self.turned
        crossings_before = self.seam_crossings
        commands_before = self.base_commands
        started = time.monotonic()
        goal = GoToPose.Goal(
            group_name=group, x=float(target[0]), y=float(target[1]), z=float(target[2]),
            roll=0.0, pitch=FORWARD_PITCH, yaw=float(hand_yaw), cartesian_path=False,
            frame_id=frame)
        self.get_logger().info(f'{name}: {group} to {target} in {frame}')
        result, status = self.send_goal(self.action, goal, timeout)
        success = result is not None and result.success
        if result is not None:
            status = result.status
        # Let the base settle before reading the travel of this step.
        time.sleep(1.0)
        step = {
            'step': name,
            'group': group,
            'frame': frame,
            'target': target,
            'success': success,
            'status': status,
            # CLOCK_MONOTONIC, so it lines up with other logs of the same machine.
            'started_monotonic': round(started, 2),
            'wall_seconds': round(time.monotonic() - started, 2),
            'base_travel_m': round(self.travel - travel_before, 4),
            'base_displacement_m': round(math.dist(self.truth, position_before), 4),
            'base_turn_rad': round(self.turned - turned_before, 4),
            'heading_change_rad': round(
                abs(math.remainder(self.heading - heading_before, 2.0 * math.pi)), 4),
            'seam_crossings': self.seam_crossings - crossings_before,
            'base_commands': self.base_commands - commands_before,
        }
        if frame == 'map':
            tip = self.tip_in_map()
            if tip is not None:
                step['tip_in_map'] = [round(value, 4) for value in tip]
                step['tip_error_m'] = round(math.dist(tip, target), 4)
        step['as_expected'] = success == expect_success and (
            base_may_move or
            (step['base_commands'] == 0 and step['base_travel_m'] < STILL_TRAVEL))
        self.get_logger().info(json.dumps(step))
        return step

    def run_named_step(self, name, group, pose_name, timeout=300.0):
        """Send group to an SRDF named state with go_to_home. The base has to stay."""
        travel_before = self.travel
        commands_before = self.base_commands
        started = time.monotonic()
        self.get_logger().info(f'{name}: {group} to {pose_name}')
        result, status = self.send_goal(
            self.home_action, GoToHome.Goal(group_name=group, pose_name=pose_name), timeout)
        success = result is not None and result.success
        if result is not None:
            status = 'reached' if success else 'failed'
        time.sleep(1.0)
        step = {
            'step': name,
            'group': group,
            'pose_name': pose_name,
            'success': success,
            'status': status,
            'started_monotonic': round(started, 2),
            'wall_seconds': round(time.monotonic() - started, 2),
            'base_travel_m': round(self.travel - travel_before, 4),
            'base_commands': self.base_commands - commands_before,
        }
        step['as_expected'] = success and step['base_commands'] == 0 and \
            step['base_travel_m'] < STILL_TRAVEL
        self.get_logger().info(json.dumps(step))
        return step

    def send_goal(self, client, goal, timeout):
        """Return (result, None), or (None, why there is no result).

        A goal without a result within the timeout is canceled, so that it does not go on
        moving the robot during the next step. main() stops the sequence after it.
        """
        goal_future = client.send_goal_async(goal)
        accepted = self.wait_future(goal_future, 10.0)
        if accepted is None:
            # A late acceptance would let the goal move the robot during the next step,
            # so wait for the response and cancel the goal if it was accepted after all.
            self.timed_out = True
            late = self.wait_future(goal_future, 30.0)
            if late is not None and late.accepted:
                self.wait_future(late.cancel_goal_async(), 10.0)
                settled = self.wait_future(late.get_result_async(), 60.0) is not None
                return None, 'goal accepted after the timeout, ' + (
                    'canceled' if settled else 'still running after the cancel request')
            return None, 'no goal response within the timeout'
        if not accepted.accepted:
            return None, 'goal not accepted'
        result_future = accepted.get_result_async()
        response = self.wait_future(result_future, timeout)
        if response is None:
            self.timed_out = True
            self.wait_future(accepted.cancel_goal_async(), 10.0)
            settled = self.wait_future(result_future, 60.0) is not None
            return None, 'no result within the timeout, ' + (
                'canceled' if settled else 'still running after the cancel request')
        return response.result, None

    @staticmethod
    def wait_future(future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        return future.result() if future.done() else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--settle', type=float, default=8.0,
                        help='seconds to let the octomap fill before the first goal')
    parser.add_argument('--tip-tolerance', type=float, default=0.03,
                        help='largest accepted end effector error of the whole-body step, in m')
    parser.add_argument('--skip-near', action='store_true', help='skip the near arm step')
    parser.add_argument('--cycles', type=int, default=0,
                        help='whole-body round trips to run after the far_body step')
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
    node = DemoClient()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    report = {'steps': []}
    if not node.wait_until_ready(180.0):
        report['error'] = 'the commander, TF or ground truth did not come up'
        finish(report, 2)
    # With plan_only the commander reports success without moving anything, and the named
    # steps only check the result and that the base stayed.
    report['commander_plan_only'] = node.commander_plans_only()
    if report['commander_plan_only']:
        report['error'] = 'the commander has plan_only set, so nothing would move'
        finish(report, 2)
    time.sleep(args.settle)
    report['start_base'] = list(node.truth)

    def record(step):
        report['steps'].append(step)
        # The robot stopped wherever the canceled goal left it, so the later steps would
        # not start where they expect to.
        if node.timed_out:
            report['error'] = f"{step['step']}: {step['status']}; the remaining steps did not run"
            report['end_base'] = list(node.truth)
            report['monitor_events'] = node.monitor_events
            report['passed'] = False
            finish(report, 1)
        return step

    def body_step(name, target, hand_yaw=0.0):
        step = node.run_step(name, 'whole_body_left', 'map', target,
                             expect_success=True, base_may_move=True, hand_yaw=hand_yaw)
        if step.get('tip_error_m', math.inf) > args.tip_tolerance:
            step['as_expected'] = False
        if step['base_travel_m'] <= STILL_TRAVEL:
            step['as_expected'] = False
        # The long way round across +-pi adds about 2 pi of turning.
        if step['base_turn_rad'] > step['heading_change_rad'] + math.pi:
            step['as_expected'] = False
        # Nothing blocks the way in this scene, so a long detour is a planning problem.
        if step['base_travel_m'] > 1.5 * step['base_displacement_m'] + 0.3:
            step['as_expected'] = False
        return record(step)

    record(node.run_step(
        'far_arm', 'left_arm_lift', 'map', FAR_TARGET_MAP,
        expect_success=False, base_may_move=False))
    if not args.skip_near:
        record(node.run_step(
            'near_arm', 'left_arm_lift', 'base_footprint', NEAR_TARGET_BASE,
            expect_success=True, base_may_move=False))
        step = node.run_step(
            'near_body', 'whole_body_left', 'base_footprint', NEAR_BODY_TARGET_BASE,
            expect_success=True, base_may_move=False)
        if 'base fixed' not in step['status']:
            step['as_expected'] = False
        record(step)
    body_step('far_body', FAR_TARGET_MAP)
    for cycle in range(args.cycles):
        body_step(f'home_{cycle}', HOME_TARGET_MAP)
        body_step(f'far_{cycle}', FAR_TARGET_MAP)
    # The base front stays outside the table up to here. deep_body may drive it under the
    # table top, so the column is measured there instead.
    report['table_clearance_m'] = round(TABLE_EDGE_X - (node.max_x + BASE_FRONT), 4)
    # A caller clears the idle arm before a goal near furniture: the whole-body goal will
    # not move it. It stays tucked until the base is away from the table.
    record(node.run_named_step('tuck_right', 'right_arm', 'standby'))
    node.max_x = -math.inf
    body_step('deep_body', DEEP_TARGET_MAP)
    report['deep_column_gap_m'] = round(TABLE_EDGE_X - (node.max_x + COLUMN_FRONT), 4)
    body_step('seam_turn', SEAM_TURN_TARGET_MAP, SEAM_TURN_HAND_YAW)
    if body_step('seam_cross', SEAM_CROSS_TARGET_MAP, SEAM_CROSS_HAND_YAW)['seam_crossings'] < 1:
        report['steps'][-1]['as_expected'] = False
    # Below a lift position of 0.50 m the fingers of an arm in home reach into the base, and
    # IK can leave the lift there (0.492 m after seam_cross in one run). right_arm_lift's
    # home raises the lift to 0.744 m in the same motion.
    record(node.run_named_step('home_right', 'right_arm_lift', 'home'))

    report['end_base'] = list(node.truth)
    report['monitor_events'] = node.monitor_events
    report['passed'] = all(step['as_expected'] for step in report['steps'])
    finish(report, 0 if report['passed'] else 1)


if __name__ == '__main__':
    main()
