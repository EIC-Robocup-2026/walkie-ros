#!/usr/bin/env python3
"""Client of the whole-body demo. Needs robot_bringup whole_body_demo.launch.py running.

Sends the same target through both options of the commander and measures each against
MuJoCo's ground truth:

  far_arm     left_arm_lift to a target on the table, out of reach. It has to fail, with
              no base command at all.
  near_arm    left_arm_lift to a reachable target. The arm moves and the base stays.
  near_body   whole_body_left to a target the arm reaches on its own. The base stays.
  far_body    whole_body_left to the first target. Base, lift and arm move together.
  seam_turn   whole_body_left with the hand pointing along -x, which turns the body to a
              heading near +pi.
  seam_cross  whole_body_left to a hand direction whose body headings all lie past the
              seam. The base has to turn through +-pi the short way.

With --cycles N, whole_body_left drives back to a target near the start and out to the
first target again, N times, between far_body and seam_turn.

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
from my_robot_interfaces.action import GoToPose
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import Odometry
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
# Front edge of the base footprint, and the table's near edge in scene_whole_body.xml.
BASE_FRONT = 0.30
TABLE_EDGE_X = 1.55
# A base standing still reads below this much ground truth travel.
STILL_TRAVEL = 0.01


class DemoClient(Node):
    """Sends the goals and collects ground truth along the way."""

    def __init__(self):
        super().__init__('whole_body_demo')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.action = ActionClient(self, GoToPose, '/go_to_pose')
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
                    self.buffer.can_transform('map', 'base_footprint', Time()):
                return True
            time.sleep(0.5)
        return False

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
        future = self.action.send_goal_async(goal)
        accepted = self.wait_future(future, 10.0)
        success = False
        status = 'goal not accepted'
        if accepted is not None and accepted.accepted:
            result = self.wait_future(accepted.get_result_async(), timeout)
            if result is None:
                status = 'no result within the timeout'
            else:
                success = result.result.success
                status = result.result.status
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
    time.sleep(args.settle)
    report['start_base'] = list(node.truth)

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
        report['steps'].append(step)
        return step

    report['steps'].append(node.run_step(
        'far_arm', 'left_arm_lift', 'map', FAR_TARGET_MAP,
        expect_success=False, base_may_move=False))
    if not args.skip_near:
        report['steps'].append(node.run_step(
            'near_arm', 'left_arm_lift', 'base_footprint', NEAR_TARGET_BASE,
            expect_success=True, base_may_move=False))
        step = node.run_step(
            'near_body', 'whole_body_left', 'base_footprint', NEAR_BODY_TARGET_BASE,
            expect_success=True, base_may_move=False)
        if 'base fixed' not in step['status']:
            step['as_expected'] = False
        report['steps'].append(step)
    body_step('far_body', FAR_TARGET_MAP)
    for cycle in range(args.cycles):
        body_step(f'home_{cycle}', HOME_TARGET_MAP)
        body_step(f'far_{cycle}', FAR_TARGET_MAP)
    body_step('seam_turn', SEAM_TURN_TARGET_MAP, SEAM_TURN_HAND_YAW)
    if body_step('seam_cross', SEAM_CROSS_TARGET_MAP, SEAM_CROSS_HAND_YAW)['seam_crossings'] < 1:
        report['steps'][-1]['as_expected'] = False

    report['end_base'] = list(node.truth)
    report['table_clearance_m'] = round(TABLE_EDGE_X - (node.max_x + BASE_FRONT), 4)
    report['monitor_events'] = node.monitor_events
    report['passed'] = all(step['as_expected'] for step in report['steps'])
    finish(report, 0 if report['passed'] else 1)


if __name__ == '__main__':
    main()
