#!/usr/bin/env python3
# Copyright 2026 Walkie.
#
# Stress-test for openarm_left_joint7.
#
# Drives joint7 back and forth between its min and max position by publishing
# trajectory_msgs/JointTrajectory straight to the joint_trajectory_controller
# (bypassing MoveIt planning).  Each full round-trip the "duty scale" is ramped
# up: a higher scale means a shorter traverse time, i.e. a higher commanded
# velocity / more aggressive motion.  Runs forever until you press Ctrl+C.
#
#     duty_scale 0.2  ->  min<->max in 2.00 s   (gentle)
#     duty_scale 0.4  ->  min<->max in 1.00 s
#     duty_scale 0.6  ->  min<->max in 0.66 s
#     duty_scale 1.0  ->  min<->max in 0.40 s   (most aggressive)
#
# Live feedback (position / velocity / tracking error / running peaks) is read
# back from /joint_states and printed continuously.
#
# NOTE: the controller does NOT have allow_partial_joints_goal enabled, so every
# trajectory must list all 7 left joints.  We therefore hold joints 1-6 at their
# last-measured positions while only joint7 moves.
#
# Run (after sourcing the workspace and with the controllers already running):
#     python3 stress_test_joint7.py
#     python3 stress_test_joint7.py --min -1.4 --max 1.4 --base-time 0.5 --duty-start 0.1
#     python3 stress_test_joint7.py --loop          # reset scale to start after max
#     python3 stress_test_joint7.py --ros-args -r /joint_states:=/some_ns/joint_states

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


TARGET_JOINT = "openarm_left_joint7"
# Full ordered joint list for the left_joint_trajectory_controller.  All of
# these must appear in every goal because allow_partial_joints_goal is false.
LEFT_JOINTS = [
    "openarm_left_joint1",
    "openarm_left_joint2",
    "openarm_left_joint3",
    "openarm_left_joint4",
    "openarm_left_joint5",
    "openarm_left_joint6",
    "openarm_left_joint7",
]


class Joint7StressTest(Node):
    def __init__(self, args):
        super().__init__("joint7_stress_test")
        self.args = args

        self.joints = list(LEFT_JOINTS)
        self.target_idx = self.joints.index(TARGET_JOINT)

        # Latest measured positions (held for the non-target joints) and the
        # latest measured state of joint7 (printed as feedback).
        self.last_pos = {}
        self.j7_pos = None
        self.j7_vel = 0.0
        self.j7_eff = None

        # Running peaks for the end-of-run summary.
        self.pos_min_seen = math.inf
        self.pos_max_seen = -math.inf
        self.vel_peak = 0.0
        self.eff_peak = 0.0

        self.cur_target = None  # what we last commanded joint7 to

        cmd_qos = QoSProfile(depth=10)
        self.cmd_pub = self.create_publisher(
            JointTrajectory, args.command_topic, cmd_qos)

        # /joint_states is published best-effort by some broadcasters; accept both.
        state_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            JointState, args.state_topic, self._on_joint_state, state_qos)

    # ---- feedback ---------------------------------------------------------
    def _on_joint_state(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name in self.last_pos or name in self.joints:
                if i < len(msg.position):
                    self.last_pos[name] = msg.position[i]
            if name == TARGET_JOINT:
                if i < len(msg.position):
                    self.j7_pos = msg.position[i]
                    self.pos_min_seen = min(self.pos_min_seen, self.j7_pos)
                    self.pos_max_seen = max(self.pos_max_seen, self.j7_pos)
                if i < len(msg.velocity):
                    self.j7_vel = msg.velocity[i]
                    self.vel_peak = max(self.vel_peak, abs(self.j7_vel))
                if i < len(msg.effort) and len(msg.effort) > 0:
                    self.j7_eff = msg.effort[i]
                    self.eff_peak = max(self.eff_peak, abs(self.j7_eff))

    def wait_for_state(self, timeout_s=10.0):
        end = self.get_clock().now() + Duration(seconds=timeout_s)
        while rclpy.ok() and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            # Require EVERY joint we will command, so build_traj never falls back
            # to 0.0 for a missing joint (which would fling that joint to zero).
            if self.j7_pos is not None and all(j in self.last_pos for j in self.joints):
                return True
        return False

    # ---- commanding -------------------------------------------------------
    def build_traj(self, j7_target, time_s):
        """One-point trajectory: joint7 -> target, all others held at last pos."""
        positions = []
        for name in self.joints:
            if name == TARGET_JOINT:
                positions.append(float(j7_target))
            else:
                positions.append(float(self.last_pos.get(name, 0.0)))

        pt = JointTrajectoryPoint()
        pt.positions = positions
        # Zero terminal velocity -> decelerate to a clean stop at each limit.
        pt.velocities = [0.0] * len(self.joints)
        pt.time_from_start = Duration(seconds=time_s).to_msg()

        traj = JointTrajectory()
        traj.joint_names = list(self.joints)
        # stamp 0 => controller starts executing immediately.
        traj.points = [pt]
        return traj

    def traverse_time_for(self, duty):
        """Higher duty -> shorter time.  Clamped to respect the velocity limit."""
        t = self.args.base_time / duty
        span = abs(self.args.max - self.args.min)
        req_v = span / t if t > 0 else math.inf
        if req_v > self.args.vel_limit:
            t = span / self.args.vel_limit  # slow down to stay within limit
        return max(t, 0.02)

    def send(self, j7_target, time_s):
        self.cur_target = j7_target
        self.cmd_pub.publish(self.build_traj(j7_target, time_s))

    # ---- spin helper ------------------------------------------------------
    def spin_for(self, duration_s, label):
        end = self.get_clock().now() + Duration(seconds=duration_s)
        next_print = self.get_clock().now()
        period = Duration(seconds=0.1)
        while rclpy.ok() and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            now = self.get_clock().now()
            if now >= next_print:
                self._print_status(label)
                next_print = now + period

    def _print_status(self, label):
        pos = self.j7_pos if self.j7_pos is not None else float("nan")
        tgt = self.cur_target if self.cur_target is not None else float("nan")
        err = (tgt - pos) if (self.j7_pos is not None and self.cur_target is not None) else float("nan")
        eff = "" if self.j7_eff is None else f" eff={self.j7_eff:+6.2f}"
        line = (f"\r{label} target={tgt:+6.3f} pos={pos:+6.3f} "
                f"vel={self.j7_vel:+6.2f} err={err:+6.3f}{eff} "
                f"| peaks pos[{self.pos_min_seen:+5.2f},{self.pos_max_seen:+5.2f}] "
                f"|v|max={self.vel_peak:4.2f}")
        sys.stdout.write(line.ljust(110))
        sys.stdout.flush()

    # ---- main loop --------------------------------------------------------
    def run(self):
        a = self.args
        self.get_logger().info(
            f"Waiting for {a.state_topic} (need {TARGET_JOINT} + joint positions)...")
        if not self.wait_for_state():
            self.get_logger().error(
                "No joint states received. Are the controllers running and is "
                f"'{a.state_topic}' correct? Aborting.")
            return

        lo = a.min + a.margin
        hi = a.max - a.margin
        self.get_logger().info(
            f"Start. joint7 range [{lo:+.4f}, {hi:+.4f}] rad, base_time={a.base_time}s "
            f"(time at duty=1.0), duty {a.duty_start}->{a.duty_max} step {a.duty_step}, "
            f"{'LOOP' if a.loop else 'HOLD at max'}. Ctrl+C to stop.")

        # Gentle homing move to the lower limit first (slow, fixed time).
        self.get_logger().info(f"Homing joint7 to lower limit ({lo:+.3f}) ...")
        self.send(lo, a.home_time)
        self.spin_for(a.home_time + 0.2, "[home ]")

        duty = a.duty_start
        target_hi = True  # next target is the upper limit
        cycle = 0
        try:
            while rclpy.ok():
                t = self.traverse_time_for(duty)
                tgt = hi if target_hi else lo
                label = f"[cyc {cycle:>3} duty {duty:0.2f}]"
                self.send(tgt, t)
                self.spin_for(t + a.dwell, label)

                target_hi = not target_hi
                if not target_hi:
                    # just sent toward hi; a full round-trip completes when we
                    # come back to lo, so count + ramp on the return.
                    pass
                else:
                    # back at lo -> one full round trip done; ramp the duty.
                    cycle += 1
                    duty += a.duty_step
                    if duty > a.duty_max + 1e-9:
                        duty = a.duty_start if a.loop else a.duty_max
        except KeyboardInterrupt:
            pass
        finally:
            self.stop_safely()

    def stop_safely(self):
        sys.stdout.write("\n")
        self.get_logger().info("Stopping: commanding joint7 to hold current position...")
        rclpy.spin_once(self, timeout_sec=0.1)
        hold = self.j7_pos if self.j7_pos is not None else (self.cur_target or 0.0)
        try:
            self.send(hold, 0.5)
            self.spin_for(0.6, "[stop ]")
            sys.stdout.write("\n")
        except Exception:
            pass
        self.get_logger().info(
            f"Summary: position range seen [{self.pos_min_seen:+.3f}, "
            f"{self.pos_max_seen:+.3f}] rad, peak |vel| = {self.vel_peak:.2f} rad/s"
            + ("" if self.eff_peak == 0.0 else f", peak |eff| = {self.eff_peak:.2f}"))


def parse_args(argv):
    p = argparse.ArgumentParser(
        description="Stress-test openarm_left_joint7 by cycling min<->max with a "
                    "ramping duty (speed) scale.")
    # joint7 limits come from arm/v10/joint_limits.yaml: +/- 1.570796 rad.
    p.add_argument("--min", type=float, default=-1.570796, help="lower limit (rad)")
    p.add_argument("--max", type=float, default=1.570796, help="upper limit (rad)")
    p.add_argument("--margin", type=float, default=0.0,
                   help="back off this many rad from each hard limit (safety)")
    p.add_argument("--base-time", dest="base_time", type=float, default=0.4,
                   help="traverse time (s) at duty=1.0 (the fastest)")
    p.add_argument("--duty-start", dest="duty_start", type=float, default=0.2)
    p.add_argument("--duty-step", dest="duty_step", type=float, default=0.2)
    p.add_argument("--duty-max", dest="duty_max", type=float, default=1.0)
    p.add_argument("--loop", action="store_true",
                   help="reset duty to start after reaching max (default: hold at max)")
    p.add_argument("--dwell", type=float, default=0.0,
                   help="pause (s) at each limit before reversing")
    p.add_argument("--home-time", dest="home_time", type=float, default=3.0,
                   help="time (s) for the initial gentle move to the lower limit")
    p.add_argument("--vel-limit", dest="vel_limit", type=float, default=20.943946,
                   help="cap on commanded joint velocity (rad/s); traverse time is "
                        "lengthened if a duty would exceed it")
    p.add_argument("--command-topic", dest="command_topic",
                   default="/left_joint_trajectory_controller/joint_trajectory")
    p.add_argument("--state-topic", dest="state_topic", default="/joint_states")
    return p.parse_known_args(argv)


def main():
    parsed, ros_args = parse_args(sys.argv[1:])
    rclpy.init(args=ros_args)
    node = Joint7StressTest(parsed)
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
