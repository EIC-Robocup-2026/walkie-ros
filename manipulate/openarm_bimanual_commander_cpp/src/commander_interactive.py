#!/usr/bin/env python3
"""Interactive CLI test node for the bimanual commander."""

import math
import threading
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from my_robot_interfaces.action import (
    GoToPose,
    GoToPoseRelative,
    GoToPoseQuaternion,
    GoToHome,
    ControlGripper,
    SetJointPosition,
)
from my_robot_interfaces.srv import GetEEPose, GetJointStates

GROUPS = ["left_arm", "right_arm", "left_arm_lift", "right_arm_lift",
          "both_arms", "both_arms_lift", "left_gripper", "right_gripper"]

MENU = """
╔══════════════════════════════════════════╗
║    Bimanual Commander Interactive CLI    ║
╚══════════════════════════════════════════╝
Groups: left_arm  right_arm  left_arm_lift  right_arm_lift
        both_arms  both_arms_lift  left_gripper  right_gripper

 [1] go_to_pose          x y z roll pitch yaw [frame] [cartesian]
 [2] go_to_pose_relative x y z roll pitch yaw [frame] [cartesian]
 [3] go_to_pose_quat     x y z qx qy qz qw   [frame] [cartesian]
 [4] go_to_home          group  (pose: home/standby/hands_up/pre-place/tray)
 [5] control_gripper     group position
 [6] set_joint_position  group j1 j2 ... jN
 [7] get_ee_pose         group [frame]
 [8] get_joint_states   group
 [q] quit
"""


def euler_to_quat(roll, pitch, yaw):
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return qx, qy, qz, qw


def quat_to_euler(qx, qy, qz, qw):
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (qw * qy - qz * qx)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)

    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def prompt(msg, default=None):
    if default is not None:
        val = input(f"  {msg} [{default}]: ").strip()
        return val if val else str(default)
    return input(f"  {msg}: ").strip()


def send_action(node, client, goal, timeout_sec=60.0):
    """Send an action goal and block until result. Returns (success, status_str)."""
    if not client.wait_for_server(timeout_sec=3.0):
        return False, "Action server not available"

    print("  >> Sending goal...")
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

    if not future.done():
        return False, "Goal send timed out"

    goal_handle = future.result()
    if not goal_handle.accepted:
        return False, "Goal rejected by server"

    print("  >> Goal accepted, waiting for result...")
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=timeout_sec)

    if not result_future.done():
        return False, "Result timed out"

    result = result_future.result()
    r = result.result
    success = getattr(r, "success", False)
    status = getattr(r, "status", "")
    return success, status


class InteractiveCommander(Node):
    def __init__(self):
        super().__init__("interactive_commander")
        self._pose_client   = ActionClient(self, GoToPose,         "go_to_pose")
        self._rel_client    = ActionClient(self, GoToPoseRelative,  "go_to_pose_relative")
        self._quat_client   = ActionClient(self, GoToPoseQuaternion,"go_to_pose_quat")
        self._home_client   = ActionClient(self, GoToHome,          "go_to_home")
        self._grip_client   = ActionClient(self, ControlGripper,    "control_gripper")
        self._joint_client  = ActionClient(self, SetJointPosition,  "set_joint_position")
        self._ee_pose_cli   = self.create_client(GetEEPose,         "get_ee_pose")
        self._jnt_state_cli = self.create_client(GetJointStates,    "get_joint_states")

    # ── helpers ──────────────────────────────────────────────────────────

    def _get_group(self):
        g = prompt("group_name").strip()
        if g not in GROUPS:
            print(f"  [warn] '{g}' not in known groups list, proceeding anyway.")
        return g

    def _get_frame(self):
        f = prompt("frame_id (enter=base_footprint)", "")
        return f

    def _get_cartesian(self):
        c = prompt("cartesian_path (0/1)", "0")
        return c.strip() in ("1", "true", "True", "yes")

    # ── actions ──────────────────────────────────────────────────────────

    def do_go_to_pose(self):
        group = self._get_group()
        try:
            x     = float(prompt("x (m)"))
            y     = float(prompt("y (m)"))
            z     = float(prompt("z (m)"))
            roll  = float(prompt("roll  (rad)"))
            pitch = float(prompt("pitch (rad)"))
            yaw   = float(prompt("yaw   (rad)"))
        except ValueError:
            print("  [error] Invalid number input."); return
        frame     = self._get_frame()
        cartesian = self._get_cartesian()

        goal = GoToPose.Goal()
        goal.group_name    = group
        goal.x, goal.y, goal.z = x, y, z
        goal.roll, goal.pitch, goal.yaw = roll, pitch, yaw
        goal.frame_id      = frame
        goal.cartesian_path = cartesian

        qx, qy, qz, qw = euler_to_quat(roll, pitch, yaw)
        print(f"  → quaternion: [{qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f}]")
        ok, status = send_action(self, self._pose_client, goal)
        print(f"  Result: {'SUCCESS' if ok else 'FAILED'} — {status}")

    def do_go_to_pose_relative(self):
        group = self._get_group()
        try:
            x     = float(prompt("dx (m)"))
            y     = float(prompt("dy (m)"))
            z     = float(prompt("dz (m)"))
            roll  = float(prompt("droll  (rad)"))
            pitch = float(prompt("dpitch (rad)"))
            yaw   = float(prompt("dyaw   (rad)"))
        except ValueError:
            print("  [error] Invalid number input."); return
        frame     = self._get_frame()
        cartesian = self._get_cartesian()
        ee_frame  = prompt("offset in EE-local frame? (1=ee / 0=world)", "0").strip() in ("1", "true", "yes")

        goal = GoToPoseRelative.Goal()
        goal.group_name    = group
        goal.x, goal.y, goal.z = x, y, z
        goal.roll, goal.pitch, goal.yaw = roll, pitch, yaw
        goal.frame_id       = frame
        goal.cartesian_path = cartesian
        goal.ee_frame       = ee_frame

        ok, status = send_action(self, self._rel_client, goal)
        print(f"  Result: {'SUCCESS' if ok else 'FAILED'} — {status}")

    def do_go_to_pose_quat(self):
        group = self._get_group()
        try:
            x  = float(prompt("x (m)"))
            y  = float(prompt("y (m)"))
            z  = float(prompt("z (m)"))
        except ValueError:
            print("  [error] Invalid number input."); return

        mode = prompt("input as (e)uler or (q)uaternion", "e").lower()
        try:
            if mode.startswith("e"):
                roll  = float(prompt("roll  (rad)"))
                pitch = float(prompt("pitch (rad)"))
                yaw   = float(prompt("yaw   (rad)"))
                qx, qy, qz, qw = euler_to_quat(roll, pitch, yaw)
                print(f"  → quaternion: [{qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f}]")
            else:
                qx = float(prompt("qx"))
                qy = float(prompt("qy"))
                qz = float(prompt("qz"))
                qw = float(prompt("qw"))
                roll, pitch, yaw = quat_to_euler(qx, qy, qz, qw)
                print(f"  → euler: roll={math.degrees(roll):.2f}° "
                      f"pitch={math.degrees(pitch):.2f}° yaw={math.degrees(yaw):.2f}°")
        except ValueError:
            print("  [error] Invalid number input."); return

        frame     = self._get_frame()
        cartesian = self._get_cartesian()

        goal = GoToPoseQuaternion.Goal()
        goal.group_name    = group
        goal.x, goal.y, goal.z = x, y, z
        goal.qx, goal.qy, goal.qz, goal.qw = qx, qy, qz, qw
        goal.frame_id       = frame
        goal.cartesian_path = cartesian

        ok, status = send_action(self, self._quat_client, goal)
        print(f"  Result: {'SUCCESS' if ok else 'FAILED'} — {status}")

    def do_go_to_home(self):
        group = self._get_group()
        # Press Enter for "home". Any SRDF named state works (passed straight
        # to setNamedTarget); pre-place / tray are defined only for some
        # groups (an unknown state aborts cleanly).
        pose = prompt("pose (home/standby/hands_up/pre-place/tray)", "home")
        goal = GoToHome.Goal()
        goal.group_name = group
        goal.pose_name  = pose
        ok, _ = send_action(self, self._home_client, goal)
        print(f"  Result: {'SUCCESS' if ok else 'FAILED'}")

    def do_control_gripper(self):
        group = self._get_group()
        try:
            pos = float(prompt("position (m, 0=closed)"))
        except ValueError:
            print("  [error] Invalid number."); return
        goal = ControlGripper.Goal()
        goal.group_name = group
        goal.position   = pos
        ok, status = send_action(self, self._grip_client, goal)
        print(f"  Result: {'SUCCESS' if ok else 'FAILED'} — {status}")

    def do_set_joint_position(self):
        group = self._get_group()
        raw = prompt("joint_positions (space-separated radians)")
        try:
            joints = [float(v) for v in raw.split()]
        except ValueError:
            print("  [error] Invalid joint values."); return
        goal = SetJointPosition.Goal()
        goal.group_name      = group
        goal.joint_positions = joints
        print(f"  Sending {len(joints)} joint values: {joints}")
        ok, status = send_action(self, self._joint_client, goal)
        print(f"  Result: {'SUCCESS' if ok else 'FAILED'} — {status}")

    def do_get_ee_pose(self):
        group = self._get_group()
        frame = self._get_frame()

        if not self._ee_pose_cli.wait_for_service(timeout_sec=3.0):
            print("  [error] get_ee_pose service not available"); return

        req = GetEEPose.Request()
        req.group_name = group
        req.frame_id   = frame

        future = self._ee_pose_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if not future.done():
            print("  [error] Service call timed out"); return

        r = future.result()
        if not r.success:
            print(f"  [error] {r.status}"); return

        roll, pitch, yaw = quat_to_euler(r.qx, r.qy, r.qz, r.qw)
        print(f"\n  EE Pose [{r.frame_id}]:")
        print(f"    position   : x={r.x:.4f}  y={r.y:.4f}  z={r.z:.4f}")
        print(f"    quaternion : qx={r.qx:.4f}  qy={r.qy:.4f}  qz={r.qz:.4f}  qw={r.qw:.4f}")
        print(f"    euler (rad): roll={roll:.4f}  pitch={pitch:.4f}  yaw={yaw:.4f}")
        print(f"    euler (deg): roll={math.degrees(roll):.2f}°  "
              f"pitch={math.degrees(pitch):.2f}°  yaw={math.degrees(yaw):.2f}°\n")


    def do_get_joint_states(self):
        group = self._get_group()

        if not self._jnt_state_cli.wait_for_service(timeout_sec=3.0):
            print("  [error] get_joint_states service not available"); return

        req = GetJointStates.Request()
        req.group_name = group

        future = self._jnt_state_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if not future.done():
            print("  [error] Service call timed out"); return

        r = future.result()
        if not r.success:
            print(f"  [error] {r.status}"); return

        print(f"\n  Joint States [{group}]:")
        for name, pos in zip(r.joint_names, r.joint_positions):
            print(f"    {name:<35} {pos:+.6f} rad  ({math.degrees(pos):+.2f}°)")
        print()


def main():
    rclpy.init()
    node = InteractiveCommander()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    HANDLERS = {
        "1": node.do_go_to_pose,
        "2": node.do_go_to_pose_relative,
        "3": node.do_go_to_pose_quat,
        "4": node.do_go_to_home,
        "5": node.do_control_gripper,
        "6": node.do_set_joint_position,
        "7": node.do_get_ee_pose,
        "8": node.do_get_joint_states,
    }

    try:
        while rclpy.ok():
            print(MENU)
            choice = input("Select [1-7 / q]: ").strip().lower()
            if choice == "q":
                break
            handler = HANDLERS.get(choice)
            if handler:
                try:
                    handler()
                except KeyboardInterrupt:
                    print("\n  [cancelled]")
            else:
                print("  Invalid choice.")
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)


if __name__ == "__main__":
    main()
