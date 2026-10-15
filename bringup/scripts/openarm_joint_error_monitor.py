#!/usr/bin/env python3
"""
OpenArm Joint State & Tracking Error Monitor

Monitors the OpenArm bimanual arm joints, reading the actual joint states
from /joint_states and comparing them against the commanded setpoints
(from JointTrajectoryControllerState, JointTrajectory, or ForwardCommandController).

Calculates instantaneous and peak error for each joint (both in radians and degrees),
as well as aggregate metrics (RMSE, MAE, Max Error).
Optionally publishes the error as a sensor_msgs/JointState topic and logs to CSV.

Usage:
  # Monitor both arms at 10 Hz:
  python3 openarm_joint_error_monitor.py

  # Monitor only right arm and record CSV:
  python3 openarm_joint_error_monitor.py --arm right --csv error_log.csv

  # Run via ros2 / pixi:
  pixi run python3 bringup/scripts/openarm_joint_error_monitor.py --arm both
"""

import argparse
import csv
import math
import os
import sys
import time
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float64MultiArray


# Joint naming conventions for OpenArm V10
ARM_JOINTS = {
    "left": [f"openarm_left_joint{i}" for i in range(1, 8)],
    "right": [f"openarm_right_joint{i}" for i in range(1, 8)],
}
GRIPPER_JOINTS = {
    "left": ["openarm_left_finger_joint1"],
    "right": ["openarm_right_finger_joint1"],
}

# ANSI colors for terminal display
RESET = "\033[0m"
BOLD = "\033[1m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
RED = "\033[31m"
CYAN = "\033[36m"
WHITE = "\033[37m"
DIM = "\033[2m"


class JointData:
    def __init__(self, name: str):
        self.name = name
        self.cmd_pos: Optional[float] = None
        self.cmd_vel: Optional[float] = None
        self.act_pos: Optional[float] = None
        self.act_vel: Optional[float] = None
        self.act_eff: Optional[float] = None
        self.error_pos: Optional[float] = None  # cmd - act
        self.peak_error_pos: float = 0.0
        self.last_update_time: float = 0.0

    def update_command(self, pos: float, vel: Optional[float] = None):
        self.cmd_pos = pos
        if vel is not None:
            self.cmd_vel = vel
        self._recompute_error()

    def update_actual(self, pos: float, vel: Optional[float] = None, eff: Optional[float] = None):
        self.act_pos = pos
        if vel is not None:
            self.act_vel = vel
        if eff is not None:
            self.act_eff = eff
        self.last_update_time = time.time()
        self._recompute_error()

    def _recompute_error(self):
        if self.cmd_pos is not None and self.act_pos is not None:
            self.error_pos = self.cmd_pos - self.act_pos
            self.peak_error_pos = max(self.peak_error_pos, abs(self.error_pos))


class OpenArmJointErrorMonitor(Node):
    def __init__(self, args):
        super().__init__("openarm_joint_error_monitor")
        self.args = args

        # Select joints to monitor
        self.sides = ["left", "right"] if args.arm == "both" else [args.arm]
        self.monitored_joints: List[str] = []
        for side in self.sides:
            self.monitored_joints.extend(ARM_JOINTS[side])
            if args.include_gripper:
                self.monitored_joints.extend(GRIPPER_JOINTS[side])

        self.joint_data: Dict[str, JointData] = {
            name: JointData(name) for name in self.monitored_joints
        }

        # Quality of Service: Best effort / keep last for fast sensor streams
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        # 1. Subscribe to actual joint states
        self.sub_joint_states = self.create_subscription(
            JointState, args.state_topic, self._on_joint_states, sensor_qos
        )
        # Also subscribe reliable in case broadcaster publishes reliable
        self.sub_joint_states_reliable = self.create_subscription(
            JointState, args.state_topic, self._on_joint_states, reliable_qos
        )

        # 2. Subscribe to JointTrajectoryControllerState (provides synchronized reference, feedback & error)
        for side in self.sides:
            jtc_state_topic = f"/{side}_joint_trajectory_controller/controller_state"
            self.create_subscription(
                JointTrajectoryControllerState,
                jtc_state_topic,
                lambda msg, s=side: self._on_jtc_state(msg, s),
                reliable_qos,
            )

            # 3. Subscribe to incoming trajectory commands
            jtc_cmd_topic = f"/{side}_joint_trajectory_controller/joint_trajectory"
            self.create_subscription(
                JointTrajectory,
                jtc_cmd_topic,
                lambda msg, s=side: self._on_joint_trajectory(msg, s),
                reliable_qos,
            )

            # 4. Subscribe to forward position controller commands (if used)
            fwd_cmd_topic = f"/{side}_forward_position_controller/commands"
            self.create_subscription(
                Float64MultiArray,
                fwd_cmd_topic,
                lambda msg, s=side: self._on_forward_command(msg, s),
                reliable_qos,
            )

        # 5. Publisher for computed errors
        self.pub_error: Optional[rclpy.publisher.Publisher] = None
        if args.publish_error:
            self.pub_error = self.create_publisher(JointState, "openarm/joint_error", 10)

        # 6. CSV logging setup
        self.csv_file = None
        self.csv_writer = None
        if args.csv:
            csv_path = os.path.abspath(args.csv)
            self.csv_file = open(csv_path, "w", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.csv_file)
            header = ["timestamp_sec"]
            for name in self.monitored_joints:
                header.extend([f"{name}_cmd_rad", f"{name}_act_rad", f"{name}_err_rad", f"{name}_eff_nm"])
            header.extend(["rmse_rad", "mae_rad", "max_abs_err_rad"])
            self.csv_writer.writerow(header)
            self.get_logger().info(f"Logging errors to CSV: {csv_path}")

        # Metrics history
        self.total_cycles = 0
        self.global_peak_error = 0.0
        self.global_peak_joint = ""

        # Timer for display and calculation loop
        period = 1.0 / max(args.rate, 0.1)
        self.timer = self.create_timer(period, self._update_loop)

        self.get_logger().info(
            f"OpenArm Joint Error Monitor initialized for arms: {self.sides}. "
            f"Listening to '{args.state_topic}' and controller command topics."
        )

    # ---- Callback Handlers --------------------------------------------------

    def _on_joint_states(self, msg: JointState):
        """Processes actual joint state measurements."""
        for i, name in enumerate(msg.name):
            if name in self.joint_data:
                pos = msg.position[i] if i < len(msg.position) else None
                vel = msg.velocity[i] if i < len(msg.velocity) else None
                eff = msg.effort[i] if i < len(msg.effort) else None
                if pos is not None:
                    self.joint_data[name].update_actual(pos, vel, eff)

    def _on_jtc_state(self, msg: JointTrajectoryControllerState, side: str):
        """Processes controller reference (setpoint) and feedback from JTC."""
        for i, name in enumerate(msg.joint_names):
            if name in self.joint_data:
                cmd_pos = msg.reference.positions[i] if i < len(msg.reference.positions) else None
                cmd_vel = msg.reference.velocities[i] if i < len(msg.reference.velocities) else None
                if cmd_pos is not None:
                    self.joint_data[name].update_command(cmd_pos, cmd_vel)

    def _on_joint_trajectory(self, msg: JointTrajectory, side: str):
        """Tracks commanded trajectory targets (final point in trajectory)."""
        if not msg.points:
            return
        last_pt = msg.points[-1]
        for i, name in enumerate(msg.joint_names):
            if name in self.joint_data and i < len(last_pt.positions):
                self.joint_data[name].update_command(last_pt.positions[i])

    def _on_forward_command(self, msg: Float64MultiArray, side: str):
        """Tracks commands from forward position controller (array of 7 values)."""
        expected_joints = ARM_JOINTS[side]
        for i, val in enumerate(msg.data):
            if i < len(expected_joints):
                name = expected_joints[i]
                if name in self.joint_data:
                    self.joint_data[name].update_command(val)

    # ---- Display and Error Metrics ------------------------------------------

    def _update_loop(self):
        # Calculate aggregate metrics
        errors = []
        squared_errors = []
        abs_errors = []

        now_stamp = self.get_clock().now()
        now_sec = now_stamp.nanoseconds * 1e-9

        error_msg = JointState() if self.pub_error else None
        if error_msg:
            error_msg.header.stamp = now_stamp.to_msg()

        for name, data in self.joint_data.items():
            if data.error_pos is not None:
                err = data.error_pos
                errors.append(err)
                abs_errors.append(abs(err))
                squared_errors.append(err * err)
                if abs(err) > self.global_peak_error:
                    self.global_peak_error = abs(err)
                    self.global_peak_joint = name

            if error_msg:
                error_msg.name.append(name)
                error_msg.position.append(data.error_pos if data.error_pos is not None else 0.0)
                error_msg.velocity.append(data.act_vel if data.act_vel is not None else 0.0)
                error_msg.effort.append(data.act_eff if data.act_eff is not None else 0.0)

        rmse = math.sqrt(sum(squared_errors) / len(squared_errors)) if squared_errors else 0.0
        mae = (sum(abs_errors) / len(abs_errors)) if abs_errors else 0.0
        max_abs_err = max(abs_errors) if abs_errors else 0.0

        if error_msg:
            self.pub_error.publish(error_msg)

        # Write CSV row
        if self.csv_writer:
            row = [f"{now_sec:.4f}"]
            for name in self.monitored_joints:
                d = self.joint_data[name]
                cmd_str = f"{d.cmd_pos:.5f}" if d.cmd_pos is not None else ""
                act_str = f"{d.act_pos:.5f}" if d.act_pos is not None else ""
                err_str = f"{d.error_pos:.5f}" if d.error_pos is not None else ""
                eff_str = f"{d.act_eff:.3f}" if d.act_eff is not None else ""
                row.extend([cmd_str, act_str, err_str, eff_str])
            row.extend([f"{rmse:.5f}", f"{mae:.5f}", f"{max_abs_err:.5f}"])
            self.csv_writer.writerow(row)

        self.total_cycles += 1
        self._render_dashboard(rmse, mae, max_abs_err)

    def _render_dashboard(self, rmse: float, mae: float, max_abs_err: float):
        use_color = not self.args.no_color
        c_reset = RESET if use_color else ""
        c_bold = BOLD if use_color else ""
        c_green = GREEN if use_color else ""
        c_yellow = YELLOW if use_color else ""
        c_red = RED if use_color else ""
        c_cyan = CYAN if use_color else ""
        c_dim = DIM if use_color else ""

        warn_th = self.args.warn_threshold
        fail_th = self.args.fail_threshold

        def col(text: str, width: int, align: str = "<", color: str = "") -> str:
            """Formats text to strict visual width first, then wraps in ANSI colors."""
            if align == "<":
                padded = f"{text:<{width}}"
            elif align == ">":
                padded = f"{text:>{width}}"
            else:
                padded = f"{text:^{width}}"
            if color and use_color:
                return f"{color}{padded}{c_reset}"
            return padded

        headers = [
            ("Joint Name", 22, "<"),
            ("Command", 24, "^"),
            ("Actual", 24, "^"),
            ("Error (rad / m)", 16, "^"),
            ("Error (deg)", 13, "^"),
            ("Status", 8, "^"),
            ("Effort (Nm)", 13, "^"),
        ]

        sep_line = "+" + "+".join("-" * (w + 2) for _, w, _ in headers) + "+"
        title_text = f" OpenArm Joint Error Monitor ({', '.join(s.upper() for s in self.sides)}) "
        title_bar = f"+{title_text.center(len(sep_line) - 2, '=')}+"

        lines = []
        if self.args.clear_screen:
            lines.append("\033[H\033[J")
        else:
            lines.append("")

        lines.append(f"{c_bold}{c_cyan}{title_bar}{c_reset}")
        lines.append(f"{c_bold}{c_cyan}{sep_line}{c_reset}")

        hdr_cols = [col(h, w, a) for h, w, a in headers]
        hdr_line = "| " + " | ".join(hdr_cols) + " |"
        lines.append(f"{c_bold}{hdr_line}{c_reset}")
        lines.append(sep_line)

        has_commands = any(d.cmd_pos is not None for d in self.joint_data.values())
        has_actuals = any(d.act_pos is not None for d in self.joint_data.values())

        for name in self.monitored_joints:
            d = self.joint_data[name]
            is_finger = "finger" in name

            # 1. Commanded column (width 24)
            if d.cmd_pos is not None:
                if is_finger:
                    cmd_val = f"{d.cmd_pos:>+7.4f} m ({d.cmd_pos * 1000:>4.1f} mm)"
                else:
                    cmd_val = f"{d.cmd_pos:>+7.4f} rad ({math.degrees(d.cmd_pos):>+5.1f}°)"
                cmd_col = col(cmd_val, 24, "^")
            else:
                cmd_col = col("No command", 24, "^", c_dim)

            # 2. Actual column (width 24)
            if d.act_pos is not None:
                if is_finger:
                    act_val = f"{d.act_pos:>+7.4f} m ({d.act_pos * 1000:>4.1f} mm)"
                else:
                    act_val = f"{d.act_pos:>+7.4f} rad ({math.degrees(d.act_pos):>+5.1f}°)"
                act_col = col(act_val, 24, "^")
            else:
                act_col = col("No data", 24, "^", c_dim)

            # 3. Error columns & Status (width 16, 13, 8)
            if d.error_pos is not None:
                err_val = d.error_pos
                abs_err = abs(err_val)

                if abs_err >= fail_th:
                    status_text = "FAIL"
                    err_color = f"{c_red}{c_bold}"
                elif abs_err >= warn_th:
                    status_text = "WARN"
                    err_color = f"{c_yellow}{c_bold}"
                else:
                    status_text = "OK"
                    err_color = c_green

                if is_finger:
                    err_rad_str = f"{err_val:>+9.4f} m"
                    err_deg_str = "--"
                else:
                    err_rad_str = f"{err_val:>+9.4f} rad"
                    err_deg_str = f"{math.degrees(err_val):>+7.2f}°"

                err_rad_col = col(err_rad_str, 16, "^", err_color)
                err_deg_col = col(err_deg_str, 13, "^", err_color if not is_finger else c_dim)
                status_col = col(status_text, 8, "^", err_color)
            else:
                err_rad_col = col("--", 16, "^", c_dim)
                err_deg_col = col("--", 13, "^", c_dim)
                status_col = col("WAIT", 8, "^", c_dim)

            # 4. Effort column (width 13)
            if d.act_eff is not None:
                eff_val = f"{d.act_eff:>+7.2f} Nm"
                eff_col = col(eff_val, 13, "^")
            else:
                eff_col = col("--", 13, "^", c_dim)

            # 5. Joint Name column (width 22)
            display_name = name.replace("openarm_", "")
            name_col = col(display_name, 22, "<")

            row_line = (
                f"| {name_col} | {cmd_col} | {act_col} | "
                f"{err_rad_col} | {err_deg_col} | {status_col} | {eff_col} |"
            )
            lines.append(row_line)

        lines.append(sep_line)

        # Status warning if topics not yet publishing
        inner_width = len(sep_line) - 4
        if not has_actuals:
            msg = f"[!] Waiting for '{self.args.state_topic}'... Are the arm motors powered and bringup launched?"
            lines.append(f"| {col(msg, inner_width, '<', c_yellow)} |")
            lines.append(sep_line)
        elif not has_commands:
            msg = "[i] Actual joint states received. Waiting for commands on controller topics..."
            lines.append(f"| {col(msg, inner_width, '<', c_yellow)} |")
            lines.append(sep_line)
        else:
            metrics_line = (
                f"Tracking Metrics:  RMSE: {rmse:.4f} rad ({math.degrees(rmse):.2f}°) | "
                f"MAE: {mae:.4f} rad ({math.degrees(mae):.2f}°) | "
                f"Max Error: {max_abs_err:.4f} rad ({math.degrees(max_abs_err):.2f}°)"
            )
            lines.append(f"| {col(metrics_line, inner_width, '<', c_bold)} |")
            if self.global_peak_joint:
                peak_line = (
                    f"Session Peak Error: {self.global_peak_error:.4f} rad "
                    f"({math.degrees(self.global_peak_error):.2f}°) on '{self.global_peak_joint.replace('openarm_', '')}'"
                )
                lines.append(f"| {col(peak_line, inner_width, '<', c_dim)} |")
            lines.append(sep_line)

        output_text = "\n".join(lines)
        sys.stdout.write(output_text + "\n")
        sys.stdout.flush()

    def print_final_summary(self):
        """Prints a summary upon termination."""
        print("\n" + "=" * 60)
        print(" OpenArm Joint Error Monitor - Session Summary")
        print("=" * 60)
        print(f" Total update cycles: {self.total_cycles}")
        print(f" Global Peak Error:   {self.global_peak_error:.4f} rad ({math.degrees(self.global_peak_error):.2f}°)")
        if self.global_peak_joint:
            print(f" Worst-performing:    {self.global_peak_joint}")
        print("-" * 60)
        for name in self.monitored_joints:
            d = self.joint_data[name]
            curr_err = f"{d.error_pos:+.4f} rad" if d.error_pos is not None else "N/A"
            peak_err = f"{d.peak_error_pos:.4f} rad ({math.degrees(d.peak_error_pos):.2f}°)"
            print(f" {name:<28} | Final Err: {curr_err:<12} | Peak Err: {peak_err}")
        print("=" * 60)
        if self.csv_file:
            self.csv_file.close()
            print(f" CSV log saved to: {self.args.csv}")


def parse_arguments(argv):
    parser = argparse.ArgumentParser(
        description="Monitor OpenArm joint positions and calculate commanded vs actual error."
    )
    parser.add_argument(
        "--arm",
        choices=["left", "right", "both"],
        default="both",
        help="Arm to monitor (default: both)",
    )
    parser.add_argument(
        "--state-topic",
        default="/joint_states",
        help="Topic for measured joint states (default: /joint_states)",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=10.0,
        help="Display update rate in Hz (default: 10.0)",
    )
    parser.add_argument(
        "--warn-threshold",
        type=float,
        default=0.035,
        help="Error warning threshold in radians (default: 0.035 rad ~ 2.0 deg)",
    )
    parser.add_argument(
        "--fail-threshold",
        type=float,
        default=0.087,
        help="Error failure threshold in radians (default: 0.087 rad ~ 5.0 deg)",
    )
    parser.add_argument(
        "--include-gripper",
        action="store_true",
        default=True,
        help="Include finger gripper joint in the monitor (default: True)",
    )
    parser.add_argument(
        "--no-gripper",
        dest="include_gripper",
        action="store_false",
        help="Exclude finger gripper joint",
    )
    parser.add_argument(
        "--publish-error",
        action="store_true",
        help="Publish computed joint errors to /openarm/joint_error (sensor_msgs/JointState)",
    )
    parser.add_argument(
        "--csv",
        type=str,
        default=None,
        help="File path to save error time-series CSV log",
    )
    parser.add_argument(
        "--clear-screen",
        action="store_true",
        default=True,
        help="Clear terminal between frames for a stable dashboard display (default: True)",
    )
    parser.add_argument(
        "--no-clear-screen",
        dest="clear_screen",
        action="store_false",
        help="Do not clear screen between frames (stream scrolling log)",
    )
    parser.add_argument(
        "--no-color",
        action="store_true",
        help="Disable ANSI color codes in console output",
    )
    return parser.parse_known_args(argv)


def main():
    parsed_args, ros_args = parse_arguments(sys.argv[1:])
    rclpy.init(args=ros_args)
    node = OpenArmJointErrorMonitor(parsed_args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.print_final_summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
