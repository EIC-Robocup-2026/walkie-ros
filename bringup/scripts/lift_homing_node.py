#!/usr/bin/env python3
"""
ROS 2 Bridge Node for TMotorCANControl with Homing Logic.
Integrates the homing sequence from demo_ak45_10_speed_until_current_diff.py.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import math
import time
import numpy as np
from TMotorCANControl.servo_can import TMotorManager_servo_can

class TMotorBridgeNode(Node):
    # int16 limit on (vel|acc) / radps_per_ERPM (5.82e-4) → ~19.07 rad/{s,s²}
    VEL_ACC_LIMIT_RAD = 19.0

    def __init__(self):
        super().__init__('tmotor_ros2_bridge')

        # ---- Parameters (can be overridden via ROS 2 params) ----
        self.declare_parameter('motor_id', 16)
        self.declare_parameter('motor_type', 'AK45-10')
        self.declare_parameter('homing_erpm', -8000)
        self.declare_parameter('current_diff_limit', 0.35)
        self.declare_parameter('abs_current_cap', 4.0)
        self.declare_parameter('control_frequency', 100.0) # Hz
        self.declare_parameter('max_velocity_cm_s', 5.0)
        self.declare_parameter('max_acceleration_cm_s2', 20.0)
        self.declare_parameter('homing_backoff_time_s', 1.0)
        # Hard ceiling for how fast the lift can physically sustain. RViz
        # publishes the trajectory directly, so if traj velocity exceeds what
        # the motor can deliver the visualization runs ahead of reality.
        # Measure your real sustained cm/s and set this to that.
        self.declare_parameter('hardware_max_cm_s', 3.0)

        # ---- Lift kinematics (URDF: prismatic, lower=0.000m, upper=0.7435m) ----
        # Position convention matches the URDF: 0 = bottom, lift_max_cm/100 = top.
        # The motor is homed against the upper hard stop, so motor_rad = 0 corresponds
        # to the lift at the top (position = lift_max_cm/100). lift_m_per_rad is signed:
        # negative because driving the motor positive (away from the upper hard stop)
        # moves the lift downward (position decreasing toward 0).
        self.declare_parameter('lift_m_per_rad', 0.0)
        self.declare_parameter('lift_min_cm', 0.0)
        self.declare_parameter('lift_max_cm', 74.35)
        self.declare_parameter('joint_name', 'lift_joint')
        self.declare_parameter('topic_joint_states', 'lift/joint_states')
        self.declare_parameter('topic_lift_controller_commands', 'lift_controller/commands')
        self.declare_parameter('topic_position_commands', 'forward_position_controller/commands')

        self.motor_id = self.get_parameter('motor_id').value
        self.motor_type = self.get_parameter('motor_type').value
        self.homing_erpm = self.get_parameter('homing_erpm').value
        self.diff_limit_A = self.get_parameter('current_diff_limit').value
        self.abs_cap_A = self.get_parameter('abs_current_cap').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.max_vel_cms = self.get_parameter('max_velocity_cm_s').value
        self.max_acc_cms2 = self.get_parameter('max_acceleration_cm_s2').value
        self.backoff_time_s = self.get_parameter('homing_backoff_time_s').value
        self.hw_max_vel_cms = self.get_parameter('hardware_max_cm_s').value
        self.m_per_rad = self.get_parameter('lift_m_per_rad').value
        self.lift_min_cm = self.get_parameter('lift_min_cm').value
        self.lift_max_cm = self.get_parameter('lift_max_cm').value
        self.joint_name = self.get_parameter('joint_name').value
        self.topic_joint_states = self.get_parameter('topic_joint_states').value
        self.topic_lift_ctrl_cmds = self.get_parameter('topic_lift_controller_commands').value
        self.topic_pos_cmds = self.get_parameter('topic_position_commands').value
        # Lift position (m) at the homed origin (motor_rad = 0). With the upper-stop
        # homing scheme this is the top of travel, i.e. lift_max_cm in meters.
        self.lift_top_m = self.lift_max_cm * 0.01
        if self.m_per_rad == 0.0:
            self.get_logger().error(
                "lift_m_per_rad is 0.0 — set this param (signed m/rad) before commanding. Commands will be rejected."
            )

        # ---- Publishers & Subscribers ----
        self.state_pub = self.create_publisher(JointState, self.topic_joint_states, 10)
        # Position command stream for a ros2_control lift_controller
        # (position_controllers/JointGroupPositionController on lift_joint).
        # With mock_components/GenericSystem the command loops back through
        # joint_state_broadcaster to /joint_states, driving RViz.
        self.lift_ctrl_pub = self.create_publisher(
            Float64MultiArray, self.topic_lift_ctrl_cmds, 10
        )
        self.cmd_sub = self.create_subscription(Float64MultiArray, self.topic_pos_cmds, self.cmd_callback, 10)

        # ---- Motor Initialization ----
        self.dev = TMotorManager_servo_can(motor_type=self.motor_type, motor_ID=self.motor_id)
        self.get_logger().info(f"Initializing {self.motor_type} on ID {self.motor_id}...")

        # Internal state
        self.state = 'IDLE' # IDLE, HOMING, CONTROL

        # SW trapezoidal trajectory state (all SI units: meters, seconds).
        # We shape motion in this node and feed the motor a slowly-advancing
        # position setpoint, because the lib's vel/acc CAN fields use a
        # different internal unit scale than the position field — calibrating
        # m_per_rad fixes position but leaves vel/acc ~100× too aggressive.
        self.traj_target_m = self.lift_top_m
        self.traj_max_vel_ms = 0.0
        self.traj_max_acc_ms2 = 0.0
        self.traj_current_pos_m = self.lift_top_m  # starts at homed origin (top)
        self.traj_current_vel_ms = 0.0
        self.traj_active = False
        self._last_traj_time = None

        # Motor pos-vel CAN field caps. Pinned to the int16 ceiling so the
        # motor will follow whatever tiny position step the SW trajectory
        # produces each loop without bottlenecking — actual lift speed is
        # governed by how fast traj_current_pos_m advances, not by these.
        self.MOTOR_VEL_CAP_LIB = self.VEL_ACC_LIMIT_RAD
        self.MOTOR_ACC_CAP_LIB = self.VEL_ACC_LIMIT_RAD

        # Enter motor context manually
        self.dev.__enter__()

        # Start the process
        self.run_homing_sequence()

        # Start periodic control/state loop
        timer_period = 1.0 / self.control_frequency
        self.timer = self.create_timer(timer_period, self.loop_callback)

    def run_homing_sequence(self):
        """
        Implements the logic from demo_ak45_10_speed_until_current_diff.py
        This is a blocking call performed during initialization.

        Phase 0: Idle-current baseline (motor enabled, velocity=0)
        Phase 1: Back off in opposite direction (in case we started at the hard stop)
        Phase 2: Drive toward hard stop; detect via current rise above running baseline
        """
        self.get_logger().info("Starting Homing Sequence...")

        SETTLE_TIME_S = 0.4
        BASELINE_WINDOW_S = 0.5
        IDLE_SAMPLE_S = 0.9
        BACKOFF_SETTLE_S = 0.6

        self.dev.enter_velocity_control()

        # --- Phase 0: Sample idle baseline (no motion) ---
        self.dev._command.velocity = 0
        idle_samples = []
        t0 = time.time()
        while time.time() - t0 < IDLE_SAMPLE_S:
            self.dev.update()
            idle_samples.append(abs(self.dev.get_current_qaxis_amps()))
            time.sleep(0.01)
        idle_baseline = sum(idle_samples) / max(len(idle_samples), 1)
        self.get_logger().info(f"Idle baseline: {idle_baseline:.3f}A")

        # Sanity check: if motor draws high current at rest, it's likely jammed at the stop already
        if idle_baseline > self.diff_limit_A:
            self.get_logger().warn(
                f"Idle current ({idle_baseline:.2f}A) exceeds diff limit ({self.diff_limit_A:.2f}A) — likely at hard stop. Backing off."
            )

        # --- Phase 1: Back off in opposite direction ---
        backoff_erpm = -self.homing_erpm
        self.get_logger().info(f"Backing off at {backoff_erpm} ERPM for {self.backoff_time_s:.2f}s...")
        self.dev._command.velocity = backoff_erpm
        t_bo = time.time()
        while time.time() - t_bo < self.backoff_time_s:
            self.dev.update()
            abs_i_q = abs(self.dev.get_current_qaxis_amps())
            if abs_i_q > self.abs_cap_A:
                self.get_logger().error(
                    f"Backoff aborted: current cap hit ({abs_i_q:.2f}A) — possible opposite hard stop."
                )
                self.dev.enter_idle_mode()
                self.state = 'IDLE'
                return
            time.sleep(0.01)

        # Brief settle at velocity=0 after backoff
        self.dev._command.velocity = 0
        t_st = time.time()
        while time.time() - t_st < BACKOFF_SETTLE_S:
            self.dev.update()
            time.sleep(0.01)

        # --- Phase 2: Drive toward hard stop ---
        self.get_logger().info(f"Driving toward hard stop at {self.homing_erpm} ERPM (waiting indefinitely)...")
        self.dev._command.velocity = self.homing_erpm

        t_start = time.time()
        baseline = None
        baseline_samples = []
        homed = False

        while True:
            self.dev.update()
            i_q = self.dev.get_current_qaxis_amps()
            abs_i_q = abs(i_q)
            t_elapsed = time.time() - t_start

            # Absolute safety cap
            if abs_i_q > self.abs_cap_A:
                self.get_logger().error(f"Homing failed: Absolute current cap hit ({abs_i_q:.2f}A)")
                break
            if t_elapsed < SETTLE_TIME_S:
                continue
            elif t_elapsed < SETTLE_TIME_S + BASELINE_WINDOW_S:
                baseline_samples.append(abs_i_q)
            else:
                if baseline is None:
                    if not baseline_samples:
                        self.get_logger().error("Homing failed: No baseline samples")
                        break
                    baseline = sum(baseline_samples) / len(baseline_samples)
                    self.get_logger().info(f"Baseline set: {baseline:.3f}A")

                # Check for trigger
                if abs_i_q - baseline > self.diff_limit_A:
                    self.get_logger().info(f"Hard stop detected (diff: {abs_i_q - baseline:.2f}A)")
                    homed = True
                    break

            time.sleep(0.002)  # 500 Hz polling — catch current rise quickly to reduce penetration

        if homed:
            # Active reverse brake: command opposite ERPM briefly to dissipate kinetic energy
            BRAKE_ERPM = -self.homing_erpm  # opposite direction
            BRAKE_PULSE_S = 0.1
            self.dev._command.velocity = BRAKE_ERPM
            t_br = time.time()
            while time.time() - t_br < BRAKE_PULSE_S:
                self.dev.update()
                time.sleep(0.002)

            # Then settle at velocity=0
            self.dev._command.velocity = 0
            for _ in range(30):  # 0.3s settle
                self.dev.update()
                time.sleep(0.01)

            self.dev._canman.comm_can_set_origin(self.dev.ID, 1) # Permanent Zero
            time.sleep(0.1)
            self.dev.update()
            self.get_logger().info(f"Homing Successful. Origin set. Current Position: {self.dev.get_motor_angle_radians():.2f} rad")

            # Transition to position-velocity control mode for ROS commands
            self.dev.enter_position_velocity_control()
            self.state = 'CONTROL'
        else:
            self.get_logger().error("Homing failed or timed out. Motor staying in IDLE.")
            self.dev.enter_idle_mode()
            self.state = 'IDLE'

    def cmd_callback(self, msg):
        """Set trajectory goal from [pos_cm, vel_cm_s, acc_cm_s2]. Vel & acc optional."""
        if self.state != 'CONTROL':
            return
        if self.m_per_rad == 0.0:
            self.get_logger().warn("Cannot command: lift_m_per_rad is 0.0")
            return
        data = list(msg.data)
        if len(data) == 0:
            self.get_logger().warn("Empty command array — ignoring")
            return

        pos_cm = data[0]
        vel_cms = abs(data[1]) if len(data) > 1 else min(self.max_vel_cms, self.hw_max_vel_cms)
        acc_cms2 = abs(data[2]) if len(data) > 2 else self.max_acc_cms2

        if pos_cm < self.lift_min_cm or pos_cm > self.lift_max_cm:
            self.get_logger().error(
                f"pos {pos_cm:.2f}cm outside limits [{self.lift_min_cm:.2f}, {self.lift_max_cm:.2f}] — rejecting"
            )
            return

        # Apply both the soft user limit and the hard hardware ceiling so the
        # SW trajectory (which is also what RViz visualizes) cannot outrun
        # what the motor can physically follow.
        vel_cms = min(vel_cms, self.max_vel_cms, self.hw_max_vel_cms)
        acc_cms2 = min(acc_cms2, self.max_acc_cms2)
        if vel_cms <= 0.0 or acc_cms2 <= 0.0:
            self.get_logger().warn(
                f"vel {vel_cms} or acc {acc_cms2} non-positive — ignoring"
            )
            return

        self.traj_target_m = pos_cm * 0.01
        self.traj_max_vel_ms = vel_cms * 0.01
        self.traj_max_acc_ms2 = acc_cms2 * 0.01
        self.traj_active = True

        self.get_logger().info(
            f"cmd → {pos_cm:.2f}cm @ {vel_cms:.3f}cm/s, acc {acc_cms2:.3f}cm/s² (SW traj)"
        )

    def _advance_trajectory(self, dt):
        """Step the trapezoidal velocity profile toward traj_target_m by dt seconds."""
        if not self.traj_active:
            return

        error = self.traj_target_m - self.traj_current_pos_m
        v = self.traj_current_vel_ms
        a = self.traj_max_acc_ms2
        v_max = self.traj_max_vel_ms

        if abs(error) < 1e-5 and abs(v) < 1e-4:
            self.traj_current_pos_m = self.traj_target_m
            self.traj_current_vel_ms = 0.0
            self.traj_active = False
            return

        dir_to_target = 1.0 if error > 0 else -1.0
        moving_wrong_way = (v * dir_to_target) < 0
        stop_dist = (v * v) / (2.0 * a) if a > 0 else 0.0

        if moving_wrong_way:
            new_v = v + dir_to_target * a * dt
        elif abs(error) <= stop_dist:
            if v > 0:
                new_v = max(0.0, v - a * dt)
            elif v < 0:
                new_v = min(0.0, v + a * dt)
            else:
                new_v = 0.0
        else:
            new_v = v + dir_to_target * a * dt
            if new_v > v_max:
                new_v = v_max
            elif new_v < -v_max:
                new_v = -v_max

        self.traj_current_vel_ms = new_v
        self.traj_current_pos_m += new_v * dt

    def loop_callback(self):
        """High-frequency control loop and state publisher."""
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_traj_time is None:
            dt = 1.0 / self.control_frequency
        else:
            dt = now - self._last_traj_time
        self._last_traj_time = now
        # Guard against clock jumps / first-tick weirdness
        if dt <= 0.0 or dt > 0.5:
            dt = 1.0 / self.control_frequency

        if self.state == 'CONTROL':
            self._advance_trajectory(dt)
            # URDF position → motor angle: subtract the homed-origin offset so
            # motor_rad = 0 when traj_current_pos_m == lift_top_m.
            target_pos_rad = (self.traj_current_pos_m - self.lift_top_m) / self.m_per_rad
            self.dev.set_output_angle_radians(
                target_pos_rad, self.MOTOR_VEL_CAP_LIB, self.MOTOR_ACC_CAP_LIB
            )

        # Always update and publish state
        self.dev.update()

        # Publish the SW-shaped trajectory as the joint position. The
        # trajectory is the authoritative motion source we feed to the motor
        # (see comment above MOTOR_VEL_CAP_LIB), so it matches commanded
        # reality. We avoid integrating motor velocity feedback because the
        # TMotor library's get_output_velocity_radians_per_second() uses a
        # hard-coded radps_per_ERPM that doesn't match AK45-10's pole/gear
        # configuration — integration would be wildly off-scale either way.
        js_msg = JointState()
        js_msg.header.stamp = self.get_clock().now().to_msg()
        js_msg.name = [self.joint_name]
        js_msg.position = [self.traj_current_pos_m]
        js_msg.velocity = [self.traj_current_vel_ms]
        js_msg.effort = [self.dev.get_motor_torque_newton_meters()]
        self.state_pub.publish(js_msg)

        # Mirror the trajectory position to the ros2_control lift_controller.
        # JointGroupPositionController expects a Float64MultiArray whose data
        # array length matches the controller's joint list (here: [lift_joint]).
        lift_cmd = Float64MultiArray()
        lift_cmd.data = [self.traj_current_pos_m]
        self.lift_ctrl_pub.publish(lift_cmd)

    def shutdown(self):
        self.get_logger().info("Shutting down bridge...")
        self.dev.enter_idle_mode()
        self.dev.update()
        self.dev.__exit__(None, None, None)

def main(args=None):
    rclpy.init(args=args)
    node = TMotorBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
