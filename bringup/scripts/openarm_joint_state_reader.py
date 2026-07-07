#!/usr/bin/env python3
"""
Free-drive joint state reader for the OpenArm bimanual arms.

Polls the Damiao motors with refresh queries only (motors are never enabled,
and are disabled on startup by default), so the arms stay torque-free and can
be moved by hand while their joint states are published as
sensor_msgs/JointState.

WARNING: do NOT run this while the real bringup (real_omnibot.launch.py /
openarm.bimanual.launch.py) is active. It shares the SocketCAN bus with
ros2_control and disables the motors on startup, which would drop a
controlled arm.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import openarm_can as oa

# V10 motor layout, mirrors openarm_hardware/v10_simple_hardware.hpp
ARM_MOTOR_TYPES = [
    oa.MotorType.DM8009,  # joint1
    oa.MotorType.DM8009,  # joint2
    oa.MotorType.DM4340,  # joint3
    oa.MotorType.DM4340,  # joint4
    oa.MotorType.DM4310,  # joint5
    oa.MotorType.DM4310,  # joint6
    oa.MotorType.DM4310,  # joint7
]
ARM_SEND_CAN_IDS = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]
ARM_RECV_CAN_IDS = [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17]
GRIPPER_MOTOR_TYPE = oa.MotorType.DM4310
GRIPPER_SEND_CAN_ID = 0x08
GRIPPER_RECV_CAN_ID = 0x18

# Gripper motor->joint mapping, mirrors OpenArm_v10HW::motor_radians_to_joint:
# motor 0 rad = closed = 0 m, motor -1.0472 rad = open = 0.044 m.
GRIPPER_SCALE = 0.044 / -1.0472


class OpenArmJointStateReader(Node):
    def __init__(self):
        super().__init__('openarm_joint_state_reader')

        self.declare_parameter('arms', 'both')  # both | left | right
        self.declare_parameter('left_can_interface', 'can1')
        self.declare_parameter('right_can_interface', 'can0')
        self.declare_parameter('can_fd', True)
        self.declare_parameter('hand', True)
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('disable_on_start', True)
        self.declare_parameter('joint_states_topic', 'openarm/joint_states')
        # > 0.0 also logs the positions to the console at this period (s)
        self.declare_parameter('print_throttle_s', 0.0)

        arms = self.get_parameter('arms').value
        can_fd = self.get_parameter('can_fd').value
        self.hand = self.get_parameter('hand').value
        self.print_throttle_s = self.get_parameter('print_throttle_s').value
        disable_on_start = self.get_parameter('disable_on_start').value

        sides = {
            'both': ['left', 'right'],
            'left': ['left'],
            'right': ['right'],
        }.get(arms)
        if sides is None:
            raise ValueError(f"invalid 'arms' parameter: {arms!r} (use both|left|right)")

        self.arms = []  # list of (prefix, oa.OpenArm)
        for side in sides:
            can_if = self.get_parameter(f'{side}_can_interface').value
            try:
                arm = oa.OpenArm(can_if, can_fd)
                arm.init_arm_motors(ARM_MOTOR_TYPES, ARM_SEND_CAN_IDS, ARM_RECV_CAN_IDS)
                if self.hand:
                    arm.init_gripper_motor(
                        GRIPPER_MOTOR_TYPE, GRIPPER_SEND_CAN_ID, GRIPPER_RECV_CAN_ID)
                arm.set_callback_mode_all(oa.CallbackMode.STATE)
                if disable_on_start:
                    arm.disable_all()
                    arm.recv_all()
            except Exception as e:
                self.get_logger().error(
                    f'{side} arm on {can_if} unavailable, skipping: {e}')
                continue
            self.get_logger().info(
                f'{side} arm on {can_if}: reading in free-drive'
                f'{" (motors disabled)" if disable_on_start else ""}')
            self.arms.append((side, can_if, arm))

        if not self.arms:
            raise RuntimeError('no arm reachable on any CAN interface')

        topic = self.get_parameter('joint_states_topic').value
        self.pub = self.create_publisher(JointState, topic, 10)
        self.first_poll = True
        self.last_print = self.get_clock().now()

        rate_hz = self.get_parameter('rate_hz').value
        self.create_timer(1.0 / rate_hz, self.poll)

    def poll(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        readings = []  # (side, can_if, [7 joint rad], gripper_m or None)
        for side, can_if, arm in self.arms:
            arm.refresh_all()
            arm.recv_all()
            joints = [m.get_position() for m in arm.get_arm().get_motors()]
            for i, motor in enumerate(arm.get_arm().get_motors()):
                msg.name.append(f'openarm_{side}_joint{i + 1}')
                msg.position.append(motor.get_position())
                msg.velocity.append(motor.get_velocity())
                msg.effort.append(motor.get_torque())
            gripper_m = None
            if self.hand:
                gripper_motors = arm.get_gripper().get_motors()
                if gripper_motors:
                    m = gripper_motors[0]
                    gripper_m = m.get_position() * GRIPPER_SCALE
                    msg.name.append(f'openarm_{side}_finger_joint1')
                    msg.position.append(gripper_m)
                    msg.velocity.append(m.get_velocity() * GRIPPER_SCALE)
                    msg.effort.append(m.get_torque())
            readings.append((side, can_if, joints, gripper_m))
        self.pub.publish(msg)

        if self.first_poll:
            self.first_poll = False
            if msg.position and all(p == 0.0 for p in msg.position):
                self.get_logger().warn(
                    'all positions are exactly zero — no state replies yet? '
                    'Check that the arms are powered and the CAN bus is up.')

        if self.print_throttle_s > 0.0:
            now = self.get_clock().now()
            if (now - self.last_print).nanoseconds >= self.print_throttle_s * 1e9:
                self.last_print = now
                self.get_logger().info(self.format_readings(readings))

    @staticmethod
    def format_readings(readings):
        lines = ['']
        for side, can_if, joints, gripper_m in readings:
            lines.append(f'{f"{side} arm ({can_if})":<17}{"rad":>10} {"deg":>10}')
            for i, rad in enumerate(joints):
                lines.append(
                    f'  joint{i + 1:<10}{rad:>10.4f} {math.degrees(rad):>10.2f}')
            if gripper_m is not None:
                lines.append(
                    f'  {"gripper":<15}{gripper_m:>10.4f} m '
                    f'({gripper_m * 1000:.1f} mm open)')
            preset = ', '.join(f'{rad:.4f}' for rad in joints)
            lines.append(f'  preset (rad): [{preset}]')
        return '\n'.join(lines)


def main():
    rclpy.init()
    try:
        node = OpenArmJointStateReader()
    except Exception as e:
        print(f'[openarm_joint_state_reader] failed to start: {e}')
        rclpy.try_shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
