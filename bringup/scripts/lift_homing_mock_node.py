#!/usr/bin/env python3
"""
Mock lift homing node — same ROS 2 interface as lift_homing_node.py but
does not touch the CAN bus. Useful for simulation and development without hardware.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class TMotorBridgeMockNode(Node):

    def __init__(self):
        super().__init__('tmotor_ros2_bridge')

        self.declare_parameter('control_frequency', 100.0)
        self.declare_parameter('max_velocity_cm_s', 5.0)
        self.declare_parameter('max_acceleration_cm_s2', 20.0)
        self.declare_parameter('hardware_max_cm_s', 3.0)
        self.declare_parameter('lift_min_cm', 0.0)
        self.declare_parameter('lift_max_cm', 74.35)
        self.declare_parameter('joint_name', 'lift_joint')
        self.declare_parameter('topic_joint_states', 'lift/joint_states')
        self.declare_parameter('topic_lift_controller_commands', 'lift_controller/commands')
        self.declare_parameter('topic_position_commands', 'lift/cmd')
        self.control_frequency = self.get_parameter('control_frequency').value
        self.max_vel_cms = self.get_parameter('max_velocity_cm_s').value
        self.max_acc_cms2 = self.get_parameter('max_acceleration_cm_s2').value
        self.hw_max_vel_cms = self.get_parameter('hardware_max_cm_s').value
        self.lift_min_cm = self.get_parameter('lift_min_cm').value
        self.lift_max_cm = self.get_parameter('lift_max_cm').value
        self.joint_name = self.get_parameter('joint_name').value
        self.lift_top_m = self.lift_max_cm * 0.01

        topic_js = self.get_parameter('topic_joint_states').value
        topic_lc = self.get_parameter('topic_lift_controller_commands').value
        topic_pc = self.get_parameter('topic_position_commands').value

        self.state_pub = self.create_publisher(JointState, topic_js, 10)
        self.lift_ctrl_pub = self.create_publisher(Float64MultiArray, topic_lc, 10)
        self.cmd_sub = self.create_subscription(
            Float64MultiArray, topic_pc, self.cmd_callback, 10
        )

        self.state = 'IDLE'
        self.traj_target_m = self.lift_top_m
        self.traj_max_vel_ms = 0.0
        self.traj_max_acc_ms2 = 0.0
        self.traj_current_pos_m = self.lift_top_m
        self.traj_current_vel_ms = 0.0
        self.traj_active = False
        self._last_traj_time = None

        self._run_mock_homing()

        timer_period = 1.0 / self.control_frequency
        self.timer = self.create_timer(timer_period, self.loop_callback)

    def _run_mock_homing(self):
        self.get_logger().info('[MOCK] Homing sequence skipped (no CAN bus). Setting origin at top of travel.')
        self.state = 'CONTROL'

    def cmd_callback(self, msg):
        if self.state != 'CONTROL':
            return
        data = list(msg.data)
        if not data:
            self.get_logger().warn('Empty command array — ignoring')
            return

        pos_cm = data[0]
        vel_cms = abs(data[1]) if len(data) > 1 else min(self.max_vel_cms, self.hw_max_vel_cms)
        acc_cms2 = abs(data[2]) if len(data) > 2 else self.max_acc_cms2

        if pos_cm < self.lift_min_cm or pos_cm > self.lift_max_cm:
            self.get_logger().error(
                f'pos {pos_cm:.2f}cm outside limits [{self.lift_min_cm:.2f}, {self.lift_max_cm:.2f}] — rejecting'
            )
            return

        vel_cms = min(vel_cms, self.max_vel_cms, self.hw_max_vel_cms)
        acc_cms2 = min(acc_cms2, self.max_acc_cms2)
        if vel_cms <= 0.0 or acc_cms2 <= 0.0:
            self.get_logger().warn(f'vel {vel_cms} or acc {acc_cms2} non-positive — ignoring')
            return

        self.traj_target_m = pos_cm * 0.01
        self.traj_max_vel_ms = vel_cms * 0.01
        self.traj_max_acc_ms2 = acc_cms2 * 0.01
        self.traj_active = True
        self.get_logger().info(f'[MOCK] cmd → {pos_cm:.2f}cm @ {vel_cms:.3f}cm/s')

    def _advance_trajectory(self, dt):
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
            new_v = max(-v_max, min(v_max, new_v))

        self.traj_current_vel_ms = new_v
        self.traj_current_pos_m += new_v * dt

    def loop_callback(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_traj_time is None:
            dt = 1.0 / self.control_frequency
        else:
            dt = now - self._last_traj_time
        self._last_traj_time = now
        if dt <= 0.0 or dt > 0.5:
            dt = 1.0 / self.control_frequency

        if self.state == 'CONTROL':
            self._advance_trajectory(dt)

        js_msg = JointState()
        js_msg.header.stamp = self.get_clock().now().to_msg()
        js_msg.name = [self.joint_name]
        js_msg.position = [self.traj_current_pos_m]
        js_msg.velocity = [self.traj_current_vel_ms]
        js_msg.effort = [0.0]
        self.state_pub.publish(js_msg)

        lift_cmd = Float64MultiArray()
        lift_cmd.data = [self.traj_current_pos_m]
        self.lift_ctrl_pub.publish(lift_cmd)

    def shutdown(self):
        self.get_logger().info('[MOCK] Shutting down.')


def main(args=None):
    rclpy.init(args=args)
    node = TMotorBridgeMockNode()
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
