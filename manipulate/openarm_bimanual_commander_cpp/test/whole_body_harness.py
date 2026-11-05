"""Shared bring-up and client of the whole-body commander launch tests.

The bring-up runs the real move_group and commander against whole_body_stub_world.py.
"""
import json
import math
import os
import threading
import time

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from my_robot_interfaces.action import GoToHome, GoToPose
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

FAR_TARGET = (1.75, 0.15, 1.05)
FAR_TARGET_RIGHT = (1.75, -0.15, 1.05)
# Exact IK solves this one from the home state with the base where it is.
NEAR_TARGET = (0.50, 0.20, 0.90)
# The target of the arm-only near step of the demo. An arm group reaches it, within its
# goal tolerances.
NEAR_ARM_TARGET = (0.45, 0.25, 1.00)
FORWARD_PITCH = math.pi / 2.0
BASE_CONTROLLER = 'base_controller'
LEFT_ARM = 'left_joint_trajectory_controller'
RIGHT_ARM = 'right_joint_trajectory_controller'
# What the stub world gives as the reason of an abort, followed by the controller name.
ABORT_REASON = 'stub world aborted '
# The controllers each arm group has to reach. The lift only moves if the named state
# differs in lift height, so it is not required here.
ARM_GROUPS = {
    'left_arm': {LEFT_ARM},
    'right_arm': {RIGHT_ARM},
    'both_arms': {LEFT_ARM, RIGHT_ARM},
    'left_arm_lift': {LEFT_ARM},
    'right_arm_lift': {RIGHT_ARM},
    'both_arms_lift': {LEFT_ARM, RIGHT_ARM},
}


def bringup(move_group_mode, commander_mode, with_base_server=True, plan_only=False,
            commander_sim_time=False):
    """Return the launch actions. Each waits for what the next one needs at startup."""
    moveit_config = (
        MoveItConfigsBuilder('openarm', package_name='openarm_bimanual_moveit_config')
        .robot_description(file_path='config/openarm_bimanual.urdf.xacro')
        .robot_description_semantic(
            file_path='config/openarm_bimanual.srdf', mappings={'base_mode': move_group_mode})
        .to_moveit_configs()
    )
    stub_world = Node(
        package='openarm_bimanual_commander_cpp',
        executable='whole_body_stub_world',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            {'with_base_server': with_base_server},
        ],
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='log',
        parameters=[moveit_config.robot_description],
    )
    wait_for_world = Node(
        package='openarm_bimanual_moveit_config',
        executable='wait_for_graph.py',
        name='wait_for_world',
        output='screen',
        arguments=['--action', f'/{LEFT_ARM}/follow_joint_trajectory'],
    )
    pkg_moveit = get_package_share_directory('openarm_bimanual_moveit_config')
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_moveit, 'launch', 'moveit_only.launch.py')),
        launch_arguments={
            'hardware_type': 'mock_components', 'base_mode': move_group_mode, 'use_rviz': 'false',
        }.items(),
    )
    wait_for_move_group = Node(
        package='openarm_bimanual_moveit_config',
        executable='wait_for_graph.py',
        name='wait_for_move_group',
        output='screen',
        arguments=['--action', '/move_action'],
    )
    pkg_commander = get_package_share_directory('openarm_bimanual_commander_cpp')
    commander = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_commander, 'launch', 'commander.launch.py')),
        launch_arguments={
            'base_mode': commander_mode, 'plan_only': 'true' if plan_only else 'false',
            'use_sim_time': 'true' if commander_sim_time else 'false',
        }.items(),
    )
    return [
        stub_world,
        robot_state_publisher,
        wait_for_world,
        RegisterEventHandler(OnProcessExit(
            target_action=wait_for_world, on_exit=[moveit, wait_for_move_group])),
        RegisterEventHandler(OnProcessExit(target_action=wait_for_move_group, on_exit=[commander])),
    ]


class CommanderClient:
    """Sends goals to the commander and reads what reached the stub controllers."""

    def __init__(self, name):
        self.node = rclpy.create_node(name)
        self.go_to_pose = ActionClient(self.node, GoToPose, '/go_to_pose')
        self.go_to_home = ActionClient(self.node, GoToHome, '/go_to_home')
        self.parameters = self.node.create_client(SetParameters, '/bimanual_commander/set_parameters')
        self.lock = threading.Lock()
        self.events = []
        self.next_id = 0
        self.commands = self.node.create_publisher(
            String, '/stub_world/command', QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))
        latched = QoSProfile(
            depth=1000, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(String, '/stub_world/events', self.on_event, latched)
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        threading.Thread(target=self.executor.spin, daemon=True).start()

    def close(self):
        self.executor.shutdown()
        self.node.destroy_node()

    def on_event(self, message):
        with self.lock:
            self.events.append(json.loads(message.data))

    def wait_ready(self, timeout=240.0):
        """Wait for the commander. It starts last, so everything else is up by then."""
        ready = self.go_to_pose.wait_for_server(timeout_sec=timeout)
        assert ready, 'The commander did not start. Is a Zenoh router running (rmw_zenohd)?'
        assert self.go_to_home.wait_for_server(timeout_sec=30.0)
        time.sleep(3.0)
        self.reset_world()

    def command(self, **command):
        """Send a command to the stub world and wait until it has applied it."""
        self.next_id += 1
        command['id'] = self.next_id
        acknowledged = {'ack': self.next_id}
        # A publisher can take a moment to match, so this sends again until it is answered.
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            self.commands.publish(String(data=json.dumps(command)))
            for _ in range(50):
                with self.lock:
                    if acknowledged in self.events:
                        return
                time.sleep(0.02)
        raise AssertionError(f'The stub world did not answer {command}')

    def reset_world(self):
        self.command(reset=True)
        self.settle()
        self.clear_events()

    def clear_events(self):
        with self.lock:
            self.events.clear()

    @staticmethod
    def settle():
        """Give move_group and the commander time to see a change of state."""
        time.sleep(1.0)

    def goals(self):
        with self.lock:
            return [event for event in self.events if 'goal' in event]

    def controllers(self):
        return {event['goal'] for event in self.goals()}

    def base_goals(self):
        return [event for event in self.goals() if event['goal'] == BASE_CONTROLLER]

    def canceled(self):
        with self.lock:
            return {event['canceled'] for event in self.events if 'canceled' in event}

    def wait_for_goal(self, timeout=60.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.goals():
                return True
            time.sleep(0.05)
        return False

    def send_pose(self, group, target, frame='map', cartesian=False):
        """Return the future of the goal handle of a pose goal."""
        goal = GoToPose.Goal(
            group_name=group, x=float(target[0]), y=float(target[1]), z=float(target[2]),
            roll=0.0, pitch=FORWARD_PITCH, yaw=0.0, cartesian_path=cartesian, frame_id=frame)
        return self.go_to_pose.send_goal_async(goal)

    @staticmethod
    def wait(future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        assert future.done(), f'No answer in {timeout} s'
        return future.result()

    def finish(self, handle_future, timeout=180.0):
        """Return the result of a pose goal, with its action status."""
        handle = self.wait(handle_future, 30.0)
        assert handle.accepted, 'The commander rejected a goal instead of aborting with a reason'
        return self.wait(handle.get_result_async(), timeout)

    def pose(self, group, target, frame='map', cartesian=False, timeout=180.0):
        return self.finish(self.send_pose(group, target, frame, cartesian), timeout).result

    def named(self, group, state, timeout=180.0):
        goal = GoToHome.Goal(group_name=group, pose_name=state)
        handle = self.wait(self.go_to_home.send_goal_async(goal), 30.0)
        assert handle.accepted
        return self.wait(handle.get_result_async(), timeout).result

    def set_parameter(self, name, value):
        assert self.parameters.wait_for_service(timeout_sec=30.0)
        if isinstance(value, str):
            parameter_value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value)
        else:
            parameter_value = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
        request = SetParameters.Request(parameters=[Parameter(name=name, value=parameter_value)])
        response = self.wait(self.parameters.call_async(request), 30.0)
        assert response.results[0].successful, response.results[0].reason
