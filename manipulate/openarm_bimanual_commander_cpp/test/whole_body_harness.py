"""Shared bring-up and client of the whole-body commander launch tests.

The bring-up runs the real move_group and commander against whole_body_stub_world.py.
"""
import json
import math
import os
import threading
import time

from ament_index_python.packages import get_package_share_directory
from launch.actions import GroupAction, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
from moveit_configs_utils import MoveItConfigsBuilder
from my_robot_interfaces.action import GoToHome, GoToPose
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import InteractiveMarkerFeedback
from visualization_msgs.srv import GetInteractiveMarkers

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
            commander_sim_time=False, commander_remappings=()):
    """Return the launch actions. Each waits for what the next one needs at startup.

    commander_remappings: (from, to) name pairs for the commander alone, for example to
    put a test's stub between the commander and a move_group service.
    """
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
    if commander_remappings:
        # Scoped, so move_group and the stub world keep their names.
        commander = GroupAction(
            [SetRemap(src=src, dst=dst) for src, dst in commander_remappings] + [commander])
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

    def send_named(self, group, state):
        """Return the future of the goal handle of a named-state goal."""
        return self.go_to_home.send_goal_async(GoToHome.Goal(group_name=group, pose_name=state))

    def named(self, group, state, timeout=180.0):
        handle = self.wait(self.send_named(group, state), 30.0)
        assert handle.accepted
        return self.wait(handle.get_result_async(), timeout).result

    def set_parameter(self, name, value):
        result = self.try_set_parameter(name, value)
        assert result.successful, result.reason

    def try_set_parameter(self, name, value):
        """Return the SetParametersResult instead of asserting that it succeeded."""
        assert self.parameters.wait_for_service(timeout_sec=30.0)
        if isinstance(value, str):
            parameter_value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value)
        elif isinstance(value, bool):
            parameter_value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
        elif isinstance(value, int):
            parameter_value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=value)
        elif isinstance(value, (list, tuple)):
            parameter_value = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=[float(v) for v in value])
        else:
            parameter_value = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
        request = SetParameters.Request(parameters=[Parameter(name=name, value=parameter_value)])
        return self.wait(self.parameters.call_async(request), 30.0).results[0]


class MarkerDriver:
    """Plays the part of RViz for whole_body_marker.

    Reads the marker from its server and moves it or picks its menu entries with feedback
    messages, as the InteractiveMarkers display does. `node` has to be spinning.
    """

    NAMESPACE = '/whole_body_hand_target'
    MARKER = 'hand_target'

    def __init__(self, node, client_id):
        self.client_id = client_id
        self.markers = node.create_client(GetInteractiveMarkers, self.NAMESPACE + '/get_interactive_markers')
        self.feedback = node.create_publisher(InteractiveMarkerFeedback, self.NAMESPACE + '/feedback', 10)

    def marker(self):
        assert self.markers.wait_for_service(timeout_sec=30.0)
        response = CommanderClient.wait(self.markers.call_async(GetInteractiveMarkers.Request()), 10.0)
        found = [m for m in response.markers if m.name == self.MARKER]
        assert len(found) == 1, [m.name for m in response.markers]
        return found[0]

    def wait_for_marker(self, check, timeout):
        """Return the marker once check(marker) is true."""
        deadline = time.monotonic() + timeout
        while True:
            marker = self.marker()
            if check(marker):
                return marker
            assert time.monotonic() < deadline, (
                f'The marker did not change in {timeout} s; last description: {marker.description}')
            time.sleep(0.2)

    def wait_for_status(self, prefix, timeout):
        return self.wait_for_marker(lambda m: m.description.startswith(prefix), timeout).description

    def send(self, event_type, pose=None, entry=''):
        message = InteractiveMarkerFeedback(
            client_id=self.client_id, marker_name=self.MARKER, event_type=event_type)
        message.header.frame_id = 'map'
        if pose is not None:
            message.pose = pose
        if entry:
            ids = [e.id for e in self.marker().menu_entries if e.title == entry]
            assert len(ids) == 1, entry
            message.menu_entry_id = ids[0]
        deadline = time.monotonic() + 10.0
        while self.feedback.get_subscription_count() == 0 and time.monotonic() < deadline:
            time.sleep(0.05)
        assert self.feedback.get_subscription_count() > 0, 'whole_body_marker is not listening'
        self.feedback.publish(message)

    def pick(self, entry):
        self.send(InteractiveMarkerFeedback.MENU_SELECT, entry=entry)
