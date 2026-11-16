#!/usr/bin/env python3
"""Hand target for whole-body goals, as an interactive marker in RViz.

Drag the marker (the box, an arrow or a ring) to where a hand should go, then
right-click it and pick "Reach with left arm" or "Reach with right arm". That sends
the pose to the commander's go_to_pose_quat action with the group whole_body_left
or whole_body_right, so the goal runs every check of a whole-body goal: the arm
alone first, then sampled base poses ranked by the whole_body_weight_*
parameters. Nothing moves while you drag. The marker's text shows the status.

The marker's z axis (the arrow) is the gripper's approach direction, the z axis
of openarm_<side>_hand. "Snap to left hand" puts the marker on the hand, which is
a good place to start dragging from.

Needs the commander running with base_mode:=whole_body, and an InteractiveMarkers
display in RViz on the topic /whole_body_hand_target/update (moveit_map.rviz has
one).
"""
import math

from geometry_msgs.msg import Pose
from interactive_markers import InteractiveMarkerServer, MenuHandler
from my_robot_interfaces.action import GoToPoseQuaternion
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker

MARKER = "hand_target"
ACTION = "go_to_pose_quat"
# The gripper pointing forward at about the height of the table demo's targets, in case
# no hand transform is available when the node starts.
DEFAULT_POSITION = (0.45, 0.25, 1.0)
DEFAULT_PITCH = math.pi / 2
# Seconds without the action server that got the goal before the goal counts as lost.
COMMANDER_LOST_AFTER = 3


def axis_control(name, mode, w, x, y, z):
    control = InteractiveMarkerControl()
    control.name = name
    control.interaction_mode = mode
    control.orientation.w, control.orientation.x = w, x
    control.orientation.y, control.orientation.z = y, z
    return control


class WholeBodyMarker(Node):
    def __init__(self):
        super().__init__("whole_body_marker")
        self.frame = self.declare_parameter("frame_id", "map").value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.client = ActionClient(self, GoToPoseQuaternion, ACTION)
        self.status_topic = self.resolve_topic_name(ACTION) + "/_action/status"
        # One goal at a time. `request` counts the goals, so the late answer of a goal this
        # node gave up on changes nothing. `goal_server` is the endpoint gid of the status
        # publisher of the action server the goal went to.
        self.busy = False
        self.request = 0
        self.goal_handle = None
        self.cancel_pending = False
        self.goal_server = None
        self.missing = 0

        self.server = InteractiveMarkerServer(self, "whole_body_hand_target")
        self.menu = MenuHandler()
        self.menu.insert("Reach with left arm", callback=lambda _: self.reach("left"))
        self.menu.insert("Reach with right arm", callback=lambda _: self.reach("right"))
        self.menu.insert("Snap to left hand", callback=lambda _: self.snap("left"))
        self.menu.insert("Snap to right hand", callback=lambda _: self.snap("right"))
        self.menu.insert("Cancel goal", callback=lambda _: self.cancel())

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = DEFAULT_POSITION
        pose.orientation.y = math.sin(DEFAULT_PITCH / 2)
        pose.orientation.w = math.cos(DEFAULT_PITCH / 2)
        self.server.insert(self.make_marker(pose))
        self.menu.apply(self.server, MARKER)
        self.server.applyChanges()

        # Starts on the left hand once its transform arrives.
        self.snap_timer = self.create_timer(0.5, self.first_snap)
        # On steady time, so that a lost goal is noticed while a simulation's /clock stands
        # still.
        self.create_timer(1.0, self.watch_commander, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def make_marker(self, pose):
        marker = InteractiveMarker()
        marker.header.frame_id = self.frame
        marker.name = MARKER
        marker.description = "Whole-body hand target: right-click to reach"
        marker.scale = 0.2
        marker.pose = pose

        # Dragging the box moves the target in the view plane. A right-click on any
        # control opens the menu.
        body = InteractiveMarkerControl()
        body.interaction_mode = InteractiveMarkerControl.MOVE_3D
        body.always_visible = True
        box = Marker()
        box.type = Marker.CUBE
        box.scale.x = box.scale.y = box.scale.z = 0.04
        box.color.r, box.color.g, box.color.b, box.color.a = 0.9, 0.6, 0.1, 0.9
        arrow = Marker()
        arrow.type = Marker.ARROW
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.12, 0.012, 0.012
        arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 0.2, 0.4, 1.0, 0.9
        # An arrow points along its own x axis. This turns it onto the hand's z axis.
        arrow.pose.orientation.y = -math.sqrt(0.5)
        arrow.pose.orientation.w = math.sqrt(0.5)
        body.markers = [box, arrow]
        marker.controls.append(body)

        s = math.sqrt(0.5)
        for axis, (w, x, y, z) in {"x": (s, s, 0.0, 0.0), "y": (s, 0.0, 0.0, s), "z": (s, 0.0, s, 0.0)}.items():
            marker.controls.append(axis_control("move_" + axis, InteractiveMarkerControl.MOVE_AXIS, w, x, y, z))
            marker.controls.append(axis_control("rotate_" + axis, InteractiveMarkerControl.ROTATE_AXIS, w, x, y, z))
        return marker

    def set_status(self, text):
        self.get_logger().info(text)
        marker = self.server.get(MARKER)
        if marker is None:
            return
        marker.description = text
        self.server.insert(marker)
        self.server.applyChanges()

    def hand_pose(self, side):
        try:
            t = self.tf_buffer.lookup_transform(self.frame, f"openarm_{side}_hand", Time())
        except TransformException:
            return None
        pose = Pose()
        pose.position.x = t.transform.translation.x
        pose.position.y = t.transform.translation.y
        pose.position.z = t.transform.translation.z
        pose.orientation = t.transform.rotation
        return pose

    def first_snap(self):
        pose = self.hand_pose("left")
        if pose is None:
            return
        self.snap_timer.cancel()
        self.server.setPose(MARKER, pose)
        self.server.applyChanges()

    def snap(self, side):
        pose = self.hand_pose(side)
        if pose is None:
            self.set_status(f"No transform from {self.frame} to openarm_{side}_hand yet")
            return
        self.server.setPose(MARKER, pose)
        self.server.applyChanges()

    def reach(self, side):
        pose = self.server.get(MARKER).pose
        if self.busy:
            self.set_status("A goal is still running: wait, or pick Cancel goal")
            return
        servers = self.server_ids()
        if not self.client.server_is_ready() or not servers:
            self.set_status("The commander's go_to_pose_quat action is not available")
            return
        if len(servers) > 1:
            # The marker could not tell which one has the goal, so it could not notice
            # that one goes away.
            self.set_status("Two or more commanders serve go_to_pose_quat: stop all but one")
            return
        goal = GoToPoseQuaternion.Goal()
        goal.group_name = f"whole_body_{side}"
        goal.frame_id = self.frame
        goal.x, goal.y, goal.z = pose.position.x, pose.position.y, pose.position.z
        q = pose.orientation
        goal.qx, goal.qy, goal.qz, goal.qw = q.x, q.y, q.z, q.w
        goal.cartesian_path = False
        self.busy = True
        self.request += 1
        request = self.request
        self.goal_handle = None
        self.cancel_pending = False
        (self.goal_server,) = servers
        self.missing = 0
        self.set_status(
            f"{goal.group_name}: sent ({goal.x:.2f}, {goal.y:.2f}, {goal.z:.2f}) in {self.frame}"
        )
        future = self.client.send_goal_async(goal)
        future.add_done_callback(lambda f: self.accepted(request, goal.group_name, f))

    def accepted(self, request, group, future):
        if request != self.request:
            return
        if future.exception() is not None:
            self.finish(f"{group}: goal not sent: {future.exception()}")
            return
        handle = future.result()
        if not handle.accepted:
            self.finish(f"{group}: goal rejected")
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self.finished(request, group, f))
        if self.cancel_pending:
            handle.cancel_goal_async()
            self.set_status(f"{group}: accepted, cancel requested")
        else:
            self.set_status(f"{group}: accepted, running")

    def finished(self, request, group, future):
        if request != self.request:
            return
        if future.exception() is not None:
            self.finish(f"{group}: outcome unknown: {future.exception()}")
            return
        result = future.result().result
        self.finish(f"{group}: {'done' if result.success else 'failed'}: {result.status}")

    def finish(self, status):
        self.busy = False
        self.goal_handle = None
        self.cancel_pending = False
        self.set_status(status)

    def cancel(self):
        if not self.busy:
            self.set_status("No goal to cancel")
            return
        if self.goal_handle is None:
            # The commander has not accepted the goal yet. accepted() cancels it.
            self.cancel_pending = True
            self.set_status("Cancel requested, waiting for the commander to accept the goal")
            return
        self.goal_handle.cancel_goal_async()
        self.set_status("Cancel requested")

    def server_ids(self):
        return {bytes(info.endpoint_gid) for info in self.get_publishers_info_by_topic(self.status_topic)}

    def watch_commander(self):
        # A commander that exits mid-goal never answers, which would leave the marker busy.
        # A restarted one serves the same action but does not know the goal, so this
        # follows the server the goal went to, by the gid of its status publisher.
        if not self.busy or self.goal_server in self.server_ids():
            self.missing = 0
            return
        self.missing += 1
        if self.missing >= COMMANDER_LOST_AFTER:
            self.request += 1
            self.finish("The commander's go_to_pose_quat action went away: outcome unknown")


def main():
    # Without rclpy's signal handlers, Ctrl-C reaches spin() as KeyboardInterrupt and only
    # this thread shuts the context down. rclpy's handlers shut it down from their own
    # thread too, and the two race: rcl_shutdown fails and the node exits with code 1.
    # spin() sees the interrupt at its next wakeup, within a second of the steady timer.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = WholeBodyMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
