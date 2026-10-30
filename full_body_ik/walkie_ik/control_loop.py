"""
Preview loop, adapted from walker_robot/sim/control_loop.py (Coordinated-Base-Arm-Motion-Control).

There, the IK command drove the sim every cycle. Here it drives only the preview viewer: the
IK keeps converging on the GUI's targets in the preview, and the solution goes to the robot
when the GUI's Execute button is pressed (lift + arms -> MoveIt, base -> Nav2, at once).
"""
import threading
import time

import numpy as np

from . import rotations
from .drag_targets import DragTargets
from .reference_frame import WORLD, convert_pose, targets_to_world
from .walkie_model import ARM_JOINTS

# base joint -> base group (as in WalkieIK.step) and top speed when following the manual slider
BASE_JOINT_GROUPS = {"x": "xy", "y": "xy", "yaw": "yaw", "lift": "lift"}
MANUAL_BASE_SPEED = {"x": 0.5, "y": 0.5, "yaw": 1.0, "lift": 0.2}     # m/s, rad/s


def run_control_loop(ik, preview, gui, bridge, ik_rate: float = 100.0) -> None:
    """
    Control thread: read targets from the GUI -> ik.step -> show in the preview; on Execute,
    send the current command to the robot. The GUI owns the main thread; closing either window
    stops both.

    The preview starts from the robot's state as soon as it is available (joint states + TF),
    and the GUI's sync button restarts it from the robot's state again. The base pose of that
    state is the IK's desired base pose: the base only moves as far as the arms need it to.

    With the GUI's drag option on, dragging an arm's hand in the preview (double-click, then
    Ctrl + left / right drag) moves that arm's target (DragTargets). The preview has no physics,
    so the drag pushes nothing.

    Targets in the GUI's frame (world = map, or robot base) are converted to map every cycle,
    using the base pose of the command the IK starts from (as in walker_robot).
    """
    period = 1.0 / ik_rate

    def worker():
        command = dict(ik.walkie.home_state)
        desired_base = None
        synced = False
        robot, robot_wait = None, None
        drag = DragTargets(preview.model)
        tick = 0

        def sync():
            nonlocal command, desired_base, synced
            command = dict(robot)
            desired_base = {joint: command[joint] for joint in ("x", "y", "yaw")}
            synced = True
            gui.set_base_values({joint: command[joint] for joint in BASE_JOINT_GROUPS})
            frame = gui.get_request()[2]
            gui.set_targets({side: convert_pose(pos, rotations.rotmat_to_euler_ZYX(R), WORLD, frame, command)
                             for side, (pos, R) in ik.forward(command).items()})

        gui.set_base_values({joint: command[joint] for joint in BASE_JOINT_GROUPS})
        with preview.launch_viewer() as viewer:
            with viewer.lock():                         # start with the whole robot in view
                viewer.cam.lookat[:] = command["x"], command["y"], 0.8
                viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 3.5, 180.0, -25.0
            # stop when either window closes; the viewer must close here, in its own thread, before exit
            while viewer.is_running() and not gui.is_closed():
                t0 = time.perf_counter()

                # ---- robot state; the first one (and every sync press) restarts the preview from it ----
                if tick % max(1, int(ik_rate // 10)) == 0:
                    robot, robot_wait = bridge.robot_state()
                sync_requested = gui.take_reset()
                if robot is not None and (sync_requested or not synced):
                    sync()
                    with viewer.lock():                 # look at the robot (it may be far off in a world)
                        viewer.cam.lookat[:] = command["x"], command["y"], 0.8

                # ---- target frame switch ----
                new_frame = gui.take_frame_request()
                if new_frame is not None:
                    gui.switch_frame(new_frame, lambda pos, rpy, src, dst:
                                     convert_pose(pos, rpy, src, dst, command))

                # ---- drag in the preview: moves a target ----
                if gui.drag_targets_enabled():
                    _drag_targets(drag, viewer, gui, command)

                # ---- manual base: joints the IK does not move head for their slider value ----
                targets, base_groups, frame, base_manual = gui.get_request()
                command = dict(command)
                for joint, group in BASE_JOINT_GROUPS.items():
                    if group not in base_groups:
                        max_step = MANUAL_BASE_SPEED[joint] * period
                        command[joint] += float(np.clip(base_manual[joint] - command[joint], -max_step, max_step))

                # ---- IK (preview only) ----
                targets = targets_to_world(targets, frame, command)
                if synced:
                    command = ik.step(command, targets, base_groups, desired_base)
                ik_base = {joint: command[joint] for joint, group in BASE_JOINT_GROUPS.items() if group in base_groups}
                if ik_base:
                    gui.set_base_values(ik_base)
                preview.show(command)

                # ---- robot ----
                if gui.take_execute():
                    if robot is None:
                        bridge.node.get_logger().warn(f"not executed: {robot_wait}")
                    else:
                        bridge.execute(dict(command), robot)
                if gui.take_cancel():
                    bridge.cancel()

                # ---- visuals + status (10 Hz) ----
                if tick % max(1, int(ik_rate // 10)) == 0:
                    _update_display(ik, preview, gui, viewer, command, targets, frame, robot)
                    gui.set_robot_status(bridge.status_text() if robot is not None or synced
                                         else f"{robot_wait}\n(the preview starts from the robot's state)")
                viewer.sync()
                tick += 1

                time.sleep(max(0.0, period - (time.perf_counter() - t0)))
        gui.close()

    thread = threading.Thread(target=worker, daemon=True)
    thread.start()
    gui.run()           # blocks until the window is closed
    thread.join(timeout=5.0)    # let the preview viewer close before the process exits


def _drag_targets(drag, viewer, gui, command):
    """Apply a hand drag in the preview to that arm's target in the GUI (in the GUI's frame)."""
    frame = gui.get_request()[2]

    def current_target(side):
        pos, rpy = convert_pose(*gui.get_target_values()[side], frame, WORLD, command)
        return pos, rotations.euler_ZYX_to_rotmat(*rpy)

    with viewer.lock():
        dragged = drag.update(viewer.perturb, current_target)
    if dragged:
        gui.set_targets({side: convert_pose(pos, rotations.rotmat_to_euler_ZYX(R), WORLD, frame, command)
                         for side, (pos, R) in dragged.items()})


def _update_display(ik, preview, gui, viewer, command, targets, frame, robot):
    """targets: map frame. The pose shown in the GUI is in `frame`, like its sliders."""
    expected = ik.forward(command)
    errors = ik.errors(command, targets)
    actual = ik.forward(robot) if robot is not None else {}

    frames = []
    for side, target in targets.items():
        R_target = None if target.rpy is None else rotations.euler_ZYX_to_rotmat(*target.rpy)
        frames.append((target.pos, R_target, 0.10, 0.9))      # target: big frame
    for pos, R in actual.values():
        frames.append((pos, R, 0.06, 0.35))                   # robot's TCP now: small, faint
    if robot is not None:
        c, s = np.cos(robot["yaw"]), np.sin(robot["yaw"])
        R_base = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        frames.append((np.array([robot["x"], robot["y"], 0.02]), R_base, 0.25, 0.35))  # robot's base now

    if gui.show_joint_frames() and targets:
        joints = ["lift"] + [j for side in targets for j in ARM_JOINTS[side]]
        for joint in joints:
            pos, R = preview.joint_frame(joint)
            frames.append((pos, R, 0.05, 0.8))                # joint frames: smallest

    with viewer.lock():
        preview.draw_frames(viewer, frames)

    for side, (pos, R) in expected.items():
        shown_pos, shown_rpy = convert_pose(pos, rotations.rotmat_to_euler_ZYX(R), WORLD, frame, command)
        lines = [f"expected pose ({frame} frame)",
                 f"pos  {shown_pos[0]:+.3f} {shown_pos[1]:+.3f} {shown_pos[2]:+.3f} m",
                 "rpy  " + " ".join(f"{a:+6.1f}" for a in np.degrees(shown_rpy)) + " deg"]
        if side in errors:
            pos_err, rot_err = errors[side]
            lines.append(f"IK err  {pos_err * 1000:6.1f} mm" +
                         ("" if rot_err is None else f"  {np.degrees(rot_err):5.1f} deg"))
        if side in actual:
            lines.append(f"robot TCP vs expected {np.linalg.norm(actual[side][0] - pos) * 1000:7.1f} mm")
        gui.set_status(side, "\n".join(lines), shown_pos, shown_rpy)
