#!/usr/bin/env python3
"""Check offline that Walkie's gripper holds a world object through a lift.

Assembles the world with Walkie through robot_bringup.world_loader, as the ROS
launch does, and runs the physics of one side grasp without ROS or MoveIt:

1. A damped least-squares IK places the right hand at the spawn, in front of
   the robot, with the gripper pointing forward and level (side grasp). The
   arm joints and the lift are set to that solution and held by their
   actuators.
2. The object's free joint is moved so that the object's origin sits between
   the open fingers, upright, at --depth along the gripper's approach axis.
3. The finger command ramps from open to closed at the commander's
   gripper_speed (0.1 units/s), as the commander's ControlGripper does in
   simulation. The object is pinned in place during the close and while the
   fingers settle, so the close cannot push it away.
4. The object is released and held for --hold seconds, then the lift command
   rises by --lift metres. The object is held again for --hold seconds, and
   then for --carry seconds, as while the robot drives back.

It reports the settled finger position (the gap the commander compares with
its grasp band), how far the object rose with the hand, and its slip: how far
it fell against the hand from its release to the end of the carry. It exits
non-zero if the gap leaves the band at any physics step from the release to
the end of the carry, the hand does not rise within --lift-tolerance of
--lift, the object does not rise with the hand, or it slips more than
--max-slip.

The fingers taper toward their tips, so a round object is pushed toward the
palm as they close. That motion is reported as moved_in_hand and is not slip.

Run with the pixi Python from the repository root; no colcon build is needed:

    pixi run python simulation/mujoco/tools/check_grasp.py chula_pat14_6

The report goes to simulation/mujoco/output/<world>/grasp_check_<object>.json.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import logging
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import world_tools
from world_tools import world_loader

ARM_JOINTS = ["lift_joint"] + [f"openarm_right_joint{i}" for i in range(1, 8)]
FINGER_ACTUATOR = "openarm_right_finger_joint1"
FINGER_JOINT = "openarm_right_finger_joint1"
HAND_BODY = "openarm_right_hand"
FINGER_OPEN = 0.044
# The commander's defaults: gripper_speed (commander.launch.py) and the grasp band
# (grasp_detect_min, grasp_detect_max).
GRIPPER_SPEED = 0.1
GRASP_BAND = (0.005, 0.044)


def body_frame(data, body: int) -> tuple[np.ndarray, np.ndarray]:
    return data.xpos[body].copy(), data.xmat[body].reshape(3, 3).copy()


def side_grasp_rotation(forward: np.ndarray, camera_up: bool) -> np.ndarray:
    """Hand rotation in world: approach (hand z) along forward, fingers closing horizontally.

    The wrist camera bracket sits on the hand's +x side. camera_up puts hand x up,
    otherwise it points down.
    """
    z_axis = forward / np.linalg.norm(forward)
    x_axis = np.array([0.0, 0.0, 1.0 if camera_up else -1.0])
    y_axis = np.cross(z_axis, x_axis)
    return np.column_stack([x_axis, y_axis, z_axis])


def solve_hand_pose(model, data, target_pos, target_rot, lift_max, iterations=400) -> float:
    """Move the arm joints and the lift so the hand reaches the pose. Returns the residual.

    The lift stays at or below lift_max, so that it can still rise for the lift step.
    """
    hand = model.body(HAND_BODY).id
    joints = [model.joint(name).id for name in ARM_JOINTS]
    qadr = [model.jnt_qposadr[j] for j in joints]
    dadr = [model.jnt_dofadr[j] for j in joints]
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    target_quat = np.zeros(4)
    mujoco.mju_mat2Quat(target_quat, target_rot.flatten())
    error = np.inf
    for _ in range(iterations):
        mujoco.mj_kinematics(model, data)
        mujoco.mj_comPos(model, data)
        pos, rot = body_frame(data, hand)
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, rot.flatten())
        local = np.zeros(3)
        mujoco.mju_subQuat(local, target_quat, quat)
        twist = np.concatenate([target_pos - pos, rot @ local])
        error = float(np.linalg.norm(twist))
        if error < 1e-5:
            break
        mujoco.mj_jacBody(model, data, jacp, jacr, hand)
        jac = np.vstack([jacp, jacr])[:, dadr]
        step = jac.T @ np.linalg.solve(jac @ jac.T + 1e-4 * np.eye(6), twist)
        for joint, adr, delta in zip(joints, qadr, step):
            low, high = model.jnt_range[joint]
            if joint == joints[0]:
                high = min(high, lift_max)
            data.qpos[adr] = np.clip(data.qpos[adr] + 0.5 * delta, low, high)
    return error


def run(model, args) -> dict:
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    mujoco.mj_forward(model, data)

    base = model.body("base_footprint").id if mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "base_footprint") >= 0 else model.body("base_link").id
    base_pos, base_rot = body_frame(data, base)
    forward = base_rot[:, 0] * np.array([1.0, 1.0, 0.0])
    left = base_rot[:, 1] * np.array([1.0, 1.0, 0.0])
    hand_rot = side_grasp_rotation(forward, args.camera_up)
    hand_pos = (base_pos[:2].tolist() + [0.0]) + args.hand_x * forward + args.hand_y * left
    hand_pos[2] = args.height
    lift_max = model.jnt_range[model.joint("lift_joint").id][1] - args.lift - 0.05
    residual = solve_hand_pose(model, data, np.asarray(hand_pos), hand_rot, lift_max)

    for name in ARM_JOINTS:
        data.ctrl[model.actuator(name).id] = data.qpos[model.jnt_qposadr[model.joint(name).id]]
    finger = model.joint(FINGER_JOINT).id
    finger_q = model.jnt_qposadr[finger]
    for joint in (FINGER_JOINT, FINGER_JOINT.replace("joint1", "joint2")):
        data.qpos[model.jnt_qposadr[model.joint(joint).id]] = FINGER_OPEN
    data.ctrl[model.actuator(FINGER_ACTUATOR).id] = FINGER_OPEN
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)

    hand = model.body(HAND_BODY).id
    obj = model.body(args.object)
    if obj.jntnum[0] != 1 or model.jnt_type[obj.jntadr[0]] != mujoco.mjtJoint.mjJNT_FREE:
        raise ValueError(f"{args.object} must have exactly one free joint")
    obj_q = model.jnt_qposadr[obj.jntadr[0]]
    obj_v = model.jnt_dofadr[obj.jntadr[0]]
    pos, rot = body_frame(data, hand)
    pinned = np.concatenate([pos + rot[:, 2] * args.depth, [1.0, 0.0, 0.0, 0.0]])

    def pin():
        data.qpos[obj_q:obj_q + 7] = pinned
        data.qvel[obj_v:obj_v + 6] = 0.0

    def object_in_hand():
        hand_pos, hand_rot = body_frame(data, hand)
        return hand_rot.T @ (data.xpos[obj.id] - hand_pos)

    def simulate(seconds, before_step=None, after_step=None):
        for _ in range(int(round(seconds / model.opt.timestep))):
            if before_step:
                before_step()
            mujoco.mj_step(model, data)
            if after_step:
                after_step()

    # Smallest and largest finger gap from the release to the end of the carry.
    gap_range = [math.inf, -math.inf]

    def track_gap():
        gap = float(data.qpos[finger_q])
        gap_range[0] = min(gap_range[0], gap)
        gap_range[1] = max(gap_range[1], gap)

    # Settle the arm with the object pinned and the fingers open.
    simulate(1.0, pin)
    contacts_open = finger_contacts(model, data, obj.id)

    # Close at gripper_speed, then wait for the controller's stall timeout (1 s).
    actuator = model.actuator(FINGER_ACTUATOR).id
    ramp_steps = int(math.ceil(FINGER_OPEN / GRIPPER_SPEED / model.opt.timestep))
    for step in range(1, ramp_steps + 1):
        data.ctrl[actuator] = FINGER_OPEN * (1.0 - step / ramp_steps)
        pin()
        mujoco.mj_step(model, data)
    simulate(1.0, pin)
    gap_closed = float(data.qpos[finger_q])

    released_in_hand = object_in_hand()
    released_drop = float(data.xpos[hand][2] - data.xpos[obj.id][2])
    track_gap()
    simulate(args.hold, after_step=track_gap)
    start_in_hand = object_in_hand()
    start_world = data.xpos[obj.id].copy()
    start_hand = data.xpos[hand].copy()
    gap_held = float(data.qpos[finger_q])

    lift = model.actuator("lift_joint").id
    lift_start = float(data.ctrl[lift])
    lift_steps = int(round(args.lift / args.lift_speed / model.opt.timestep))
    for step in range(1, lift_steps + 1):
        data.ctrl[lift] = lift_start + args.lift * step / lift_steps
        mujoco.mj_step(model, data)
        track_gap()
    simulate(args.hold, after_step=track_gap)
    lifted_in_hand = object_in_hand()
    hand_rise = float(data.xpos[hand][2] - start_hand[2])
    object_rise = float(data.xpos[obj.id][2] - start_world[2])
    gap_lifted = float(data.qpos[finger_q])

    simulate(args.carry, after_step=track_gap)
    carried_in_hand = object_in_hand()
    gap_carried = float(data.qpos[finger_q])
    # The tapered fingers push a round object toward the palm, so the object moves in
    # the hand without loosening. Only a fall against the hand counts as slip.
    slip = float(data.xpos[hand][2] - data.xpos[obj.id][2]) - released_drop
    mimic = model.jnt_qposadr[model.joint(FINGER_JOINT.replace("joint1", "joint2")).id]
    return {
        "object": args.object,
        "ik_residual": residual,
        "hand_height": args.height,
        "depth": args.depth,
        "camera_up": args.camera_up,
        "finger_contacts_while_open": contacts_open,
        "gap_closed_pinned": gap_closed,
        "gap_after_release": gap_held,
        "gap_after_lift": gap_lifted,
        "gap_after_carry": gap_carried,
        "gap_min_from_release": gap_range[0],
        "gap_max_from_release": gap_range[1],
        "mimic_finger_after_carry": float(data.qpos[mimic]),
        "grasp_band": list(GRASP_BAND),
        "lift": args.lift,
        "hand_rise": hand_rise,
        "object_rise": object_rise,
        "slip": slip,
        "moved_in_hand": float(np.linalg.norm(carried_in_hand - released_in_hand)),
        "object_in_hand_at_release": released_in_hand.round(4).tolist(),
        "object_in_hand_before_lift": start_in_hand.round(4).tolist(),
        "object_in_hand_after_lift": lifted_in_hand.round(4).tolist(),
        "object_in_hand_after_carry": carried_in_hand.round(4).tolist(),
        "object_geoms": geom_settings(model, obj.id),
        "checks": grasp_checks(
            gap_range=tuple(gap_range), hand_rise=hand_rise, object_rise=object_rise,
            slip=slip, lift=args.lift, lift_tolerance=args.lift_tolerance,
            max_slip=args.max_slip),
    }


def grasp_checks(*, gap_range, hand_rise, object_rise, slip, lift, lift_tolerance,
                 max_slip) -> dict[str, bool]:
    """Pass or fail of each check, from the measured grasp.

    gap_range is the smallest and largest finger gap from the release to the end of
    the carry; every gap in it must be in the commander's grasp band. The hand must
    rise within lift_tolerance of the commanded lift, and the object with it.
    """
    return {
        "gap_in_grasp_band": GRASP_BAND[0] <= gap_range[0] and gap_range[1] <= GRASP_BAND[1],
        "hand_rose_commanded_lift": abs(hand_rise - lift) <= lift_tolerance,
        "object_rose_with_hand": object_rise > 0.8 * hand_rise > 0.0,
        "slip_below_limit": slip <= max_slip,
    }


def finger_contacts(model, data, body: int) -> int:
    count = 0
    for contact in data.contact[:data.ncon]:
        bodies = {model.geom_bodyid[contact.geom1], model.geom_bodyid[contact.geom2]}
        if body in bodies and any("finger" in model.body(b).name for b in bodies):
            count += 1
    return count


def geom_settings(model, body: int) -> list[dict]:
    settings = []
    for geom in range(model.ngeom):
        if model.geom_bodyid[geom] != body or not model.geom_contype[geom]:
            continue
        settings.append({
            "name": model.geom(geom).name,
            "friction": model.geom_friction[geom].round(5).tolist(),
            "condim": int(model.geom_condim[geom]),
            "priority": int(model.geom_priority[geom]),
            "solref": model.geom_solref[geom].round(5).tolist(),
            "solimp": model.geom_solimp[geom][:3].round(5).tolist(),
        })
    return settings


def main(argv=None) -> int:
    logging.basicConfig(level=logging.WARNING, format="%(levelname)s: %(message)s")
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    world_tools.add_world_arguments(parser)
    parser.add_argument("--variant", help="World variant; default: the manifest's default_variant")
    parser.add_argument("--robot-mjcf", type=Path, default=world_tools.ROBOT_MJCF,
                        help="Robot MJCF (default: %(default)s)")
    parser.add_argument("--object", default="task_water_bottle_1",
                        help="Body with a free joint to grasp (default: %(default)s)")
    parser.add_argument("--height", type=float, default=0.86,
                        help="Hand and object height in m (default: %(default)s)")
    parser.add_argument("--hand-x", type=float, default=0.45,
                        help="Hand distance ahead of the base in m (default: %(default)s)")
    parser.add_argument("--hand-y", type=float, default=-0.25,
                        help="Hand offset to the base's left in m (default: %(default)s)")
    parser.add_argument("--depth", type=float, default=0.06,
                        help="Object origin along the approach axis from the hand frame in m, "
                             "as in bringup/scripts/sim_bottle_pick.py; the hand's TCP is "
                             "at 0.08 (default: %(default)s)")
    parser.add_argument("--camera-down", dest="camera_up", action="store_false",
                        help="Point the wrist camera bracket down instead of up")
    parser.add_argument("--lift", type=float, default=0.10,
                        help="Lift travel in m (default: %(default)s)")
    parser.add_argument("--lift-speed", type=float, default=0.05,
                        help="Lift speed in m/s (default: %(default)s)")
    parser.add_argument("--lift-tolerance", type=float, default=0.005,
                        help="Largest accepted difference between the hand's rise and "
                             "--lift, in m (default: %(default)s)")
    parser.add_argument("--hold", type=float, default=2.0,
                        help="Seconds to hold before and after the lift (default: %(default)s)")
    parser.add_argument("--carry", type=float, default=20.0,
                        help="Seconds to hold the lifted object, as while driving back "
                             "(default: %(default)s)")
    parser.add_argument("--max-slip", type=float, default=0.01,
                        help="Largest accepted fall against the hand from release to the end "
                             "of the carry, in m (default: %(default)s)")
    args = parser.parse_args(argv)

    directory, manifest, output = world_tools.open_world(args)
    variant = args.variant or manifest["default_variant"]
    if variant not in manifest["variants"]:
        parser.error(f"unknown variant {variant!r}")
    robot_path = args.robot_mjcf.resolve()
    pose = world_loader.resolve_spawn(args.world, manifest, None, None)
    tree = world_loader.assemble_world(
        robot_path, directory / manifest["variants"][variant], robot_path.parent / "scene.xml",
        pose)
    model = mujoco.MjModel.from_xml_string(ET.tostring(tree, encoding="unicode"))

    report = run(model, args)
    report.update({
        "world": args.world,
        "variant": variant,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "mujoco_version": mujoco.__version__,
    })
    failed = sorted(name for name, passed in report["checks"].items() if not passed)
    report["status"] = "FAIL" if failed else "PASS"
    world_tools.write_json(output / f"grasp_check_{args.object}.json", report)
    print(f"{report['status']}: gap after lift {report['gap_after_lift']:.4f} m, "
          f"object rose {report['object_rise']:.4f} m of the hand's {report['hand_rise']:.4f} m, "
          f"slip {report['slip']:.4f} m")
    for name in failed:
        print(f"  failed: {name}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
