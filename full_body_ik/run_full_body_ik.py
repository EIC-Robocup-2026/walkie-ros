#!/usr/bin/env python3
"""
Walkie full-body IK: target GUI + MuJoCo preview, executed with MoveIt hybrid planning (lift + arms)
and Nav2 (base).

Start the walkie MuJoCo sim, Nav2 and MoveIt first (see README.md), then, from the repository root:

    pixi run python full_body_ik/run_full_body_ik.py
    pixi run python full_body_ik/run_full_body_ik.py --world chula_pat14_6   # sim started with world:=chula_pat14_6
"""
import argparse
import os
import signal
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from ament_index_python.packages import get_package_share_directory

from walkie_ik import Target, WalkieIK, WalkieModel, rotations
from walkie_ik.control_loop import run_control_loop
from walkie_ik.preview_sim import PreviewSim
from walkie_ik.ros_bridge import RobotBridge
from walkie_ik.target_gui import DEFAULT_XY_RANGE, TargetGUI
from walkie_ik.world import map_xy_range, prepare_world


def main():
    default_model = os.path.join(get_package_share_directory("walkie_description"), "mujoco", "scene.xml")
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model", default=default_model,
                        help="walkie MJCF scene of the IK model, and of the preview without --world (default: %(default)s)")
    parser.add_argument("--world", default="",
                        help="show the preview robot in this named world, as the sim's world:=NAME (e.g. chula_pat14_6)")
    parser.add_argument("--variant", default=None, help="world variant, as the sim's variant:= (default: the world's default)")
    parser.add_argument("--worlds-dir", default=None, help="as the sim's worlds_dir:= (default: installed walkie_mujoco_worlds)")
    parser.add_argument("--rate", type=float, default=100.0, help="IK / preview rate in Hz")
    parser.add_argument("--velocity-scaling", type=float, default=0.5, help="MoveIt max velocity scaling")
    parser.add_argument("--acceleration-scaling", type=float, default=0.3, help="MoveIt max acceleration scaling")
    parser.add_argument("--planning-time", type=float, default=5.0, help="MoveIt allowed planning time [s]")
    parser.add_argument("--no-precise-base", dest="precise_base", action="store_false",
                        help="keep Nav2's default goal checker (0.1 m) instead of precise_goal_checker (0.02 m)")
    parser.add_argument("--no-hybrid", dest="hybrid", action="store_false",
                        help="send the arms + lift to move_group (/move_action) instead of hybrid planning "
                             "(/run_hybrid_planning)")
    args = parser.parse_args()

    walkie = WalkieModel(args.model)
    ik = WalkieIK(walkie)
    xy_range = DEFAULT_XY_RANGE
    if args.world:
        # the IK keeps the robot-only model; only the preview shows the world
        print(f"Preparing world {args.world}...")
        world = prepare_world(args.world, args.variant, worlds_dir=args.worlds_dir)
        preview = PreviewSim(WalkieModel(world["model_path"]))
        xy_range = map_xy_range(world["map_path"])
    else:
        preview = PreviewSim(walkie)
    bridge = RobotBridge(args.velocity_scaling, args.acceleration_scaling, args.planning_time,
                         args.precise_base, args.hybrid)

    # targets at the home pose until the robot's state arrives (the loop then syncs to it)
    gui = TargetGUI({side: Target(pos, rotations.rotmat_to_euler_ZYX(R))
                     for side, (pos, R) in ik.forward(walkie.home_state).items()}, xy_range=xy_range)
    for sig in (signal.SIGINT, signal.SIGTERM):          # Ctrl+C closes the GUI, which ends the app
        signal.signal(sig, lambda *_: gui.close())
    try:
        run_control_loop(ik, preview, gui, bridge, args.rate)   # preview thread in the background, GUI here
    finally:
        bridge.shutdown()


if __name__ == "__main__":
    main()
