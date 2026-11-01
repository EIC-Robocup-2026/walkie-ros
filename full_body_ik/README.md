# Full-body IK (MuJoCo + Nav2 + MoveIt)

Full-body IK for Walkie: one damped-least-squares solver moves both OpenArm arms, the lift and the base
(x, y, yaw) towards end-effector targets. The solution is shown in a MuJoCo preview. On **Execute**, the
lift and both arms go to MoveIt hybrid planning and the base pose goes to Nav2, and both execute at the same time.

The IK comes from [Coordinated-Base-Arm-Motion-Control](https://github.com/MonochromePie/Coordinated-Base-Arm-Motion-Control)
(`walker_robot/`). This folder is a standalone script with no colcon package and needs no build.

## Run

Four terminals, from the repository root:

```bash
pixi run ros2 run rmw_zenoh_cpp rmw_zenohd
pixi run ros2 launch robot_bringup nav_mujoco_omnibot.launch.py
pixi run ros2 launch openarm_bimanual_moveit_config moveit_only.launch.py hardware_type:=mujoco
pixi run python full_body_ik/run_full_body_ik.py
```

With `hardware_type:=mujoco`, MoveIt plans in the `map` frame (base pose from TF `map -> base_footprint`),
keeps its octomap (head camera cloud) in `map`, and starts hybrid planning. See
[Manipulation: Hybrid planning](../docs/guides/manipulation.md#hybrid-planning). Start Nav2 first: MoveIt
needs the `map` frame.

Two windows open:

- **Walkie full-body IK** (Tkinter): the control interface.
- **MuJoCo preview**: a second, kinematic-only walkie that shows the IK solution (the expected outcome).
  It draws the targets as large frames. It also draws faint frames at the real robot's TCPs and its base
  pose, read from `/joint_states` and TF, so you can compare them.

Workflow:

1. The preview starts from the robot's current state as soon as `/joint_states` and TF `map -> base_footprint` arrive.
2. Move the target sliders (map frame, or relative to the robot base) and tick the base groups the IK may move
   (x/y, yaw, lift). Unticked base joints follow the manual base sliders. The IK converges continuously in the preview.
   Or tick **Drag targets in MuJoCo** (Display box) and drag the targets in the preview. Double-click a hand
   to select it, then use **Ctrl + left drag** to rotate that arm's target and **Ctrl + right drag** to move it
   (add Shift for the other plane). The sliders follow.
3. **Execute (MoveIt + Nav2)**: sends the preview's current solution to the robot.
4. **Sync preview to robot**: restarts the preview from the robot's state, for example after execution.
5. **Cancel**: cancels both goals.

**With a world.** If the sim runs a named world (`nav_mujoco_omnibot.launch.py world:=chula_pat14_6`), pass the
same name, and the same `--variant` if you set one:

```bash
pixi run python full_body_ik/run_full_body_ik.py --world chula_pat14_6
```

The preview then shows the same room, with the robot drawn at its map pose (the MuJoCo world frame is the map
frame). The preview robot is a ghost: no physics runs and contacts are off, so it passes through walls and
furniture and moves nothing. It only shows where the robot is being sent. The x/y sliders cover the world's map
plus 1 m. The world is built by `robot_bringup.world_loader` into `~/.ros/mujoco_worlds_full_body_ik`, so the
sim's own generated files are never touched. Without `--world`, the x/y sliders cover ±15 m.

Options: `--velocity-scaling`, `--acceleration-scaling`, `--planning-time` (MoveIt), `--rate` (IK/preview Hz),
`--model` (MJCF scene, default: the installed `walkie_description/mujoco/scene.xml`), `--no-precise-base` (see below),
`--no-hybrid` (send the arms and lift to `move_group` instead of hybrid planning).

## What is sent

| IK joints | Sent to | How |
|---|---|---|
| `lift`, `q_l1..7`, `q_r1..7` | MoveIt hybrid planning | `HybridPlanner` action `/run_hybrid_planning`: one joint goal for `both_arms_lift` (`lift_joint`, `openarm_{left,right}_joint1..7`). The global planner plans it, the local planner follows it and holds the arms when the octomap or a collision object blocks the path. With `--no-hybrid`: `MoveGroup` action `/move_action`, plan + execute |
| `x`, `y`, `yaw` | Nav2 | `NavigateToPose` action `/navigate_to_pose`: goal pose in `map` |

A base move under 1 cm and 1 deg is not sent to Nav2.

Nav2's default goal checker (`general_goal_checker`) stops within 0.1 m, which is too coarse for the arms. While
an IK base goal runs, the tool publishes `precise_goal_checker` (0.02 m, 0.01 rad) on the behavior tree's
`goal_checker_selector` topic and switches back to `general_goal_checker` when the goal ends. `--no-precise-base` disables this.

## How it differs from `walker_robot`

- **Kinematic model**: the walkie arms mount differently from the `walker_robot` scene, and link 3 to 4 is
  0.204 m instead of 0.154 m, so the DH tables do not apply. FK and Jacobians come from walkie's own MJCF
  through MuJoCo (`mj_kinematics` + `mj_jacBody` on `openarm_*_hand_tcp`). The joint limits (lift 0 to 0.7435 m,
  arm ranges) come from the same model. The base x, y, yaw are the free joint of `base_footprint`, restricted
  to the plane. This also avoids sympy, which is not in the pixi environment.
- **Solver**: `walkie_ik/inverse_kinematics.py` is an unchanged copy (selective DLS, joint-limit damping,
  null-space desired pose). `WalkieIK` (`full_body_ik.py`) is `BimanualIK` with the same tuning constants.
- **Null-space base pose**: the base is pulled towards where the robot is now (set at each sync), not
  towards the map origin. The base therefore moves only as far as the arms need it to.
- **Null-space arm posture**: the arms are pulled towards a chef-like rest pose (`ARM_REST_POSE` in
  `full_body_ik.py`): elbows at 90°, upper arms about 16° out from the body, forearms forward and about 11° inward,
  wrists neutral. The pull uses only the freedom the targets leave, so the hands still reach their targets.
  It shapes the posture most with position-only targets. With full 6D targets, only the elbow swivel is left
  free. `ARM_REST_GAIN = 0` turns it off.
- **The IK drives a preview**, not the robot directly. The robot moves only on Execute, through MoveIt and Nav2.

## Limitations

- The base pose comes from odometry (`map -> odom` is a static identity in `nav_mujoco_omnibot.launch.py`, and
  the omni controller runs open loop). The base therefore lands within Nav2's tolerance of the goal as
  measured by odometry. In a test that moved the base 0.8 m, the hands ended 17 mm from the preview's
  expected pose, which is Nav2's 2 cm tolerance.
- MoveIt plans the arms with the base where it is at planning time, in the `map` frame, and Nav2 plans the
  base. Neither knows about the other's motion. The hybrid local planner checks the rest of the arm path with
  that planning-time base pose, so while the base drives, obstacles are checked where the arms would be if the
  base had not moved.
- To correct the residual base error, press **Sync preview to robot**, untick the base groups and execute
  again. Only the arms and lift then move.

## Files

| File | Contents |
|---|---|
| `run_full_body_ik.py` | Entry point |
| `walkie_ik/walkie_model.py` | Walkie MJCF → IK joint mapping, limits, `ChainFK` (MuJoCo FK/Jacobian) |
| `walkie_ik/full_body_ik.py` | `WalkieIK.step()` (the IK API), `Target` |
| `walkie_ik/inverse_kinematics.py` | Solver (copy from `walker_robot/IK/`) |
| `walkie_ik/rotations.py`, `reference_frame.py` | Rotation helpers, map/base target frames |
| `walkie_ik/preview_sim.py` | Kinematic MuJoCo preview |
| `walkie_ik/target_gui.py` | Tkinter GUI (from `walker_robot/sim/`, with Execute / Cancel / Sync) |
| `walkie_ik/control_loop.py` | GUI → IK → preview loop, Execute handling |
| `walkie_ik/world.py` | Prepare the sim's named world for the preview; map extent for the sliders |
| `walkie_ik/drag_targets.py` | Drag the targets in the preview (copy from `walker_robot/sim/`) |
| `walkie_ik/ros_bridge.py` | ROS 2 node: `/joint_states`, TF, MoveGroup and NavigateToPose clients |
