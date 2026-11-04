# 7. Add Whole-Body Goals to MoveIt

**Goal:** let a caller reach a target the arm cannot reach from where the robot
stands, with one MoveIt plan for the base, the lift, and one arm, in a scene whose
octomap stays in `map`. Choosing not to move the base stays possible, and it stays
the default.

This entry records what changed on 2026-11-05, why, and what you need to do after
pulling. For day-to-day use, read
[Move the base with a whole-body goal](../guides/manipulation.md#whole-body-goals).

## In short

- **Changed:** a new launch argument, `base_mode` (`fixed`, `map`, `whole_body`), on
  `moveit_only.launch.py` and `commander.launch.py`. Two new group names on
  `/go_to_pose` and `/go_to_pose_quat`: `whole_body_left` and `whole_body_right`. A new
  node, `whole_body_base_follower`, and a new demo, `whole_body_demo.launch.py`.
- **Why:** arm groups plan as if the base were welded to the floor, so a target out of
  reach could only fail. Task code had to move the base first and hope the arm could
  reach from there.
- **After pulling:** `pixi install`, update the submodules, and build
  ([After pulling this change](#after-pulling-this-change)).
- **Watch out:** arm moves take about twice as long, because this branch also takes the
  speed scaling from `feat/commander-speed-scaling`. Whole-body goals have not run on the
  real robot, so keep `base_mode:=fixed` there.
- **Result:** the MuJoCo demo passed all ten steps in its last run, and the launch tests
  pass ([Verification](#verification)). The plan's larger measurements have not been run
  ([Still open](#still-open)).

## After pulling this change

1. Install the new dependency, `ros-jazzy-depth-image-proc`:

    ```bash
    pixi install
    ```

2. Update the submodules. `description/` has the new scene:

    ```bash
    git submodule update --init --recursive
    ```

3. Build. `move_group` under `base_mode:=whole_body` loads a plugin from the commander
   package, so build both:

    ```bash
    pixi run build
    ```

Launch commands that do not name `base_mode` get `fixed`. With `fixed`, the robot model
that MoveIt loads is byte-identical to the one before this change.

!!! warning "The arms no longer run at full speed"
    The commander now starts with the scaling in `config/joint_limits.yaml`: 0.5 velocity
    and 0.3 acceleration, instead of 1.0. See
    [Change commander parameters](../guides/manipulation.md#change-commander-parameters)
    for how to go back to full speed.

## What changed

| File | Change |
| --- | --- |
| `manipulate/openarm_bimanual_moveit_config/config/openarm_bimanual.srdf` | The virtual joint depends on `base_mode`: fixed from `world`, or planar from `map`. New groups `whole_body_left` and `whole_body_right`. |
| `manipulate/openarm_bimanual_moveit_config/launch/moveit_only.launch.py` | `base_mode`, the octomap in `map`, the floor gate, the jump guard, the wait for `map`, and `depth_image_proc` under MuJoCo. `fixed` keeps its values. |
| `manipulate/openarm_bimanual_moveit_config/scripts/` | New: `cloud_floor_gate.py`, `octomap_jump_guard.py`, `wait_for_graph.py` |
| `manipulate/openarm_bimanual_moveit_config/config/` | Base limits in `joint_limits.yaml`, `base_controller` in `moveit_controllers.yaml`, OMPL entries, and `moveit_map.rviz` |
| `manipulate/openarm_bimanual_commander_cpp/src/whole_body_option.hpp` | The class behind the new group names: the gate, the base sampling, the repairs, and the stops |
| `manipulate/openarm_bimanual_commander_cpp/src/whole_body_base_follower.cpp` | The action server of `base_controller`. It closes the loop on `map` to `base_footprint` and publishes to `/cmd_vel_smoothed`. |
| `manipulate/openarm_bimanual_commander_cpp/src/unwrap_planar_heading.cpp` | A planning response adapter that keeps the base from turning the long way across ±π |
| `manipulate/openarm_bimanual_commander_cpp/src/whole_body_demo.py` | The demo client |
| `manipulate/openarm_bimanual_commander_cpp/test/` | Six launch files and a stub world, see [Tests](../guides/testing.md#whole-body-launch-tests) |
| `bringup/launch/whole_body_demo.launch.py` | Starts the demo. `bringup/scripts/ground_truth_map_odom.py` feeds `map` to `odom` from MuJoCo's ground truth. |
| `bringup/launch/nav_mujoco_omnibot.launch.py` | Sets `state_topic` on the collision monitor, so the follower can see a stop |
| `description/mujoco/scene_whole_body.xml` | The scene: a table, a free box, and a wall |
| `pixi.toml`, `pixi.lock` | `ros-jazzy-depth-image-proc` |
| `scripts/tmux_walkie.sh`, `.tmuxp.yaml` | Pass `base_mode:=fixed` explicitly to the MoveIt and commander panes |

??? info "The choices, and what was rejected"
    | Question | Chosen | Rejected, and why |
    | --- | --- | --- |
    | How to model the base | A planar virtual joint in the SRDF. The URDF does not change, so nothing outside MoveIt sees a different robot. | Explicit base joints in the URDF. They would change the description for every consumer. They stay the fallback if the planar model cannot be made robust. |
    | How base commands reach the wheels | Through the collision monitor, on `/cmd_vel_smoothed` | Straight to the omni controller. Faster to build, but the base would ignore the stop and slowdown polygons. |
    | Where the option lives | A small class inside the commander, selected by `group_name` | A separate node, or a new action field. Both need every caller to change. |
    | What "the base stays still" means | Per request: an arm group has no base variables, so no plan can command the base | A lock on the base for the duration of an arm goal. A system-wide lock was not needed for the first experiment, and it would stop Nav2 and the joystick. |
    | The default | `fixed`, planar modes opt-in | A planar default. It would move the octomap and the planning frame for every existing launch. |

??? info "What MuJoCo showed that the stubs did not"
    | Problem | Effect | Fix |
    | --- | --- | --- |
    | RRTConnect samples the base uniformly in the workspace box | 2.18 m of travel for a 1.02 m displacement | Plan in a small box first, then in the 5 m box |
    | OMPL normalizes the base heading to [-π, π], and time parameterization treats it as a plain number | The base would turn the long way | The `UnwrapPlanarHeading` response adapter |
    | The MuJoCo lift settled 2.65 mm below its command | The idle arm's fingers touched `base_link`, and every plan failed on the start state | The commander starts a plan from a slightly raised lift |
    | A stale voxel behind the robot's surface gets no ray through it | Every plan failed on the start state | The commander clears the octree and logs a warning |
    | A simulation at a fraction of real time can stall `/clock` for longer than a fixed wall-clock wait | A false `clock is not advancing` refusal | The clock check runs on the node's clock, with a 5 s wall-clock limit |
    | `MoveGroupInterface` requests for the `OCTOMAP` component of `/get_planning_scene` race the updater | `move_group` segfaulted | Nothing requests that component |

??? info "What the review found"
    An independent review of the first working version found six problems. Tests against
    stub controllers confirmed the first two.

    | Finding | Fix |
    | --- | --- |
    | The base kept following its plan after the commander died | The commander publishes a heartbeat. The follower refuses goals and stops without it. |
    | An aborted arm or lift goal did not stop the base, and the reverse. `move_group` waits for its controllers one at a time, in name order, so no stop came within 10 s. | The commander watches each controller's action status and stops the motion |
    | A cancel could arrive while a request was on its way to `move_group`. Not reproduced with stubs. | A second check before execution, a stop repeated until `move_group` finishes, and a motion slot held until then |
    | An active collision monitor stop was forgotten when a goal started. The monitor still zeroed the commands, so the base did not move, but the arm and lift ran on. | A goal that starts during a stop sends one zero command so the monitor looks again, and is rejected if the stop stays |
    | Nav2 could take the base during planning | The Nav2 check runs again before execution and during it, and the commander compares the base with where it was when planning started |
    | `whole_body_planning_time` was read without the lock | It is read once, under the lock |

    The follower also ends a goal directly on a `stop` event, which covers a hung
    `move_group`.

## CI

`openarm_bimanual_commander_cpp` joins the tested packages. Its launch tests add about 12
minutes to the **Test** step, and they use the router that CI already starts. These tests
have not run in CI yet, because the branch has not been pushed. They are timing-sensitive
in places, so a slower runner may show problems that this laptop did not.

## Verification

Checked on 2026-11-05 on a laptop with an RTX 2060, with `navigation` at `7f15b1c`, in
MuJoCo with `rmw_zenoh_cpp`, the viewer, and RViz. The real-time factor was between 0.16
and 0.30, depending on what else the laptop ran.

| Check | Result |
| --- | --- |
| Whole-body launch tests | Option 13 of 13, gate 10 of 10 with 2 skipped by design, plan-only 1 of 1, clock 2 of 2, stopping 11 of 11, follower 13 of 13 |
| `openarm_bimanual_moveit_config` tests | 34 of 34 |
| `fixed` regression | `snapshot_robot_model.py` output byte-identical between the old and the new commit, before the stop fixes, which touch only whole-body paths |
| Red tests | The stop tests failed on the code before the stop fixes: follower 6 of 13, stopping 7 of 11. The clock regression test failed before its fix. |
| `pixi run -e docs docs-build` | Builds without warnings |

The last demo run, with `--cycles 2`:

| Step | Group | Status | Base travel / displacement | Turn | Tip error |
| --- | --- | --- | --- | --- | --- |
| `far_arm` | `left_arm_lift` | `Planning Failed`, as expected | 0 | 0 | Not scored |
| `near_arm` | `left_arm_lift` | `Success` | 1.8 / 0.3 mm, 0 base commands | 0.004 rad | Not scored |
| `near_body` | `whole_body_left` | `Success (base fixed)` | 3.6 / 0.6 mm, 0 base commands | 0.015 rad | Not scored |
| `far_body` | `whole_body_left` | Success | 1.022 / 1.020 m | 0.77 rad | 10.5 mm |
| `home_0` | `whole_body_left` | Success | 1.016 / 1.014 m | 0.17 rad | 8.9 mm |
| `far_0` | `whole_body_left` | Success | 1.138 / 1.136 m | 0.34 rad | 11.3 mm |
| `home_1` | `whole_body_left` | Success | 1.093 / 1.092 m | 0.04 rad | 9.4 mm |
| `far_1` | `whole_body_left` | Success | 0.987 / 0.986 m | 0.50 rad | 11.3 mm |
| `seam_turn` | `whole_body_left` | Success | 0.147 / 0.147 m | 2.92 rad, heading change 2.92 rad | 8.7 mm |
| `seam_cross` | `whole_body_left` | Success, one crossing of ±π | 0.860 / 0.858 m | 1.74 rad, heading change 1.74 rad | 13.3 mm |

- Base, tip, and arm positions are from MuJoCo's ground truth. `map` to `odom` was fed from
  the same truth, so these numbers contain no localization error.
- Most of the tip error is height: the tip stops 7 to 11 mm below the target. The lift's
  own sag explains about 3 mm. The rest was not separated, and is probably gravity sag in
  the arm.
- The collision monitor did not act in this run. In the run before it, the slowdown polygon
  turned on and off three times and the follower kept tracking.
- The three runs before it each missed something, and each miss led to a fix: a false clock
  refusal, a demo target that did not make the base cross ±π, and `near_body` (see
  [Still open](#still-open)). This is one clean run, not a success rate.

## Commits

To be listed once the branch lands.

## Still open

These items from the plan have not been run:

- Ghost voxels after a 5 m loop.
- The n = 30 pose-goal comparison on `left_arm` and `left_arm_lift` against the old binary.
- The reach set: at least 95% success, with p95 planning time at or under 2.5 s.
- Replay of accepted plans with a full-state validity check.
- A test of `right_joint2_fixed` on `whole_body_right`.
- The comparison with Nav2 followed by an arm goal, over 30 runs. This is the question the
  experiment was meant to answer, so the whole-body option has not yet been shown to beat it.
- A measurement of how fast a collision monitor stop halts arm, lift, and base. The target is
  0.5 s.
- Arm goals with the follower frozen or killed, in MuJoCo.
- A `move_group` restart path. Today, restart the commander along with `move_group`.

Known limits:

- In one run, `near_body` moved the base by 3 cm instead of reporting `base fixed`. The log
  showed no arm-only attempt, and 8 attempts against stubs did not reproduce it. The first
  attempt now tries a second IK seed and logs why it gave up. The next run passed, which does
  not prove that this was the cause.
- A goal near the edge of the arm's reach can move the base by a few centimeters, because the
  first attempt needs an exact IK solution. Callers who need the base to stay still use an arm
  group.
- The real-robot Nav2 parameters set no `state_topic`, so the follower would not see a
  collision monitor stop there. The real lift has its own driver, which does not accept a
  cancel.
- The demo's `table_clearance_m` ignores the base heading, so it is only a rough number. It was
  0.14 m in the last run and -0.02 m in the run before it. No contact was seen, but the box pose
  is not recorded.
- `manipulate/README.md` still lists old group names. It was out of date before this change.
- Migration Log number 7 is also claimed by draft PR #63. Whichever lands second takes the next
  number.
