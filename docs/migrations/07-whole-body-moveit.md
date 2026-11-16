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
  speed scaling from `feat/commander-speed-scaling`. A whole-body goal does not move the
  other arm, so send that arm to `standby` first (added 2026-11-11, see
  [Planning near the table](#planning-near-the-table)). A pose goal to a single-arm group
  is now solved by IK first, and can end in a different arm configuration than before
  (added 2026-11-14, `ik_first_enable` turns it off, see
  [IK first for arm pose goals](#ik-first-for-arm-pose-goals)). Whole-body goals have not
  run on the real robot, so keep `base_mode:=fixed` there.
- **Added 2026-11-16:** weights that rank the sampled base poses of a whole-body goal, a
  rest posture, and a hand target to drag in RViz. The defaults pick the same base pose
  as before. See
  [Weighted whole-body goals and a hand target](#weighted-whole-body-goals-and-a-hand-target).
- **Result:** the MuJoCo demo passed all ten steps in the last run here and in six runs of
  three independent checks, and the launch tests pass ([Verification](#verification)). A
  collision monitor stop missed its 0.5 s target in four of seven runs, by up to 0.09 s of
  simulation time, on the lift or the arm. The plan's larger measurements have not been run
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
| `manipulate/openarm_bimanual_commander_cpp/src/whole_body_option.hpp` | The class behind the new group names: the gate, the base sampling, the repairs, and the stops. Its refusal names the idle arm when it collided or lacked clearance in an IK solution. |
| `manipulate/openarm_bimanual_commander_cpp/src/whole_body_base_follower.cpp` | The action server of `base_controller`. It closes the loop on `map` to `base_footprint` and publishes to `/cmd_vel_smoothed`. |
| `manipulate/openarm_bimanual_commander_cpp/src/unwrap_planar_heading.cpp` | A planning response adapter that keeps the base from turning the long way across ±π |
| `manipulate/openarm_bimanual_commander_cpp/src/whole_body_demo.py` | The demo client. It tucks the right arm to `standby` before a target over the table, and sends `right_arm_lift` home at the end. It refuses to run when the commander has `plan_only` set. |
| `manipulate/openarm_bimanual_commander_cpp/src/benchmark_planning.cpp`, `manipulate/openarm_bimanual_moveit_config/launch/run_planning_benchmark.launch.py` | A planning benchmark against the demo's table scene, see [Measure planning speed](../guides/manipulation.md#measure-planning-speed) |
| `manipulate/openarm_bimanual_commander_cpp/src/ik_first_goal.hpp`, `commander_template.cpp` | IK first for single-arm pose goals, and the parameters `joint_goal_planning_time` and `ik_first_enable`, see [IK first for arm pose goals](#ik-first-for-arm-pose-goals) |
| `manipulate/openarm_bimanual_commander_cpp/test/` | Seven launch files and a stub world, see [Tests](../guides/testing.md#whole-body-launch-tests) |
| `bringup/launch/whole_body_demo.launch.py` | Starts the demo. `bringup/scripts/ground_truth_map_odom.py` feeds `map` to `odom` from MuJoCo's ground truth. |
| `bringup/launch/nav_mujoco_omnibot.launch.py` | Sets `state_topic` on the collision monitor, so the follower can see a stop |
| `description/mujoco/scene_whole_body.xml` | The scene: a table, a free box, and a wall |
| `pixi.toml`, `pixi.lock` | `ros-jazzy-depth-image-proc` |
| `scripts/tmux_walkie.sh`, `.tmuxp.yaml` | Pass `base_mode:=fixed` explicitly to the MoveIt and commander panes |
| `manipulate/openarm_bimanual_commander_cpp/src/whole_body_goal.hpp` | The base pose sampling, the weighted cost, and the candidate search, without ROS. See [Weighted whole-body goals](#weighted-whole-body-goals-and-a-hand-target). |
| `manipulate/openarm_bimanual_commander_cpp/src/whole_body_marker.py` | The RViz hand target. `whole_body_demo.launch.py` starts it with `use_rviz:=true`, and `moveit_map.rviz` shows it. |
| `manipulate/openarm_bimanual_commander_cpp/test/test_whole_body_goal.cpp`, `whole_body_marker.test.py`, `whole_body_marker_client.test.py` | The gtest of the candidate search, and the hand target's launch tests |

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

??? info "What the second review found"
    A second independent review, of the version with the fixes above, found nine more
    problems. The last column says whether a test failed on the code before the fix. The
    old follower failed every test that sends the new heartbeat type, so its tests show
    nothing about the follower fixes one by one.

    | Finding | Fix | Failing test before the fix |
    | --- | --- | --- |
    | The start repairs raised the lift and clamped joints for every group. A `left_arm` goal then planned from a lift that was not there, and nothing closed the difference. | Repairs touch only the joints of the planned group. The lift is raised only for groups that move it. | Yes, option |
    | The heartbeat only covered a running base goal. A whole-body motion that keeps the base still ran to its end after the commander died. | The heartbeat is a `std_msgs/Bool` that says whether a whole-body goal executes. When it stops during one, the follower sends one `stop`. | Yes, stopping |
    | `halt` gave up after 10 s and sent no more stops, so a request still on its way to `move_group` could start afterwards | A thread keeps sending a stop every 500 ms until `move_group` finishes, and the status says so | No test |
    | A follower that started during a collision monitor stop never heard of it: the monitor reports changes only | A stall check: a base that lags the plan and does not move under the commands ends the goal | Not on its own |
    | Clearing the octree waited 1 s of wall time. At a low real-time factor the octomap had not refilled. | The wait is 1 s of the node's clock, with a 10 s wall-time cap | No test |
    | The motion ticket's deleter captured the commander object, which an execute thread can outlive | The motion slot lives in a shared state that the tickets and threads hold | No test, by construction |
    | Any aborted goal on any controller stopped the motion, including another client's goal on a controller the plan does not use, and old aborts still listed in a status | Only an abort of a goal that is new since the execution started, on a controller of the plan, stops it | Yes, stopping |
    | The follower re-sent every `stop` it heard and heard its own. An event carries no time, so a late one could end the next motion. | It ignores its own events, does not re-send an incoming one, and sends at most one per heartbeat period | Not on its own |
    | A Nav2 server that died during a goal left an `EXECUTING` status, which refused every later whole-body goal | A status counts only while its server is still in the graph | Yes, option |

    The follower now sends the `stop` event right after its first zero command instead of
    after all five, so the arm and the lift get it about 40 ms sooner.

??? info "What the third review found"
    A third review, of the second review's fixes, found four gaps in them and three wrong
    statements in these pages.

    | Finding | Fix | Failing test before the fix |
    | --- | --- | --- |
    | The heartbeat reported a motion up to 200 ms after `move_group` got it. A commander that died in that window left an arm-only plan running, with no `stop` from the follower. | The commander sends a beat when the motion starts, under the same lock as the periodic beats, and sends the plan 50 ms later | Yes, stopping, in 5 of 5 attempts |
    | A restarted Nav2 server publishes no status before its first goal. The dead server's `EXECUTING` stayed, and a publisher was on the topic again, so whole-body goals stayed refused. | A status counts only while the publisher that sent it is in the graph, compared by GID | Yes, option |
    | The stop thread after a 10 s `halt` could send one more stop after `move_group` finished, when a new goal could already have started | The thread holds the motion slot, and checks for the end before every stop | No test: a window of microseconds |
    | The late status of an older goal, which the commander had never listed, counted as an abort of the running motion | The goal's accept stamp has to be at most 0.1 s before the execution started | Yes, stopping |
    | The pages said the follower keeps up under a `PolygonSlow` ratio of 0.4. With `max_linear_speed` 0.25 it cannot. | The shipped ratio is 0.8, which leaves 0.2 m/s against the plan's 0.17 m/s. The guide now says so, and a test runs the diagonal speed at 0.8. | Not a code fix |
    | The second review's table claimed a failing test for every fix | The table above says which fixes had one | Not a code fix |
    | The guide said a cancel sends a `stop` event, and that the heartbeat covers every motion | A cancel sends none, because it comes from `move_group`. The flag covers whole-body goals only. | Not a code fix |

    Another client's new goal on a controller of the plan still stops the motion. It
    preempts the plan's goal on that controller, so the motion could not finish anyway.

??? info "What the second MuJoCo check found"
    An independent check in MuJoCo of the second review's fixes, at `adb32f8`, found two
    problems that the stub world could not show. Its numbers are under
    [Verification](#verification).

    | Finding | Fix | Failing test before the fix |
    | --- | --- | --- |
    | Every follower stop reached the caller as `Execution Failed, MoveIt code -7`. The reason, such as a collision monitor stop or a stall, was only in the follower's log. | The commander asks the aborted goal's action server for its result by the goal's UUID, and appends the server's reason to the status | Yes, stopping, for an abort of the base and of the arm |
    | The thread that carries on after a 10 s `halt` sent its first stop 1.4 ms after `halt`'s last one | The thread waits 500 ms before its first stop | No test: the extra stop did no harm |

??? info "What the fourth review found"
    A review of the third review's fixes and the two above found three problems in them.

    | Finding | Fix | Failing test before the fix |
    | --- | --- | --- |
    | The abort reason was read through a reference into a response that was already freed | The response is kept until the reason is copied | Yes, stopping: the commander read a reason with null characters, threw while sending the result, and died at the first arm abort |
    | After a 10 s `halt`, fetching the reason could hold back the stop thread for up to 500 ms, so the stops paused for about 1 s | The thread starts before the reason is fetched | No test: needs a frozen `move_group` |
    | When `move_group` failed on its own, another client's abort on a controller of the plan in the next 300 ms was reported as the cause | An abort counts only if its goal was accepted before `move_group` returned | No test: the stub world cannot make `move_group` fail without an abort |

    The review also confirmed from the `rmw_zenoh_cpp` 0.2.11 and `rmw_fastrtps_cpp` 8.4.4
    sources that a message's publisher GID and the graph's endpoint GID are the same bytes,
    which the Nav2 restart fix relies on.

## Planning near the table

Added on 2026-11-11, after plans failed more often close to the table. A new benchmark,
`run_planning_benchmark.launch.py`, measured why and how fast planning is
([Measure planning speed](../guides/manipulation.md#measure-planning-speed)). No collision
mesh was wrong. Most failures came from the idle arm and from targets placed too low:

- **The idle arm:** a whole-body goal for one arm leaves the other where it is. In `home`
  that arm hangs to 0.61 m, below the table top at 0.74 m, so the column cannot come closer
  than about 0.2 m to the table edge. Callers now send it to `standby` first, and the
  refusal names it ([Clear the idle arm](../guides/manipulation.md#clear-the-idle-arm)).
- **Low targets:** a target is for the hand frame, not the fingertips. The guide now lists
  the lowest hand height over a surface for each gripper orientation, and the margin a
  whole-body goal needs on top
  ([Targets close to a surface](../guides/manipulation.md#targets-close-to-a-surface)).
- **The idle arm after a low target:** `home` puts an arm's fingers into the base below a
  lift position of 0.50 m, so send the idle arm home with its lift group
  (`right_arm_lift` or `left_arm_lift`), which raises the lift too.
- **Arm planning time:** stays at 5 s, and arm plans use all of it. A pose goal also has
  to find its IK goal state within that time. At 1.5 s, one goal with the idle arm in
  `home` failed 4 of 12 times in the benchmark and 2 of 8 times in the launch tests
  ([Planning time and attempts](../guides/manipulation.md#planning-time-and-attempts)).
  The faster path for single-arm pose goals came later, see
  [IK first for arm pose goals](#ik-first-for-arm-pose-goals).

??? info "Causes found, and what was done"
    | Cause | Effect | Done |
    | --- | --- | --- |
    | The idle arm in `home` reaches 0.61 m high and 0.22 m in front of the column | In the box scene, facing the table 0.08 m away, IK reached none of 18 targets with the gripper 3 cm over the table, against 13 of 18 with the idle arm in `standby` (up to 16 IK calls per target). From 1 m before the table on the octomap, whole-body goals over the far half of the table succeeded 13 of 18 times, against 15 of 18. | The refusal counts the IK solutions the idle arm was in collision with and names the arm. The demo tucks the right arm before a target over the table. |
    | Targets are for the hand frame | Pointing forward, the wrist camera bracket reaches 0.111 m below the hand frame. Pointing down, the fingertips reach 0.095 m below it, plus 0.01 m of padding. | Documented, with the lowest hand height for each orientation |
    | The octomap places the table top up to about 0.04 m high: 0.03 m voxels, and `base_footprint` 6 mm below the floor in MuJoCo | A target a few centimeters over the real top collides with the octomap | Documented margin |
    | A second collision box on the column, from y -0.23 to 0 m in `base_link`, sticks out 0.13 m to the robot's right | Approaches with the right side to the table stop earlier than the column would | Left as is, see [Still open](#still-open) |
    | OMPL checks motions at discrete points, 0.003 of the joint space's extent apart | About 4% of plans with 10000 attempts (2 of 56) and about 10% of plans with 1 or 4 attempts at that spacing (16 of 156) failed `ValidateSolution` with `INVALID_MOTION_PLAN`, most likely because the path grazes the table between two checks | Not changed, see the choices below |
    | A whole-body goal must also be valid with the lift 0.01 m lower, because the lift settles short | With the octomap reading the table high, the demo's first deep target, its bracket 0.05 m over the real top, failed 9 of 9 times with `No base pose found` | Documented: about 0.05 m in all over a surface. The demo target went up to 0.10 m over the top. |
    | In `home`, an arm's fingers reach into the base below a lift position of 0.50 m | After a low target the idle arm cannot go back to `home`: `Failed to plan path to 'home'`, because MoveIt finds no valid goal state | Documented. The demo sends `right_arm_lift` home, which raises the lift in the same motion. |
    | The demo's deep target at y 0.30 put the base near a table leg | The collision monitor stopped 3 of 9 deep goals at y 0.30 and 1 of 5 at y 0.10. The one stop examined lasted 20 ms, a single scan, and the follower aborts the goal on any stop | The demo target moved to the table's centre line, y 0. See [Still open](#still-open). |

??? info "The choices, and what was rejected"
    | Question | Chosen | Rejected, and why |
    | --- | --- | --- |
    | Who moves the idle arm | The caller, with `/go_to_home` | The commander tucking it on its own. It would move an arm the caller did not name. |
    | Which link a target is for | The hand frame, as before | A fingertip frame. Every existing caller's targets would shift. |
    | Arm planning time and attempts | 10000 attempts in 5 s, as before | 1.5 s: joint goals planned in 1.6 to 1.7 s with motions about as long as at 5 s, but a pose goal with the idle arm in `home` failed 4 of 12 times in the benchmark and 2 of 8 times in the launch tests, because no IK goal state turned up in time. 4 attempts: 0.14 to 0.31 s per plan, but motions 0.5 to 2.0 s longer and about 12% of plans failing validation. 4 attempts with `longest_valid_segment_fraction` 0.0005: no validation failures, but 1.0 to 1.8 s per plan, longer motions, and whole-body goals 3 to 4 times slower. |
    | TRAC-IK solve type | `Distance`, as before | `Speed`. Whole-body goals took 0.5 to 0.8 s instead of 0.5 to 0.9 s, hardly faster, and its solutions are not the nearest to the current arm pose, so motions are larger and less repeatable. |
    | The side box on the column | Unchanged | Removing it. Nobody here knows what it stands for. |

## IK first for arm pose goals

Added on 2026-11-14. In the planning benchmark, a `left_arm_lift` pose goal now plans in
a median of about 1.2 s instead of about 5.2 s, and it succeeded at least as often. On a
0.02 m octomap half the goals fell back, and the median stayed at about 5.3 s. The
benchmark leaves out the commander's service calls and measured `left_arm_lift` only;
`left_arm`, `right_arm`, and `right_arm_lift` take the same path. Every arm plan uses
its full planning time, and a pose goal needed the 5 s of `arm_planning_time` because
OMPL samples its IK goal states inside the plan. A joint goal skips that sampling, so
the commander now turns the pose into a joint goal first:

- **IK:** up to four calls to `move_group`'s `/compute_ik` with `avoid_collisions`,
  seeded from the current state, from `home` with the lift where it is, and from both
  again. `/compute_ik` checks only the links of the group, so each solution also has to
  pass `/check_state_validity` for the whole robot; for `left_arm_lift` and
  `right_arm_lift` also with the lift 0.01 m lower, as whole-body goals do. The calls
  take about 50 ms per goal. All of them share 2 s, the commander stops at the first
  call without an answer and sends none after the 2 s, and a cancel stops them within
  about 20 ms.
- **Plan:** to the first solution that passes, for the new parameter
  `joint_goal_planning_time` (1.0 s), with a joint goal tolerance of 0.001 rad, so the
  arm ends on the checked solution.
- **Fallback:** when no solution passes, IK gets no answer, or that plan fails, the pose
  goal itself for `arm_planning_time` (5 s), as before. The commander logs which way a
  goal went.
- **Relative pose goals** (`/go_to_pose_relative`) take the same path, also as the
  fallback of a Cartesian path that fails.
- **Named states and joint goals** (`/go_to_home`, `/set_joint_position`) of every arm
  group, `both_arms` and `both_arms_lift` included, plan for `joint_goal_planning_time`
  too, and plan again for `arm_planning_time` when that fails. Gripper groups keep 1 s.
- **Unchanged:** pose goals to `both_arms` and `both_arms_lift`, which are not chains
  and have no IK solver; Cartesian paths that succeed; and whole-body goals.
- **Turn it off** with `ik_first_enable false`. A `joint_goal_planning_time` at or above
  `arm_planning_time` turns off the shorter plans, so the commander then plans as it did
  before this change.

Each goal now holds a lock on its MoveGroupInterface from setting it up until its plan
is done, and applies `planner_id`, `arm_planning_time`, and the two scaling factors to
it under that lock. The parameter callback used to set them on the groups itself, which
could change a group in the middle of another goal's plan; it now only checks and logs
them. Two goals on the same group plan one after the other, and execute as before. A
goal waits for the lock in 20 ms steps, so a cancel ends the wait within about 20 ms,
and a goal canceled by then plans nothing.

The benchmark numbers are in
[Planning time and attempts](../guides/manipulation.md#planning-time-and-attempts).
With the idle arm in `home`, a target close to the body still takes about 5.5 s: the four
IK calls found no solution with clearance for the launch tests' near target in 47 of 48
tries, and the fallback then planned it.

??? info "The choices, and what was rejected"
    | Question | Chosen | Rejected, and why |
    | --- | --- | --- |
    | Planning time of the joint goal | 1.0 s | 0.5 s: medians of 0.69 to 0.79 s, and its joint plans failed about as often (4 of 91 IK solutions, against 3 of 93 at 1.0 s). Not chosen because the robot's computer has not been measured: a slower one runs fewer searches in 0.5 s, and each failed plan costs the 5 s fallback. 1.5 s: about 0.5 s slower than 1.0 s, with about as many fallbacks. |
    | Where IK runs | `move_group`'s `/compute_ik` and `/check_state_validity` | IK in the commander's process. The commander has no planning scene of its own, and the services check against `move_group`'s, octomap included. |
    | Lowering `arm_planning_time` instead | Kept at 5 s | It is the planning time of every fallback, and the fallback gets exactly the goals that IK first could not solve, such as the near target with the idle arm in `home`, which planned 8 of 12 times at 1.5 s. |
    | Which arm configuration a goal ends in | The first IK solution that passes, from the seeds in order: current state, `home`, current state, `home`. TRAC-IK's Distance mode returns a solution near its seed, so the first seed gives one near the current joints, and the `home` seed one near `home`. | Leaving the goal state to OMPL. A pose goal planned directly ends anywhere within the 0.02 m and 0.02 rad tolerances. In the benchmark the motions were about as long on average either way, and longer for some goals with IK first. |

## Weighted whole-body goals and a hand target

Added on 2026-11-16. A whole-body goal that the arm cannot reach alone used to plan to
the nearest base pose with a usable IK solution. It now ranks the usable poses by a
weighted cost of how far each part moves, and the RViz bring-up has a hand target that
sends whole-body goals:

- **Weights:** six new commander parameters, `whole_body_weight_base_xy`, `_base_yaw`,
  `_lift`, `_arm_joints` (seven values, joint1 to joint7), `_rest`, and
  `whole_body_candidates`. The cost of a base pose is the weighted sum of base travel,
  base turn, lift travel, each arm joint's motion, and the arm's distance from the rest
  posture. The defaults weigh base travel alone, which picks the same pose as before.
- **Rest posture:** `whole_body_rest_state` (`home`) names the state the arm should stay
  near. It seeds IK at every sampled base pose and in the arm-alone attempt, and
  `whole_body_weight_rest` turns it into a cost. It has to be a state of both
  `left_arm_lift` and `right_arm_lift`.
- **Search:** the samples of each tier stay sorted by base cost. The commander solves IK
  in that order and holds up to `whole_body_candidates` usable poses, until a pose's base
  cost alone reaches the cheapest one held. It plans to the cheapest, and after a failed
  plan goes on to the next cheapest. With the defaults that is the nearest usable pose,
  with no extra IK call.
- **Hand target:** `whole_body_marker` shows a box in RViz. Drag it, right-click it, and
  pick **Reach with left arm** or **Reach with right arm**, see
  [Drag a hand target in RViz](../guides/manipulation.md#drag-a-hand-target-in-rviz).
- **Benchmark:** `wb_presets` plans the same whole-body goals once per weight preset, see
  [Compare weights in the benchmark](../guides/manipulation.md#compare-weights-in-the-benchmark).
- **Unchanged:** arm groups, goals the arm reaches from where the robot stands, and every
  plan's limits and checks. A weight cannot keep the base still; an arm group does.

In the planning benchmark, a turn weight (`base_yaw=0.3`) more than halved the base turn
at no extra planning time. Arm and rest weights cut arm motion by 12 to 16 percent, for 2
to 5 cm more base travel and about 0.5 s more per goal. See
[Compare weights in the benchmark](../guides/manipulation.md#compare-weights-in-the-benchmark).

??? info "The choices, and what was rejected"
    | Question | Chosen | Rejected, and why |
    | --- | --- | --- |
    | Where the weights act | On the base poses after IK, as a cost that ranks them | Weighted IK, so the arm and the base share one solve. TRAC-IK has no joint weights, and the planar base is not part of its chain. |
    | What the rest posture does | Seeds IK and, with a weight, adds a cost | A seed alone. TRAC-IK's Distance mode returns a solution near its seed, but a different seed can still win, so the seed cannot guarantee a posture. |
    | How many poses get IK | Only while an unsolved pose could still be cheaper, at most `whole_body_candidates` held | IK at every sample before planning. 80 samples cost far more IK calls, and most of them cannot beat the poses already found. |
    | How the base stays still | An arm group, as before | A weight on the base. Any finite weight still lets the base move when the arm cannot reach. |
    | How RViz sends a goal | An interactive marker that sends `/go_to_pose_quat` when a menu entry is picked | RViz's MotionPlanning gizmo on the whole-body groups. It needs an IK solver for the planar base plus the arm, which MoveIt does not have. Planning on every drag. A plan takes about a second and moves the robot. |

??? info "What the review found"
    An independent review of the first version found six problems.

    | Finding | Fix |
    | --- | --- |
    | After a failed plan, the next candidate came from the poses already solved, though an unsolved pose could be cheaper | The search resumes after a failed plan, and bounds the unsolved poses by the cheapest candidate still held |
    | The benchmark gave each preset other sampled poses, and planned some goals the commander refuses because the base turns too far | One sampler seed per target, shared by every preset, and the commander's turn check |
    | **Cancel goal** before the commander accepted the goal did nothing, and the goal ran on | The cancel waits for the acceptance and is sent then |
    | A failed goal request, a failed result, or a commander that died left the marker busy for good. A late reply to an old goal could change the text of a new one. | Each goal has a number, and replies to an older one are dropped. Errors end the goal. When the action server is gone for 3 s during a goal, the text says `outcome unknown` and the marker takes new goals. |
    | A `ros2 param set` of several whole-body parameters at once was applied one parameter at a time, so a goal could start with half of them | The whole set is applied under one lock |
    | The guide said to drag the box, which had no control of its own | The box moves in the view plane |

    A second review, of the fixed version, found two gaps in the marker's lost-goal
    check. Both tests failed on the code before the fix.

    | Finding | Fix |
    | --- | --- |
    | A commander that restarted within a second or two served the action again before the check counted three misses, and the new one never answers the old goal, so the marker stayed busy | The marker remembers the endpoint gid of the status publisher of the server it sent the goal to, and counts a miss while that server is gone, whatever else serves the action |
    | The check ran on the node's clock, so on simulation time a paused `/clock` stopped it | It runs on steady time |

    A third review, of that fix, found that the marker took the gids of every publisher on
    the action's status topic. With a second one, such as a second commander, the dead
    server stayed hidden behind the live one. The tests below failed on the code before
    the fix.

    | Finding | Fix |
    | --- | --- |
    | With two status publishers, the commander that had the goal could exit and the marker stayed busy | The marker sends a goal only when exactly one server publishes the status, and follows that one |
    | The restart test let the new commander start after the old one exited, so it relied on the restart being fast | The new commander starts before the old one exits, so a check that only asks whether the action is served fails it |
    | Ctrl-C ended the marker with exit code 1: rclpy's signal handler and `main()` both shut the context down, and the second `rcl_shutdown` failed. The same happened in 10 of 10 tries with the usual `except (KeyboardInterrupt, ExternalShutdownException)`. | The marker installs no rclpy signal handler, so Ctrl-C is a `KeyboardInterrupt` and `main()` alone shuts down. 10 of 10 tries exited with 0, within 0.64 s. A post-shutdown test checks the exit code. |

    Not changed: the marker takes the gid just before it sends the goal. A commander that
    exits and a new one that appears in that moment would make it follow the wrong one.
    A restart takes about a second, so this needs both to land in under a millisecond.

## CI

`openarm_bimanual_commander_cpp` joins the tested packages. Its launch tests add about 11
minutes to the **Test** step, and they use the router that CI already starts. These tests
have not run in CI yet, because the branch has not been pushed. They are timing-sensitive
in places, so a slower runner may show problems that this laptop did not.

## Verification

Checked on 2026-11-05 on a laptop with an RTX 2060, with `navigation` at `7f15b1c`, in
MuJoCo with `rmw_zenoh_cpp`, the viewer, and RViz. The real-time factor was between 0.16
and 0.30, depending on what else the laptop ran.

| Check | Result |
| --- | --- |
| Whole-body launch tests | Option 16 of 16, gate 10 of 10 with 2 skipped by design, plan-only 1 of 1, clock 2 of 2, stopping 15 of 15, follower 17 of 17, in 13 min 31 s |
| `openarm_bimanual_moveit_config` tests | 32 of 32: 26 contract tests and 6 floor-gate tests |
| `fixed` regression | `snapshot_robot_model.py` output byte-identical between the old and the new commit, before the stop fixes, which touch only whole-body paths |
| Red tests | The stop tests failed on the code before the stop fixes: follower 6 of 13, stopping 7 of 11. The clock regression test failed before its fix. The tests of the second review's fixes failed on the code before them: option 2 of 15, stopping 2 of 12. The follower file failed 14 of 16 there, because the old follower expects the old heartbeat type. The tests of the third review's fixes and the second MuJoCo check's failed on the code before them: option 1 of 16, stopping 3 of 15. |
| `pixi run -e docs docs-build` | Builds without warnings |

An independent check in MuJoCo, of the version before the second review's fixes, gave the
following. "Quiet" means the base below 0.01 m/s and 0.02 rad/s, the lift below 0.001 m/s,
and every left arm joint below 0.01 rad/s, for 0.5 s. Times are wall time from the event
and include settling.

| Check | Result |
| --- | --- |
| Demo, three fresh runs | 3 of 3 passed every step. `near_body` reported `base fixed` with no base command each time. Largest tip error 14.1 to 14.8 mm. |
| 30 min soak with the viewer and RViz | Seven whole-body goals succeeded. No process restarted. `move_group` grew by 49 MiB. |
| Cancel during a far goal | Base, lift, and arm quiet after 0.48, 0.70, and 0.85 s |
| Commander killed during a far goal | The follower aborted on the heartbeat. Base, lift, and arm quiet after 2.25, 2.52, and 2.61 s. A restarted commander ran a new goal without a `move_group` restart. |
| Follower killed or frozen | Arm-group goals succeeded in all four cases, with no base command |
| Nav2 goal during a motion | Stopped 0.10 s after the Nav2 goal was sent. A whole-body goal during a Nav2 goal was refused within 0.10 s. |
| Real collision monitor stop, two runs | Base 0.27 and 0.61 s, lift 0.72 and 0.87 s, arm 0.63 and 0.95 s of wall time. The simulation ran at about 0.62 of real time, so in simulation time the second run is about 0.38, 0.54, and 0.59 s. |
| `fixed` bring-up | `left_arm_lift` succeeded and `whole_body_left` was refused with `Whole-body mode unavailable` |
| The standalone commands in the guide | The documented goal succeeded, with a router already running |

A second independent check in MuJoCo, of the second review's fixes at `adb32f8`, used a
stricter "quiet": the same thresholds held for 0.5 s of simulation time. Times are
simulation time from the event, then wall time, and include settling. The real-time factor
was 0.30 to 0.40.

| Check | Result |
| --- | --- |
| Demo, two fresh runs with `--cycles 2` | 2 of 2 passed every step. `near_body` reported `base fixed` with no base command. Largest tip error 21.7 and 13.8 mm. |
| Real collision monitor stop, three runs | Base 0.32, 0.32, 0.24 s. Lift 0.48, 0.49, 0.48 s. Arm 0.55, 0.56, 0.46 s, so the arm missed the 0.5 s target in two runs. The status was `Execution Failed, MoveIt code -7`, without the follower's reason (now fixed). |
| Commander killed during a goal that keeps the base still | The follower sent exactly one `stop`, 2 s of wall time after the kill. Arm quiet after 0.49 s, lift after 0.88 s. No base command before or after. |
| `move_group` frozen (`SIGSTOP`), then the goal canceled | The caller got `Canceled (move_group has not finished: ...)` after 10.1 s. The follower stopped the base after 0.29 s, but the arm and the lift ran on until `move_group` resumed. Other goals were refused meanwhile. The first goal after resuming succeeded. |
| Collision monitor stop that a restarted follower never heard | The stall check ended the goal 1.57 s after it started (`the base is not moving under its commands`). Arm quiet after 1.78 s, lift after 2.02 s. |
| `bt_navigator` killed during a Nav2 goal | Whole-body goals were refused with `Nav2 has an active goal on follow_path` for the 25 s observed, because `controller_server` kept following the path, and the base kept driving. The dead `navigate_to_pose` server no longer refused anything. |

A third independent check in MuJoCo, of the fixes from the third review, the second check,
and the fourth review at `773716c`, used the stricter "quiet" of the second check. Times
are simulation time from the event, then wall time. The real-time factor was 0.52 to 0.77.

| Check | Result |
| --- | --- |
| Demo, one fresh run with `--cycles 2` | Passed every step. `near_body` reported `base fixed` with no base command. Largest tip error 14.1 mm. |
| Real collision monitor stop, two runs | Base 0.19 and 0.31 s, lift 0.49 and 0.53 s, arm 0.43 and 0.50 s, so the lift missed the 0.5 s target by 26 ms in one run. Both statuses read `Execution Failed, MoveIt code -7: base_controller aborted its goal (collision monitor stop on polygon VelocityPolygonStop)`. |
| `move_group` frozen (`SIGSTOP`), then the goal canceled | Stops every 499 to 516 ms, 499 and 502 ms across the 10 s handoff. The caller got `Canceled (move_group has not finished: ...)` after 10.1 s, the base was quiet after 0.23 s, and the first goal after resuming succeeded. A cancel also sends four stops within 3 ms: the commander's cancel handler, which predates this change, stops each arm interface. |
| Collision monitor stop that a restarted follower never heard | The stall check ended the goal 1.58 s after it started. The status read `Execution Failed, MoveIt code -7: base_controller aborted its goal (the base is not moving under its commands (0.104818 m, -0.070184 rad behind): a collision monitor stop or a dead command path)`. |
| `bt_navigator` and `controller_server` killed during a Nav2 goal | No whole-body goal was refused for a Nav2 reason. The first four were refused with `The base is moving` while the base coasted to rest. The fifth started 2.4 s after the kill and succeeded. |

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

### Planning near the table { #verification-near-the-table }

Checked on 2026-11-11 on the same laptop, in MuJoCo with `rmw_zenoh_cpp`, headless and
without RViz, for the changes in [Planning near the table](#planning-near-the-table).

| Check | Result |
| --- | --- |
| Arm plans in the benchmark | See [Planning time and attempts](../guides/manipulation.md#planning-time-and-attempts). The benchmark numbers on this page and in the guide are from runs with the commander's 0.01 m finger padding; the first runs had none and were redone. Two exceptions: the empty-scene pose goals pool a run with and one without padding (MoveIt applies link padding only against world objects, and that scene has none), and the `path_tolerance` comparison ran without padding. |
| Whole-body goals in the benchmark, from 1 m before the table | 18 of 18 targets per idle-arm pose, on the boxes and on the octomap with 0.03 m voxels. Median 0.5 to 0.9 s per goal, slowest 1.2 s, IK and planning together. Targets over the far half of the table: see the causes table above. |
| Commander launch tests | 67 tests, 2 skipped, none failed, with `arm_planning_time` 5 s. With 1.5 s, `test_near_goal_after_an_arm_goal_keeps_the_base_still` failed 2 of 8 tries with `Planning Failed`. |
| Demo, three fresh runs with `--cycles 1` | Runs 1 and 3 passed every step. Run 2 passed every step up to `home_right`, which then sent `right_arm` home and failed with the lift at 0.492 m. The demo now sends `right_arm_lift`, and run 3 passed with it. |
| `deep_body` in the three runs | Base travel 0.37, 0.49 and 0.42 m. The column came within 0.100, 0.061 and 0.137 m of the table edge. Tip error 11.3, 11.2 and 9.0 mm. |
| Collision monitor in the three runs | No stop. The slowdown polygon turned on 5, 3 and 4 times. |

### IK first { #verification-ik-first }

Checked on 2026-11-14 on the same laptop, for the changes in
[IK first for arm pose goals](#ik-first-for-arm-pose-goals). Nothing ran in MuJoCo since
these changes: the checks are the planning benchmark and the commander launch tests.

| Check | Result |
| --- | --- |
| Planning benchmark | See [Planning time and attempts](../guides/manipulation.md#planning-time-and-attempts), run of 2026-11-14 |
| `arm_ik_first.test.py` | 8 of 8 in about 70 s |
| Red test | With a lock wait that ignored the cancel, the test of a goal canceled while it waits for another goal's plan failed: the goal ended 7.7 s after the cancel. |
| Commander launch tests | 76 tests, 2 skipped, none failed, in 11 min 17 s. Before the review fixes: 72 tests, 2 skipped, none failed, in 10 min 57 s. |
| `pixi run docs-build` | Builds without warnings |

### Weighted goals and the hand target { #verification-weighted-goals }

Checked on 2026-11-16 on the same laptop, for the changes in
[Weighted whole-body goals and a hand target](#weighted-whole-body-goals-and-a-hand-target).

| Check | Result |
| --- | --- |
| `test_whole_body_goal` (gtest) | 11 of 11 |
| Commander launch tests | Option 18 of 18, gate 10 of 10, plan-only 1 of 1, clock 2 of 2, stopping 15 of 15, follower 17 of 17, IK first 8 of 8, marker 3 of 3, marker client 3 of 3. After the third review's fixes, marker client 4 of 4 and its exit-code check, in 23 s. In the full run, two files failed and passed when run again: stopping lost `test_near_goal_after_an_arm_goal_keeps_the_base_still` to `Planning Failed` in the `left_arm_lift` fallback, a path this change does not touch, while other tests ran on the laptop; marker client ran against an installed marker older than the source. |
| Red tests | The tests of the second review's fixes failed on the code before them. With the paused simulation clock, the lost and the restarted commander both failed. With the marker on wall time, the restarted commander still failed. The tests of the third review's fixes failed on the code before them: two status publishers, and the exit code. The new restart test failed on a copy of the marker that only asks whether the action is served. |
| Planning benchmark | See [Compare weights in the benchmark](../guides/manipulation.md#compare-weights-in-the-benchmark), results of 2026-11-16 |
| `pixi run docs-build` | Builds without warnings |

A smoke run in MuJoCo, headless and without an RViz window: `whole_body_marker` ran on
simulation time next to the demo launch file, and a script sent it the feedback messages
that RViz sends. Base motion is from MuJoCo's ground truth. One run of each goal, in
this order, so it is not a comparison of the weights.

| Goal through the marker | Weights | Result | Base displacement | Turn | Lift | Arm | Commander log |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Far target, from the start pose | Defaults | Success after 27 s | 1.05 m | 0.83 rad | 0.23 m | 5.6 rad | `candidate 1 of 1 usable` |
| Back to the start | Defaults | Success after 24 s | 1.00 m | 0.65 rad | 0.08 m | 4.4 rad | `candidate 1 of 1 usable` |
| Far target | `base_yaw` 0.3, `arm_joints` 0.1 each, set with `ros2 param set` | Success after 29 s | 1.17 m | 0.19 rad | 0.08 m | 1.9 rad | `candidate 1 of 8 usable` |
| Back to the start | The same | Success after 29 s | 1.21 m | 0.09 rad | 0.02 m | 1.0 rad | `candidate 1 of 3 usable` |
| **Cancel goal** 8 s into a far goal | The same | `failed: Canceled` 0.2 s after the pick | | | | | |

The arm ended 5.6 to 7.2 rad from `home` after every goal, as no run set a rest weight.
RViz itself, the restart of a commander during a goal, and the rest weights have not run
in MuJoCo.

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
- A `move_group` restart path. Today, restart the commander along with `move_group`.

Known limits:

- A collision monitor stop missed its 0.5 s target in four of seven runs over three checks.
  The base met it every time. The lift came to rest after up to 0.54 s and the arm after up
  to 0.59 s of simulation time. The stop event leaves about 40 ms sooner since the first
  check, and the later checks still saw the arm at up to 0.56 s and the lift at 0.53 s.
- When `move_group` itself freezes, nothing stops the arm and the lift: the commander's and
  the follower's stops both go through `move_group`. The follower still stops the base. A
  direct cancel to the arm and lift controllers would close this, and is not built.
- Clearing the octree to repair a voxel inside the robot forgets everything out of view.
- The abort check compares a controller's accept stamp with the commander's clock, so both
  have to run on simulation time or both on wall time. With the controllers on simulation
  time and the commander on wall time, no abort counts, and only the deadline stops the motion.
- `FootprintApproach` slows the base when it heads for an obstacle within about 0.2 m. A plan
  that ends that close to the table can then stop with `not moving under its commands` or a
  tracking error. Not seen in MuJoCo so far.

- In one run, `near_body` moved the base by 3 cm instead of reporting `base fixed`. The log
  showed no arm-only attempt, and 8 attempts against stubs did not reproduce it. The first
  attempt now tries a second IK seed and logs why it gave up. The next run passed, which does
  not prove that this was the cause.
- A goal near the edge of the arm's reach can move the base by a few centimeters, because the
  first attempt needs an exact IK solution. In the second MuJoCo check, the target
  (0.45, 0.25, 1.00) in `base_footprint` moved it by 7.5 cm. Callers who need the base to
  stay still use an arm group.
- The real-robot Nav2 parameters set no `state_topic`, so the follower would not see a
  collision monitor stop there. The real lift has its own driver, which does not accept a
  cancel.
- The demo's `table_clearance_m` and `deep_column_gap_m` ignore the base heading, so they are
  only rough numbers. `table_clearance_m` was 0.14 m in one run and -0.02 m in the run before
  it. No contact was seen, but the box pose is not recorded.
- About 4% of arm plans near the table with the commander's 10000 attempts (2 of 56 in the
  benchmark) fail with `INVALID_MOTION_PLAN`, most likely because OMPL's motion checks miss a
  graze that the final check finds. Denser checks (`longest_valid_segment_fraction` 0.0005)
  removed it, but made each search 6 to 9 times slower and whole-body goals 3 to 4 times
  slower. Since IK first, named states and joint goals plan again with `arm_planning_time`
  after any failed plan, and a pose goal solved by IK falls back to the pose goal. Pose goals
  that OMPL plans directly, and Cartesian paths, still get no second plan.
- With the idle arm in `home`, the four IK calls of IK first miss solutions that the
  pose-goal fallback then finds, so such goals take about 5.5 s. More IK calls from random
  seeds would likely catch some of them, and are not built.
- Close to the table legs, the collision monitor stopped short base moves of whole-body
  goals: 4 of 14 deep goals at y 0.10 to 0.30 m. The one stop examined lasted a single scan
  (20 ms), and the follower aborts the goal on any stop. MuJoCo disables the monitor's
  point cloud source, so these stops come from `/scan`. Which band of
  `VelocityPolygonStop` fired was not recorded.
- A second collision box on the column, at y -0.23 to 0 m in `base_link`, sticks out 0.13 m to
  the robot's right. Ask whoever added it to `base.urdf.xacro` what it stands for before
  removing it.
- `manipulate/README.md` still lists old group names. It was out of date before this change.
- Migration Log number 7 is also claimed by draft PR #63. Whichever lands second takes the next
  number.
