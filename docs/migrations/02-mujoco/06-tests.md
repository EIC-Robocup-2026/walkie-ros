# 2.6 Tests

This entry records the tests that the MuJoCo work added, and why. They guard the
contracts between the URDF, the controller files, the generated MJCF model and the SRDF,
so a change on one side cannot silently break the other.

**Goal:** make every assumption the MuJoCo model depends on a test that fails with a
clear message, and remove the old test that did not test this robot.

## In short

- **Added:** `test_description.py`, `test_mjcf.py` and `test_srdf_urdf_contract.py`.
  `test_xacro.py` was removed: it tested Husarion's rosbot, not Walkie.
- **Key decisions:** check each controller file against the mode it is loaded with; mark
  known broken pairings as strict xfails; fail when the checked-in `walkie.xml` is stale;
  register tests with a CMake glob; pin pytest below 9.
- **Result:** 109 tests in three files. Two [known issues](#known-issues) remain.

## Which tests were added { #overview }

| File | Package | Build first | Tests |
| --- | --- | --- | --- |
| `description/test/test_description.py` | `walkie_description` | Yes | 76 |
| `description/test/test_mjcf.py` | `walkie_description` | No | 23 |
| `manipulate/openarm_bimanual_moveit_config/test/test_srdf_urdf_contract.py` | `openarm_bimanual_moveit_config` | Yes | 10 |

The counts are what `pytest --collect-only` reports for each file. `test_pep257.py` in
`description/test/` also still runs. It checks docstring style with `ament_pep257`.

## Run only these tests { #running-only-these-tests }

```bash
# No build needed
pixi run python -m pytest description/test/test_mjcf.py
pixi run python -m pytest description/test/test_mjcf.py -k lidar    # one group

# Build first
pixi run python -m pytest description/test/test_description.py
pixi run python -m pytest manipulate/openarm_bimanual_moveit_config/test/test_srdf_urdf_contract.py
pixi run colcon test --packages-select walkie_description
pixi run colcon test --packages-select walkie_description --ctest-args -R test_mjcf    # one file
pixi run colcon test-result --verbose    # show the results after a colcon run
```

!!! warning "Build first"
    Before the commands under `# Build first`, run `pixi run build` (or
    `pixi run build-pkg walkie_description`) so the installed share directory is current.
    Those tests read the **installed** files, so a stale install tests the old description.

## test_description.py

This file tests the URDF and the controller files. It does not use MuJoCo. It expands
`robots/gz_walkie.urdf.xacro` from the installed `walkie_description` share directory, so
the package must be built and the workspace sourced first.

### Configurations

The file expands nine configurations (`CONFIGURATIONS`): the five ros2_control modes and
four variants. [Tests: test_description.py](../../guides/testing.md#test_descriptionpy)
lists them with their xacro arguments. Seven tests run for every configuration.

??? info "The seven tests that run for every configuration"
    | Test | What it checks |
    | --- | --- |
    | `test_xacro_expands` | The configuration expands without xacro errors and contains links |
    | `test_link_and_joint_names_are_unique` | No two links and no two joints share a name |
    | `test_kinematic_tree_is_connected` | Joints connect existing links into one tree with one root |
    | `test_mesh_files_exist` | Every mesh URI resolves to an installed file |
    | `test_no_unresolved_xacro_expressions` | No `${...}` or `$(...)` survives the expansion |
    | `test_gazebo_references_exist` | Every `<gazebo reference>` names an existing link or joint |
    | `test_ros2_control_joints_are_movable_urdf_joints` | Every ros2_control joint is a non-fixed URDF joint, owned by exactly one block |

    Each expansion is cached with `functools.lru_cache`, so each configuration is expanded
    once per run.

### Controller pairings

**Decision:** check each controller file together with the ros2_control mode that the
launch files load it with. `CONTROLLER_PAIRINGS` records the pairs: `gazebo` with
`sim_controllers.yaml`, `mujoco` with `mujoco_controllers.yaml`, `topic_base` with
`isaac_controllers.yaml`, and both `real_robot` and `mock` with `real_controllers.yaml`.

??? info "Where each pairing is wired"
    | Mode | Wired by |
    | --- | --- |
    | `gazebo` | `sim_controllers_yaml` default of `gz_walkie.urdf.xacro`; `simulation/gazebo/launch/gzsim_omnibot.launch.py` |
    | `mujoco` | the `mujoco_ros2_control` controller manager (launch file not written yet) |
    | `topic_base` | `bringup/launch/isaac_omnibot.launch.py` |
    | `real_robot` | `bringup/launch/real_omnibot.launch.py` |
    | `mock` | `bringup/launch/test_zed.launch.py` |

??? info "What the two pairing tests check"
    | Test | What it checks |
    | --- | --- |
    | `test_every_controller_config_is_paired` | Every `.yaml` file in `config/ros2_controller/` has a pairing, and every pairing names an existing file |
    | `test_controller_config_matches_ros2_control[<mode>-<file>]` | See the list below |

    For each controller in the file, `test_controller_config_matches_ros2_control` checks
    that:

    - its type is known to the test;
    - every joint it names exists in the URDF and in a ros2_control block;
    - every command and state interface it claims is declared;
    - IMU broadcasters claim only declared sensor interfaces;
    - every frame parameter names a URDF link, except `odom_frame_id`, `map_frame_id` and
      `world_frame_id`.

### The strict xfail

`KNOWN_PAIRING_MISMATCHES` lists pairings that are known to be broken, each marked
`xfail(strict=True)`. Once a pairing is fixed, the run fails until its entry is removed.
[Tests: Strict expected failure](../../guides/testing.md#strict-expected-failure)
explains the mark. The one listed pairing today is described under
[Known issues](#known-issues).

### Omni geometry

`test_omni_wheel_geometry_matches_urdf[<file>]` runs once per controller file that
configures an `omni_wheel_drive_controller`. It compares `robot_radius`, the wheel order
and angles, and `wheel_radius` with the wheel joints in the URDF, measured in
`base_frame_id`.

**Why:** this check would have caught the old `robot_radius` of 0.252 m. The wheels sit
0.185 m from the center. The value was fixed in `13244ae` of the `description` submodule.

??? info "Checked parameters and tolerances"
    | Parameter | Check | Tolerance |
    | --- | --- | --- |
    | `robot_radius` | Equals the distance of each wheel joint from the center | 0.001 m |
    | `wheel_names` and `wheel_offset` | Wheels are listed counterclockwise, the first at `wheel_offset` from the X axis, then every 360 / n degrees | 1 degree |
    | `wheel_radius` | Equals the radius of the wheel's cylinder or sphere collision | 0.002 m |

    The wrong `robot_radius` of 0.252 m made the controller over-rotate by about 1.36
    times.

!!! note
    The `wheel_radius` mismatch passes this 2 mm check. See [Known issues](#known-issues).

### MuJoCo branch

Three tests check the `mujoco` mode only: its ros2_control interfaces, its hardware
plugin, and its `mujoco_model` parameter.

??? info "What the three MuJoCo tests check"
    | Test | What it checks |
    | --- | --- |
    | `test_mujoco_ros2_control_interfaces` | In `mujoco` mode, all four wheels are declared; wheels have only a `velocity` command, every other joint only `position`; no mimic joint is declared; state interfaces include `position` and `velocity`, and nothing outside {position, velocity} for wheels or {position, velocity, effort} for other joints |
    | `test_mujoco_hardware_plugin` | There is exactly one hardware plugin, `mujoco_ros2_control/MujocoSystemInterface` |
    | `test_mujoco_model_parameter_points_to_installed_scene` | There is exactly one `mujoco_model` parameter, it equals the installed `mujoco/scene.xml`, and that file exists |

## test_mjcf.py

This file tests the generated model and its default world. It locates every file
relative to itself, so it runs from a source checkout without a build. Most tests start
from the `home` keyframe of `description/mujoco/scene.xml`.

- It imports `mujoco` and `numpy` with `pytest.importorskip`. If either is missing, the
  whole file is **skipped**, not failed.
- It loads the generator, `description/scripts/build_mjcf.py`, which needs `xacro` to
  expand the description. If `xacro` cannot be imported, the tests that run the
  generator are skipped.

### (a) The default world compiles and the robot rests

The scene compiles with a plausible mass, `walkie.xml` loads alone, and the base stays
still at rest and during lift steps.

??? info "Tests and tolerances for the default world"
    | Test | What it checks | Tolerance |
    | --- | --- | --- |
    | `test_scene_compiles_with_plausible_mass` | Robot mass between 25 and 60 kg; `floating_base_joint` is a free joint on `base_footprint`; time step 0.002 s, `implicitfast` integrator, elliptic friction cones | exact for the options |
    | `test_model_loads_without_scene` | `walkie.xml` loads on its own and has no `floor` geom | exact |
    | `test_rest_is_stable` | 2 s at rest: state stays finite, no MuJoCo warnings, the base height changes less than 3 mm, the base drifts less than 1 cm | 3 mm, 1 cm |
    | `test_imu_at_rest` | See [2.4 Sensors](04-sensors.md) | 0.05 m/s², 0.01 rad/s |
    | `test_lift_step_keeps_base_on_floor[up]`, `[down]` | A lift setpoint step of ±0.2 m held for 3 s: state stays finite, the base height changes less than 5 mm, the base drifts less than 1 cm, and the lift moves slower than 0.2 m/s (twice the URDF limit) | 5 mm, 1 cm, 0.2 m/s |

    The down step lowers the hands onto the base, so it also checks that the lift stalls
    instead of pushing the robot.

### (b) The omni drive follows the controller kinematics

Each wheel sits where `mujoco_controllers.yaml` expects it, and the controller's wheel
speeds move the base at the commanded twist. [2.2 Omni Wheels](02-omni-wheels.md)
explains the wheel model.

??? info "Tests, commands and tolerances for the omni drive"
    | Test | What it checks | Tolerance |
    | --- | --- | --- |
    | `test_wheel_layout_matches_controller` | Each wheel of `wheel_names` sits at the angle and radius `mujoco_controllers.yaml` expects, with its axle pointing outward | 1e-3 rad, 0.005 m, 1e-3 on the axis |
    | `test_drive_follows_omni_kinematics[forward]`, `[sideways]`, `[rotation]`, `[combined]` | Wheel speeds from the controller's inverse kinematics; after 1.5 s to settle, the base twist averaged over 0.5 s matches the command | 3 % + 0.002 m/s linear, 4 % + 0.005 rad/s angular |

    The commands, as (vx m/s, vy m/s, wz rad/s):

    | Case | Command |
    | --- | --- |
    | forward | (0.3, 0, 0) |
    | sideways | (0, 0.3, 0) |
    | rotation | (0, 0, 0.5) |
    | combined | (0.2, -0.15, 0.4) |

!!! note
    The tolerances cover the drive shortfall from the `wheel_radius` mismatch. See
    [Known issues](#known-issues).

### (c) Lidar emulation

`test_lidar_beam_layout`, `test_lidar_sees_nothing_in_empty_scene` and
`test_lidar_ranges_to_wall` are described with their tolerances in
[2.4 Sensors](04-sensors.md).

### (d) Contract with the ros2_control block

The MJCF actuators match the ros2_control joints and interfaces, the `home` keyframe
uses the ros2_control initial positions, and each camera sits at its optical frame.

??? info "Tests and tolerances for the ros2_control contract"
    | Test | What it checks | Tolerance |
    | --- | --- | --- |
    | `test_ros2_control_contract` | See the list below | exact |
    | `test_home_keyframe_matches_initial_positions` | Every `initial_value` of a position state interface equals the `home` keyframe position of that joint | pytest default |
    | `test_cameras_at_optical_frames` | See [2.4 Sensors](04-sensors.md) | 1e-9, 1e-4 rad |

    `test_ros2_control_contract` checks that:

    - the set of MJCF actuators equals the set of ros2_control joints;
    - wheels have exactly `[velocity]` as command interface, the rest exactly
      `[position]`; state interfaces include position and velocity;
    - each actuator drives its own joint and is a velocity or position actuator, judged
      from its gain and bias;
    - mimic joints have no actuator and have a `<name>_mimic` equality;
    - IMU sensors exist;
    - `mujoco_model` ends with `walkie_description/mujoco/scene.xml`.

### (e) The checked-in model is up to date

`test_generated_model_is_up_to_date` runs the generator into a temporary directory and
compares the text with `description/mujoco/walkie.xml`. If they differ, it fails with:

```text
mujoco/walkie.xml is out of date; regenerate it with "pixi run mjcf"
```

**Why:** this is what makes it safe to check in `walkie.xml`. Anyone who changes the
xacro but forgets to regenerate gets a failing test.

### (f) Collision classes

These tests check the floor and wheel pairs, the class bitmasks, and real contacts with
props. [2.3 Collisions](03-collisions.md) explains the classes.

??? info "What the collision tests check"
    | Test | What it checks |
    | --- | --- |
    | `test_floor_and_wheel_pairs` | `floor` is a plane in the FLOOR class; each wheel contact is a capsule with `contype` = `conaffinity` = 0; each has a pair with the floor, `condim` 3, and axle friction below 0.1 times the rolling friction |
    | `test_collision_classes_by_bitmask` | BASE and BODY geoms collide with nothing in the world (FURN, OBJ, FLOOR); ARM and GRIP geoms collide with FURN, OBJ and all robot classes; every colliding robot geom is in exactly one class |
    | `test_collision_classes_in_contact` | Adds props to the scene: a FURN box inside the base is ignored; a FURN box at the left hand and an OBJ box at the right hand do touch; an OBJ box rests on the floor |

## The SRDF/URDF contract test

Commit `9a405ff` of the superproject removed 12 stale `disable_collisions` entries from
the OpenArm SRDF and added `test_srdf_urdf_contract.py`. The entries named `bottombase`,
which was removed from the description in `f5a69e3`, and `world`, which was never a link
of `gz_walkie`.

**Why a test:** MoveIt only warns about such entries and skips them, so they survived
unnoticed. The test checks every link, joint and group name in the SRDF. It expands the
installed `walkie_description` xacro, so build that package first.

??? info "What the SRDF tests check, and how they are registered"
    | Test | What it checks |
    | --- | --- |
    | `test_srdf_names_exist_in_urdf[<mode>-<right_joint2_fixed>]` | For `mock`, `gazebo`, `real_robot` and `mujoco`, each with `right_joint2_fixed` false and true: every link and joint the SRDF names exists in the expanded URDF |
    | `test_srdf_group_references_are_defined[false]`, `[true]` | Every subgroup, group state group, end effector group and end effector `parent_group` names a group defined in the SRDF |

    `test_srdf_names_exist_in_urdf` covers virtual joint child links, group chains, links
    and joints, group state joints, end effector parent links, passive joints and
    `disable_collisions` pairs. A group may list the SRDF virtual joint next to URDF
    joints, so the test accepts virtual joint names in groups.

    The link and joint sets do not depend on the mode today. Running all four modes keeps
    that assumption checked.

    The test is registered in `manipulate/openarm_bimanual_moveit_config/CMakeLists.txt`
    with
    `ament_add_pytest_test(test_srdf_urdf_contract test/test_srdf_urdf_contract.py TIMEOUT 120)`.

## Why test_xacro.py was deleted

`description/test/test_xacro.py` was copied from Husarion's `rosbot_description`. It
expanded `rosbot` and `rosbot_xl`, with `mecanum` true and false, from that package. The
package is not in this repository, so the test tested nothing about Walkie.
`test_description.py` replaces it.

## Registration and dependencies

- `description/CMakeLists.txt` registers every `test/test_*.py` file with a glob, each
  as its own ament pytest test. A new test file needs no CMake edit.
- `TIMEOUT 300` (was 60), because the new files run nine xacro expansions and several
  seconds of simulated physics.
- The `mujoco` directory was added to the `install(DIRECTORY ...)` list, so
  `share/walkie_description/mujoco/scene.xml` exists for the `mujoco_model` parameter.
- `description/package.xml` gained test dependencies, but none on `mujoco`.

??? info "CMake and package.xml changes in detail"
    The test files are no longer listed by name:

    ```cmake
    file(GLOB _pytest_tests CONFIGURE_DEPENDS
      RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_*.py)
    list(SORT _pytest_tests)
    ```

    - `CONFIGURE_DEPENDS` makes CMake re-run the glob on each build, so a new file is
      picked up without a manual reconfigure. This option needs CMake 3.12, hence
      `cmake_minimum_required(VERSION 3.12)` (was 3.11).
    - `WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}` is unchanged: the tests run from the
      package source directory.
    - The new test dependencies in `description/package.xml` are `ament_pep257`,
      `ament_index_python`, `python3-yaml` and `python3-numpy`. In the pixi environment,
      `mujoco` comes from `pixi.toml` instead.

### The pytest pin

Commit `a926459` pins `pytest = ">=8,<9"` in `pixi.toml`, because with pytest 9 every
ament pytest test aborted.
[Tests: Why pytest is pinned below 9](../../guides/testing.md#why-pytest-is-pinned-below-9)
explains the cause.

## Commits

| Repository | Commit | Change |
| --- | --- | --- |
| `description` submodule | `2b15ba1` | `test_description.py`, `test_mjcf.py`, CMake glob, test dependencies; `test_xacro.py` removed |
| superproject | `9a405ff` | Stale SRDF entries removed; `test_srdf_urdf_contract.py` added |
| superproject | `a926459` | pytest pinned below 9 |

## Known issues

| Issue | Effect | Workaround |
| --- | --- | --- |
| `mock` mode does not match `real_controllers.yaml` | `bringup/launch/test_zed.launch.py` cannot activate `joint_broad`. The pairing test is a strict xfail. | None yet. After the fix, delete the entry from `KNOWN_PAIRING_MISMATCHES`. |
| The controller `wheel_radius` is 0.076 m, the wheel collision radius 0.075 m | In MuJoCo the base drives about 1.5 % slower and turns about 2.4 % slower than commanded. The tests still pass. | See [2.2 Omni Wheels](02-omni-wheels.md#known-issues). |

??? info "Details of the two known issues"
    **`mock` with `real_controllers.yaml`:** `joint_broad` lists effort state interfaces
    for the wheels and velocity/effort for `head_servo_joint`, which the mock branch does
    not declare. The fix needs a decision outside the test: a controller file per mode, or
    different hardware interfaces.

    **`wheel_radius`:** `test_omni_wheel_geometry_matches_urdf` passes because of its
    2 mm tolerance. The drive shortfall stays inside the 3 % and 4 % drive tolerances of
    `test_drive_follows_omni_kinematics`.

## Troubleshooting

| Symptom | Cause | Fix |
| --- | --- | --- |
| Tests in `test_mjcf.py` are skipped | pytest runs outside the pixi environment, so `mujoco`, `numpy` or `xacro` cannot be imported | Run the tests with `pixi run python -m pytest ...` |
| `test_generated_model_is_up_to_date` fails with `mujoco/walkie.xml is out of date` | The xacro changed, but `walkie.xml` was not regenerated | Run `pixi run mjcf` and commit `walkie.xml` in the `description` submodule |
| `test_controller_config_matches_ros2_control` fails with `controller type ... is not modelled` | A controller file uses a controller type the test does not know | Teach `_claimed_interfaces` in `test_description.py` which interfaces the new type claims |
| `test_description.py` or `test_srdf_urdf_contract.py` still sees an old description | The installed share directory is stale | `pixi run build-pkg walkie_description`, then run the tests again |
