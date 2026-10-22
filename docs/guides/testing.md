# Tests

Run the workspace tests, see what each suite checks, and fix the common
failures. Most tests check the robot model: the URDF, the controller files, the
MoveIt SRDF, and the MuJoCo model. They run in seconds and need no robot and no
simulator window.

## Run the tests

After a change to the robot model, build, test, and read the summary:

```bash
pixi run build-pkg walkie_description openarm_bimanual_moveit_config
pixi run test --packages-select walkie_description openarm_bimanual_moveit_config
pixi run colcon test-result --verbose
```

| Command | What it does |
| --- | --- |
| `pixi run test` | `colcon test` for every built package, including the Nav2 packages |
| `pixi run test --packages-select <package> ...` | Only the listed packages. Much faster. |
| `pixi run colcon test-result --verbose` | Summary of the last run, with the failures |
| `pixi run python -m pytest description/test/test_mjcf.py` | One pytest file, without colcon. See [Run one test file with pytest](#running-pytest-directly). |

`pixi run test` passes extra arguments to `colcon test`, so every colcon test
option works after it.

!!! warning
    A plain `pixi run test` also runs the slow Nav2 tests and `urg_node2_test`,
    which needs a real Hokuyo lidar. Use `--packages-select`, or add
    `--packages-skip urg_node2`. See [Known issues](#known-issues).

!!! note "Some tests need a Zenoh router"
    Pixi sets `RMW_IMPLEMENTATION=rmw_zenoh_cpp`, so nodes in separate
    processes find each other only through a Zenoh router. A test that starts
    nodes in more than one process needs one. Two examples, checked on
    2026-10-19: the launch test of `topic_based_ros2_control`, and the launch
    test `test_collision_checker` of `nav2_costmap_2d`. Without a router, each
    one waits until CTest stops it at its timeout (120 s and 60 s). Other
    `nav2_costmap_2d` integration tests and the tests in `nav2_system_tests`
    also start several nodes from a launch file. Start the router in another
    terminal before `pixi run test`:

    ```bash
    pixi run ros2 run rmw_zenoh_cpp rmw_zenohd
    ```

    The `walkie_description`, `openarm_bimanual_moveit_config` and
    `robot_bringup` tests start no nodes and need no router.

!!! note "CI runs the tests of four packages"
    On every pull request and every push to `main`, CI builds the workspace
    and runs the tests of `walkie_description`,
    `openarm_bimanual_moveit_config`, `topic_based_ros2_control`, and
    `robot_bringup`. Lint tests run but do not fail the job. See [CI](#ci).

??? info "Where the result files are"
    - `build/<package>/test_results/<package>/*.xunit.xml`: JUnit XML result
      per test. `colcon test-result` reads these.
    - `build/<package>/ament_cmake_pytest/<test>.txt`: full pytest output of
      each ament pytest test.

## Build before you test

`colcon test` runs the tests that CMake registered during the build. A
package that is not built has no registered tests. Build it first with
`pixi run build` or `pixi run build-pkg <package>`.

Some tests also read files from `install/`. Every `pixi run` sources
`install/setup.bash` at start, so it sees the new build. A `pixi shell` opened
before the build does not. Run `source install/setup.bash` in it, or open a new
shell.

| Test file | Needs a build? |
| --- | --- |
| `description/test/test_description.py` | yes |
| `manipulate/openarm_bimanual_moveit_config/test/test_srdf_urdf_contract.py` | yes (`walkie_description`) |
| `description/test/test_mjcf.py` | no |
| `description/test/test_pep257.py` | only to register it with colcon |
| `bringup/test/test_mujoco_world.py`, `test_mujoco_launch_worlds.py` | yes (`robot_bringup`) |
| `bringup/test/test_mujoco_world_assets.py` | yes (`robot_bringup`, `walkie_description`, `walkie_mujoco_worlds`) |

??? info "Why each file needs a build or not"
    - `test_description.py` expands `gz_walkie.urdf.xacro` from the installed
      `walkie_description` share directory.
    - `test_srdf_urdf_contract.py` expands the installed `gz_walkie.urdf.xacro`.
      The SRDF is read from the source tree.
    - `test_mjcf.py` finds every file relative to itself and loads
      `scripts/build_mjcf.py` from the source tree.
    - `test_pep257.py` checks the folder it runs in. See
      [Run one test file with pytest](#running-pytest-directly).
    - The `robot_bringup` tests import the installed Python package
      `robot_bringup`. `test_mujoco_world_assets.py` also finds `walkie.xml`
      and every installed world through the package index, and is skipped when
      `mujoco` cannot be imported.

## Run one test file with pytest { #running-pytest-directly }

```bash
pixi run python -m pytest description/test/test_mjcf.py
pixi run python -m pytest description/test/test_description.py -k omni -v
pixi run python -m pytest manipulate/openarm_bimanual_moveit_config/test/test_srdf_urdf_contract.py
pixi run python -m pytest bringup/test/test_mujoco_world.py bringup/test/test_mujoco_launch_worlds.py bringup/test/test_mujoco_world_assets.py
```

This is faster than colcon while you work on one test. pytest options work:
`-k` selects tests by name, `-x` stops at the first failure, and `-v` prints
one line per test.

`test_mjcf.py` works right after a clone. `test_description.py` and
`test_srdf_urdf_contract.py` need a built `walkie_description` (see
[Build before you test](#build-before-you-test)).

!!! warning "Run lint tests from the package folder"
    `test_pep257.py` checks `.`, the folder you run it from. Run from the
    repository root, it checks the Python files of the whole repository, not
    only `walkie_description`. Run it through colcon, or change to
    `description/` first.

## Add tests

| You want to | Do this |
| --- | --- |
| Add a test file to `walkie_description` | Create `description/test/test_<name>.py`, then run `pixi run build-pkg walkie_description` to register it |
| Add a controller file to `config/ros2_controller/` | Add a `(mode, file)` entry to `CONTROLLER_PAIRINGS` in `test_description.py`. Otherwise `test_every_controller_config_is_paired` fails. |
| Use a new controller type in a controller file | Teach `_claimed_interfaces` in `test_description.py` which interfaces it claims. Otherwise the test reports "controller type ... is not modelled". |
| Record a pairing that cannot be fixed now | Add it to `KNOWN_PAIRING_MISMATCHES` with a reason |
| Change the xacro files | Run `pixi run mjcf` and commit `mujoco/walkie.xml` too |
| Add a test file to `robot_bringup` | Create `bringup/test/test_<name>.py` and add an `ament_add_pytest_test` line for it in `bringup/CMakeLists.txt`. `robot_bringup` lists its tests by name. |
| Add a named MuJoCo world | Nothing to add. After `pixi run build-pkg walkie_mujoco_worlds`, `test_mujoco_world_assets` checks every variant of the new world. |

??? info "How walkie_description registers its tests"
    `description/CMakeLists.txt` does not list the test files by name. It
    registers every `test/test_*.py` file with a glob:

    ```cmake
    file(GLOB _pytest_tests CONFIGURE_DEPENDS
      RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_*.py)
    ```

    Each file becomes one test with `ament_add_pytest_test`, named after the
    file (`test_description`, `test_mjcf`, `test_pep257`). Each one has:

    - `TIMEOUT 300`: CTest stops the test after 300 seconds.
    - `WORKING_DIRECTORY` set to the `description/` source folder.
    - `PYTHONPATH` extended with the package build folder.

    `CONFIGURE_DEPENDS` tells CMake to check the glob again on every build. A
    new `test/test_*.py` file is registered by the next
    `pixi run build-pkg walkie_description`. You do not need to edit
    `CMakeLists.txt`, but you do need that build.

## CI

`.github/workflows/ci.yml` builds the workspace and runs the tests on every
pull request, whatever its base branch, and on every push to `main`. You can
also start it by hand in the Actions tab of the repository.

| Step | What it does |
| --- | --- |
| Checkout | All submodules, recursively, then the Git LFS files inside them (the meshes in `description/`) |
| pixi | Installs the `default` environment from `pixi.lock` with `--locked`. The job fails if `pixi.toml` and `pixi.lock` disagree. |
| Build | `colcon build` of the whole workspace, Nav2 included |
| Start the Zenoh router | Starts `rmw_zenohd` with the default config (port 7447) in the background and waits until the port accepts connections. The router runs until the job ends. |
| Test | The tests of `walkie_description`, `openarm_bimanual_moveit_config`, `topic_based_ros2_control`, and `robot_bringup`, without their lint tests. `topic_based_system_test` runs on Fast DDS, the others on Zenoh (see [Known issues](#known-issues)). A failure fails the job. |
| Lint (not enforced) | The lint tests (CTest label `linter`) of `robot_bringup`, `robot_simulation`, `robot_navigation`, `dual_laser_merger`, and `joy_interface`. A failure is reported, but the job still passes. |

Run the enforced tests the way CI does. Start a Zenoh router in another
terminal first (`pixi run ros2 run rmw_zenoh_cpp rmw_zenohd`), because the
launch test of `topic_based_ros2_control` needs one:

```bash
pixi run build
pixi run test --packages-select walkie_description openarm_bimanual_moveit_config topic_based_ros2_control robot_bringup --ctest-args " -LE" linter " -E" "^topic_based_system_test$"
pixi run env RMW_IMPLEMENTATION=rmw_fastrtps_cpp colcon test --packages-select topic_based_ros2_control --ctest-args " -R" "^topic_based_system_test$"
pixi run colcon test-result --verbose
```

`-LE linter` tells CTest to skip the tests with the label `linter`, which
every ament lint test has. `-E` skips the tests whose name matches, and `-R`
runs only those. So the second command runs `topic_based_system_test` alone,
on Fast DDS (see [Known issues](#known-issues)). colcon passes an argument
that starts with `-` on to CTest only with a space in front of it, so keep
the quotes.

To see why a run failed, open the failed step in the job log. The Test step
prints `colcon test-result --verbose`. The run summary shows the test counts
of both test steps. The artifact `colcon-logs`, kept for 14 days, holds the
colcon logs (`log/`), the router log (`log/rmw_zenohd.log`), and the result
files (`build/*/test_results/`).

??? info "Why CI starts a Zenoh router"
    Pixi sets `RMW_IMPLEMENTATION=rmw_zenoh_cpp` for every step that runs
    with `pixi run`. The launch test `test/ros2_control.test.py` of
    `topic_based_ros2_control` starts `robot_state_publisher`,
    `ros2_control_node`, two spawners, and a test node, each in its own
    process. Without a router they cannot find each other:
    `ros2_control_node` waits for `robot_description` until CTest stops the
    test after 120 s.

    CI starts one router instead of switching the tests to DDS, so the tests
    run on the same middleware as the robot and as a local `pixi run test`.
    The one exception is `topic_based_system_test`, which aborts at exit
    under Zenoh (see [Known issues](#known-issues)). It runs in one process,
    so Fast DDS changes nothing that it checks. A step-level `env:` could not
    switch the middleware, because pixi replaces the value from the calling
    shell. The Test step sets it inside `pixi run` instead.

    GitHub Actions keeps a background process from one step running in the
    later steps, and stops it when the job ends.

??? info "Tests that CI does not run"
    - The upstream Nav2 tests in `navigation/navigation2/`: slow, and not
      Walkie code. CI builds Nav2 without them.
    - `urg_node2_test`: needs a real Hokuyo lidar (see
      [Known issues](#known-issues)).
    - The lint tests of third-party packages: `lakibeam1`, the Dynamixel
      packages, `spatio_temporal_voxel_layer`, and `topic_based_ros2_control`.
    - Launch files: no step starts a simulation or the robot.

??? info "Why lint is not enforced"
    The lint tests fail in most first-party packages. On 2026-10-15,
    `ament_flake8` reported 1783 errors in `bringup/`, 127 in
    `simulation/gazebo/`, and 71 in `tools/joy_interface/`. An enforced lint
    step would keep every run red. When the lint tests of a package pass,
    they can be made enforced: add a step to `ci.yml` that runs them without
    `continue-on-error`.

??? info "Why the build runs in two passes"
    The first pass builds every package with `-DBUILD_TESTING=OFF`. This
    skips the test targets of Nav2, which are a large part of its build. The
    second pass builds only the packages in `TEST_PACKAGES` and
    `LINT_PACKAGES` again, with `-DBUILD_TESTING=ON`, which registers their
    tests. colcon configures a package again when its CMake arguments change,
    so the second pass only configures these packages and compiles their
    test targets.

??? info "Caches"
    | Cache | Contents | Written by |
    | --- | --- | --- |
    | ccache | Compiled objects, up to 2 GB | Every run, right after the build, also when a test fails |
    | pixi | The `default` environment, keyed on `pixi.lock` | Pushes to `main` only |
    | LFS | The Git LFS objects of the submodules, keyed on their object IDs | Runs that download new objects |

    On 2026-10-15, a run without a ccache took 37 min, 32 of them in the
    build. Run again with the ccache of that run, it took 8 min, 5.5 of them
    in the build, with 90 % of the compilations taken from the cache. The
    Build step ends with `ccache --show-stats`, which shows the hit rate of
    the run.

    A pull request restores the newest cache of its own branch, or else the
    newest cache of `main`. GitHub keeps up to 10 GB of caches per repository
    and removes the least recently used ones first. The pixi environment is
    written only from `main`, so pull requests do not fill that space with
    copies of it. With the LFS cache, a run downloads only the LFS objects
    that changed since the cached ones.

## What each suite checks

The four suites below are in `walkie_description` and
`openarm_bimanual_moveit_config`.

??? info "Tests in the other packages"
    | Package | Tests | Registered by |
    | --- | --- | --- |
    | `openarm_bimanual_moveit_config` | `test_srdf_urdf_contract` | `ament_add_pytest_test(... TIMEOUT 120)` in its `CMakeLists.txt` |
    | `robot_bringup` | `test_mujoco_world`, `test_mujoco_launch_worlds`, and `test_mujoco_world_assets` (`TIMEOUT 180`): the MuJoCo world loader, the world arguments of the launch files, and every installed named world. See [MuJoCo Simulation: Tests](simulation-mujoco.md#tests). Also lint tests. | `ament_add_pytest_test` in its `CMakeLists.txt`, and `ament_lint_auto_find_test_dependencies()` |
    | `robot_simulation`, `robot_navigation`, `lakibeam1`, `dual_laser_merger`, `dynamixel_hardware_interface`, `dynamixel_interfaces`, `dynamixel_sdk_examples`, `spatio_temporal_voxel_layer` | Lint tests (some linters turned off per package) | `ament_lint_auto_find_test_dependencies()` |
    | `topic_based_ros2_control` | Lint tests, the gtest `topic_based_system_test`, and the launch test `test/ros2_control.test.py` (`TIMEOUT 120`) | Its `CMakeLists.txt` (submodule) |
    | `urg_node2` | gtest `urg_node2_test` (`TIMEOUT 200`) | Its `CMakeLists.txt`. Needs a real lidar. |
    | `joy_interface` | `test_flake8.py`, `test_pep257.py`, and `test_copyright.py` (always skipped) | ament_python: colcon runs pytest on `test/` |
    | Nav2 packages in `navigation/navigation2/` | Upstream Nav2 tests, including `nav2_system_tests` | Upstream |

    `walkie_perception`, `my_robot_interfaces`, `walkie_tf_interfaces`,
    `openarm_bimanual_commander_cpp`, `cubemars_hardware`, `openarm_hardware`,
    and `unitree_lidar_ros2` list `ament_lint_auto` in `package.xml`, but their
    `CMakeLists.txt` registers no tests. The other packages, including
    `walkie_tf`, have no tests.

### test_description.py

File: `description/test/test_description.py`. It expands the URDF for each
configuration in `CONFIGURATIONS` and checks the result and the controller
files. Expansions are cached, so each configuration is expanded once per run.
The pairings of modes and controller files are listed in
[Robot Description](../reference/robot-description.md#controller-files).

??? info "Configurations (`CONFIGURATIONS`) and their xacro arguments"
    | Name | xacro arguments |
    | --- | --- |
    | `mock` | `ros2_control:=mock` |
    | `gazebo` | `ros2_control:=gazebo` |
    | `topic_base` | `ros2_control:=topic_base` |
    | `real_robot` | `ros2_control:=real_robot` |
    | `mujoco` | `ros2_control:=mujoco` |
    | `gazebo_single_lidar` | `ros2_control:=gazebo dual_lidar:=False` |
    | `gazebo_no_arm` | `ros2_control:=gazebo use_arm:=False` |
    | `real_robot_arm_hardware` | `ros2_control:=real_robot use_fake_arm_hardware:=False` |
    | `gazebo_right_joint2_fixed` | `ros2_control:=gazebo right_joint2_fixed:=True` |

??? info "All tests in test_description.py"
    | Test | Runs for | Checks |
    | --- | --- | --- |
    | `test_xacro_expands` | each configuration | xacro runs without errors and the result has links |
    | `test_link_and_joint_names_are_unique` | each configuration | No two links and no two joints share a name |
    | `test_kinematic_tree_is_connected` | each configuration | Joints connect existing links into one tree with one root |
    | `test_mesh_files_exist` | each configuration | Every `package://` or `file://` mesh path points to an installed file |
    | `test_no_unresolved_xacro_expressions` | each configuration | No `${...}` or `$(...)` is left after expansion |
    | `test_gazebo_references_exist` | each configuration | Every `<gazebo reference>` names an existing link or joint |
    | `test_ros2_control_joints_are_movable_urdf_joints` | each configuration | Every ros2_control joint is a non-fixed URDF joint, owned by exactly one ros2_control block |
    | `test_every_controller_config_is_paired` | once | Every `.yaml` file in `config/ros2_controller/` appears in `CONTROLLER_PAIRINGS`, and every file named there exists |
    | `test_controller_config_matches_ros2_control` | each pairing in `CONTROLLER_PAIRINGS` | Each controller only claims joints, command and state interfaces, sensors, and frames that the paired mode declares |
    | `test_omni_wheel_geometry_matches_urdf` | each controller file | `robot_radius` within 0.001 m, wheel angles within 1 degree, and `wheel_radius` within 0.002 m of the URDF |
    | `test_mujoco_ros2_control_interfaces` | `mujoco` | Wheels have only a velocity command, other joints only a position command, mimic joints are not declared |
    | `test_mujoco_hardware_plugin` | `mujoco` | Exactly one hardware plugin, `mujoco_ros2_control/MujocoSystemInterface` |
    | `test_mujoco_model_parameter_points_to_installed_scene` | `mujoco` | The `mujoco_model` parameter is the installed `mujoco/scene.xml`, and that file exists |

### test_mjcf.py

File: `description/test/test_mjcf.py`. It checks the generated MuJoCo model
`mujoco/walkie.xml` and the default world `mujoco/scene.xml`. Most tests run a
short physics simulation.

The file uses `pytest.importorskip('mujoco')`. Outside the pixi environment,
where `mujoco` is not installed, the whole file is skipped instead of failing.

A `mujoco` without the `mujoco_3d_lidar` extension (`ros-jazzy-mujoco-3d-lidar`),
for example one from `pip`, is not skipped: every test that loads `walkie.xml`
fails with `plugin mujoco.plugin.lidar not found`. Run `pixi install` and use
the pixi environment.

??? info "All tests in test_mjcf.py"
    | Test | Checks |
    | --- | --- |
    | `test_scene_compiles_with_plausible_mass` | The scene compiles. Robot mass is between 25 and 60 kg. `floating_base_joint` is a free joint on `base_footprint`. Timestep 0.002 s, `implicitfast` integrator, elliptic friction cones. |
    | `test_model_loads_without_scene` | `walkie.xml` loads alone and has no `floor` geom |
    | `test_rest_is_stable` | Standing still for 2 s: height changes less than 3 mm, drift is less than 1 cm, no MuJoCo warnings |
    | `test_imu_at_rest` | The IMU reads gravity (9.81 m/s² up), zero rotation, and the site orientation |
    | `test_lift_step_keeps_base_on_floor[up/down]` | A 0.2 m lift step does not lift or push the base, and the lift moves slower than 0.2 m/s |
    | `test_wheel_layout_matches_controller` | Wheel positions and axes match `wheel_names`, `wheel_offset`, and `robot_radius` in `mujoco_controllers.yaml` |
    | `test_drive_follows_omni_kinematics[forward/sideways/rotation/combined]` | Wheel speeds from the omni kinematics move the base at the commanded velocity, within 3% (linear) and 4% (angular) |
    | `test_lidar_beam_layout[front_lidar/back_lidar]` | Beam count, angles, and update rate match the URDF and the `walkie.xml` header |
    | `test_lidar_sees_nothing_in_empty_scene[front_lidar/back_lidar]` | No beam hits the robot or the floor |
    | `test_lidar_ranges_to_wall` | Readings match the distance to a wall 1 m in front of the robot |
    | `test_lidar_3d_layout` | The head 3D lidar `unitree_l2` is one extension sensor on the site `unitree4d_l2_imu_initial` with 180 x 32 readings. Its plugin config and the `walkie.xml` header match the generator, and its rays cover the hemisphere z ≥ 0 of the site. |
    | `test_lidar_3d_sees_floor` | Empty scene: every hit is within range and no farther than the floor along its ray. More than 60 % of the rays end on the floor. Every hit on the robot lies inside the self-filter box of `unitree_self_filter.yaml`. |
    | `test_lidar_3d_ranges_to_box` | Readings match the distance to a box 1.5 m to the left of the robot |
    | `test_ros2_control_contract` | MuJoCo actuators and the ros2_control joints match one to one. Mimic joints become equality constraints. The IMU sensors exist. |
    | `test_home_keyframe_matches_initial_positions` | The `home` keyframe uses the `initial_value` positions from the ros2_control block |
    | `test_cameras_at_optical_frames` | Each MuJoCo camera sits at its ROS optical frame and has the same horizontal field of view as the Gazebo sensor |
    | `test_generated_model_is_up_to_date` | Running `build_mjcf.py` again gives exactly the checked-in `walkie.xml` |
    | `test_floor_and_wheel_pairs` | The floor is a plane in the FLOOR class. Each wheel capsule touches the floor only through its `<pair>`, with low axle friction. |
    | `test_collision_classes_by_bitmask` | Base and body geoms ignore furniture, objects, and the floor. Arm and gripper geoms collide with furniture, objects, and the robot. |
    | `test_collision_classes_in_contact` | Adds boxes to the scene and steps it. Furniture inside the base makes no contact. Furniture and objects at the hands make contact. A free object rests on the floor. |

??? info "Accepted speed error in the drive test"
    The drive test has a known, accepted error. The controller uses
    `wheel_radius` 0.076 m, but the wheel collision radius is 0.075 m. The
    comment in the test says this gives about 1.5% (linear) and 2.4% (angular)
    less speed than commanded. The tolerances are set to catch larger errors.

### test_srdf_urdf_contract.py

File: `manipulate/openarm_bimanual_moveit_config/test/test_srdf_urdf_contract.py`.
MoveIt only prints a warning when the SRDF names a link or joint that the URDF
does not have, so such mistakes often go unnoticed. This test catches them.

The SRDF `config/openarm_bimanual.srdf` uses xacro conditions, so the test
expands it with xacro first.

??? info "All tests in test_srdf_urdf_contract.py"
    | Test | Runs for | Checks |
    | --- | --- | --- |
    | `test_srdf_names_exist_in_urdf` | `ros2_control` in `mock`, `gazebo`, `real_robot`, `mujoco`, times `right_joint2_fixed` in `false`, `true` | Every link and joint named in the SRDF exists in the URDF: virtual joint child, chains, group links and joints, group states, end effector parent links, passive joints, and disabled collision pairs |
    | `test_srdf_group_references_are_defined` | `right_joint2_fixed` in `false`, `true` | Every group named by a subgroup, group state, or end effector is defined |

### test_pep257.py

File: `description/test/test_pep257.py`. It runs `ament_pep257` on `.` and
`test`, which checks the docstring style of the Python files. Under colcon the
working folder is `description/`, so it checks the `walkie_description`
package.

## Strict expected failure

One pairing in `test_controller_config_matches_ros2_control` is known to be
wrong: `mock` with `real_controllers.yaml`. The test marks it with
`pytest.mark.xfail(strict=True)`:

- While the mismatch exists, the test fails as expected. pytest reports it as
  `xfailed`, and the run passes.
- When someone fixes the mismatch, the test passes. Because the mark is
  strict, pytest reports `XPASS(strict)` and the run **fails**. Remove the
  entry from `KNOWN_PAIRING_MISMATCHES` in `test_description.py` in the same
  change.

The details of the mismatch are in
[Robot Description](../reference/robot-description.md#known-issues).

??? info "Why the mark is strict"
    The reason for the mismatch is stored in `KNOWN_PAIRING_MISMATCHES` in
    `test_description.py`. The comment above that dictionary says each fix
    needs a decision outside the test (a controller file per mode, or
    different hardware interfaces).

    The strict mark keeps the list of known problems honest: an entry cannot
    stay after its problem is gone.

??? info "Results when this page was written (2026-10-12)"
    | Suite | Result |
    | --- | --- |
    | `test_description` | 75 passed, 1 xfailed |
    | `test_mjcf` | 23 passed |
    | `test_pep257` | 1 passed |
    | `test_srdf_urdf_contract` | 10 passed |

    After the move to RoboStack build set `_25`, the three pytest files
    (`test_description.py`, `test_mjcf.py`, `test_srdf_urdf_contract.py`)
    gave the same total: 108 passed, 1 xfailed.

## Why pytest is pinned below 9

`pixi.toml` has `pytest = ">=8,<9"`. pytest 9 rejects the pytest plugins of
Jazzy `launch_testing` and `launch_testing_ros`, and every ament pytest test
aborts. Keep the pin until the RoboStack Jazzy packages work with pytest 9.

??? info "How pytest 9 breaks the Jazzy plugins"
    The comment above the pin in `pixi.toml` says: "pytest 9 rejects the
    pytest11 plugins of Jazzy launch_testing / launch_testing_ros."

    `pytest11` is the entry point group that Python packages use to register
    pytest plugins. pytest loads every registered plugin when it starts.
    Jazzy's `launch_testing` and `launch_testing_ros` register such plugins.
    The commit that added the pin says that with pytest 9 every ament pytest
    test aborted.

## Known issues

| Issue | Effect | Workaround |
| --- | --- | --- |
| `urg_node2_test` connects to a Hokuyo lidar at `192.168.0.10`. Per the `urg_node2` README, it expects a connected, powered-on UTM-30LX-EW. | Without that lidar the test does not pass. CTest stops it after 200 seconds at most. | `pixi run test --packages-skip urg_node2`, or use `--packages-select` for the packages you changed |
| `mock` with `real_controllers.yaml` (`test_zed.launch.py`) does not work | `test_controller_config_matches_ros2_control[mock-real_controllers]` is a strict expected failure | None yet. See [Strict expected failure](#strict-expected-failure). |
| The fixture of `test_action_follow_path_action` in `nav2_behavior_tree` never sets `cancel_timeout`. See [Navigation: Known issues](navigation.md#known-issues). | The test can hang until CTest stops it | `pixi run test --packages-skip nav2_behavior_tree`, or skip only this test with `--packages-select nav2_behavior_tree --ctest-args -E test_action_follow_path_action` |
| The `main()` of the gtest `topic_based_system_test` (in the `topic_based_ros2_control` submodule) calls `rclcpp::init()` but never `rclcpp::shutdown()`. ROS then shuts down only at process exit, which `rmw_zenoh_cpp` does not support. | Under Zenoh, the test passes, then the process aborts with `The Thread Local Storage inside Tokio is destroyed`. CTest reports a failure. Checked on 2026-10-19 with `rmw_zenoh_cpp` 0.2.11: 6 of 6 runs aborted, and with Fast DDS 3 of 3 passed. | Run this test on Fast DDS, as CI does: `pixi run env RMW_IMPLEMENTATION=rmw_fastrtps_cpp colcon test --packages-select topic_based_ros2_control --ctest-args " -R" "^topic_based_system_test$"`. A `rclcpp::shutdown()` call at the end of `main()` fixes it under Zenoh, but the file is in the upstream submodule. |

## Troubleshooting

| Symptom | Cause | Fix |
| --- | --- | --- |
| `test_description.py` or `test_srdf_urdf_contract.py` fails with `PackageNotFoundError` for `walkie_description` | Package not built, or the shell was opened before the build | `pixi run build-pkg walkie_description`, then run the test with `pixi run`, or `source install/setup.bash` in the old shell |
| `colcon test-result` shows no results for a package | The package was not built, or its tests were never run | Build it, then `pixi run test --packages-select <package>` |
| `test_generated_model_is_up_to_date` fails with `mujoco/walkie.xml is out of date; regenerate it with "pixi run mjcf"` | The xacro changed, but `mujoco/walkie.xml` was not regenerated | `pixi run mjcf`, then commit `walkie.xml` |
| `test_controller_config_matches_ros2_control[mock-real_controllers]` fails with `XPASS(strict)` | The known mismatch was fixed | Remove the entry from `KNOWN_PAIRING_MISMATCHES` |
| `test_every_controller_config_is_paired` fails with "add a CONTROLLER_PAIRINGS entry" | A new file in `config/ros2_controller/` | Add the pairing to `CONTROLLER_PAIRINGS` |
| `test_controller_config_matches_ros2_control` reports "is not modelled" | A controller type the test does not know | Add the type to `_claimed_interfaces` |
| `test_mesh_files_exist` fails | `walkie_description` not built, or a wrong mesh path in the xacro | `pixi run build-pkg walkie_description`, or fix the path named in the failure |
| Tests pass, but RViz or Gazebo cannot load the arm, hand, or wheel meshes | The `.dae` files are Git LFS pointers. `test_mesh_files_exist` only checks that each file exists, so pointer files pass. | `git submodule foreach --recursive git lfs pull`, then rebuild |
| Every ament pytest test aborts at start-up | pytest 9 is installed | Keep `pytest = ">=8,<9"` in `pixi.toml` and run `pixi install` |
| `test_mjcf.py` is reported as skipped | `mujoco` cannot be imported, for example outside pixi | Run it with `pixi run python -m pytest ...` |
| `urg_node2_test` fails or times out | No Hokuyo UTM-30LX-EW at `192.168.0.10` | `pixi run test --packages-skip urg_node2` |
| `pixi run test` takes a long time | It also runs the Nav2 tests | Use `--packages-select` |
| A launch test, such as the one of `topic_based_ros2_control`, fails after its timeout. Its log shows `Unable to connect to a Zenoh router`. | No Zenoh router runs, so the test nodes cannot find each other | Start `pixi run ros2 run rmw_zenoh_cpp rmw_zenohd` in another terminal and run the test again |
| `topic_based_system_test` fails with `return code -6` after `[  PASSED  ] 1 test.` | It ran on Zenoh, and its process aborts at exit | Run it on Fast DDS. See [Known issues](#known-issues). |
| CI fails in the `setup-pixi` step with `lock-file not up-to-date with the workspace` | `pixi.toml` changed, but `pixi.lock` was not updated and committed | Run `pixi install`, then commit `pixi.lock` together with `pixi.toml` |
