# 4. Move to RoboStack Build Set _25

**Goal:** move the pixi environment from RoboStack build set `_21` to `_25`, so
that the MuJoCo ros2_control runtime can be installed, and pin the build set so
that it changes only on purpose.

This entry records why the team moved, what changed, and what was tested. For
day-to-day use of the environment, read [Working with Pixi](../guides/pixi.md).
For the steps of a future move, read
[Changing the build set](../guides/pixi.md#changing-the-build-set).

## In short

- **Changed:** `pixi.toml` pins `ros2-distro-mutex = "0.19.*"`, which holds the
  environment on build set `_25`, and adds `mujoco_ros2_control` 0.1.2.
  `pixi.lock` was solved again on `_25`.
- **Why:** `mujoco_ros2_control` 0.1.x exists only in `_25`. The move also
  brings a Zenoh fix and a tf2 fix, and moves the environment closer to the
  apt Jazzy stack on the robot.
- **After pulling:** run `pixi install`, then
  `pixi run clean && pixi run build`
  ([After pulling this change](#after-pulling-this-change)).
- **Watch out:** OpenCV is 5.0 under pixi but 4.6 on apt
  ([Consequences](#consequences)). Do not `pip install` into the environment
  ([pip in the environment](#pip-in-the-environment)).
- **Result:** the whole workspace builds, and the description, MJCF, and SRDF
  tests pass. MuJoCo runs headless with `mujoco_ros2_control`. At the time of
  the move, no launch file started it ([Verification](#verification)).

!!! note "Update 2026-10-13: MuJoCo launch files added"
    `robot_bringup` now starts MuJoCo with `mujoco_omnibot.launch.py`, and
    MuJoCo with Nav2 with `nav_mujoco_omnibot.launch.py`. See
    [MuJoCo Simulation: Run with ROS 2](../guides/simulation-mujoco.md#run-with-ros-2)
    and [2.7 Launch Files and Nav2](02-mujoco/07-launch.md).

## After pulling this change

1. Install the new environment:

    ```bash
    pixi install
    ```

2. Rebuild the whole workspace:

    ```bash
    pixi run clean && pixi run build
    ```

    `pixi run build-pkg` is not enough. The CMake caches in `build/` point at
    the configuration folders of VTK 9.6 and OpenCV 4, and `behaviortree_cpp`
    and `realtime_tools` changed their ABI.

3. Do not `pip install` into the pixi environment (see
   [pip in the environment](#pip-in-the-environment)).

A branch that is still on the `_21` lock, such as `feat/Pixi`, conflicts on
`pixi.lock` when it is merged. `pixi.lock` is merged as a binary file, so
resolve the conflict by solving the environment again, as described in
[Resolve a pixi.lock conflict](../guides/pixi.md#lock-file-conflicts).

??? info "How long the full rebuild took"
    The trial build took 1 h 19 min on a 12-core computer with 15 GB of RAM,
    with `--parallel-workers 1` (colcon builds one package at a time) and
    `MAKEFLAGS=-j3`.

## Consequences

- **OpenCV 5 under pixi, OpenCV 4.6 on apt.** apt Jazzy and the Docker images
  use Ubuntu's OpenCV 4.6. OpenCV code in the workspace must work with both.
  For the perception nodes, see
  [Perception](../guides/perception.md#opencv-under-pixi).
- **A new build set is a deliberate change.** With the mutex pinned,
  `pixi add` and `pixi update` stay in `_25`. Moving to another build set
  means changing the pin, solving the lock again, and a full rebuild
  ([Changing the build set](../guides/pixi.md#changing-the-build-set)).

## Why move

1. **`mujoco_ros2_control` 0.1.x exists only in `_25`.** `_21` has only 0.0.3,
   which cannot be solved together with the workspace's MuJoCo pin and has no
   sensor plugins.
2. **A Zenoh fix.** `rmw_zenoh_cpp` 0.2.11 fixes a lost-wakeup bug in 0.2.10
   that can stall an executor.
3. **Closer to the robot.** `_25` is closer to apt Jazzy `main`, the stack on
   the robot and in the Docker images.
4. **`_21` gets no more packages.** This is an observed RoboStack pattern, not
   a documented policy.
5. **A tf2 fix.** `tf2_ros` 0.36.23 fixes a race in `waitForTransform`.

??? info "Details of each reason"
    - **`mujoco_ros2_control` 0.0.3.** Its `mujoco_vendor` 0.0.9 pins libmujoco
      3.10.0, which conflicts with `mujoco = ">=3.14"`, the pin in `pixi.toml`
      before the move, so the environment cannot be solved. 0.0.3 also lacks
      the `CameraPlugin` and `RangefinderLidarPlugin` sensor plugins. And the
      MjSpec serializer of MuJoCo 3.10 writes a different `walkie.xml`.
    - **The Zenoh bug.** The lost wakeup can stall an executor until another
      event arrives. A process without timers can stall indefinitely.
    - **Distance to apt.** Compared with the apt sync of October 2026, `_25`
      has no package older than apt. `_21` had 87.
    - **No more packages for `_21`.** RoboStack has stopped publishing to a
      build set once a newer one exists. Old packages have so far stayed
      downloadable.
    - **The tf2 race.** Nav2 nodes that use `tf2_ros::MessageFilter` reach
      that code.

??? info "Alternatives that were rejected"
    | Alternative | Why it was rejected |
    | --- | --- |
    | Stay on `_21` and install the `mujoco_ros2_control` 0.0.3 package | Cannot be solved together with `mujoco = ">=3.14"`. No sensor plugins, and a different `walkie.xml` (see reason 1). |
    | Stay on `_21` and build `mujoco_ros2_control` 0.1.2 from source | Works. But upstream requires the `mujoco_vendor` package, so two CMake files need a patch. That means a fork or a patch step, plus a MuJoCo pin. The Zenoh bug and the gap to apt stay. |
    | A Python MuJoCo node with `topic_based_ros2_control` | Acceptable only as a stopgap. The team would own the simulator plumbing. |
    | Run the simulator from the apt Docker image | apt has `mujoco_ros2_control` 0.1.2, with MuJoCo 3.12. The simulator would lose the pixi workflow. |

    [2.5 ros2_control](02-mujoco/05-ros2-control.md#packaging-status) records
    why a separate pixi environment for the simulator was rejected too.

## What changed

Commit `fe423e3` in this repository moved `pixi.toml` and `pixi.lock` to build
set `_25`.

| | Before | After |
| --- | --- | --- |
| Build set | `_21` (`ros2-distro-mutex` 0.16.0) | `_25` (`ros2-distro-mutex` 0.19.0, published 2026-10-08) |
| Build set pin in `pixi.toml` | None | `ros2-distro-mutex = "0.19.*"` |
| `mujoco_ros2_control` | Not installed | 0.1.2 |
| `mujoco` pin | `>=3.14` | `3.15.*` |
| `rmw_zenoh_cpp` | 0.2.10 | 0.2.11 |
| OpenCV | 4.13.0 | 5.0.0 |
| Packages in the `default` environment | 1276 | 1179 |

A build set is one rebuild of every ROS package of the distribution.
[Working with Pixi](../guides/pixi.md#the-robostack-build-set) explains how
build sets work.

??? info "The state before the move"
    - The pixi setup is not on `main` yet. It lives on `feat/Pixi`, which the
      team shares, and on branches based on it, such as `feat/mujoco`. Their
      locks were on `_21`. The robot and the Docker images use ROS 2 Jazzy
      from apt (see [Docker Images](../guides/docker.md)).
    - `pixi.toml` did not pin `ros2-distro-mutex`. An unpinned `pixi update`,
      or `pixi add ros-jazzy-mujoco-ros2-control`, solved the whole
      environment again and moved it to the newest build set.
    - The MuJoCo ros2_control runtime was not installed. See
      [2.5 ros2_control](02-mujoco/05-ros2-control.md#packaging-status).

??? info "The new and changed entries in pixi.toml"
    The first entry of `[dependencies]`:

    ```toml title="pixi.toml"
    # RoboStack build set _25. Every build set rebuilds all ROS packages, so the
    # mutex is pinned: adding or updating a package must not silently move the
    # environment to another set. Changing it requires `pixi run clean && pixi run build`.
    # See docs/migrations/04-robostack-build-25.md.
    ros2-distro-mutex = "0.19.*"
    ```

    Further down, next to the Zenoh entry:

    ```toml title="pixi.toml"
    ros-jazzy-rmw-zenoh-cpp = ">=0.2.11,<0.3"
    # Must match the libmujoco that ros-jazzy-mujoco-vendor is built against (3.15.0 in _25).
    mujoco = "3.15.*"
    ros-jazzy-mujoco-ros2-control = ">=0.1.2"
    # `ros2 control view_controller_chains` imports graphviz; RoboStack does not map python3-graphviz.
    python-graphviz = "*"

    # Available since build set _25, but still built from source (walkie_external.repos):
    # ros-jazzy-laser-proc = "*"
    ```

    | Entry | Before | After | Why |
    | --- | --- | --- | --- |
    | `ros2-distro-mutex` | not listed | `0.19.*` | Pins build set `_25`. Adding or updating a package can no longer move the environment to another build set. |
    | `ros-jazzy-rmw-zenoh-cpp` | `>=0.2.10,<0.3` | `>=0.2.11,<0.3` | Requires the lost-wakeup fix |
    | `mujoco` | `>=3.14` | `3.15.*` | Must match libmujoco 3.15.0, which `ros-jazzy-mujoco-vendor` 0.1.0 is built against |
    | `ros-jazzy-mujoco-ros2-control` | not listed | `>=0.1.2` | The MuJoCo ros2_control runtime. It brings `mujoco_ros2_control_plugins`, `mujoco_ros2_control_msgs`, and `mujoco_vendor`. |
    | `python-graphviz` | not listed | `*` | See [One break and its fix](#one-break-and-its-fix) |
    | `# ros-jazzy-laser-proc` | In the "packages were not found" list | Own comment | `laser_proc` 1.0.2 is on the channel in `_25`. The workspace still builds it from source through `walkie_external.repos`. |

??? info "Why the environment has fewer packages"
    The `default` environment went from 1276 to 1179 packages, and `pixi.lock`
    from 1317 to 1219 package entries. In `_25`, the `ros-jazzy-*` packages are
    aliases without files that depend on the matching `ros2-*` packages. The
    `_21` lock installed 256 packages twice, about 364 MB of duplicates.
    `pixi.toml` keeps the `ros-jazzy-*` names. The `docs` environment did not
    change (63 packages).

??? info "Package versions that changed"
    | Package | `_21` | `_25` | Notes |
    | --- | --- | --- | --- |
    | OpenCV (`libopencv`, `py-opencv`) | 4.13.0 | 5.0.0 | Required by the `_25` mutex. See [Consequences](#consequences). |
    | VTK | 9.6.2 | 9.7.0 | |
    | ros2_control (`controller_manager`, `hardware_interface`) | 4.47.0 | 4.49.0 | |
    | ros2_controllers (omni wheel drive, joint trajectory) | 4.42.1 | 4.43.0 | |
    | `control_msgs` | 5.9.0 | 5.10.0 | |
    | `realtime_tools` | 3.11.0 | 3.13.0 | ABI change |
    | `behaviortree_cpp` | 4.9.1 | 4.10.0 | The library's SONAME has no version. ABI change. |
    | `rmw_zenoh_cpp` | 0.2.10 | 0.2.11 | Same zenoh-c (`libzenohc` 1.9.0) |
    | `tf2_ros` | 0.36.22 | 0.36.23 | `waitForTransform` race fix |
    | `rclcpp` | 28.1.21 | 28.1.22 | |
    | `gz_ros2_control` | 1.2.19 | 1.2.20 | Adds an environment hook for `GZ_SIM_SYSTEM_PLUGIN_PATH` |
    | `foxglove_bridge` | 3.4.3 | 3.6.0 | |
    | `mujoco_ros2_control`, `mujoco_ros2_control_plugins`, `mujoco_ros2_control_msgs` | not installed | 0.1.2 | New |
    | `mujoco_vendor` | not installed | 0.1.0 | New. Built against libmujoco 3.15.0. |
    | `python-graphviz` | not installed | 0.21 | New |
    | `pygraphviz` | 2.0.2 | not installed | No longer a dependency of `ros2controlcli` |

    Unchanged: Python 3.12.14, numpy 2.5.3, PCL 1.15.1, Boost 1.90.0, Eigen
    5.0.1, OpenVDB 13.0.0, MuJoCo 3.15.0, GCC 13.4, gz-sim 8.10.0, and MoveIt
    2.12.4. `cv_bridge` stays at 4.1.0, rebuilt against OpenCV 5.

### pip in the environment

Do not `pip install` into the pixi environment. Packages installed that way
are not in `pixi.lock`. Use a separate virtual environment, as described in
[Python virtual environments next to pixi](../guides/pixi.md#python-virtual-environments-next-to-pixi).

The `_25` environment adds `pip`, `virtualenv`, `pip-compile`, `pip-sync`, and
`pyproject-build` to `.pixi/envs/default/bin`.

## One break and its fix

`ros2 control view_controller_chains` failed on `_25` with
`KeyError: 'view_controller_chains'`. `python-graphviz = "*"` in `pixi.toml`
adds the missing Python module, and the verb loads again. A build does not
catch this break. No other break was found.

??? info "Why the verb broke"
    - `ros2controlcli` 4.49.0 imports the Python module `graphviz` and
      declares `python3-graphviz` as a dependency.
    - RoboStack does not map `python3-graphviz` to a conda package. The `_25`
      package also dropped the `pygraphviz` dependency that `_21` had.

## Verification

The `_25` lock was checked twice:

- **Trial, 2026-10-12:** on a scratch copy of the workspace, before the move.
- **Repeat, 2026-10-13:** in the repository, on the `feat/robostack-25`
  branch, after `pixi run clean`.

Both runs built all 76 workspace packages. The description, MJCF, and SRDF
contract tests gave 108 passed and 1 xfailed, and `pixi run mjcf` reproduced
`walkie.xml` byte for byte. MuJoCo with `mujoco_ros2_control` ran headless,
drove the base, and published camera and lidar data.
[MuJoCo Simulation](../guides/simulation-mujoco.md#run-with-ros-2) describes
that bringup and the parameters it needs.

The MuJoCo run also showed a model issue: commanded down from 0.51 m, the lift
stalls near 0.48 m, because the fingers hit `base_link`.

??? info "Trial results, 2026-10-12"
    | Check | Result |
    | --- | --- |
    | Full workspace build | 76 of 76 packages, no failures. This includes all Nav2 packages, which build with `-Werror`. For the 43 packages that have a `_21` build log to compare with, the warning counts are the same as on `_21`. |
    | `test_description.py`, `test_mjcf.py`, `test_srdf_urdf_contract.py` (see [Tests](../guides/testing.md)) | 108 passed, 1 xfailed |
    | `pixi run mjcf` | `walkie.xml` is byte-identical |
    | Message, service, and action type hashes | 0 of 630 types in the environment and 0 of 92 types in the workspace changed |
    | Headless MuJoCo bringup with `mujoco_ros2_control` | The 9 controllers that the trial spawned (of the 14 in `mujoco_controllers.yaml`) became active. `/joint_states` for 22 joints at about 55 Hz (`update_rate` 60). Real-time factor about 1.0. A 0.2 m/s `cmd_vel` for 4 s gave controller odometry within about 1.5 % of the ground truth. |
    | MuJoCo cameras and lidars | They publish when a `mujoco_plugins` parameter file is supplied. The lidars measured about 28 Hz against 40 Hz configured. |
    | Mock ros2_control bringup | Unchanged. `joint_broad` still fails to activate because of the known mock effort interface gap (see [Robot Description](../reference/robot-description.md#known-issues)). |
    | Headless Gazebo with `gz_ros2_control` | With the plugin path that pixi sets, `joint_broad`, `omni_wheel_drive_controller` and `lift_controller` became active. The trial started Gazebo with its own script, not with `gzsim_omnibot.launch.py`, because the launch file overrides that path (see [Pre-existing issues](#pre-existing-issues-found-by-the-trial)). |
    | Zenoh | A local `rmw_zenohd` router starts and relays |

??? info "Repeat results, 2026-10-13"
    | Check | Result |
    | --- | --- |
    | Full workspace build | 76 of 76 packages |
    | `test_description.py`, `test_mjcf.py`, `test_srdf_urdf_contract.py` | 108 passed, 1 xfailed |
    | `pixi run mjcf` | `walkie.xml` is byte-identical |
    | Headless MuJoCo bringup from the installed workspace, with a `mujoco_plugins` parameter file | `joint_broad`, `omni_wheel_drive_controller`, `imu_sensor_broadcaster`, `lift_controller` and `head_servo_controller` became active |
    | Base drive | A 0.2 m/s `cmd_vel` for 4 s moved the ground-truth pose 0.85 m forward, with no lateral drift |
    | Cameras | The head camera published color and depth images, the bottom camera color images |
    | Front lidar | Published 241 beams, all -1 (no return), because the default scene has only a floor |
    | `ros2 control view_controller_chains` | Loads |

Not verified on `_25` yet:

- a Nav2 simulation run, with `bt_navigator` on `behaviortree_cpp` 4.10;
- the team Zenoh router across machines;
- MoveIt at runtime;
- the real hardware plugins (only compiled);
- the RViz and Foxglove user interfaces.

!!! note "Update 2026-10-13: Nav2 in MuJoCo"
    A headless run of `nav_mujoco_omnibot.launch.py` on `_25` brought up Nav2.
    `bt_navigator`, on `behaviortree_cpp` 4.10, reported "Goal succeeded" for a
    goal on the blank map. The run needed two workarounds for the Nav2 fork in
    `navigation/`, whose `navigation2` was bumped to upstream `main` on
    2026-10-08; they do not come from the build set. Until the MPPI parameter
    files in the navigation repository are fixed, the Gazebo and real-robot
    Nav2 launches fail to configure `controller_server`. See
    [2.7 Launch Files and Nav2](02-mujoco/07-launch.md).

    **Update 2026-10-14:** every MPPI parameter file now sets the Omni plugin
    key, and the Nav2 launches that include `navigation_launch.py` or
    `localization_launch.py` directly start their own `lifecycle_manager_nav2`.
    See
    [5. Repair Nav2 After the navigation2 Bump](05-nav2-repair.md#mppi-motion-models-are-plugins).
    Under Fast DDS, the real-robot launches still fail on an older, separate
    problem ([entry 5: Still open](05-nav2-repair.md#still-open)).

## Pre-existing issues found by the trial

The trial found three problems that do not come from the move. This change
documents them; it does not fix them.

| Issue | Where it is listed |
| --- | --- |
| `gzsim_omnibot.launch.py` and `world_preview_gz.launch.py` set `GZ_SIM_SYSTEM_PLUGIN_PATH` to `/opt/ros/jazzy/lib/`. On a computer with `/opt/ros/jazzy`, Gazebo crashes. | [Gazebo Simulation](../guides/simulation-gazebo.md#known-issues), [Troubleshooting](../troubleshooting.md#gazebo-problems-only-on-computers-with-a-system-ros) |
| `robot_state_publisher.launch.py` in `description/` defaults `dual_lidar` to `true`, which the xacro cannot evaluate with `ros2_control:=gazebo` | [Gazebo Simulation](../guides/simulation-gazebo.md#known-issues) |
| A `nav2_behavior_tree` test fixture never sets `cancel_timeout`, so the test can hang, on `_21` and on `_25` | [Navigation](../guides/navigation.md#known-issues) |

The fix for the first issue is to remove the override. New in `_25`:
`gz_ros2_control` ships an environment hook, so `pixi run` already sets the
variable to the `lib` folder of the pixi environment.

## Still open

!!! note "Update 2026-10-13: launch file done"
    This list began with "A launch file that starts MuJoCo with
    `mujoco_ros2_control`". That item is done: `mujoco_omnibot.launch.py` in
    `robot_bringup`. See
    [MuJoCo Simulation: Run with ROS 2](../guides/simulation-mujoco.md#run-with-ros-2)
    and [2.7 Launch Files and Nav2](02-mujoco/07-launch.md).

- The checks listed as not verified under [Verification](#verification).
- The fixes for the three pre-existing issues above.
- `laser_proc` could come from the channel now. It is still built from source.
