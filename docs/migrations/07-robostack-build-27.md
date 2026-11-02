# 7. Move to RoboStack Build Set _27

**Goal:** move the pixi environment from RoboStack build set `_25` to `_27`, so
that RTAB-Map can come from RoboStack in the main environment.

This entry records why the team moved, what changed, and what was tested. For
day-to-day use of the environment, read [Working with Pixi](../guides/pixi.md).
For the steps of a future move, read
[Changing the build set](../guides/pixi.md#changing-the-build-set).

!!! warning "No real hardware was tested"
    Every check on this page ran on a development computer without a robot. No
    motor moved, and no CAN bus, servo, camera, or lidar was connected. The
    hardware plugins were only compiled and loaded. Test the robot before you
    rely on this move ([Still open](#still-open)).

## In short

- **Changed:** `pixi.toml` pins `ros2-distro-mutex = "0.20.*"`, which holds the
  environment on build set `_27`, and `mujoco = "3.16.*"`. `pixi.lock` was
  solved again. `walkie.xml` was regenerated with MuJoCo 3.16. The lift venv,
  its launch file, `perception/setup_env.sh`, `zed_watchdog.py`, and
  `head_touch_beep_node.py` no longer name a Python version.
- **Why:** the `rtabmap_ros` packages exist on RoboStack only in `_27`. This
  change does not add them.
- **After pulling:** update the submodules, run `pixi install`, then
  `pixi run clean && pixi run build`, and recreate `bringup/.venv`
  ([After pulling this change](#after-pulling-this-change)).
- **Watch out:** the environment is now Python 3.14. The perception venv cannot
  be created under pixi ([Consequences](#consequences)).
- **Before you merge:** push and merge the `description` submodule first
  ([Commit order](#commit-order)).
- **Result:** all 74 workspace packages build, in 1 h 9 min. The tests give the
  totals of `main` plus the new `robot_bringup` tests. MuJoCo and Nav2 on MuJoCo ran
  headless. Gazebo and MoveIt on mock hardware ran headless from copies of their
  launch files ([Verification](#verification)).

## After pulling this change

1. Update the submodules. `description` moves with this change, because
   `walkie.xml` must match the MuJoCo version in the lock:

    ```bash
    git submodule update --init --recursive
    git submodule foreach --recursive git lfs pull
    ```

2. Install the new environment:

    ```bash
    pixi install
    ```

3. Rebuild the whole workspace:

    ```bash
    pixi run clean && pixi run build
    ```

    `pixi run build-pkg` is not enough. The CMake caches in `build/` point at
    the old environment, and the Python packages in `install/` are in
    `python3.12` folders.

4. If the computer has `bringup/.venv`, recreate it. The old venv is on Python
   3.12, and `lift_homing.launch.py` now looks in the folder of the Python that
   runs it, `lib/python3.14`:

    ```bash
    cd bringup
    pixi run uv venv --python python --system-site-packages --clear
    pixi run uv sync --python python
    ```

    `--clear` replaces the existing venv. Without it, uv stops with
    `A virtual environment already exists`. `--python python` keeps `uv sync` on
    that venv. A bare `uv sync` replaces it, with another Python and without
    access to `rclpy`, when `.python-version` asks for another version or the
    venv's base interpreter is gone.

A branch that is still on the `_25` lock conflicts on `pixi.lock` when it is
merged. Resolve the conflict by solving the environment again, as described in
[Resolve a pixi.lock conflict](../guides/pixi.md#lock-file-conflicts).

??? info "How long the full rebuild took"
    The clean build took 1 h 9 min 13 s for 74 packages on a 12-core computer
    with 15 GB of RAM, in use at the same time. It ran at `nice -n 10`, with
    `--parallel-workers 1`, `MAKEFLAGS=-j3`, and tests enabled. The `_25` build
    of [entry 4](04-robostack-build-25.md#after-pulling-this-change) took
    1 h 19 min on a computer of the same size.

## Consequences

!!! warning "The perception venv cannot be created under pixi"
    `perception/uv.lock` pins numpy 1.26.4 and open3d 0.19.0. Neither has a
    wheel for Python 3.14, so `pixi run ./perception/setup_env.sh` stops at its
    lock check. It creates and removes nothing:

    ```text
    ERROR: 'uv sync --locked --dry-run' failed in perception/ on Python 3.14.10 (.../.pixi/envs/default/bin/python3):
    error: Distribution `numpy==1.26.4 @ registry+https://pypi.org/simple` can't be installed because it is marked as `--no-build` but has no binary distribution
    Nothing was created or removed.
    ```

    A venv built earlier from Python 3.12 does not work under pixi either:
    `import rclpy` fails with `No module named 'rclpy._rclpy_pybind11'`. So
    `yolo_node`, `obb3d`, `grasp_server`, and `test_grasp_viz` do not run under
    pixi on `_27`. The rest of `walkie_perception` builds, and its services
    import on Python 3.14. See
    [Perception: Set up the grasp pipeline](../guides/perception.md#set-up-the-grasp-pipeline)
    and [Still open](#still-open).

- **Python 3.14 everywhere in the environment.** A venv created from the pixi
  Python on `_25` must be recreated. Python packages with compiled code need
  a `cp314` wheel, or they build from source.
- **OpenCV is a headless build.** The version stays 5.0.0, but the solver now
  picks the `headless` build instead of `qt6`. `cv2.getBuildInformation()`
  reports `GUI: NONE`, so `cv2.imshow` cannot open a window. No Python or C++
  file in the workspace calls the OpenCV GUI functions. `pixi.toml` does not pin
  the variant ([Still open](#still-open)).
- **`walkie.xml` and MuJoCo move together.** MuJoCo 3.15 cannot load the new
  file. It stops with
  `input is only available for so3, pid and dcmotor actuators`. MuJoCo 3.16
  loads the old file, but `test_generated_model_is_up_to_date` fails. So
  `pixi.toml`, `pixi.lock`, and the `description` pointer belong in one commit
  ([Commit order](#commit-order)). The move is one-way: a branch on `_25` cannot
  take the new `description` commit until it takes the new lock too.
- **Repositories still on `_25` can talk to this one, as far as tested.**
  `walkie-vision` and `walkie-database` use `_25` with `libzenohc` 1.9.0. This
  environment uses `libzenohc` 1.10.1, with the same `rmw_zenoh_cpp` 0.2.11.
  Stand-in nodes on one computer exchanged topics, services, and actions in both
  directions. Two computers, the team router config, and the repositories' own
  nodes were not tested ([Verification](#verification)).

??? info "What the MuJoCo 3.16 writer changed in `walkie.xml`"
    The diff is 25 insertions and 25 deletions in 1366 lines:

    - `meshdir="../"` became `meshdir=".."`.
    - Two `exclude` lines changed places.
    - The 22 general actuators gained `input="pos"` (18) or `input="vel"` (4
      wheels).
    - The 18 position actuators now store the computed velocity gain instead of
      the `dampratio` marker. `lift_joint` went from `biasprm="0 -50000 1"` to
      `biasprm="0 -50000 -1295.92"`. Editing `kp` in the XML by hand no longer
      changes the damping. Change the constant in `build_mjcf.py` and regenerate.

    The old and new files compile to the same model on 3.16 in 581 of 583
    fields. `actuator_biasprm` differs in 18 elements, by at most 0.00383
    (relative 3e-06). `actuator_ctrlspec` differs in 22 elements, which are the
    new `input` values.

    A 4500-step rollout stayed finite without warnings. The old file gives
    bit-identical results on 3.15 and 3.16. The new file differs from the old
    one on 3.16 by 6.5e-5 m in the base at rest, 1.7e-4 m in the base during an
    arm motion, and 1.95e-3 rad in `fr_wheel`.

## Why move

**RTAB-Map comes from RoboStack.** `ros2-rtabmap-slam`, `-sync`, `-util`,
`-odom`, `-conversions`, and `-ros` have one build, 0.23.13, in `_27`. Their
`ros-jazzy-` aliases depend on `ros2-distro-mutex` 0.20. With the `_25` pin,
`pixi add` cannot install them. Only the core library, `ros2-rtabmap`, also has
a `_25` build.

This change only makes the packages reachable. In a copy of the manifest,
`pixi add` of `ros-jazzy-rtabmap-slam`, `-sync`, `-util`, `-odom`, and
`-conversions` solves in 3.3 s and adds 30 lock entries. RTAB-Map was not
installed or run.

??? info "Alternatives that were rejected"
    | Alternative | Why it was rejected |
    | --- | --- |
    | Keep `_25` and put RTAB-Map in a second pixi environment on `_27` | A second full ROS installation next to the main one, with its own Python version. [2.5 ros2_control](02-mujoco/05-ros2-control.md#packaging-status) rejected the same idea for the simulator. |

## What changed

Branch `feat/robostack-build-27` holds the change in this repository. Branch
`feat/mujoco-3.16` of the `description` submodule holds the regenerated
`walkie.xml`.

| | Before | After |
| --- | --- | --- |
| Build set | `_25` (`ros2-distro-mutex` 0.19.0) | `_27` (`ros2-distro-mutex` 0.20.0) |
| Build set pin in `pixi.toml` | `0.19.*` | `0.20.*` |
| `mujoco` pin | `3.15.*` | `3.16.*` |
| Python | 3.12.14 | 3.14.10 |
| OpenCV | 5.0.0, `qt6` build | 5.0.0, `headless` build |
| `libzenohc` | 1.9.0 | 1.10.1 |
| Packages in the `default` environment | 1191 | 1187 |

| File | Change |
| --- | --- |
| `pixi.toml`, `pixi.lock` | The two pins above, and a new solve. No package was added. |
| `description/mujoco/walkie.xml` | Regenerated by `pixi run mjcf` on MuJoCo 3.16 |
| `description/mujoco/scene.xml` | The comment that quotes the `meshdir` line, which the writer changed |
| `bringup/launch/lift_homing.launch.py` | Builds the venv path from the running interpreter. `ROBOT_BRINGUP_VENV_SITE` still overrides it. |
| `bringup/test/test_lift_homing_launch.py`, `bringup/CMakeLists.txt`, `bringup/package.xml` | New test of the venv path. It needs no build and no hardware. |
| `bringup/scripts/zed_watchdog.py`, `bringup/scripts/head_touch_beep_node.py` | `#!/usr/bin/python3` becomes a shell header that runs `$CONDA_PREFIX/bin/python3`, or `/usr/bin/python3` without `CONDA_PREFIX` (see "Why these two scripts have a shell header" below) |
| `bringup/test/test_script_interpreter.py` | New test of that header. It needs no build and no hardware. |
| `.github/workflows/ci.yml` | `robot_bringup` joins the tested packages, so CI runs those tests |
| `bringup/pyproject.toml`, `bringup/uv.lock` | `requires-python = ">=3.12"`, without the upper bound below 3.13 |
| `perception/setup_env.sh` | Builds the venv from the active `python3`. Checks the target, `rclpy`, `uv`, and the lock before it touches an existing venv. New variable `PERCEPTION_VENV_DIR`. |
| `.gitignore` | Adds `MUJOCO_LOG.TXT` |

??? info "The changed entries in pixi.toml"
    The first entry of `[dependencies]`:

    ```toml title="pixi.toml"
    # RoboStack build set _27. Every build set rebuilds all ROS packages, so the
    # mutex is pinned: adding or updating a package must not silently move the
    # environment to another set. Changing it requires `pixi run clean && pixi run build`.
    # docs/migrations/07-robostack-build-27.md records this move;
    # docs/migrations/04-robostack-build-25.md explains why the mutex is pinned.
    ros2-distro-mutex = "0.20.*"
    ```

    Further down:

    ```toml title="pixi.toml"
    # Must match the libmujoco that ros-jazzy-mujoco-vendor is built against (3.16.0 in _27).
    mujoco = "3.16.*"
    ```

??? info "Package versions that changed"
    | Package | `_25` | `_27` | Notes |
    | --- | --- | --- | --- |
    | `ros2-distro-mutex` | 0.19.0 `jazzy_25` | 0.20.0 `jazzy_27` | The only build of 0.20.0 |
    | Python | 3.12.14 | 3.14.10 | |
    | numpy | 2.5.3 | 2.5.4 | |
    | `libopencv` | 5.0.0 `qt6_hbf336e1_606` | 5.0.0 `headless_h6b8f503_7` | |
    | `py-opencv` | 5.0.0 `qt6_h6945e2d_606` | 5.0.0 `headless_h413394a_7` | |
    | `mujoco`, `libmujoco`, `mujoco-python` | 3.15.0 | 3.16.0 | |
    | `libzenohc` | 1.9.0 | 1.10.1 | |
    | `ros2-rmw-zenoh-cpp` | 0.2.11 `np2py312h815c85b_25` | 0.2.11 `np2py314hc614d3d_27` | Rebuild |
    | `ros2-rclcpp` | 28.1.22 `np2py312h1bab3cf_25` | 28.1.22 `np2py314h53a0733_27` | Rebuild |
    | `ros2-moveit-core` | 2.12.4 `np2py312h7de5627_25` | 2.12.4 `np2py314h62963f8_27` | Rebuild |
    | `openvdb` | 13.0.0 | 13.2.0 | |
    | `ros2-tl-expected` | installed | not installed | `<tl/expected.hpp>` now comes from `cpp-expected` 1.3.1. The workspace does not use it. |

    The only `ros2-*` package that changed version is the mutex. All others are
    rebuilds for Python 3.14. Unchanged: Boost 1.90.0, PCL 1.15.1, GCC 13.4.0,
    CMake 3.31.8, setuptools 79.0.1, pytest 8.4.2. The `docs` environment did
    not change (63 packages).

    The table shows the packages the team is most likely to notice. Between the
    two locks, 60 versions in `default` changed, 59 of them outside ROS, among
    them pillow 12.3.0 to 13.0.0, multidict 6.7.1 to 7.2.1, ffmpeg 9.0.1 to 9.0.2,
    and `libopenvino` 2026.3 to 2026.4. Seven names are new: `cgns`, `exprtk`,
    `fast_float`, `libharu`, `libmpdec`, `libopenvino-gguf-frontend`, and `ujson`.
    Eleven are gone: `dav1d`, `gnutls`, `libidn2`, `libmicrohttpd`, `libtasn1`,
    `libunistring`, `nettle`, `p11-kit`, `ros2-tl-expected`, `unicodedata2`, and
    `zfp`. For a full list, compare the `pixi.lock` of the two commits.

??? info "Why the lock mixes the build numbers _21, _25, _27, and _28"
    The `ros2-*` packages hold the files: 417 carry `_27`, and 6 carry `_28`
    (`ros2-ament-cmake-gmock`, `ros2-ament-cmake-gtest`,
    `ros2-gz-ogre-next-vendor`, `ros2-rviz-ogre-vendor`, `ros2-urdf`,
    `ros2-urdf-parser-plugin`). All of them depend on mutex 0.20.

    The `ros-jazzy-*` packages are aliases without files: 126 carry `_25`, and
    13 carry `_21`. They are the same downloads as in the `_25` lock. Their
    suffix says nothing about the build set.

??? info "Why these two scripts have a shell header"
    Both scripts started with `#!/usr/bin/python3`. pixi's `PYTHONPATH` now holds
    Python 3.14 packages, so Ubuntu's Python 3.12 failed with
    `No module named 'numpy._core._multiarray_umath'`. On `_25`, it could import
    the numpy from pixi, because both were Python 3.12. The watchdog owns the ZED
    process, so with the default arguments the ZED would not have started.

    `#!/usr/bin/env python3` is not enough. Without pixi, these nodes can run with
    apt Jazzy under the `walkie-commander` environment, where `python3` on `PATH`
    is a venv on Python 3.11 that cannot load `rclpy`. The header covers both
    cases, with the same trick as the perception nodes. `sh` runs the second line,
    and Python reads it as a string:

    ```python
    #!/bin/sh
    "exec" "${CONDA_PREFIX:-/usr}/bin/python3" "$0" "$@"
    ```

    A conda environment that is active next to apt Jazzy also sets `CONDA_PREFIX`,
    so these scripts would run on its Python. Deactivate it first.

## Commit order { #commit-order }

!!! warning "Push `description` first"
    This repository names a commit on branch `feat/mujoco-3.16` of `description`.
    Until that branch is on the remote, `git submodule update` fails on every other
    computer and in CI. The pointer, the two pins, and the lock are in one commit,
    because the `_27` lock with the old `walkie.xml` fails
    `test_generated_model_is_up_to_date`.

1. Push the submodule branch:

    ```bash
    git -C description push -u origin feat/mujoco-3.16
    ```

2. Push this branch and open both pull requests.

    After a test run, `git status` can also show
    ` ? hardware/actuator/topic_based_ros2_control`. Its launch test leaves a
    `__pycache__` folder in that submodule. Neither `git add -A` nor a commit
    picks it up.

3. Merge the `description` pull request with a merge commit, or keep its branch,
   so that the commit this repository names stays reachable. A squash or a rebase
   followed by a deleted branch breaks `git submodule update`. The branch also
   carries `feat: hand gain` (`86c2745`), which is on `feat/hand-gain` but not on
   `main` of `description`. `description` has no CI, so nothing tests that pull
   request by itself.
4. Merge this repository right after it.
5. Add the commits or pull requests to [What changed](#what-changed), as the
   [Migration Log](index.md) asks.

A branch still on `_25` must not move its `description` pointer to the new commit
before it rebases on this change, because MuJoCo 3.15 cannot load the new file.
`feat/chula-pat14-6-simulation` and `feat/full-body-control` are such branches
today, both with the pointer at `526c976`.

## Verification

All checks ran on 2026-11-02 and 2026-11-03, on a 12-core computer with 15 GB of
RAM, without a robot, a window, or a GPU workload. Each comment shows the result
of the command.

```bash
pixi install --locked                       # The default environment has been installed.
pixi lock --check                           # Lock-file was already up-to-date
pixi list ros2-distro-mutex                 # 0.20.0, build jazzy_27
pixi run mjcf                               # wrote ... nq=31 nv=30 nu=22 ... mass=36.664 kg
pixi run build                              # after the clean build: Summary: 74 packages finished [9.20s]
pixi run python -m pytest description/test/test_description.py description/test/test_mjcf.py \
    manipulate/openarm_bimanual_moveit_config/test/test_srdf_urdf_contract.py    # 111 passed, 1 xfailed
```

The clean build ran inside the environment with the flags of the `build` task,
one package at a time, and tests enabled:

```bash
MAKEFLAGS=-j3 pixi run bash -c 'colcon build --symlink-install --parallel-workers 1 \
    --continue-on-error --cmake-args -DCMAKE_PREFIX_PATH="$CONDA_PREFIX"'
# Summary: 74 packages finished [1h 9min 13s]
```

The robot stacks ran with these commands:

```bash
pixi run ros2 launch robot_bringup mujoco_omnibot.launch.py headless:=true use_rviz:=false
pixi run ros2 launch robot_bringup nav_mujoco_omnibot.launch.py headless:=true use_rviz:=false
pixi run ros2 launch robot_bringup lift_homing.launch.py mock:=true
```

??? info "Environment and build results"
    | Check | Result |
    | --- | --- |
    | Lock | 1187 packages in `default`, 63 in `docs`. Download about 1.7 GB, installed size 7.3 GB. |
    | Upgrade of an existing environment | In a copy of the manifest, `pixi install --locked` on the `main` lock gave mutex 0.19.0 and Python 3.12.14. The same command on the new lock, in the same folder, took 14 s and gave mutex 0.20.0, Python 3.14.10, and numpy 2.5.4. `rclpy` created a node, and only `lib/python3.14` remained. |
    | Full workspace build | 74 of 74 packages, no source edit needed. Nav2 builds with `-Werror`. |
    | Build warnings | No warning that the `_25` build logs lack. This is a rough comparison of stderr. |
    | Shared libraries | `ldd -r` on 286 shared objects: 0 not found. `dlopen` of 256 libraries with `RTLD_NOW`: 0 failed. |
    | Launch files | 46 of 46 first-party launch files parse. In a wider sweep of 78 files, only `nav2_bringup`'s `cloned_multi_tb3_simulation_launch.py` fails, on an import that `_25` lacks too. |
    | Message types | `rosidl` round trip works. 13 console scripts resolve. |
    | Not built | The ZED wrapper (`pixi run setup-real`), because the ZED SDK is not on this computer |

??? info "Test results"
    | Check | Result |
    | --- | --- |
    | `test_description.py`, `test_mjcf.py` before `pixi run mjcf` | 1 failed (`test_generated_model_is_up_to_date`), 100 passed, 1 xfailed |
    | `test_description.py`, `test_mjcf.py` after `pixi run mjcf` | 101 passed, 1 xfailed |
    | `pixi run mjcf` again | `walkie.xml` is byte-identical |
    | The CI test step of `.github/workflows/ci.yml`, with a Zenoh router and `topic_based_system_test` on Fast DDS | 134 tests, 0 errors, 0 failures, 1 skipped: `walkie_description` 106, `openarm_bimanual_moveit_config` 11, `topic_based_ros2_control` 4, `robot_bringup` 13. The first three match the 121 of the CI run on `main` at `fcbac5c`. `robot_bringup` is new here: 5 cases in `test_lift_homing_launch`, 6 in `test_script_interpreter`, and their 2 CTest entries. |
    | `test_lift_homing_launch` against a broken launch file | In a copy, a hard-coded `python3.12` fails 3 of the 5 tests, and an ignored `ROBOT_BRINGUP_VENV_SITE` fails 1 |
    | The CI lint step (not enforced) | `Summary: 2877 tests, 0 errors, 2695 failures, 8 skipped`. `main` has 2870 tests and the same 2695 failures. The 7 new tests all pass. With `bringup/.venv` present when CMake configures `robot_bringup`, as on a robot computer, `ament_lint_auto` also finds its C headers and adds `cppcheck` and `uncrustify`, which end in 2 errors (`No files found`). A CI runner has no venv there. The shell header of `zed_watchdog.py` and `head_touch_beep_node.py` came after that count. It adds 48 failures, and the two new test files add none. With the venv present, the step now gives `2936 tests, 2 errors, 2745 failures, 8 skipped`. Most are `E402`, because the header is the first statement of each file. The perception nodes with the same header have them too. |
    | Python syntax | `compileall` on 84 first-party files, with `SyntaxWarning` as an error, passes |

??? info "Simulation and MoveIt results"
    | Stack | Result |
    | --- | --- |
    | MuJoCo, `mujoco_omnibot.launch.py` | 9 controllers active. Real-time factor 1.001. `/joint_states` 54.65 Hz, `/scan` 27.48 Hz, head camera 4.82 Hz, `/unilidar/cloud` and its filtered topic 11.83 Hz. |
    | MuJoCo base drive | 0.2 m/s for 5 s, in four directions: ground truth over odometry 0.984, 0.985, 0.986, 0.977. The guide's earlier values are 0.985, 0.986, 0.978. |
    | MuJoCo with Nav2, `nav_mujoco_omnibot.launch.py` | Active in about 9 s. Goal (2.0, 0.5, 0°) succeeded in 15.66 s with a final TF error of 0.077 m. Goal (-1.5, -1.5, 90°) succeeded in 19.16 s with a final TF error of 0.048 m. Against ground truth, the second goal was off by 0.337 m, which is odometry drift (the guide gives 0.17 to 0.43 m). 0 recoveries. 84 control-loop rate warnings (median 19.23 Hz). |
    | Controller aborts during those goals | "0 recoveries" does not mean the controller never gave up. `controller_server` aborted `follow_path` once with `error_code:105` (`Failed to make progress`) in 5 of 9 goals over 5 starts, each time close to the goal. It then received the path again, and all 9 goals ended `SUCCEEDED`. The message is also in older `controller_server` logs on this computer, the newest from 2026-10-29. Goal counts on `_25` were not compared. |
    | Nav2 starts | `_27`: 18 of 19 MuJoCo starts became active, in 8 to 9 s in the batch of 12 that measured it. `_25`: 11 of 12, in 7.5 to 9 s. See [Other issues found](#other-issues-found) for the hangs. |
    | Obstacle avoidance on MuJoCo | In a copy of the scene with a wall across the direct route, 1 m wide at x = 1.5, the goal (3, 0) succeeded in 26.2 s. The robot drove around the wall as far as y = -1.07, and its center stayed at least 0.436 m from the wall. Ground truth ended 0.114 m from the goal. `/scan` saw the wall at 1.35 m. |
    | Camera and lidar content | Head camera rgb: mean 73.2, standard deviation 25.8. Head depth: all finite. Bottom camera rgb: mean 103.4, standard deviation 35.0. `/unilidar/cloud/filtered`: 3862 finite points. Through EGL, the path that logs `OpenGL error 0x502`, the image values stay within 0.2 of these, and the point count is the same. |
    | `rosbag2` | `ros2 bag record` of `/scan`, the odometry, and `/tf` wrote 2142 messages in 10.8 s to an MCAP file. `ros2 bag info` lists them. `ros2 bag play` with a remap gave the odometry at about 48 Hz. |
    | `rosbridge_server` with roslibpy 2.0.0, the version in the `walkie-sdk` lock, from a Python 3.12 venv outside pixi | Connected and listed 147 topics and 457 services. Received `/scan` and odometry. Read the `bt_navigator` state. A `NavigateToPose` goal to (1.0, 1.0) succeeded in 15.79 s, 0.072 m from the goal by odometry. |
    | Gazebo as shipped, `gzsim_omnibot.launch.py` | Segmentation fault (exit 139) in the apt plugin `libgz_ros2_control-system.so`, the same as on `_25`. See [Gazebo Simulation: Known issues](../guides/simulation-gazebo.md#known-issues). |
    | Gazebo, a copy of the launch file without the plugin path override | Real-time factor 0.448, `/scan` 17.94 Hz, `/joint_states` 26.38 Hz. `_25` measured 0.423 to 0.444, 16.92 to 17.77 Hz, and 24.88 Hz. |
    | Nav2 on that Gazebo copy | 5 of 6 starts became active in 30 to 33 s. One hung on `collision_monitor/get_state` for over 130 s. Goals (2, 0, 0) and (0, -2, -90°) succeeded, at a real-time factor of 0.29. |
    | MoveIt on mock hardware, a copy of the launch file without RViz | 6 controllers active. TRAC-IK loads. `/compute_ik` succeeds for all four groups. Joint plan 0.05 s, pose plan 0.13 s, plan and execute 2.22 s. |
    | Commander on MoveIt | `/go_to_home`, `/set_joint_position`, and `/go_to_pose` succeed. See [Other issues found](#other-issues-found) for one pose. |
    | Lift, `lift_homing.launch.py mock:=true` | `/lift/joint_states` at 100.03 Hz. `/lift/cmd` `[40, 3, 2]` moved the position from 0.7435 to 0.6419. The mock node does not use the venv, so this run says nothing about the venv path. `test_lift_homing_launch` covers the path, and the next table covers the real node's imports. |

??? info "Lift venv and perception results"
    | Check | Result |
    | --- | --- |
    | Lift venv on Python 3.14 | `pixi run uv venv --python python --system-site-packages --clear` and `pixi run uv sync --python python` install 9 packages. `TMotorCANControl`, `can`, and `rclpy` import together. |
    | `uv sync` and the lift venv, in a copy of the project | Bare `uv sync` keeps the venv while nothing asks for another Python. With `.python-version` set to 3.13, or with the base interpreter gone, it replaces the venv: Python 3.13.13, `include-system-site-packages = false`. `uv sync --python python` keeps Python 3.14.10 and `true` with the same `.python-version`. With no venv, it creates one with `false`, so run `uv venv` first. |
    | `numpy` in the lift venv | The venv's numpy 2.4.6 shadows pixi's 2.5.4. Messages, `cv_bridge`, and `mujoco` still worked with it. |
    | Lift node without CAN | All imports pass. The node stops at `can.interface.Bus(channel='can2', ...)` with `OSError: [Errno 19] No such device`. `sudo` was replaced by a stub, no motor was driven, and no CAN traffic was sent. |
    | `pixi run ./perception/setup_env.sh` | Stops at its lock check, with exit code 1, and creates nothing. The `--grasp` option does the same. |
    | `PERCEPTION_VENV_DIR` | The script refuses `$HOME`, `.`, any other directory that is not empty and has no `pyvenv.cfg`, a file, a dangling symlink, and `<venv>/bin/python3`, with exit code 1 and nothing removed. It accepts a missing path, an empty directory, an existing venv, and a symlink to a venv, and prints the `export PERCEPTION_VENV=` line for another location. These runs used real `uv`, a project without dependencies, and a stand-in `rclpy`. |
    | `setup_env.sh` with `.python-version` set to 3.13 | The venv keeps the base Python, 3.14.10 under pixi and 3.12.3 from Ubuntu, with `include-system-site-packages = true` |
    | Perception with relaxed pins, in a separate venv | `numpy>=1.26` resolves 107 wheels for Python 3.14 (numpy 2.5.4, open3d 0.20.0, torch 2.11.0, ultralytics 8.4.198). With CPU-only torch, `yolo_node` and `obb3d` started with a synthetic camera and random weights. They made 0 detections, so the mask and clustering code did not run. `grasp_server` stops at `No module named 'models'`, because GraspNet is not on the computer. |

??? info "ZED watchdog and head beep node results"
    No ZED was present, so no frame reached the watchdog. It ran with
    `usb_reset_enabled: False`, and the beep node played to the `null` audio device.

    | Check | Result |
    | --- | --- |
    | `pixi run ros2 run` of both nodes, the watchdog with `monitor_only:=true` | Both run on the pixi `python3.14` and stay up until SIGINT |
    | A small launch file with the `Node` actions of `real_omnibot.launch.py`, under pixi | Both nodes start. The watchdog runs `ros2 launch zed_wrapper` with pixi's `ros2`, sees the exit (`Package 'zed_wrapper' not found`), and relaunches. |
    | Both scripts with apt Jazzy, `walkie-commander`'s Python 3.11 venv first on `PATH`, and no `CONDA_PREFIX` | Both run on `/usr/bin/python3.12` and stay up until SIGINT |

??? info "Results for repositories on `_25`"
    The `_25` side ran from a separate checkout on the same computer.

    | Check | Result |
    | --- | --- |
    | RIHS01 type hashes | 0 of 659 environment interfaces and 0 of 100 workspace interfaces differ |
    | `_25` and `_27` nodes, one router | Topic, transient local topic, service, and Fibonacci action work in both directions. `LaserScan` and `Image` too. |
    | `_25` router on port 7448, linked to a `_27` router on port 7447 | The same result |
    | `eclipse-zenoh` 1.7.2 client, the version in the `walkie-sdk` lock, through the `_27` router | Received 21 samples and 57 liveliness tokens |
    | Team router config | Differs from the default only in the connect endpoint `tcp/10.0.0.6:7447`. Not run. |
    | Zenoh deprecation warnings | Present on both `_25` and `_27`, so they are not new with `libzenohc` 1.10.1 |

### Real-robot launch files

These parse without error. None of them started, and no hardware was present:
`real_omnibot`, `bringup_drivebase`, `dual_lidar_hokuyo_lakibeam`,
`lift_homing`, `octomap_3d_mapping`, `openarm.bimanual`,
`openarm_joint_state_reader`, `test_zed`, `isaac_omnibot`, `mapping_Nav2_real`,
`localization_Nav2_real`, `slam_cartographer`, `slam_toolbox`, and
`nav2_loopback`.

## Other issues found

This change documents these issues; it does not fix them. The largest is an
occasional hang when Nav2 starts, on `_25` as well as on `_27`.

??? info "The issues, with what is known about `_25`"
    | Issue | On `_25` |
    | --- | --- |
    | `perception/perception/test_grasp_viz.py:202` calls `z.ptp()`, which numpy 2 removed | Not compared. Under `pixi run` on `_25`, the venv also imports pixi's numpy 2. |
    | `model.track()` in Ultralytics needs `lap`, which is not in `perception/uv.lock`. `lap` 0.5.13 has a `cp314` wheel. | Not compared |
    | `gzsim_omnibot.launch.py` and `world_preview_gz.launch.py` set `GZ_SIM_SYSTEM_PLUGIN_PATH`, and Gazebo crashes. See [Gazebo Simulation](../guides/simulation-gazebo.md#known-issues). | Same |
    | A Nav2 start can hang: one node never offers `<node>/get_state`, so `lifecycle_manager_nav2` waits for it forever. It is not always the same node: `behavior_server` in 1 of 19 MuJoCo starts, and `collision_monitor` in 1 of 6 Gazebo starts. Where the ROS logs were kept, the node has no log file, so it stops before ROS logging starts. No stack trace was taken. `kernel.yama.ptrace_scope=1` lets a process trace only its descendants, so start the node under `gdb` rather than attach to it after the hang. | Same: `map_server` in 1 of 12 MuJoCo starts |
    | The `go_to_pose` example at (0.4, -0.2, 0.5), pitch 1.57, with group `right_arm` aborts with `Planning Failed` (-31). The same pose with `right_arm_lift` succeeds in 17.7 s. | Not compared |
    | `/gazebo_ground_truth/odom` has no data. `gz_bridge.yaml` uses `/model/my_bot/pose`, and the model is `walkie_bot`. | The code is the same |
    | Spawners can abort with `Aborting, no controller is switched! (::STRICT switch)` | Same |
    | Stopping a launch prints a Zenoh TLS panic, `rcl_shutdown already called`, and `Received Data for unknown expr_id` at ERROR level. The last one also appears when any other node process exits. | Not compared |
    | With no window system at all, MuJoCo falls back from GLFW to EGL, logs `OpenGL error 0x502 in or before mjr_makeContext`, and writes it to `MUJOCO_LOG.TXT` in the working directory. The camera content does not change ([Simulation and MoveIt results](#verification)). In a desktop session, GLFW still starts with `DISPLAY` and `WAYLAND_DISPLAY` unset. EGL appeared only with `XDG_RUNTIME_DIR` and `XDG_SESSION_TYPE` unset too. | Same warning on EGL |
    | `walkie-agent-ros` gets `nav2_msgs` from RoboStack, and this workspace builds it from `navigation/navigation2`. 17 interfaces differ in their RIHS01 type hash (12 actions and 5 services), among them `NavigateToPose`, which `src/walkie_agent/tools/navigation.py` uses. A goal across the two was not tried. | Same hash in the `_25` install |
    | `scripts/tmux_walkie.sh`, `.tmuxp.yaml`, and `walkie-run.sh` in the `walkie-run` repository run outside pixi from `~/robocup2026/walkie-ws`, so they do not follow this move. `walkie-run.sh` also starts `localization_Nav2_stvl_real.launch.py`, which no longer exists. | The code is the same |
    | `gzsim_omnibot.launch.py` has no headless argument, and the MoveIt demo launch always starts RViz | The code is the same |
    | `bringup/scripts/zed_watchdog copy.py` is a stale copy | The code is the same |
    | An arm position step of 2.0 rad launches the base in MuJoCo | Same on MuJoCo 3.15 |
    | The Python modules of the Dynamixel SDK need `pyserial`, which pixi does not provide | Same |
    | The vendored Dynamixel test file `multi_port.py` is Python 2 and does not compile (line 165) | Same |

## Still open

Decisions to take:

- **Perception on `_27`.** `perception/` is planned for removal from this
  repository, with GraspNet moving to `walkie-vision`. That work is separate.
  Until then, the pins could change for Python 3.14:
  `numpy>=1.26,<2.0; python_version < '3.14'`, `numpy>=2; python_version >= '3.14'`,
  and `uv lock -P open3d -P numpy -P opencv-python -P opencv-python-headless -P scipy`.
  This resolved in a copy. It is not in the repository, and `lap` and `z.ptp()`
  above would need fixes too.
- **OpenCV variant.** `headless` is what the solver picks, and the workspace
  does not call the GUI functions. If someone needs `cv2.imshow`,
  `libopencv = { version = "5.0.*", build = "qt6*" }` solves in a copy of the
  manifest and moves 28 packages, among them `qt6-main` 6.11.2 to 6.12.0. It was
  not installed. The pin in `pixi.toml` does not fix the variant, so a later
  `pixi update` could change it.

Not tested on `_27`:

- **All real hardware:** CubeMars wheels and lift motor, Dynamixel servos,
  OpenArm, ZED, Hokuyo, Lakibeam, and Unitree lidar. The hardware plugins were
  only compiled and loaded with `dlopen`. The real launch files were only parsed.
- `real_omnibot.launch.py` under pixi, which also stops at
  `realsense2_camera`
  ([Known issues](../guides/real-robot.md#known-issues)).
  `zed_watchdog.py` and `head_touch_beep_node.py` ran with `ros2 run` and from a
  small launch file, not from a real launch file, and without a ZED or a head
  servo.
- `pixi run setup-real` and the ZED wrapper build.
- The Docker images and any apt build. The apt Python 3.12 path of
  `perception/setup_env.sh` was only emulated, with real `uv`, a project without
  dependencies, and a stand-in `rclpy`. The script has passed `bash -n`, not
  `shellcheck`, which is not installed here. Host and container can no longer
  share `bringup/.venv` ([Docker Images](../guides/docker.md#known-issues)).
- PyTorch with CUDA (`cu128`) and any GPU work. `obb3d` with real detections.
- The GraspNet CUDA extensions and `grasp_server`. `graspnetAPI` on numpy 2.
  open3d 0.20 on Python 3.12.
- Zenoh across two computers, and the team router config under `libzenohc`
  1.10.1. The nodes of `walkie-vision`, `walkie-database`, and `walkie-sdk`
  themselves. Mixed zenoh-c versions were only tested on one computer.
- A `_25` run of the MoveIt results above.
- RTAB-Map itself. Its `ros2-nav2-msgs` 1.3.13 would sit next to the workspace's
  upstream `nav2_msgs`, and their type hashes were not compared.
- Any window: RViz, the MuJoCo viewer, the Gazebo GUI, and `ogre2` with a
  display.
- `nav_sim_omnibot.launch.py` as shipped. `isaac_omnibot.launch.py` beyond
  parsing.
- The CI job itself, so the first pull request run is its real test. The two-pass
  build with `BUILD_TESTING=OFF`, `MAKEFLAGS=-j2`, and ccache has not run on `_27`,
  and neither has a pixi environment restored from a cold cache. The CI test step
  only ran here, on a build with tests enabled. Recent CI runs took 7.3 to 29.9
  min with a filled ccache. How much of that ccache still hits with the new lock
  is not known. The local clean build took 1 h 9 min, and the job timeout is 300
  min. Only pushes to `main` write the pixi cache, so each pull request run
  downloads about 1.7 GB until this change is merged.
- Nav2's own tests and `urg_node2_test`. The rclpy console scripts beyond
  imports and `--help`.
- Whether pytest 9 still rejects the launch plugins. The pin stays, and
  `launch_testing/pytest/hooks.py` is byte-identical to `_25`.
