# Troubleshooting

Fixes for problems that are not tied to one part of the robot: the
environment, pixi, Git, the build, Zenoh, Nav2 bringup, tests, and this site. Search for a
distinctive part of your error message to find the right entry.

## By topic

Problems with one simulator, launch file, or device are in the guides.

| Topic | Where |
| --- | --- |
| pixi, helper scripts, Python venvs | [Working with Pixi: Known issues](guides/pixi.md#known-issues) |
| Zenoh router and connecting to the robot | [Middleware: Known issues](guides/middleware.md#known-issues), [Troubleshooting](guides/middleware.md#troubleshooting) |
| Gazebo | [Gazebo Simulation: Known issues](guides/simulation-gazebo.md#known-issues), [Troubleshooting](guides/simulation-gazebo.md#troubleshooting) |
| MuJoCo model, viewer, custom worlds, and the launch files | [MuJoCo Simulation: Troubleshooting](guides/simulation-mujoco.md#troubleshooting) |
| Nav2, maps, localization | [Navigation: Known issues](guides/navigation.md#known-issues), [Troubleshooting](guides/navigation.md#troubleshooting) |
| MoveIt and the arms | [Manipulation: Known issues](guides/manipulation.md#known-issues), [Troubleshooting](guides/manipulation.md#troubleshooting) |
| Object detection and grasping | [Perception: Known issues](guides/perception.md#known-issues), [Troubleshooting](guides/perception.md#troubleshooting) |
| Devices, lift, bringup | [Real Robot: Known issues](guides/real-robot.md#known-issues) |
| Docker images and CI | [Docker Images: Known issues](guides/docker.md#known-issues) |
| Single test failures | [Tests: Known issues](guides/testing.md#known-issues), [Troubleshooting](guides/testing.md#troubleshooting) |

## Environment and pixi

### `pixi: command not found` after installing pixi { #pixi-is-not-found-after-installing-it }

Close the terminal and open a new one.

??? info "Why the current terminal does not find pixi"
    The installer adds pixi to `PATH` in your shell startup file. The terminal
    you ran it in started before that change.

### `ros2: command not found` { #ros2-is-not-found }

The same applies to `colcon: command not found`. Start the command with
`pixi run`, or open a `pixi shell` first. Run it inside the repository folder,
so pixi finds `pixi.toml`.

```bash
pixi run ros2 topic list
```

??? info "Why ros2 is only available through pixi"
    ROS 2 is installed inside the project, in `.pixi/envs/default/`. It is only
    on `PATH` inside the pixi environment.

### WARN Could not find activation scripts

Every pixi command prints
`WARN Could not find activation scripts: .../install/setup.bash`. Nothing is
broken. Run `pixi run build`, and the message goes away.

??? info "Why pixi looks for install/setup.bash"
    pixi sources `install/setup.bash` on activation. The workspace has not been
    built yet, or `pixi run clean` removed `install/`.

### A package is not found after a build

In a `pixi shell` that was open during the build, `ros2 launch` or `ros2 run`
says `Package '<name>' not found`. The same command works with `pixi run`.

Reload the setup file in that shell, or open a new `pixi shell`:

```bash
source install/setup.bash
```

??? info "Why only an open shell is affected"
    A shell reads `install/setup.bash` once, when it starts. Packages that were
    built for the first time after that are not on its search paths.
    `pixi run` is not affected, because it activates the environment again for
    each command.

### Packages from a system ROS show up

Something works on your computer but fails on a teammate's, or paths under
`/opt/ros` appear in error messages.

1. Remove the `source /opt/ros/...` line from `~/.bashrc`.
2. Open a new terminal.
3. Check the search path. Every entry must be inside the repository:

    ```bash
    pixi run bash -c 'echo $AMENT_PREFIX_PATH'
    ```

See [Working with Pixi](guides/pixi.md#never-source-optros-in-a-pixi-shell).

??? info "Why a system ROS breaks the pixi environment"
    A system ROS 2 installation was sourced into the pixi environment, usually
    from `~/.bashrc`. ROS then finds packages and libraries in both places.

### A helper script mixes in the system ROS

`scripts/bringup/sim_nav_gazebo.sh`, or another script in `scripts/`, behaves
differently from the same launch command run with `pixi run`.

Run the launch command directly. For example, instead of `sim_nav_gazebo.sh`:

```bash
pixi run ros2 run rmw_zenoh_cpp rmw_zenohd          # terminal 1
export RMW_IMPLEMENTATION=rmw_zenoh_cpp             # terminal 2
pixi run ros2 launch robot_bringup nav_sim_omnibot.launch.py
```

??? info "Why some scripts mix the two installations"
    Some helper scripts source `/opt/ros/$ROS_DISTRO/setup.bash`, and pixi sets
    `ROS_DISTRO=jazzy`. On a computer with a system ROS 2 Jazzy, the two
    installations are mixed. The list of scripts is in
    [Working with Pixi: Known issues](guides/pixi.md#known-issues).

### Gazebo problems only on computers with a system ROS

The Gazebo simulation crashes with a segmentation fault, or misbehaves, on a
computer that also has ROS 2 Jazzy in `/opt/ros/jazzy`, and works on a
computer without it.

Check whether the system plugin exists:

```bash
ls /opt/ros/jazzy/lib/libgz_ros2_control-system.so
```

If it does, remove the line `ld.add_action(gz_system_plugin_cmd)` locally
from the launch file (`gzsim_omnibot.launch.py` or
`world_preview_gz.launch.py` in `simulation/gazebo/launch/`). The launch file
then no longer sets `GZ_SIM_SYSTEM_PLUGIN_PATH`, and Gazebo uses the value
from the pixi environment. Or test on a computer without a system ROS.

??? info "Why Gazebo loads the system plugin"
    `gzsim_omnibot.launch.py` and `world_preview_gz.launch.py` set
    `GZ_SIM_SYSTEM_PLUGIN_PATH` to `/opt/ros/jazzy/lib/`. `ros_gz_sim` puts
    that value before the other plugin folders, so Gazebo can load the system
    `libgz_ros2_control-system.so`. That library is built against different
    libraries than the pixi environment.

    Since RoboStack build set `_25`, `gz_ros2_control` ships an environment
    hook, so `pixi run` already sets `GZ_SIM_SYSTEM_PLUGIN_PATH` to the `lib`
    folder of the pixi environment (`.pixi/envs/default/lib`). The launch
    files replace that value. Removing the override from both launch files is
    the fix; it has not been made yet.

### real_omnibot.launch.py stops with package not found

`pixi run ros2 launch robot_bringup real_omnibot.launch.py` stops with
`Caught exception in launch` and a message such as
`package 'realsense2_camera' not found`.

Run the full bringup in the physical Docker image. It has `zed_wrapper`,
`realsense2_camera`, and `topic_tools`, and `pixi.toml` has none of them.
Under pixi, `use_zed:=false` skips the ZED camera, but the launch still needs
the other two packages.

??? info "Which packages the launch needs, and their status in pixi"
    The launch file starts `zed_wrapper` (with the default `use_zed:=true`),
    `realsense2_camera`, and a `topic_tools` relay. See
    [Packages that pixi does not provide](guides/real-robot.md#packages-that-pixi-does-not-provide).

    `ros-jazzy-realsense2-camera` and `ros-jazzy-topic-tools` exist on
    `robostack-jazzy` for the locked RoboStack build, but nobody has added
    them to `pixi.toml` or tried them on the robot.

### The lift node cannot import TMotorCANControl

The lift homing node exits with
`ModuleNotFoundError: No module named 'TMotorCANControl'`.

1. Create the venv once, from the repository root. Install uv first; see
   [Real Robot: Create the lift Python environment](guides/real-robot.md#python-environment).

    ```bash
    cd bringup
    pixi run uv venv --python python3.12 --system-site-packages
    pixi run uv sync
    ```

2. Build with `pixi run build` or `pixi run build-pkg robot_bringup`.

If you build without `--symlink-install`, set `ROBOT_BRINGUP_VENV_SITE` to the
venv's `site-packages` folder:

```bash
export ROBOT_BRINGUP_VENV_SITE=$PWD/bringup/.venv/lib/python3.12/site-packages
```

??? info "Why the node cannot find the library"
    TMotorCANControl is not in the pixi environment. It lives in the uv venv
    at `bringup/.venv`, which `lift_homing.launch.py` adds to the node's
    `PYTHONPATH`. The venv does not exist yet, or the launch file cannot find
    it, because the package was built without `--symlink-install`.

### Lock file not up-to-date with the workspace

`pixi install --locked`, or the documentation workflow, stops with
`lock file not up-to-date with the workspace`.

Run `pixi lock` (or any `pixi install` or `pixi run` without `--locked`).
Then commit `pixi.toml` and `pixi.lock` together.

??? info "Why the lock file is out of date"
    `pixi.toml` was changed, but `pixi.lock` was not regenerated and committed
    with it.

### pixi.lock has a merge conflict

A merge or rebase reports a conflict in `pixi.lock`. Resolve `pixi.toml`
first, then regenerate the lock file:

```bash
git checkout --theirs pixi.lock
pixi lock
git add pixi.toml pixi.lock
```

`pixi lock` solves for the `ros2-distro-mutex` pin in the merged `pixi.toml`.
If one branch was on another RoboStack build set, such as `feat/Pixi` on
`_21`, the result can be on a different build set than before. Then rebuild
everything, as in
[Build errors after pulling a new pixi.lock](#build-errors-after-pulling-a-new-pixilock).

??? info "Why Git does not merge pixi.lock"
    `.gitattributes` sets `merge=binary` for `pixi.lock`, so Git never merges
    it line by line.

## Git, LFS, and submodules

### Cloning a submodule fails with Permission denied

`git clone --recurse-submodules` or `git submodule update` prints
`git@github.com: Permission denied (publickey).`

1. Add an SSH key to your GitHub account.
2. Test it. When the key works, GitHub greets you by user name.

    ```bash
    ssh -T git@github.com
    ```

3. Run `git submodule update --init --recursive` again.

Without an SSH key, you can tell Git to use HTTPS for GitHub instead:

```bash
git config --global url."https://github.com/".insteadOf "git@github.com:"
```

!!! warning
    This applies to every GitHub repository on your computer, including
    pushes, which then need HTTPS credentials.

??? info "Why the submodules need an SSH key"
    The `description` and `navigation` submodules, and the map submodule
    inside `navigation`, use SSH URLs. Git needs an SSH key that your GitHub
    account accepts. The repositories themselves are public.

### Robot meshes do not load

Gazebo or RViz cannot load some parts of the robot model. The mesh files in
`description/` are tiny text files.

```bash
git lfs install
git submodule foreach --recursive git lfs pull
```

??? info "Why the meshes are pointer files, and how to check"
    The meshes (`*.dae` files in `description/`) are stored in Git LFS. When
    LFS was not set up, or a tool skipped the LFS download, Git leaves small
    pointer files in their place. A pointer file starts like this:

    ```bash
    head -c 100 description/meshes/walkie/omni_wheel_optimize.dae
    ```

    ```text
    version https://git-lfs.github.com/spec/v1
    ...
    ```

    `git -C description lfs ls-files` marks a downloaded file with `*` and a
    pointer-only file with `-`.

### No url found for submodule path tools/twist_stamper

`git submodule update --init --recursive` stops with
`fatal: No url found for submodule path 'tools/twist_stamper' in .gitmodules`.

Use a branch that contains commit `8ead77e`, which removes the cause. On an
affected branch, update the real submodules by path:

```bash
git submodule update --init --recursive -- description navigation \
    hardware/actuator/topic_based_ros2_control hardware/actuator/openarm/openarm_can
```

`tools/twist_stamper` was an obsolete package and has been removed from the repository.

??? info "Why Git cannot clone tools/twist_stamper"
    On `feat/Pixi` and on branches made from it before commit `8ead77e`,
    `tools/twist_stamper` was committed as a gitlink: a submodule entry with
    no line in `.gitmodules`. Git cannot clone it. `8ead77e` removed the
    gitlink.

## Build

### A new launch or config file is not found

`ros2 launch` says
`file '<name>.launch.py' was not found in the share directory of package '<package>'`,
but the file exists in the source tree. Rebuild the package:

```bash
pixi run build-pkg <package>
```

[What needs a rebuild](guides/pixi.md#what-needs-a-rebuild) lists the other
cases.

??? info "Why a new file needs a rebuild"
    `--symlink-install` links each installed file back to its source. A file
    created after the last build has no link in `install/` yet.

### CMake cannot find laser_proc

Building `urg_node2` fails with
`Could not find a package configuration file provided by "laser_proc"`.

```bash
pixi install
pixi run build-pkg urg_node2
```

??? info "Why CMake cannot find laser_proc"
    `ros-jazzy-laser-proc` is installed as a conda package via pixi. Run `pixi install`
    to ensure the package is present in `.pixi/envs/default/`. If you previously had
    a source clone in `hardware/lidar/laser_proc`, remove it.

### The build runs out of memory or crashes during initial setup

Compiling all packages in parallel (especially the 46 Nav2 packages in
`navigation/navigation2/`) consumes large amounts of RAM and can trigger the
Linux Out-Of-Memory (OOM) killer, terminating the compiler.

Build packages one by one instead:

```bash
pixi run build-setup
```

Or specify a worker count that fits your machine's memory:

```bash
pixi run build-setup 2
```

??? info "Why the first build needs more memory"
    colcon processes multiple packages in parallel by default. During the initial
    setup, heavy C++ packages compile simultaneously. `build-setup` sets
    `--parallel-workers 1` by default so packages compile one at a time.

### A plain colcon build behaves differently

A package builds with `pixi run build`, but not with `colcon build` typed in a
`pixi shell`.

Use `pixi run build-setup`, `pixi run build`, or `pixi run build-pkg <package>`. If you must run colcon
yourself, run the same steps as the `build` task:

```bash
pixi run setup
pixi run import
colcon build --symlink-install --cmake-args -DCMAKE_PREFIX_PATH=$CONDA_PREFIX
```

??? info "What the build tasks do beyond colcon build"
    The build tasks first create the `libpthread.so` link in the compiler
    sysroot (`setup`) and clone the external packages (`import`). Then they
    run colcon with `--symlink-install` and
    `-DCMAKE_PREFIX_PATH=$CONDA_PREFIX`.

    The `libpthread.so` link lives inside `.pixi/envs/default/`, so a
    reinstall of the environment removes it.

### Build errors after pulling a new pixi.lock

After a `git pull` that changed the `ros2-distro-mutex` pin in `pixi.toml`,
`pixi run build-pkg` fails, or a package that was built before the pull fails
at run time.

Install the new environment and rebuild everything. `pixi run build-pkg` is
not enough.

```bash
pixi install
pixi run clean
pixi run build
```

`pixi list ros2-distro-mutex` shows the build set of the installed
environment. See [Changing the build set](guides/pixi.md#changing-the-build-set).

??? info "Why a new build set needs a full rebuild"
    The new lock is on another RoboStack build set. Every ROS package was
    rebuilt, and many libraries changed version. The CMake caches in `build/`
    still point at the configuration folders of the old library versions, such
    as VTK 9.6 and OpenCV 4. Some libraries also changed their ABI. In build
    set `_25`, these are `behaviortree_cpp` and `realtime_tools`.
    `behaviortree_cpp` keeps the same library file name across versions, so
    the loader does not reject Nav2 packages that were built against the old
    version. They must be rebuilt.

## Zenoh

### Nodes cannot see each other

`pixi run ros2 topic list` in one terminal does not show topics from a node in
another terminal, or from the robot.

Start one router, and use the same settings in every terminal:

```bash
pixi run ros2 run rmw_zenoh_cpp rmw_zenohd   # in its own terminal
export RMW_IMPLEMENTATION=rmw_zenoh_cpp      # in every other terminal
```

Set the same `ROS_DOMAIN_ID` everywhere. The team uses domain 23. See
[Middleware (Zenoh)](guides/middleware.md).

??? info "The three causes and how to recognize each"
    | Cause | How to recognize it |
    | --- | --- |
    | No Zenoh router is running | A node using Zenoh prints ``Unable to connect to a Zenoh router. Have you started a router with `ros2 run rmw_zenoh_cpp rmw_zenohd`?`` |
    | `RMW_IMPLEMENTATION` is not set in one terminal | pixi does not set it. Without it, ROS 2 Jazzy uses its default DDS middleware (`rmw_fastrtps_cpp`), which cannot talk to Zenoh nodes |
    | Different `ROS_DOMAIN_ID` values | The team uses domain 23. `scripts/bringup/connect_walkie_zenoh_router.sh` sets it only for its own router process; an unset variable means domain 0 |

## Nav2 and launch files

These problems come from the Nav2 fork in `navigation/navigation2/` and hit
more than one launch file.

### Nav2 stops with Failed to load motion model plugin 'Omni' { #nav2-fails-to-load-the-omni-motion-model }

`controller_server` fails to configure, Nav2 stops during bringup, and the log
shows `Can not get 'plugin' param value for FollowPath.Omni` and
`Failed to load motion model plugin 'Omni'`. Every Nav2 launch except
`nav_mujoco_omnibot.launch.py` stops this way.

Add `Omni: plugin: "mppi::OmniMotionModel"` under `FollowPath` in the MPPI
parameter file. See
[Navigation: controller_server cannot load the Omni motion model](guides/navigation.md#mppi-omni-motion-model-plugin).

### Nav2 waits forever for smoother_server/get_state { #nav2-waits-for-smoother-server }

Nav2 never becomes active, and the log repeats
`Waiting for service smoother_server/get_state...`.

Pass `use_composition:=False` to `nav2_bringup`'s `bringup_launch.py`. See
[Navigation: Nav2 bringup hangs at smoother_server/get_state](guides/navigation.md#nav2-composition-deadlock).

### Nav2 waits forever for amcl/get_state or crashes with version GLIBC_2.38 not found { #nav2-amcl-glibc-238 }

The lifecycle manager hangs on `Waiting for service amcl/get_state...`, or Nav2 nodes (`amcl`, `controller_server`) crash on startup with:
`version 'GLIBC_2.38' not found (required by .../libamcl_core.so)`.

The conda build sysroot (GLIBC 2.39) links `fmod` and ISO C23 `strtol` functions to `GLIBC_2.38`, which do not exist on Ubuntu 22.04's host libraries (GLIBC 2.35).

This is resolved via `nav2_common/cmake/glibc_compat.h` forced-include in `nav2_package.cmake`. Rebuild the affected package:
```bash
pixi run build-pkg -- nav2_amcl
```

### RViz Navigation 2 panel fails to load { #rviz-nav2-panel-fails-to-load }

RViz shows an error when opening the Nav2 panel:
`The class required for this panel, 'nav2_rviz_plugins/Navigation 2', could not be loaded.`
`dlopen error: /lib/x86_64-linux-gnu/libc.so.6: version 'GLIBC_2.38' not found`

Resolved via dynamic runtime fallback wrappers in `nav2_rviz_plugins/src/nav2_panel.cpp`. Rebuild the plugin:
```bash
pixi run build-pkg -- nav2_rviz_plugins
```

## Tests

### Every Python test aborts at start-up

All ament pytest tests fail before running, with `PluginValidationError` and
`Argument(s) {'path'} are declared in the hookimpl but can not be found in the hookspec`.

Keep `pytest = ">=8,<9"` in `pixi.toml` and run `pixi install`.

??? info "Why pytest 9 breaks the tests"
    The pytest plugins of Jazzy `launch_testing` and `launch_testing_ros` use
    a hook argument (`path`) that pytest 9 removed. See
    [Why pytest is pinned below 9](guides/testing.md#why-pytest-is-pinned-below-9).

### pixi run test takes very long or fails in urg_node2

`pixi run test` runs for a long time, or `urg_node2_test` fails or times out.
Test only the packages you changed, or skip `urg_node2`:

```bash
pixi run test --packages-select <package>
pixi run test --packages-skip urg_node2
```

See [Tests](guides/testing.md).

??? info "Why the full test run is slow and urg_node2 fails"
    The task tests every package, including the 46 Nav2 packages in
    `navigation/navigation2/`. `urg_node2_test` connects to a Hokuyo lidar at
    `192.168.0.10` and has a 200 s timeout, so it fails without the lidar.
    `test_action_follow_path_action` in `nav2_behavior_tree` can hang until
    CTest stops it (see [Tests: Known issues](guides/testing.md#known-issues)).

## Documentation site

### docs-build fails in strict mode

`pixi run docs-build` ends with `Aborted with 1 warnings in strict mode!` (or
another count). The lines above that message name each problem. Fix each one,
then run `pixi run docs-build` again.

| Message | Fix |
| --- | --- |
| `contains a link '...', but the target ... is not found among documentation files` | Correct the relative path. Links are relative to the current page. |
| `contains a link '...', but the doc '...' does not contain an anchor '#...'` | Check the heading on the target page. An anchor is the heading in lowercase, with spaces as dashes and most punctuation removed. |
| `The following pages exist in the docs directory, but are not included in the "nav" configuration` | Add the page to `nav` in `mkdocs.yml`. |
| `contains an absolute link '...'` | Use a relative `.md` link instead. |
| `contains an unrecognized relative link '...'` | Link to the `.md` file, not to a folder or a path without extension. |

??? info "Why a broken link fails the build"
    `mkdocs.yml` turns link problems into warnings, and `--strict` fails on
    any warning. See [3. Add the Documentation Site](migrations/03-documentation.md).

### The docs deploy job is rejected

A run of the Docs workflow on a branch other than `main` (such as `feat/Pixi`)
builds the site, but the `deploy` job fails with a message such as
`Deployment to github-pages was rejected: branch feat/Pixi is not allowed to deploy`.

To allow `feat/Pixi` to deploy to GitHub Pages:

1. Open the repository on GitHub and go to **Settings → Environments → github-pages**.
2. Under **Deployment branches and tags**, add `feat/Pixi` (or select "No restriction").

??? info "Why only main can deploy by default"
    By default, GitHub creates the `github-pages` environment with deployment
    protection rules that restrict deployments to the default branch (`main`).
    Adding `feat/Pixi` allows documentation changes made during development to
    publish automatically.

### Port 8000 is already in use

`pixi run docs` fails to start because another program uses port 8000. Pick
another port. Words after the task name are added to the command:

```bash
pixi run docs -a 127.0.0.1:8001
```

??? info "Why port 8000"
    `mkdocs serve` listens on `127.0.0.1:8000` by default.
