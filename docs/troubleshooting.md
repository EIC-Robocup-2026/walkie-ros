# Troubleshooting

This page covers problems that are not tied to one part of the robot: the
environment, pixi, Git, the build, Zenoh, tests in general, and this site.
Problems with one simulator, launch file, or device are in the guides; see
[By topic](#by-topic). Each entry gives the symptom, the cause, and the fix.
Use the search box with a distinctive part of your error message to find the
right entry.

## By topic

| Topic | Where |
| --- | --- |
| pixi, helper scripts, Python venvs | [Working with Pixi: Known issues](guides/pixi.md#known-issues) |
| Zenoh router and connecting to the robot | [Middleware: Known issues](guides/middleware.md#known-issues), [Troubleshooting](guides/middleware.md#troubleshooting) |
| Gazebo | [Gazebo Simulation: Known issues](guides/simulation-gazebo.md#known-issues), [Troubleshooting](guides/simulation-gazebo.md#troubleshooting) |
| MuJoCo model, viewer, and custom worlds | [MuJoCo Simulation: Troubleshooting](guides/simulation-mujoco.md#troubleshooting) |
| Nav2, maps, localization | [Navigation: Known issues](guides/navigation.md#known-issues), [Troubleshooting](guides/navigation.md#troubleshooting) |
| MoveIt and the arms | [Manipulation: Known issues](guides/manipulation.md#known-issues), [Troubleshooting](guides/manipulation.md#troubleshooting) |
| Object detection and grasping | [Perception: Known issues](guides/perception.md#known-issues), [Troubleshooting](guides/perception.md#troubleshooting) |
| Devices, lift, bringup | [Real Robot: Known issues](guides/real-robot.md#known-issues) |
| Docker images and CI | [Docker Images: Known issues](guides/docker.md#known-issues) |
| Single test failures | [Tests: Known issues](guides/testing.md#known-issues), [Troubleshooting](guides/testing.md#troubleshooting) |

## Environment and pixi

### pixi is not found after installing it

**Symptom:** `pixi: command not found` right after running the installer.

**Cause:** the installer adds pixi to `PATH` in your shell startup file. The
terminal you ran it in started before that change.

**Fix:** close the terminal and open a new one.

### ros2 is not found

**Symptom:** `ros2: command not found`, or `colcon: command not found`.

**Cause:** ROS 2 is installed inside the project, in `.pixi/envs/default/`.
It is only on `PATH` inside the pixi environment.

**Fix:** start the command with `pixi run`, for example
`pixi run ros2 topic list`, or open a `pixi shell` first. Run it inside the
repository folder, so pixi finds `pixi.toml`.

### WARN Could not find activation scripts

**Symptom:** every pixi command prints
`WARN Could not find activation scripts: .../install/setup.bash`.

**Cause:** pixi sources `install/setup.bash` on activation, and the workspace
has not been built yet, or `pixi run clean` removed `install/`.

**Fix:** nothing is broken. Run `pixi run build`, and the message goes away.

### A package is not found after a build

**Symptom:** in a `pixi shell` that was open during the build,
`ros2 launch` or `ros2 run` says `Package '<name>' not found`. The same
command works with `pixi run`.

**Cause:** a shell reads `install/setup.bash` once, when it starts. Packages
that were built for the first time after that are not on its search paths.

**Fix:** run `source install/setup.bash` in that shell, or open a new
`pixi shell`. `pixi run` is not affected, because it activates the
environment again for each command.

### Packages from a system ROS show up

**Symptom:** something works on your computer but fails on a teammate's, or
paths under `/opt/ros` appear in error messages.

**Cause:** a system ROS 2 installation was sourced into the pixi environment,
usually from `~/.bashrc`. ROS then finds packages and libraries in both
places.

**Fix:** remove the `source /opt/ros/...` line from `~/.bashrc` and open a new
terminal. Check the search path; every entry must be inside the repository:

```bash
pixi run bash -c 'echo $AMENT_PREFIX_PATH'
```

See [Working with Pixi](guides/pixi.md#never-source-optros-in-a-pixi-shell).

### A helper script mixes in the system ROS

**Symptom:** `scripts/bringup/sim_nav_gazebo.sh`, or another script in
`scripts/`, behaves differently from the same launch command run with
`pixi run`.

**Cause:** some helper scripts source `/opt/ros/$ROS_DISTRO/setup.bash`, and
pixi sets `ROS_DISTRO=jazzy`. On a computer with a system ROS 2 Jazzy, the two
installations are mixed. The list of scripts is in
[Working with Pixi: Known issues](guides/pixi.md#known-issues).

**Fix:** run the launch command directly. For example, instead of
`sim_nav_gazebo.sh`:

```bash
pixi run ros2 run rmw_zenoh_cpp rmw_zenohd          # terminal 1
export RMW_IMPLEMENTATION=rmw_zenoh_cpp             # terminal 2
pixi run ros2 launch robot_bringup nav_sim_omnibot.launch.py
```

### Gazebo problems only on computers with a system ROS

**Symptom:** the Gazebo simulation misbehaves on a computer that also has
ROS 2 Jazzy in `/opt/ros/jazzy`, and works on a computer without it.

**Cause:** `gzsim_omnibot.launch.py` and `world_preview_gz.launch.py` set
`GZ_SIM_SYSTEM_PLUGIN_PATH` to `/opt/ros/jazzy/lib/`. `ros_gz_sim` puts that
value before the other plugin folders, so Gazebo can load the system
`libgz_ros2_control-system.so`, which is built against different libraries
than the pixi environment.

**Fix:** check whether the system plugin exists:

```bash
ls /opt/ros/jazzy/lib/libgz_ros2_control-system.so
```

If it does, test on a computer without a system ROS, or remove the
`GZ_SIM_SYSTEM_PLUGIN_PATH` line from the launch file locally.

### real_omnibot.launch.py stops with package not found

**Symptom:** `pixi run ros2 launch robot_bringup real_omnibot.launch.py` stops
with `Caught exception in launch` and a message such as
`package 'realsense2_camera' not found`.

**Cause:** the launch file starts `zed_wrapper` (with the default
`use_zed:=true`), `realsense2_camera`, and a `topic_tools` relay. None of the
three is in `pixi.toml`. See
[Packages that pixi does not provide](guides/real-robot.md#packages-that-pixi-does-not-provide).

**Fix:** run the full bringup in the physical Docker image, which has all
three. Under pixi, `use_zed:=false` skips the ZED camera, but the launch still
needs the other two packages. `ros-jazzy-realsense2-camera` and
`ros-jazzy-topic-tools` exist on `robostack-jazzy` for the locked RoboStack
build, but nobody has added them to `pixi.toml` or tried them on the robot.

### The lift node cannot import TMotorCANControl

**Symptom:** the lift homing node exits with
`ModuleNotFoundError: No module named 'TMotorCANControl'`.

**Cause:** TMotorCANControl is not in the pixi environment. It lives in the uv
venv at `bringup/.venv`, which `lift_homing.launch.py` adds to the node's
`PYTHONPATH`. The venv does not exist yet, or the launch file cannot find it,
because the package was built without `--symlink-install`.

**Fix:** create the venv as described in
[Real Robot: Python environment](guides/real-robot.md#python-environment),
and build with `pixi run build` or `pixi run build-pkg robot_bringup`. Without
`--symlink-install`, set `ROBOT_BRINGUP_VENV_SITE` to the venv's
`site-packages` folder.

### Lock file not up-to-date with the workspace

**Symptom:** `pixi install --locked`, or the documentation workflow, stops
with `lock file not up-to-date with the workspace`.

**Cause:** `pixi.toml` was changed, but `pixi.lock` was not regenerated and
committed with it.

**Fix:** run `pixi lock` (or any `pixi install` or `pixi run` without
`--locked`), then commit `pixi.toml` and `pixi.lock` together.

### pixi.lock has a merge conflict

**Symptom:** a merge or rebase reports a conflict in `pixi.lock`.

**Cause:** `.gitattributes` sets `merge=binary` for `pixi.lock`, so Git never
merges it line by line.

**Fix:** resolve `pixi.toml` first, then regenerate the lock file:

```bash
git checkout --theirs pixi.lock
pixi lock
git add pixi.toml pixi.lock
```

## Git, LFS, and submodules

### Cloning a submodule fails with Permission denied

**Symptom:** `git clone --recurse-submodules` or `git submodule update` prints
`git@github.com: Permission denied (publickey).`

**Cause:** the `description` and `navigation` submodules, and the map
submodule inside `navigation`, use SSH URLs. Git needs an SSH key that your
GitHub account accepts. The repositories themselves are public.

**Fix:** add an SSH key to your GitHub account, then test it:

```bash
ssh -T git@github.com
```

When the key works, GitHub greets you by user name. Then run
`git submodule update --init --recursive` again.

Without an SSH key, you can tell Git to use HTTPS for GitHub instead:

```bash
git config --global url."https://github.com/".insteadOf "git@github.com:"
```

This applies to every GitHub repository on your computer, including pushes,
which then need HTTPS credentials.

### Robot meshes do not load

**Symptom:** Gazebo or RViz cannot load some parts of the robot model. The
mesh files in `description/` are tiny text files.

**Cause:** the meshes (`*.dae` files in `description/`) are stored in Git
LFS. When LFS was not set up, or a tool skipped the LFS download, Git leaves
small pointer files in their place. A pointer file starts like this:

```bash
head -c 100 description/meshes/walkie/omni_wheel_optimize.dae
```

```text
version https://git-lfs.github.com/spec/v1
...
```

`git -C description lfs ls-files` marks a downloaded file with `*` and a
pointer-only file with `-`.

**Fix:**

```bash
git lfs install
git submodule foreach --recursive git lfs pull
```

### No url found for submodule path tools/twist_stamper

**Symptom:** `git submodule update --init --recursive` stops with
`fatal: No url found for submodule path 'tools/twist_stamper' in .gitmodules`.

**Cause:** on `feat/Pixi` and on branches made from it before commit
`8ead77e`, `tools/twist_stamper` was committed as a gitlink: a submodule entry
with no line in `.gitmodules`. Git cannot clone it.

**Fix:** use a branch that contains `8ead77e`, which removes the gitlink. On
an affected branch, update the real submodules by path:

```bash
git submodule update --init --recursive -- description navigation \
    hardware/actuator/topic_based_ros2_control hardware/actuator/openarm/openarm_can
```

`tools/twist_stamper` itself comes from `pixi run import`.

## Build

### A new launch or config file is not found

**Symptom:** `ros2 launch` says
`file '<name>.launch.py' was not found in the share directory of package '<package>'`,
but the file exists in the source tree.

**Cause:** `--symlink-install` links each installed file back to its source.
A file created after the last build has no link in `install/` yet.

**Fix:** rebuild the package:

```bash
pixi run build-pkg <package>
```

[What needs a rebuild](guides/pixi.md#what-needs-a-rebuild) lists the other
cases.

### CMake cannot find laser_proc

**Symptom:** building `urg_node2` fails with
`Could not find a package configuration file provided by "laser_proc"`.

**Cause:** `laser_proc` is not installed from the channels. `pixi run import`
clones it into `hardware/lidar/laser_proc`, and colcon builds it with the
workspace. `pixi run build` runs the import first. `pixi run build-pkg` does
not, and it builds only the packages you name.

**Fix:**

```bash
pixi run import
pixi run build-pkg laser_proc urg_node2
```

### A plain colcon build behaves differently

**Symptom:** a package builds with `pixi run build`, but not with
`colcon build` typed in a `pixi shell`.

**Cause:** the build tasks do more than `colcon build`. They first create the
`libpthread.so` link in the compiler sysroot (`setup`), clone the external
packages (`import`), and pass `--symlink-install` and
`-DCMAKE_PREFIX_PATH=$CONDA_PREFIX`. The `libpthread.so` link lives inside
`.pixi/envs/default/`, so a reinstall of the environment removes it.

**Fix:** use `pixi run build` or `pixi run build-pkg <package>`. If you must
run colcon yourself, run `pixi run setup` and `pixi run import` first, and
use the same flags.

## Zenoh

### Nodes cannot see each other

**Symptom:** `pixi run ros2 topic list` in one terminal does not show topics
from a node in another terminal, or from the robot.

**Cause:** one of these three:

| Cause | How to recognize it |
| --- | --- |
| No Zenoh router is running | A node using Zenoh prints ``Unable to connect to a Zenoh router. Have you started a router with `ros2 run rmw_zenoh_cpp rmw_zenohd`?`` |
| `RMW_IMPLEMENTATION` is not set in one terminal | pixi does not set it. Without it, ROS 2 Jazzy uses its default DDS middleware (`rmw_fastrtps_cpp`), which cannot talk to Zenoh nodes |
| Different `ROS_DOMAIN_ID` values | The team uses domain 23. `scripts/bringup/connect_walkie_zenoh_router.sh` sets it only for its own router process; an unset variable means domain 0 |

**Fix:** start one router, and use the same settings in every terminal:

```bash
pixi run ros2 run rmw_zenoh_cpp rmw_zenohd   # in its own terminal
export RMW_IMPLEMENTATION=rmw_zenoh_cpp      # in every other terminal
```

Set the same `ROS_DOMAIN_ID` everywhere. See
[Middleware (Zenoh)](guides/middleware.md).

## Tests

### Every Python test aborts at start-up

**Symptom:** all ament pytest tests fail before running, with
`PluginValidationError` and
`Argument(s) {'path'} are declared in the hookimpl but can not be found in the hookspec`.

**Cause:** pytest 9. The pytest plugins of Jazzy `launch_testing` and
`launch_testing_ros` use a hook argument (`path`) that pytest 9 removed.

**Fix:** keep `pytest = ">=8,<9"` in `pixi.toml` and run `pixi install`. See
[Why pytest is pinned below 9](guides/testing.md#why-pytest-is-pinned-below-9).

### pixi run test takes very long or fails in urg_node2

**Symptom:** `pixi run test` runs for a long time, or `urg_node2_test` fails
or times out.

**Cause:** the task tests every package, including the 46 Nav2 packages in
`navigation/navigation2/`. `urg_node2_test` connects to a Hokuyo lidar at
`192.168.0.10` and has a 200 s timeout, so it fails without the lidar.

**Fix:** test only the packages you changed, or skip `urg_node2`:

```bash
pixi run test --packages-select <package>
pixi run test --packages-skip urg_node2
```

See [Tests](guides/testing.md).

## Documentation site

### docs-build fails in strict mode

**Symptom:** `pixi run docs-build` ends with
`Aborted with 1 warnings in strict mode!` (or another count).

**Cause:** `mkdocs.yml` turns link problems into warnings, and `--strict`
fails on any warning. The lines above the abort message name the problem:

| Message | Fix |
| --- | --- |
| `contains a link '...', but the target ... is not found among documentation files` | Correct the relative path. Links are relative to the current page. |
| `contains a link '...', but the doc '...' does not contain an anchor '#...'` | Check the heading on the target page. An anchor is the heading in lowercase, with spaces as dashes and most punctuation removed. |
| `The following pages exist in the docs directory, but are not included in the "nav" configuration` | Add the page to `nav` in `mkdocs.yml`. |
| `contains an absolute link '...'` | Use a relative `.md` link instead. |
| `contains an unrecognized relative link '...'` | Link to the `.md` file, not to a folder or a path without extension. |

**Fix:** fix each message, then run `pixi run docs-build` again. See
[Add the Documentation Site](migrations/03-documentation.md).

### The docs deploy job is rejected

**Symptom:** a manual run of the Docs workflow on a branch other than `main`
builds the site, but the `deploy` job fails.

**Cause:** by default, the `github-pages` environment accepts deployments
only from the default branch, `main`.

**Fix:** merge the change to `main`. To check a branch, run
`pixi run docs-build` locally instead.

### Port 8000 is already in use

**Symptom:** `pixi run docs` fails to start because another program uses
port 8000.

**Cause:** `mkdocs serve` listens on `127.0.0.1:8000` by default.

**Fix:** pick another port. Words after the task name are added to the
command:

```bash
pixi run docs -a 127.0.0.1:8001
```
