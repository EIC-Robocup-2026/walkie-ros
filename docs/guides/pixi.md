# Working with Pixi

This page explains how to use pixi in this repository every day: how to run
commands, which tasks exist, how the two environments differ, and how to add a
dependency. It assumes you have finished [Getting Started](../getting-started.md).

## Background

- **pixi** is a package manager. It installs packages into the project folder
  (`.pixi/envs/`), not into the system. `pixi.lock` records the exact version
  of every package, so everyone on the team gets the same environment.
- **conda packages** are prebuilt binary packages: libraries, programs, and
  Python modules, each with a list of its dependencies. pixi installs conda
  packages.
- A **channel** is a server that hosts conda packages. This workspace uses two
  channels. pixi looks in them in the order they are listed in `pixi.toml`.

| Channel | What it provides |
| --- | --- |
| `https://prefix.dev/robostack-jazzy` | ROS 2 Jazzy packages |
| `https://prefix.dev/conda-forge` | Everything else: Python, compilers, CMake, common libraries, MuJoCo |

- **RoboStack** is a project that builds ROS packages as conda packages and
  publishes them on the `robostack-jazzy` channel. A RoboStack package name is
  `ros-jazzy-` followed by the ROS package name, with dashes instead of
  underscores. For example, the ROS package `topic_tools` is
  `ros-jazzy-topic-tools`.

## Important files

| File or folder | What it is | In Git |
| --- | --- | --- |
| `pixi.toml` | The workspace manifest: channels, dependencies, tasks, environments. You edit this file. | Yes |
| `pixi.lock` | The exact version of every package in every environment. pixi writes it. Never edit it by hand. | Yes |
| `.pixi/envs/default/` | The installed ROS environment, about 8 GB | No |
| `.pixi/envs/docs/` | The installed documentation environment | No |
| `walkie_external.repos` | Source packages that are not on the channels. `pixi run import` clones them. | Yes |
| `install/setup.bash` | Created by the first build. pixi sources it when it activates the environment. | No |

## pixi run and pixi shell

There are two ways to use the environment:

| Command | What it does |
| --- | --- |
| `pixi run <command>` | Activates the environment, runs one command, then exits. |
| `pixi shell` | Opens a new shell with the environment activated. Type `exit` to leave. |

```bash
# One command at a time
pixi run ros2 topic list

# Or a whole shell session
pixi shell
ros2 topic list
exit
```

Things to know:

- If the first word after `pixi run` is a task name, pixi runs the task.
  Otherwise it runs the command as it is.
- Tasks always run from the repository root. Other commands run in your
  current directory.
- `pixi run` keeps the variables of your terminal. If you
  `export RMW_IMPLEMENTATION=rmw_zenoh_cpp` first, the command sees it.
- `pixi run` activates the environment again for every command. A
  `pixi shell` is activated once, when it starts. This matters after a build;
  see the next section.
- `pixi run -e docs <command>` runs a command in the `docs` environment
  instead of the default one.

To see what is installed:

```bash
pixi list                      # all packages in the default environment
pixi list ros-jazzy-rmw        # only packages whose name matches
pixi list -e docs              # packages in the docs environment
```

## Activation and the workspace packages

**Activation** is what pixi does before it runs a command: it sets `PATH`,
`CONDA_PREFIX`, and the ROS variables of the environment. This workspace also
asks pixi to source the colcon setup script:

```toml title="pixi.toml"
[activation]
scripts = ["install/setup.bash"]
```

Sourcing `install/setup.bash` adds the packages you built to
`AMENT_PREFIX_PATH`, `PYTHONPATH`, and the other ROS search paths. This is why
you never type `source install/setup.bash` yourself in normal use.

This has two consequences:

1. **Before the first build** (and after `pixi run clean`), `install/` does
   not exist. pixi prints the following message and continues. It is
   expected, and it goes away after `pixi run build`.

    ```text
    WARN Could not find activation scripts: .../install/setup.bash
    ```

2. **A shell opened before a build does not see new packages.** colcon's
   `setup.bash` adds each package in `install/` one by one, at the moment it
   is sourced. A package built later is not in the lists of an older shell.
   In that shell, run `source install/setup.bash`, or open a new
   `pixi shell`. `pixi run` does not have this problem, because it activates
   again every time.

## Tasks

Tasks are named commands defined in `pixi.toml`. `pixi task list` shows them
all with their descriptions.

| Task | Environment | Command | Notes |
| --- | --- | --- | --- |
| `setup` | default | `ln -sf libpthread.so.0 $CONDA_PREFIX/x86_64-conda-linux-gnu/sysroot/usr/lib/libpthread.so` | Adds the `libpthread.so` link that the conda compiler sysroot is missing. Safe to run again. |
| `import` | default | `vcs import < walkie_external.repos` | Clones or updates the source packages listed in `walkie_external.repos`. |
| `build` | default | `colcon build --symlink-install --cmake-args -DCMAKE_PREFIX_PATH=$CONDA_PREFIX` | Runs `setup` and `import` first. Builds the whole workspace. |
| `build-pkg` | default | The `build` command plus `--packages-select` | Runs `setup` first, but not `import`. Give the package names after the task name. |
| `test` | default | `colcon test` | Runs the tests of every package. Extra arguments are added at the end. |
| `clean` | default | `rm -rf build install log` | Removes all build output. |
| `mjcf` | default | `python description/scripts/build_mjcf.py` | Regenerates `description/mujoco/walkie.xml` from the robot Xacro. No build needed. |
| `mjcf-view` | default | A Python one-liner | Opens `description/mujoco/scene.xml` in the MuJoCo viewer, at the `home` keyframe. |
| `docs` | docs | `mkdocs serve` | Previews this site at <http://127.0.0.1:8000>. |
| `docs-build` | docs | `mkdocs build --strict` | Builds the site into `site/`. Fails on any warning, such as a broken link. |

Words after the task name are added to the end of the task's command:

```bash
pixi run build-pkg robot_bringup
pixi run build-pkg walkie_description robot_simulation
pixi run test --packages-select walkie_description
pixi run colcon test-result --verbose
```

To see what a task would run without running it, use a dry run:

```bash
pixi run -n build
```

`-DCMAKE_PREFIX_PATH=$CONDA_PREFIX` tells CMake to look for libraries in the
pixi environment (`$CONDA_PREFIX` is the path of the active environment). Use
the `build` and `build-pkg` tasks instead of typing `colcon build` yourself, so
the flags are always the same.

## What needs a rebuild

Both build tasks use `--symlink-install`. colcon then installs symbolic links
that point back to your source files, instead of copies. When you edit a source
file, the installed "copy" changes with it.

| You changed | Rebuild needed? |
| --- | --- |
| An existing Python, launch, Xacro, YAML, or RViz file | No. Restart the node or the launch file. |
| A script that CMake installs under a new name (`install(PROGRAMS ... RENAME ...)`), such as the `walkie_perception` nodes and the `openarm_bimanual_commander_cpp` scripts | Yes, `pixi run build-pkg <package>`. colcon copies these files instead of linking them. |
| A new file: a launch file, a config file, a script | Yes, `pixi run build-pkg <package>`. A new file has no link in `install/` until the next build. |
| C++ code | Yes. C++ must be compiled. |
| `package.xml`, `CMakeLists.txt`, or `setup.py` | Yes. |

## Environments

pixi groups packages and tasks into **features**. An **environment** is a set
of features that is installed together. This workspace has two environments:

| Environment | Features | Contents | Size |
| --- | --- | --- | --- |
| `default` | The default feature | ROS 2 Jazzy, Gazebo, MoveIt, compilers, MuJoCo, and the other build and simulation dependencies | 1276 packages, about 1.7 GB download |
| `docs` | `docs` only | MkDocs and the Material theme | 63 packages, about 70 MB download |

The default feature is everything at the top level of `pixi.toml`:
`[dependencies]`, `[tasks]`, and `[activation]`. The `docs` environment is
declared with `no-default-feature = true`:

```toml title="pixi.toml"
[environments]
docs = {features = ["docs"], no-default-feature = true}
```

So the `docs` environment has no ROS, and it does not source
`install/setup.bash`. It is a small download, which keeps the documentation CI
fast. [Add the Documentation Site](../migrations/03-documentation.md) explains
this setup.

How pixi chooses the environment:

- `pixi run docs` and `pixi run docs-build` use `docs` automatically, because
  those tasks exist only there.
- Everything else uses `default`, unless you pass `-e docs`.
- `pixi install` installs only `default`. Use `pixi install -e docs` or
  `pixi install --all` for the others.

## Adding a dependency

### From the channels

First check that the package exists, then add it:

```bash
pixi search ros-jazzy-<package-name>
pixi add ros-jazzy-<package-name>
```

`pixi add` adds a line to `[dependencies]` in `pixi.toml`, solves the
environment again, updates `pixi.lock`, and installs the package.

Then:

1. Declare the dependency in the `package.xml` of the package that uses it.
   colcon uses `package.xml` to order the build, and the Docker images still
   install dependencies with `rosdep` from `package.xml`.
2. Commit `pixi.toml` and `pixi.lock` together. Teammates get the same
   versions from `pixi.lock`. If the lock file no longer matches `pixi.toml`,
   `pixi install --locked` stops with
   `lock file not up-to-date with the workspace`. The documentation workflow
   installs with `--locked`.

### Lock file conflicts

`.gitattributes` contains this line:

```text
pixi.lock merge=binary linguist-language=YAML linguist-generated=true -diff
```

| Setting | Effect |
| --- | --- |
| `merge=binary` | Git never merges `pixi.lock` line by line. On a conflict, it leaves the current branch's version in place and marks the file as conflicted. |
| `-diff` | `git diff` does not print the file's changes. |
| `linguist-generated=true` | GitHub collapses the file in pull request diffs. |
| `linguist-language=YAML` | GitHub treats the file as YAML. |

A lock file merged line by line can look correct and still describe an
environment that cannot exist. So you do not fix the file by hand. You
regenerate it:

```bash
# pixi.lock is conflicted during a merge or rebase.
# 1. Fix any conflict in pixi.toml by hand first.
# 2. Take either version of the lock file:
git checkout --theirs pixi.lock
# 3. Solve the environment again from pixi.toml:
pixi lock
git add pixi.toml pixi.lock
```

### Packages that are not on the channels

Some ROS packages are not published on `robostack-jazzy`. They are cloned as
source code and built with the rest of the workspace. `walkie_external.repos`
lists them in the [vcstool](https://github.com/dirk-thomas/vcstool) format:

```yaml title="walkie_external.repos"
repositories:
  tools/twist_stamper:
    type: git
    url: https://github.com/joshnewans/twist_stamper.git
    version: 0.0.5
```

| Path | Version | Needed by |
| --- | --- | --- |
| `hardware/lidar/laser_proc` | A fixed commit | `urg_node2` (the Hokuyo lidar driver) |
| `tools/twist_stamper` | Tag `0.0.5` | `robot_bringup` and `robot_simulation` |

To add a package:

1. Add an entry with `type`, `url`, and `version`. Use a tag or a commit hash
   as the version, so everyone builds the same code.
2. Add the path to `.gitignore`. The clone is not part of this repository.
3. Run `pixi run import`, or `pixi run build`, which runs it.

!!! note "laser_proc is on the channel, but not for the locked build"
    `robostack-jazzy` has `ros-jazzy-laser-proc` only in a newer RoboStack
    build (build number 25, which requires `ros2-distro-mutex` 0.19).
    `pixi.lock` uses the older build (build number 21, `ros2-distro-mutex`
    0.16). Moving `laser_proc` to `pixi.toml` means moving the whole ROS
    environment to the newer build first. `ros-jazzy-twist-stamper` is not on
    the channel.

### The "packages were not found" list

At the end of `[dependencies]`, `pixi.toml` has a commented block that starts
with `# The following packages were not found:`. When the dependency list was
created, these dependency names from the workspace `package.xml` files had no
package with the same name on the channels. pixi does not read the block. It is
a record:

| Name | Declared in | Notes |
| --- | --- | --- |
| `ros-jazzy-laser-proc` | `hardware/lidar/urg_node2/package.xml` | Built from source through `walkie_external.repos` |
| `libblosc-dev`, `liblzma-dev`, `libopenvdb-dev` | `navigation/spatio_temporal_voxel_layer/openvdb_vendor/package.xml` | Debian package names. The OpenVDB library comes from the `openvdb` conda package. |
| `libopenexr-dev` | `navigation/spatio_temporal_voxel_layer/spatio_temporal_voxel_layer/package.xml` | Debian package name |
| `libqt6-core`, `libqt6gui6t64`, `libqt6statemachine6`, `libqt6widgets6t64`, `qt6-base-dev`, `qt6-scxml-dev` | `navigation/navigation2/nav2_rviz_plugins/package.xml` | Debian package names |

## Python virtual environments next to pixi

Two parts of the robot need Python packages from PyPI that are not in the pixi
environment. They use [uv](https://docs.astral.sh/uv/) virtual environments
(venvs) next to pixi. uv itself is not in the pixi environment either, so
install it separately on the computers that need it.

| | Lift homing node | Perception nodes |
| --- | --- | --- |
| Package | `robot_bringup` | `walkie_perception` |
| Python packages | TMotorCANControl, python-can, and others (`bringup/pyproject.toml`) | PyTorch, Ultralytics, Open3D, and others (`perception/pyproject.toml`) |
| venv location | `bringup/.venv` | `~/perception-venv` |
| Created by | `pixi run uv venv --python python3.12 --system-site-packages`, then `pixi run uv sync`, inside `bringup/` | `perception/setup_env.sh` |
| How the node finds it | `bringup/launch/lift_homing.launch.py` adds the venv's `site-packages` to the node's `PYTHONPATH` | The venv-based node scripts start with a shell line that runs the script again with `$PERCEPTION_VENV`, or `~/perception-venv/bin/python3` if it is not set. `PERCEPTION_VENV` is the path of the Python executable, not of the venv folder. |

For the lift node, run uv **through** `pixi run`. Inside `pixi run`, uv picks
the pixi Python 3.12 as the base interpreter of the venv. This is the same
Python that runs the node. Outside pixi, uv picks another Python 3.12, such as
the system `/usr/bin/python3.12`. With `--system-site-packages`, the venv can
also import the packages of its base interpreter, so a pixi-based venv sees
`rclpy` and the other ROS packages from pixi.

[Real Robot: Python environment](real-robot.md#python-environment) has the
full lift setup, including how the launch file finds the venv.

`perception/setup_env.sh` builds its venv on the system Python instead. See
[Perception](perception.md#what-a-pixi-user-must-do) for how to run it on a
computer with pixi only.

## Never source /opt/ros in a pixi shell

Do not run `source /opt/ros/jazzy/setup.bash` (or any other distribution) in a
shell where you use pixi. Remove it from `~/.bashrc` too.

Why: that script adds the system ROS folders to `PATH`, `AMENT_PREFIX_PATH`,
`PYTHONPATH`, and `LD_LIBRARY_PATH`. The system ROS packages are built against
Ubuntu's libraries. The pixi packages are built against conda-forge libraries.
With both on the search paths, ROS loads some packages and libraries from each
place. Two problems follow:

- A package that is missing from `pixi.toml` is found in `/opt/ros`. It works
  on your computer and fails on every computer without a system ROS.
- Libraries from two different builds end up in one process, which causes
  errors that are hard to trace.

To check your environment, print the ROS search path. Every entry should be
inside the repository (`install/...` or `.pixi/envs/default`):

```bash
pixi run bash -c 'echo $AMENT_PREFIX_PATH'
```

Some helper scripts in `scripts/` still source `/opt/ros`. See
[Known issues](#known-issues).

## Known issues

| Issue | Effect | Workaround |
| --- | --- | --- |
| `scripts/bringup/sim_nav_gazebo.sh`, `scripts/bringup/isaac_sim_bringup.sh`, and `scripts/map/record_mapping_bag.sh` source `/opt/ros/$ROS_DISTRO/setup.bash` when it exists. pixi sets `ROS_DISTRO=jazzy`. `scripts/bringup/real_slam_nav.sh` always sources `/opt/ros/<distro>/setup.bash`. | On a computer that also has a system ROS Jazzy, these scripts mix the two installations. | Run the `ros2 launch` commands from the guides directly. |
| `.gitignore` has `.pixi/*` followed by `!.pixi/config.toml`, but an earlier `.pixi` line ignores the whole folder. Git does not re-include a file inside an ignored folder. | A project-level `.pixi/config.toml` is never picked up by Git. | Add it with `git add -f .pixi/config.toml`, or remove the earlier `.pixi` line. |
| `realsense2_camera` and `topic_tools` are not in `pixi.toml`. | `real_omnibot.launch.py` fails under pixi. | See [Packages that pixi does not provide](real-robot.md#packages-that-pixi-does-not-provide). |
| `perception/setup_env.sh` creates its venv from `/usr/bin/python3.12`, not from the pixi Python. | Run from a plain shell on a computer without a system ROS, the script stops at its `import rclpy` check. | See [What a pixi user must do](perception.md#what-a-pixi-user-must-do). |
