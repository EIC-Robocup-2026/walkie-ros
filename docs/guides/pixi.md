# Working with Pixi

Use this page to run commands in the pixi environment, build the workspace, and
add dependencies. It assumes you have finished
[Getting Started](../getting-started.md).

??? info "What pixi, conda packages, channels, and RoboStack are"
    - **pixi** is a package manager. It installs packages into the project
      folder (`.pixi/envs/`), not into the system. `pixi.lock` records the
      exact version of every package, so everyone on the team gets the same
      environment.
    - **conda packages** are prebuilt binary packages: libraries, programs,
      and Python modules, each with a list of its dependencies. pixi installs
      conda packages.
    - A **channel** is a server that hosts conda packages. This workspace uses
      two channels. pixi looks in them in the order they are listed in
      `pixi.toml`.
    - **RoboStack** is a project that builds ROS packages as conda packages
      and publishes them on the `robostack-jazzy` channel.

    | Channel | What it provides |
    | --- | --- |
    | `https://prefix.dev/robostack-jazzy` | ROS 2 Jazzy packages |
    | `https://prefix.dev/conda-forge` | Everything else: Python, compilers, CMake, common libraries, MuJoCo |

??? info "Files and folders that pixi uses"
    | File or folder | What it is | In Git |
    | --- | --- | --- |
    | `pixi.toml` | The workspace manifest: channels, dependencies, tasks, environments. You edit this file. | Yes |
    | `pixi.lock` | The exact version of every package in every environment. pixi writes it. Never edit it by hand. | Yes |
    | `.pixi/envs/default/` | The installed ROS environment, about 8 GB | No |
    | `.pixi/envs/docs/` | The installed documentation environment | No |
    | `walkie_external.repos` | Source packages that are not on the channels. `pixi run import` clones them. | Yes |
    | `install/setup.bash` | Created by the first build. pixi sources it when it activates the environment. | No |

## Run commands in the environment { #pixi-run-and-pixi-shell }

```bash
# One command: activate the environment, run the command, exit
pixi run ros2 topic list

# A whole shell session with the environment activated
pixi shell
ros2 topic list
exit
```

- If the first word after `pixi run` is a [task](#tasks) name, pixi runs the
  task. Otherwise it runs the command as it is.
- Tasks always run from the repository root. Other commands run in your
  current directory.
- `pixi run` keeps your terminal's variables, such as an exported
  `RMW_IMPLEMENTATION=rmw_zenoh_cpp`.
- `pixi run` activates the environment for every command. `pixi shell`
  activates once, when it starts. This matters
  [after a build](#activation-and-the-workspace-packages).
- `pixi run -e docs <command>` uses the `docs` environment instead of the
  default one.

To see what is installed:

```bash
pixi list                      # all packages in the default environment
pixi list ros-jazzy-rmw        # only packages whose name matches
pixi list -e docs              # packages in the docs environment
```

## Never source /opt/ros in a pixi shell

!!! warning
    Do not run `source /opt/ros/jazzy/setup.bash` (or any other distribution)
    in a shell where you use pixi. Remove it from `~/.bashrc` too.

With a system ROS and pixi on the same search paths, two problems follow:

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

??? info "Why system ROS and pixi ROS do not mix"
    The `/opt/ros` setup script adds the system ROS folders to `PATH`,
    `AMENT_PREFIX_PATH`, `PYTHONPATH`, and `LD_LIBRARY_PATH`. The system ROS
    packages are built against Ubuntu's libraries. The pixi packages are built
    against conda-forge libraries. With both on the search paths, ROS loads
    some packages and libraries from each place.

## How the environment finds your built packages { #activation-and-the-workspace-packages }

pixi sources `install/setup.bash` every time it activates the environment, so
in normal use you never source it yourself. Two consequences:

1. **Before the first build** (and after `pixi run clean`), `install/` does
   not exist. pixi prints this message and continues. It is expected, and it
   goes away after `pixi run build`.

    ```text
    WARN Could not find activation scripts: .../install/setup.bash
    ```

2. **A shell opened before a build does not see new packages.** In that
   shell, run `source install/setup.bash`, or open a new `pixi shell`.
   `pixi run` does not have this problem, because it activates again every
   time.

??? info "What activation does"
    **Activation** is what pixi does before it runs a command: it sets `PATH`,
    `CONDA_PREFIX`, and the ROS variables of the environment. This workspace
    also asks pixi to source the colcon setup script:

    ```toml title="pixi.toml"
    [activation]
    scripts = ["install/setup.bash"]
    ```

    Sourcing `install/setup.bash` adds the packages you built to
    `AMENT_PREFIX_PATH`, `PYTHONPATH`, and the other ROS search paths.
    colcon's `setup.bash` adds each package in `install/` one by one, at the
    moment it is sourced. A package built later is not in the lists of an
    older shell.

## Build and run other tasks { #tasks }

Tasks are named commands defined in `pixi.toml`. Words after the task name
are added to the end of the task's command:

```bash
pixi run build                                          # build the whole workspace
pixi run build-pkg robot_bringup                        # rebuild selected packages
pixi run build-pkg walkie_description robot_simulation
pixi run test --packages-select walkie_description
pixi run colcon test-result --verbose
pixi task list                                          # all tasks with their descriptions
pixi run -n build                                       # dry run: show what the task would run
```

Use the `build` and `build-pkg` tasks instead of typing `colcon build`
yourself, so the flags are always the same.

| Task | What it does |
| --- | --- |
| `build` | Runs `setup` and `import`, then builds the whole workspace |
| `build-pkg` | Runs `setup` (not `import`), then builds the named packages |
| `test` | Runs the tests of every package |
| `clean` | Removes all build output |
| `import` | Clones or updates the packages in `walkie_external.repos` |
| `setup` | Adds the `libpthread.so` link that the conda compiler sysroot is missing. Safe to run again. |
| `mjcf` | Regenerates `description/mujoco/walkie.xml` from the robot Xacro. No build needed. |
| `mjcf-view` | Opens `description/mujoco/scene.xml` in the MuJoCo viewer, at the `home` keyframe |
| `docs` | Previews this site at <http://127.0.0.1:8000> |
| `docs-build` | Builds the site into `site/`. Fails on any warning, such as a broken link. |

??? info "The command behind each task"
    | Task | Environment | Command |
    | --- | --- | --- |
    | `setup` | default | `ln -sf libpthread.so.0 $CONDA_PREFIX/x86_64-conda-linux-gnu/sysroot/usr/lib/libpthread.so` |
    | `import` | default | `vcs import < walkie_external.repos` |
    | `build` | default | `colcon build --symlink-install --cmake-args -DCMAKE_PREFIX_PATH=$CONDA_PREFIX` |
    | `build-pkg` | default | The `build` command plus `--packages-select` |
    | `test` | default | `colcon test` |
    | `clean` | default | `rm -rf build install log` |
    | `mjcf` | default | `python description/scripts/build_mjcf.py` |
    | `mjcf-view` | default | A Python one-liner |
    | `docs` | docs | `mkdocs serve` |
    | `docs-build` | docs | `mkdocs build --strict` |

    `-DCMAKE_PREFIX_PATH=$CONDA_PREFIX` tells CMake to look for libraries in
    the pixi environment (`$CONDA_PREFIX` is the path of the active
    environment).

## What needs a rebuild

Both build tasks use `--symlink-install`: colcon installs symbolic links to
your source files instead of copies. When you edit a source file, the
installed "copy" changes with it. To rebuild one package, run
`pixi run build-pkg <package>`.

| You changed | Rebuild needed? |
| --- | --- |
| An existing Python, launch, Xacro, YAML, or RViz file | No. Restart the node or the launch file. |
| A script installed under a new name (`install(PROGRAMS ... RENAME ...)`), such as the `walkie_perception` nodes and the `openarm_bimanual_commander_cpp` scripts | Yes. colcon copies these files instead of linking them. |
| A new file, such as a launch file, config file, or script | Yes. It has no link in `install/` until the next build. |
| C++ code | Yes. C++ must be compiled. |
| `package.xml`, `CMakeLists.txt`, or `setup.py` | Yes. |

## Add a dependency { #adding-a-dependency }

### From the channels

1. Find the package name. A RoboStack package name is `ros-jazzy-` followed
   by the ROS package name, with dashes instead of underscores. For example,
   the ROS package `topic_tools` is `ros-jazzy-topic-tools`. Check that it
   exists:

    ```bash
    pixi search ros-jazzy-<package-name>
    ```

2. Add it. This adds a line to `[dependencies]` in `pixi.toml`, solves the
   environment again, updates `pixi.lock`, and installs the package.

    ```bash
    pixi add ros-jazzy-<package-name>
    ```

3. Declare the dependency in the `package.xml` of the package that uses it.
   colcon uses it to order the build, and the Docker images still install
   dependencies from it with `rosdep`.
4. Commit `pixi.toml` and `pixi.lock` together. Teammates get the same
   versions from `pixi.lock`. If the lock file no longer matches `pixi.toml`,
   `pixi install --locked` stops with
   `lock file not up-to-date with the workspace`. The documentation workflow
   installs with `--locked`.

### Resolve a pixi.lock conflict { #lock-file-conflicts }

A lock file merged line by line can look correct and still describe an
environment that cannot exist. So do not fix `pixi.lock` by hand. Regenerate
it:

```bash
# pixi.lock is conflicted during a merge or rebase.
# 1. Fix any conflict in pixi.toml by hand first.
# 2. Take either version of the lock file:
git checkout --theirs pixi.lock
# 3. Solve the environment again from pixi.toml:
pixi lock
git add pixi.toml pixi.lock
```

??? info "How .gitattributes handles pixi.lock"
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

### Packages that are not on the channels

Some ROS packages are not published on `robostack-jazzy`. They are cloned as
source code and built with the rest of the workspace. `walkie_external.repos`
lists them in the [vcstool](https://github.com/dirk-thomas/vcstool) format.

| Path | Version | Needed by |
| --- | --- | --- |
| `hardware/lidar/laser_proc` | A fixed commit | `urg_node2` (the Hokuyo lidar driver) |
| `tools/twist_stamper` | Tag `0.0.5` | `robot_bringup` and `robot_simulation` |

To add a package:

1. Add an entry to `walkie_external.repos` with `type`, `url`, and `version`.
   Use a tag or a commit hash as the version, so everyone builds the same
   code.

    ```yaml title="walkie_external.repos"
    repositories:
      tools/twist_stamper:
        type: git
        url: https://github.com/joshnewans/twist_stamper.git
        version: 0.0.5
    ```

2. Add the path to `.gitignore`. The clone is not part of this repository.
3. Run `pixi run import`, or `pixi run build`, which runs it.

??? info "Why laser_proc and twist_stamper are built from source"
    `robostack-jazzy` has `ros-jazzy-laser-proc` only in a newer RoboStack
    build (build number 25, which requires `ros2-distro-mutex` 0.19).
    `pixi.lock` uses the older build (build number 21, `ros2-distro-mutex`
    0.16). Moving `laser_proc` to `pixi.toml` means moving the whole ROS
    environment to the newer build first. `ros-jazzy-twist-stamper` is not on
    the channel.

??? info "The 'packages were not found' block in pixi.toml"
    At the end of `[dependencies]`, `pixi.toml` has a commented block that
    starts with `# The following packages were not found:`. When the
    dependency list was created, these dependency names from the workspace
    `package.xml` files had no package with the same name on the channels.
    pixi does not read the block. It is a record:

    | Name | Declared in | Notes |
    | --- | --- | --- |
    | `ros-jazzy-laser-proc` | `hardware/lidar/urg_node2/package.xml` | Built from source through `walkie_external.repos` |
    | `libblosc-dev`, `liblzma-dev`, `libopenvdb-dev` | `navigation/spatio_temporal_voxel_layer/openvdb_vendor/package.xml` | Debian package names. The OpenVDB library comes from the `openvdb` conda package. |
    | `libopenexr-dev` | `navigation/spatio_temporal_voxel_layer/spatio_temporal_voxel_layer/package.xml` | Debian package name |
    | `libqt6-core`, `libqt6gui6t64`, `libqt6statemachine6`, `libqt6widgets6t64`, `qt6-base-dev`, `qt6-scxml-dev` | `navigation/navigation2/nav2_rviz_plugins/package.xml` | Debian package names |

## Python virtual environments next to pixi

The lift homing node and the perception nodes need Python packages from PyPI
that are not in pixi. They use [uv](https://docs.astral.sh/uv/) virtual
environments (venvs). uv is not in pixi either: install it separately on the
computers that need it.

| | Lift homing node | Perception nodes |
| --- | --- | --- |
| Package | `robot_bringup` | `walkie_perception` |
| venv location | `bringup/.venv` | `~/perception-venv` |
| Created by | `pixi run uv` inside `bringup/` (below) | `perception/setup_env.sh`, on the system Python |
| Full setup | [Real Robot](real-robot.md#python-environment) | [Perception](perception.md#what-a-pixi-user-must-do), including a computer with pixi only |

Create the lift node venv inside `bringup/`:

```bash
cd bringup
pixi run uv venv --python python3.12 --system-site-packages
pixi run uv sync
```

!!! warning "Run uv through pixi run for the lift node"
    Inside `pixi run`, uv uses the pixi Python 3.12, the one that runs the
    node, as the venv's base interpreter. Outside pixi, uv picks another
    Python 3.12, such as the system `/usr/bin/python3.12`. With
    `--system-site-packages`, the venv can also import the packages of its
    base interpreter, so a pixi-based venv sees `rclpy` and the other ROS
    packages from pixi.

??? info "What each venv contains and how the nodes find it"
    - **Lift homing node:** TMotorCANControl, python-can, and others
      (`bringup/pyproject.toml`). `bringup/launch/lift_homing.launch.py` adds
      the venv's `site-packages` to the node's `PYTHONPATH`.
    - **Perception nodes:** PyTorch, Ultralytics, Open3D, and others
      (`perception/pyproject.toml`). The venv-based node scripts start with a
      shell line that runs the script again with `$PERCEPTION_VENV`, or
      `~/perception-venv/bin/python3` if it is not set. `PERCEPTION_VENV` is
      the path of the Python executable, not of the venv folder.

## The default and docs environments { #environments }

| Environment | Contents | Size |
| --- | --- | --- |
| `default` | ROS 2 Jazzy, Gazebo, MoveIt, compilers, MuJoCo, and the other build and simulation dependencies | 1276 packages, about 1.7 GB download |
| `docs` | MkDocs and the Material theme. No ROS, and it does not source `install/setup.bash`. | 63 packages, about 70 MB download |

How pixi chooses the environment:

- `pixi run docs` and `pixi run docs-build` use `docs` automatically, because
  those tasks exist only there.
- Everything else uses `default`, unless you pass `-e docs`.
- `pixi install` installs only `default`. Use `pixi install -e docs` or
  `pixi install --all` for the others.

??? info "How the two environments are defined"
    pixi groups packages and tasks into **features**. An **environment** is a
    set of features that is installed together. The `default` environment
    uses the default feature: everything at the top level of `pixi.toml`
    (`[dependencies]`, `[tasks]`, and `[activation]`). The `docs` environment
    uses only the `docs` feature, because it is declared with
    `no-default-feature = true`:

    ```toml title="pixi.toml"
    [environments]
    docs = {features = ["docs"], no-default-feature = true}
    ```

    This keeps `docs` a small download, which keeps the documentation CI fast.
    [3. Add the Documentation Site](../migrations/03-documentation.md) explains
    this setup.

## Known issues

| Issue | Effect | Workaround |
| --- | --- | --- |
| `scripts/bringup/sim_nav_gazebo.sh`, `isaac_sim_bringup.sh`, `real_slam_nav.sh`, and `scripts/map/record_mapping_bag.sh` source `/opt/ros` | With a system ROS Jazzy installed, they mix the two installations | Run the `ros2 launch` commands from the guides directly |
| `.gitignore` ignores `.pixi/config.toml` | A project-level `.pixi/config.toml` is never picked up by Git | `git add -f .pixi/config.toml`, or remove the earlier `.pixi` line from `.gitignore` |
| `realsense2_camera` and `topic_tools` are not in `pixi.toml` | `real_omnibot.launch.py` fails under pixi | [Packages that pixi does not provide](real-robot.md#packages-that-pixi-does-not-provide) |
| `perception/setup_env.sh` builds its venv from `/usr/bin/python3.12`, not the pixi Python | From a plain shell without a system ROS, it stops at its `import rclpy` check | [Perception: Create the venv](perception.md#what-a-pixi-user-must-do) |

??? info "When each helper script sources /opt/ros"
    `scripts/bringup/sim_nav_gazebo.sh`, `scripts/bringup/isaac_sim_bringup.sh`,
    and `scripts/map/record_mapping_bag.sh` source
    `/opt/ros/$ROS_DISTRO/setup.bash` when it exists. pixi sets
    `ROS_DISTRO=jazzy`. `scripts/bringup/real_slam_nav.sh` always sources
    `/opt/ros/<distro>/setup.bash`.

??? info "Why Git ignores .pixi/config.toml"
    `.gitignore` has `.pixi/*` followed by `!.pixi/config.toml`, but an
    earlier `.pixi` line ignores the whole folder. Git does not re-include a
    file inside an ignored folder.
