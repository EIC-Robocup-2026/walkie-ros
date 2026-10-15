# Working with Pixi

Use this page to run commands in the pixi environment, build the workspace, and
add dependencies. It assumes you have finished
[Getting Started](../getting-started.md).

??? info "What pixi, conda packages, channels, RoboStack, and build sets are"
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
    - A **build set** is one complete rebuild of all RoboStack ROS packages.
      Its number is the suffix of the package build strings, for example `_25`
      in `ros2-rmw-zenoh-cpp-0.2.11-np2py312h815c85b_25`. Every `ros2-`
      package depends on the matching version of the `ros2-distro-mutex`
      package, so one environment holds the ROS packages of one build set.
      See [The RoboStack build set](#the-robostack-build-set).
    - In build set `_25`, a `ros-jazzy-<name>` package contains no files. It
      is an alias that depends on `ros2-<name>`, the package with the files.
      `pixi.toml` uses the `ros-jazzy-` names, which still resolve, and
      `pixi list` shows both names. A few aliases, such as
      `ros-jazzy-ros-base`, still carry the build number `_21`. The `ros2-`
      packages they depend on are from build set `_25`.

    | Channel | What it provides |
    | --- | --- |
    | `https://prefix.dev/robostack-jazzy` | ROS 2 Jazzy packages |
    | `https://prefix.dev/conda-forge` | Everything else: Python, compilers, CMake, common libraries, MuJoCo |

??? info "Files and folders that pixi uses"
    | File or folder | What it is | In Git |
    | --- | --- | --- |
    | `pixi.toml` | The workspace manifest: channels, dependencies, tasks, environments. You edit this file. | Yes |
    | `pixi.lock` | The exact version of every package in every environment. pixi writes it. Never edit it by hand. | Yes |
    | `.pixi/envs/default/` | The installed ROS environment, about 7 GB | No |
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
pixi list ros2-rmw             # only packages whose name matches
pixi list ros2-distro-mutex    # the build set: version 0.19.0, build jazzy_25
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
pixi run build-setup                                    # initial setup: build sequentially (1 worker)
pixi run build-setup 3                                  # initial setup with 3 parallel workers
pixi run build                                          # build the whole workspace
pixi run build-pkg robot_bringup                        # rebuild selected packages
pixi run build-pkg walkie_description robot_simulation
pixi run test --packages-select walkie_description
pixi run colcon test-result --verbose
pixi task list                                          # all tasks with their descriptions
pixi run -n build-setup                                 # dry run: show what the task would run
```

Use the `build-setup`, `build`, and `build-pkg` tasks instead of typing `colcon build`
yourself, so the flags are always the same.

| Task | What it does |
| --- | --- |
| `build-setup` | Runs `setup` and `import`, then builds the workspace sequentially (default: 1 worker; e.g. `pixi run build-setup 3`) |
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
    | `build-setup` | default | `colcon build --symlink-install --parallel-workers {{ workers }} --cmake-args -DCMAKE_PREFIX_PATH=$CONDA_PREFIX` (default: 1) |
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
| The `ros2-distro-mutex` pin in `pixi.toml`, for example through a `git pull`. `pixi.lock` is then on another RoboStack build set. | Yes, everything: `pixi install`, then `pixi run clean && pixi run build`. `build-pkg` is not enough. See [Changing the build set](#changing-the-build-set). |

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
   environment again, updates `pixi.lock`, and installs the package. It fails
   for a ROS package that is not in the pinned
   [build set](#the-robostack-build-set).

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

A branch that is still on an older build set, such as `feat/Pixi` on `_21`,
conflicts on `pixi.lock` when it is merged with a branch on `_25`. Resolve it
the same way. `pixi lock` solves for the `ros2-distro-mutex` pin in the merged
`pixi.toml`. If that moves the lock to another build set, run `pixi install`,
then `pixi run clean && pixi run build`.

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

Some ROS packages are not published on `robostack-jazzy`, or were not when the
workspace moved to pixi. They are cloned as source code and built with the
rest of the workspace. `walkie_external.repos` lists them in the
[vcstool](https://github.com/dirk-thomas/vcstool) format.

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
    Build set `_25` has `ros-jazzy-laser-proc` 1.0.2, but the workspace still
    clones and builds `laser_proc` from source. Moving it to `pixi.toml` means
    adding `ros-jazzy-laser-proc`, removing its entry from
    `walkie_external.repos`, and deleting `hardware/lidar/laser_proc` on every
    computer. colcon still builds a clone that is left in place, and that
    build shadows the conda package. `ros-jazzy-twist-stamper` is not on the
    channel.

??? info "The 'packages were not found' block in pixi.toml"
    At the end of `[dependencies]`, `pixi.toml` has a commented block that
    starts with `# The following packages were not found:`. When the
    dependency list was created, these dependency names from the workspace
    `package.xml` files had no package with the same name on the channels.
    pixi does not read the block. It is a record:

    | Name | Declared in | Notes |
    | --- | --- | --- |
    | `libblosc-dev`, `liblzma-dev`, `libopenvdb-dev` | `navigation/spatio_temporal_voxel_layer/openvdb_vendor/package.xml` | Debian package names. The OpenVDB library comes from the `openvdb` conda package. |
    | `libopenexr-dev` | `navigation/spatio_temporal_voxel_layer/spatio_temporal_voxel_layer/package.xml` | Debian package name |
    | `libqt6-core`, `libqt6gui6t64`, `libqt6statemachine6`, `libqt6widgets6t64`, `qt6-base-dev`, `qt6-scxml-dev` | `navigation/navigation2/nav2_rviz_plugins/package.xml` | Debian package names |

    `ros-jazzy-laser-proc` (declared in `hardware/lidar/urg_node2/package.xml`)
    was in this block until the move to build set `_25`. It now has its own
    commented line above the block, because the package is on the channel but
    is still built from source.

## The RoboStack build set

`pixi.lock` uses RoboStack build set `_25`. The first entry of
`[dependencies]` pins it through the `ros2-distro-mutex` package:

```toml title="pixi.toml"
ros2-distro-mutex = "0.19.*"
```

`pixi list ros2-distro-mutex` shows the build set of the installed
environment: version 0.19.0, build `jazzy_25`.

- `pixi add` and `pixi update` stay inside build set `_25`. A ROS package
  that exists only in another build set cannot be added until the pin
  changes.
- A new build set changes every ROS package and needs a full rebuild. See
  [Changing the build set](#changing-the-build-set).
- Build set `_25` requires OpenCV 5.0 (`libopencv`, `py-opencv`). apt Jazzy
  and the [Docker images](docker.md) use Ubuntu's OpenCV 4.6, so code that
  uses OpenCV must work with both versions. See
  [Perception: OpenCV under pixi](perception.md#opencv-under-pixi).

??? info "Why the mutex is pinned"
    RoboStack rebuilds every ROS package for a new build set. So far, it has
    stopped publishing to a build set once a newer one exists. This is an
    observed pattern, not a documented RoboStack policy. The packages of older
    build sets have so far stayed downloadable. New ROS packages and new
    versions therefore appear only in the newest build set.

    Without the pin, `pixi update`, or `pixi add` of a package that only the
    newest build set has, solves the whole environment again for the newest
    build set. Every ROS package changes, and the workspace needs a clean
    rebuild. Before the pin, `pixi update` and
    `pixi add ros-jazzy-mujoco-ros2-control` both did this.
    [4. Move to RoboStack Build Set _25](../migrations/04-robostack-build-25.md)
    records the move to `_25` and the pin.

### Entries that depend on the build set

| Entry in `pixi.toml` | Why |
| --- | --- |
| `ros2-distro-mutex = "0.19.*"` | Keeps the environment in build set `_25` |
| `mujoco = "3.15.*"` | Must match the libmujoco 3.15.0 that `ros-jazzy-mujoco-vendor` 0.1.0 is built against |
| `ros-jazzy-mujoco-ros2-control = ">=0.1.2"` | The MuJoCo ros2_control runtime. Version 0.1.x exists only in build set `_25`. See [MuJoCo Simulation](simulation-mujoco.md#run-with-ros-2). |
| `ros-jazzy-mujoco-3d-lidar = ">=0.1.2,<0.2"` | The MuJoCo extension of the simulated head 3D lidar, which every load of `walkie.xml` needs. Version 0.1.2 exists only in build set `_25` and is built against libmujoco 3.15.0. See [2.8 Head 3D Lidar](../migrations/02-mujoco/08-3d-lidar.md#packaged-extension). |
| `ros-jazzy-rmw-zenoh-cpp = ">=0.2.11,<0.3"` | 0.2.11 fixes a lost-wakeup bug of 0.2.10. See [Middleware (Zenoh)](middleware.md#turn-on-zenoh-in-every-terminal). |
| `python-graphviz = "*"` | Provides the Python `graphviz` module for `ros2 control view_controller_chains`. Without it, the verb fails with `KeyError: 'view_controller_chains'`. A build does not catch this. See [One break and its fix](../migrations/04-robostack-build-25.md#one-break-and-its-fix). |

### Changing the build set

Change the build set on purpose, in its own commit. These are the steps used
for build set `_25`:

1. Find the mutex version of the new build set with
   `pixi search ros2-distro-mutex`. Set the pin in `pixi.toml` to it, and
   update the comment above the pin.
2. Check the other
   [entries that depend on the build set](#entries-that-depend-on-the-build-set).
   For example, `mujoco` must match the libmujoco that the new
   `mujoco_vendor` is built against.
3. Run `pixi lock`. Compare the old and new versions of the libraries that
   the workspace uses. Build set `_25`, for example, moved OpenCV from 4.13
   to 5.0.
4. Install the new environment and rebuild everything. `build-pkg` is not
   enough: the CMake caches in `build/` point at the old library versions,
   and libraries can change their ABI between build sets.

    ```bash
    pixi install
    pixi run clean && pixi run build
    ```

    [After pulling this change](../migrations/04-robostack-build-25.md#after-pulling-this-change)
    records how long the full build took for build set `_25`.

5. Run the tests (see [Tests](testing.md)). `test_generated_model_is_up_to_date`
   in `test_mjcf.py` also catches a MuJoCo version that writes a different
   `walkie.xml`.
6. Commit `pixi.toml` and `pixi.lock` together. Record the move in the
   [Migration Log](../migrations/index.md), with what was checked and what
   was not, as in
   [4. Move to RoboStack Build Set _25](../migrations/04-robostack-build-25.md).
7. Tell the team. After they pull the change, every computer runs
   `pixi install`, then `pixi run clean && pixi run build`.

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

!!! warning "Do not pip install into the pixi environment"
    The default environment contains `pip`, `virtualenv`, `pip-compile`, and
    similar tools in `.pixi/envs/default/bin`, because `mujoco_ros2_control`
    depends on them. `pixi run pip install ...` finds this `pip` and installs
    into `.pixi/envs/default/`. pixi does not track such packages: they are
    not in `pixi.lock`, and no other computer gets them. Add conda packages
    with `pixi add`, and put PyPI packages in a venv.

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
| `default` | ROS 2 Jazzy, Gazebo, MoveIt, compilers, MuJoCo, and the other build and simulation dependencies | 1179 packages, about 1.7 GB download |
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
