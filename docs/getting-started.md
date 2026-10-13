# Getting Started

Follow these steps to go from an empty computer to a running Walkie simulation.
To learn why the workspace is set up this way, read the
[Migration Log](migrations/index.md) afterwards.

## 1. Check the requirements

| Requirement | Why it is needed |
| --- | --- |
| Linux on x86_64 (a normal Intel or AMD computer). Ubuntu 24.04 is the tested host. | The environment is locked for `linux-64` only. |
| Git and Git LFS | The robot meshes are stored in Git LFS. |
| An SSH key on your GitHub account | The `description` and `navigation` submodules, and the map submodule inside `navigation`, use SSH URLs. |
| [uv](https://docs.astral.sh/uv/getting-started/installation/), only for the lift node and the perception nodes | They need extra Python packages from PyPI. See [Working with Pixi](guides/pixi.md#python-virtual-environments-next-to-pixi). |

You do **not** install ROS 2 yourself. pixi installs ROS 2 Jazzy, Gazebo,
MoveIt, the compilers, and the other build and simulation dependencies into
the project folder.

!!! warning "Do not source /opt/ros"
    If your `~/.bashrc` contains a line like `source /opt/ros/jazzy/setup.bash`,
    remove it and open a new terminal. A system ROS mixed into the pixi
    environment makes things work on your computer and fail on a teammate's.
    See [Working with Pixi](guides/pixi.md#never-source-optros-in-a-pixi-shell).

## 2. Install pixi and Git LFS

```bash
curl -fsSL https://pixi.sh/install.sh | sh
```

This installs pixi, the tool that installs ROS 2 and everything else for this
project. You do this once per computer. Close and reopen your terminal, then
check that it works:

```bash
pixi --version
```

Install Git LFS and turn it on for your user:

```bash
sudo apt install git git-lfs
git lfs install
```

`git lfs install` registers Git LFS in your Git configuration, once per
computer. Without it, Git downloads small text "pointer" files instead of the
real meshes.

## 3. Get the code

```bash
git clone --recurse-submodules git@github.com:EIC-Robocup-2026/walkie-ros.git
cd walkie-ros
git submodule foreach --recursive git lfs pull
```

This clones the repository with all its submodules, then downloads any Git LFS
mesh that is still a pointer file. You can clone anywhere. The repository root
is the colcon workspace, so you do not need a `ros2_ws/src/` folder around it.

??? info "What the clone commands do, and the list of submodules"
    `--recurse-submodules` also clones the submodules, and the submodules
    inside them. The repository has these submodules:

    | Path | Contents |
    | --- | --- |
    | `description/` | Robot model (`walkie_description`), meshes, MuJoCo model |
    | `navigation/` | Nav2 configuration (`robot_navigation`), with `navigation/navigation2/` (Nav2 source) and `navigation/walkie-navigation/map/` (maps) inside it |
    | `hardware/actuator/openarm/openarm_can/` | CAN library for the OpenArm |
    | `hardware/actuator/topic_based_ros2_control/` | ros2_control plugin |

    `git submodule foreach --recursive git lfs pull` runs `git lfs pull` inside
    every submodule. The `*.dae` meshes in `description/` are stored in Git
    LFS. If Git LFS was not set up when you cloned, these files stay small text
    pointers, and Gazebo and RViz cannot load parts of the robot. The command
    downloads anything that is missing. When all files are already there, it
    does nothing.

To update an existing clone later:

```bash
git pull --recurse-submodules
git submodule update --init --recursive
git submodule foreach --recursive git lfs pull
```

!!! warning "If the pull changed the ros2-distro-mutex pin"
    A changed `ros2-distro-mutex` pin in `pixi.toml` means the environment
    moved to another RoboStack build set. Install it and rebuild everything;
    `pixi run build-pkg` is not enough:

    ```bash
    pixi install
    pixi run clean && pixi run build
    ```

    See [What needs a rebuild](guides/pixi.md#what-needs-a-rebuild).

## 4. Install the environment

```bash
pixi install
```

This installs the exact package versions listed in `pixi.lock` into
`.pixi/envs/default/`. Expect a download of about 1.7 GB. The installed
environment takes about 7 GB of disk space.

??? info "Where pixi downloads from and what it stores"
    The packages come from the `robostack-jazzy` and `conda-forge` channels.
    Outside the project folder, pixi only keeps a download cache (in
    `~/.cache/rattler/` on Linux).

## 5. Build the workspace

```bash
pixi run build-setup
```

This builds every package in the workspace with colcon. On initial setup,
`pixi run build-setup` compiles packages one by one (`--parallel-workers 1` by default)
to prevent high parallel compilation memory usage from triggering out-of-memory crashes
(the workspace includes 46 Nav2 packages from `navigation/navigation2/`).

If your computer has enough RAM, you can specify more parallel workers (e.g.
`pixi run build-setup 3`) or use `pixi run build` for default parallel compilation.
Later builds are much faster, because only changed files are compiled again.

After the build, `pixi run` and `pixi shell` source `install/setup.bash` for
you, so ROS finds the workspace packages. Before the first build, pixi prints
this message. It is expected:

```text
WARN Could not find activation scripts: .../install/setup.bash
```

??? info "What the build tasks run, and where the output goes"
    `build-setup` and `build` are pixi tasks. They run two helper tasks first:

    | Task | What it does |
    | --- | --- |
    | `setup` | Adds a `libpthread.so` symlink that the conda compiler sysroot is missing. |
    | `import` | Runs `vcs import < walkie_external.repos`. This clones `hardware/lidar/laser_proc` and `tools/twist_stamper`, two source packages that are not installed from the pixi channels. |
    | `build-setup` | Runs `colcon build --symlink-install --parallel-workers 1` (or custom workers, e.g. `pixi run build-setup 3`). |
    | `build` | Runs `colcon build --symlink-install` against the pixi environment with default parallel workers. |

    `build/`, `install/`, and `log/` are created next to `pixi.toml` and are
    ignored by Git.

## 6. Run the simulation

Walkie uses the Zenoh middleware (`rmw_zenoh_cpp`) instead of the default DDS.
Zenoh needs a router process, and the `RMW_IMPLEMENTATION` variable in every
terminal that runs ROS nodes.

In the first terminal, start the Zenoh router and leave it running:

```bash
pixi run ros2 run rmw_zenoh_cpp rmw_zenohd
```

In a second terminal, start the Gazebo simulation:

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
pixi run ros2 launch robot_simulation gzsim_omnibot.launch.py
```

This starts Gazebo with the `aws_small_house` world, the robot with its
controllers, the ros_gz bridge, and RViz. Add `use_rviz:=false` to skip RViz.

To run the MuJoCo simulation instead, start this in the second terminal:

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
pixi run ros2 launch robot_bringup mujoco_omnibot.launch.py
```

This opens the MuJoCo viewer with the robot on an empty floor, and starts its
controllers and RViz. Add `headless:=true` to skip the viewer window. See
[MuJoCo Simulation](guides/simulation-mujoco.md#run-with-ros-2).

!!! tip "Quick check without ROS"
    `pixi run mjcf-view` opens the MuJoCo model of the robot in the MuJoCo
    viewer. It reads the model files from the source tree, so it works even
    before the first build. See [MuJoCo Simulation](guides/simulation-mujoco.md).

## 7. Preview the documentation (optional)

```bash
pixi run docs
```

Open <http://127.0.0.1:8000> in your browser. The page reloads by itself when
you save a file in `docs/`. Press ++ctrl+c++ to stop. The first run installs a
separate, small `docs` environment (about 70 MB download, no ROS).

To add or change a page, read
[Writing a new page](migrations/03-documentation.md#writing-a-new-page).

## Next

The [Guides](guides/index.md) list every task guide. Start with these:

- [Working with Pixi](guides/pixi.md): `pixi run`, `pixi shell`, tasks, and
  adding dependencies.
- [Middleware (Zenoh)](guides/middleware.md): the router, `ROS_DOMAIN_ID`, and
  connecting to the real robot.
- [Gazebo Simulation](guides/simulation-gazebo.md) and
  [MuJoCo Simulation](guides/simulation-mujoco.md): the other simulation launch
  files, including Nav2.
- [Real Robot](guides/real-robot.md): device setup and bringup on the physical
  robot.
- [Troubleshooting](troubleshooting.md): known errors and their fixes.
- [Migration Log](migrations/index.md): how the workspace got the way it is.
