# walkie-ros

<img width="2048" height="1365" alt="FB_IMG_1783438479888" src="https://github.com/user-attachments/assets/4141053d-d985-4ba4-ae8d-bf04ddfef6bd" />

ROS 2 Jazzy software stack for the Walkie robot (EIC RoboCup 2026): robot description, ros2_control hardware drivers, Gazebo simulation, Nav2 navigation, MoveIt manipulation for the bimanual OpenArm, perception, and the bringup that ties them together.

The workspace is managed with [pixi](https://pixi.sh) and [RoboStack](https://robostack.github.io) Jazzy. pixi installs ROS 2, Gazebo, MoveIt, the compiler toolchain, and all other dependencies into a project-local environment (`.pixi/`) pinned by `pixi.lock`. No system ROS installation or `rosdep` step is required.

## Prerequisites

- Linux on x86_64. The environment is locked for `linux-64`; Ubuntu 24.04 is the tested host.
- [pixi](https://pixi.sh/latest/#installation):

  ```bash
  curl -fsSL https://pixi.sh/install.sh | sh
  ```

- Git and Git LFS. The robot meshes are stored in Git LFS.

  ```bash
  sudo apt install git git-lfs
  git lfs install
  ```

- An SSH key registered on GitHub with access to the EIC-Robocup-2026 organization. The `description` and `navigation` submodules, and the map submodule nested in `navigation`, use SSH URLs.
- Real robot only: [uv](https://docs.astral.sh/uv/getting-started/installation/), for the lift homing node's Python dependencies.

Do not source `/opt/ros/<distro>/setup.bash` in a shell you use with pixi, including from `~/.bashrc`. It puts the system ROS libraries and Python packages ahead of the pixi environment, and packages missing from pixi silently resolve to the system copies.

## Getting started

### 1. Clone

```bash
git clone --recurse-submodules git@github.com:EIC-Robocup-2026/walkie-ros.git
cd walkie-ros
git submodule foreach --recursive git lfs pull
```

The last command makes sure the LFS meshes (`*.dae` under `description/meshes/`, plus the D435 camera mesh under `description/urdf/walkie/depth_camera/`) are downloaded in every submodule. Without them the mesh files are small text pointers, and Gazebo and RViz cannot load the robot model.

The repository root is the colcon workspace: `build/`, `install/`, and `log/` are created next to `pixi.toml` and are git-ignored. Clone it anywhere; a surrounding `ros2_ws/src/` directory is not needed.

To update an existing clone:

```bash
git pull --recurse-submodules
git submodule update --init --recursive
```

### 2. Install the environment

```bash
pixi install
```

This creates `.pixi/envs/default` from `pixi.lock`, using the `robostack-jazzy` and `conda-forge` channels. The first install downloads about 1.7 GB of packages, which take about 8 GB once installed.

### 3. Build

```bash
pixi run build
```

`build` first runs two helper tasks:

- `setup` adds a `libpthread.so` symlink that the conda compiler sysroot is missing.
- `import` runs `vcs import < walkie_external.repos`, which clones source packages that are not published on the `robostack-jazzy` channel (`hardware/lidar/laser_proc` and `tools/twist_stamper`). These directories are git-ignored.

It then runs `colcon build --symlink-install` against the pixi environment. The first build compiles Nav2 from source (`navigation/navigation2`) and takes a while.

To rebuild selected packages only:

```bash
pixi run build-pkg robot_bringup
pixi run build-pkg walkie_description robot_simulation
```

`build-pkg` runs `setup` but not `import`. Because of `--symlink-install`, edits to existing Python, launch, Xacro, and YAML files take effect without a rebuild. New files and C++ changes need one.

To remove all build artifacts (`build/`, `install/`, `log/`):

```bash
pixi run clean
```

## Running commands in the environment

Prefix commands with `pixi run`, or open a shell with the environment activated:

```bash
pixi run ros2 launch robot_simulation gzsim_omnibot.launch.py

pixi shell    # run ros2 commands directly; `exit` to leave
```

Activation sources `install/setup.bash`, so workspace packages are available without manual sourcing. Before the first build, pixi prints `Could not find activation scripts: .../install/setup.bash`; this is expected. A shell opened before a build does not see newly built packages: run `source install/setup.bash` in it or open a new one.

Tasks defined in `pixi.toml` always run from the repository root; any other command runs in the current directory. `pixi task list` shows all tasks.

### Adding dependencies

Add binary packages with `pixi add` and commit both `pixi.toml` and `pixi.lock`:

```bash
pixi add ros-jazzy-<package-name>
```

RoboStack package names use the `ros-jazzy-` prefix with dashes instead of underscores. Packages that are not available on the channel are built from source by adding them to `walkie_external.repos`.

## Middleware (Zenoh)

Walkie uses `rmw_zenoh_cpp` instead of a DDS implementation. The pixi environment includes it but does not select it; the default remains Fast DDS. Set it in every shell that runs ROS nodes. Zenoh nodes discover each other through a router, so run one router per machine:

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
pixi run ros2 run rmw_zenoh_cpp rmw_zenohd    # in its own terminal
```

To connect a development machine to the robot, start the router with the team configuration instead:

```bash
pixi run bash scripts/bringup/connect_walkie_zenoh_router.sh
```

It loads `scripts/bringup/WALKIE_RMW_ZENOH_ROUTER_CONFIG.json5`, which listens on port 7447 and connects to the robot's router at `tcp/10.0.0.6:7447`. The robot uses `ROS_DOMAIN_ID=23`; export the same value in every shell that talks to it.

## Simulation (Gazebo)

Gazebo is the simulator for navigation and manipulation. Set up the middleware as described above, then:

| Command | Starts |
|---|---|
| `pixi run ros2 launch robot_simulation gzsim_omnibot.launch.py` | Gazebo (`aws_small_house` world), the robot with `gz_ros2_control`, the ros_gz bridge, and RViz (`use_rviz:=false` to skip RViz) |
| `pixi run ros2 launch robot_bringup nav_sim_omnibot.launch.py` | The above, plus Nav2 localization and rosbridge |
| `pixi run ros2 launch openarm_bimanual_moveit_config moveit_demo.launch.py` | MoveIt with mock arm hardware, no simulator |
| `pixi run ros2 launch openarm_bimanual_moveit_config moveit_demo.launch.py hardware_type:=gazebo` | MoveIt with its own Gazebo instance |
| `pixi run ros2 launch walkie_description robot_preview_rviz.launch.py` | Robot model preview in RViz |

`scripts/bringup/sim_nav_gazebo.sh` wraps the Nav2 simulation: it starts `rmw_zenohd` if no router is running, then launches `nav_sim_omnibot.launch.py`. The script predates pixi and sources `/opt/ros/<distro>/setup.bash` whenever that file exists, which puts a system ROS installation ahead of the pixi environment. Use it only on machines without `/opt/ros`:

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
pixi run ./scripts/bringup/sim_nav_gazebo.sh
```

On machines with a system ROS installation, start the router and `nav_sim_omnibot.launch.py` directly as shown above.

## Simulation (MuJoCo, in progress)

A MuJoCo model of the robot is generated from the same Xacro description:

```bash
pixi run mjcf         # regenerate description/mujoco/walkie.xml from the Xacro
pixi run mjcf-view    # open description/mujoco/scene.xml in the MuJoCo viewer, at the home keyframe
```

`walkie.xml` is generated and checked in. Rerun `pixi run mjcf` after changing the URDF/Xacro instead of editing the file by hand. `scene.xml` is a hand-written minimal world (floor and light) that includes the robot.

The MuJoCo ros2_control runtime is not wired up yet. Gazebo remains the simulator for navigation and MoveIt for now.

## Real robot

### CAN and USB devices

| Interface | Device | Bus |
|---|---|---|
| `can2` | CubeMars wheel motors | CAN, 1 Mbit/s |
| `can0`, `can1` | OpenArm right and left arm | CAN FD, 1 Mbit/s nominal, 5 Mbit/s data |
| `/dev/ttyDynamixel` | Dynamixel head servo | USB serial |

Install persistent device names once per robot computer. The scripts add udev rules and systemd units that bring each CAN interface up at the correct bitrate whenever its adapter is plugged in:

```bash
sudo ./scripts/setup_device/setup_all_device.sh          # interactive menu
sudo ./scripts/setup_device/setup_all_device.sh all      # all devices
sudo ./scripts/setup_device/setup_device.sh status --alias can2
```

These scripts run with the system shell, not through pixi. To bring a classic CAN interface up manually for the current session:

```bash
sudo ip link set can2 up type can bitrate 1000000
```

### Python environment for the lift node

`bringup/scripts/lift_homing_node.py` depends on TMotorCANControl and python-can, which are not part of the pixi environment. They live in a uv virtual environment at `bringup/.venv`, created once:

```bash
cd bringup
pixi run uv venv --python python3.12 --system-site-packages
pixi run uv sync
```

Running uv through `pixi run` makes the pixi Python the venv's base interpreter, so `--system-site-packages` exposes the pixi packages (including `rclpy`) inside the venv. `lift_homing.launch.py` adds the venv's `site-packages` to the node's `PYTHONPATH` automatically. It locates the venv by resolving the `--symlink-install` symlink back to the source tree; for a non-symlinked install, set `ROBOT_BRINGUP_VENV_SITE` to the venv's `site-packages` path.

### Bringup

Run each command in its own terminal, with `RMW_IMPLEMENTATION=rmw_zenoh_cpp` and `ROS_DOMAIN_ID=23` exported:

```bash
pixi run ros2 run rmw_zenoh_cpp rmw_zenohd
pixi run ros2 launch robot_bringup lift_homing.launch.py
pixi run ros2 launch robot_bringup real_omnibot.launch.py
pixi run ros2 launch robot_navigation localization_Nav2_real.launch.py
pixi run ros2 launch openarm_bimanual_moveit_config moveit_only.launch.py hardware_type:=real_robot
pixi run ros2 launch openarm_bimanual_commander_cpp commander.launch.py
pixi run ros2 launch joy_interface joy_control_smooth.launch.py
```

- `lift_homing.launch.py` drives the lift to its upper hard stop, detects the current spike, and sets the origin. It then accepts `[pos_cm, vel_cm_s, acc_cm_s2]` commands (`Float64MultiArray`) on `lift/cmd`, serves `lift_controller/follow_joint_trajectory` for MoveIt, and publishes `lift/joint_states`. All tuning values are launch arguments; `lift_m_per_rad` is a signed, calibrated ratio and must match the hardware. `mock:=true` runs without CAN and does not need the venv. It provides the topics (`lift/cmd`, `lift/joint_states`) but not the `follow_joint_trajectory` action, so MoveIt cannot move the lift against the mock.
- `real_omnibot.launch.py` starts ros2_control with the real hardware plugins, the lidars, cameras, the OpenArm, and RViz. Switches include `use_zed`, `use_arm`, `use_fake_arm_hardware`, `use_unitree_lidar`, and `use_rviz`. The ZED ROS wrapper (`zed_wrapper`) is not available from pixi because it needs the ZED SDK and CUDA; the physical Docker image provides it. Without it, pass `use_zed:=false`. The launch also always starts `realsense2_camera` (bottom camera) and a `topic_tools` relay for the lift joint state; neither package is in the pixi environment yet, so the launch only runs where they are installed, such as the physical Docker image.

## Tests

```bash
pixi run test                                  # colcon test for every package
pixi run test --packages-select <package>      # selected packages only
pixi run colcon test-result --verbose          # show results and failures
```

`pixi run test` includes the Nav2 packages built from source, so narrowing it with `--packages-select` is much faster.

pytest is pinned below 9 in `pixi.toml`: pytest 9 rejects the pytest plugins shipped by Jazzy's `launch_testing` and `launch_testing_ros` and aborts every pytest run.

To run a package's pytest files directly:

```bash
pixi run python -m pytest description/test/test_description.py description/test/test_mjcf.py
```

Run it after a build: `test_description.py` reads the installed `walkie_description` share directory.

## Docker images

`.github/workflows/build-docker.yml` builds two images on every push to `main` (or on manual dispatch) and publishes them to the GitHub Container Registry:

| Image | Dockerfile | Contents |
|---|---|---|
| `ghcr.io/eic-robocup-2026/walkie-ros/walkie-simulation` | `docker/Dockerfile.simulation` | `osrf/ros:jazzy-desktop-full` base, for simulation |
| `ghcr.io/eic-robocup-2026/walkie-ros/walkie-physical` | `docker/Dockerfile.physical` | `stereolabs/zed` base (ZED SDK, CUDA), for the real robot |

The images are still built with apt, rosdep, and colcon, not pixi. Compose files and `just` recipes for running them are described in [docker/readme.md](docker/readme.md).

## Project structure

| Path | Packages | Purpose |
|---|---|---|
| `bringup/` | `robot_bringup` | Top-level launch files and configuration for the real robot, Gazebo, and Isaac Sim; lift homing node |
| `description/` (submodule) | `walkie_description` | URDF/Xacro model, meshes (Git LFS), ros2_control controller configuration, MuJoCo model |
| `hardware/actuator/` | `cubemars_hardware`, `dynamixel_hardware_interface`, `dynamixel_interfaces`, `dynamixel_sdk`, `openarm_hardware`, `openarm_can` (submodule), `topic_based_ros2_control` (submodule) | ros2_control hardware plugins and actuator drivers |
| `hardware/lidar/` | `urg_node2`, `lakibeam1`, `dual_laser_merger`, `robot_self_filter`, `unitree_lidar_ros2`, `laser_proc` (imported) | Lidar drivers; Hokuyo (front) and Lakibeam (rear) scans merged into `/scan`; point cloud self-filtering |
| `hardware/camera/` | none | RealSense configuration |
| `manipulate/` | `openarm_bimanual_moveit_config`, `openarm_bimanual_commander_cpp`, `my_robot_interfaces` | MoveIt configuration and commander for the bimanual OpenArm |
| `navigation/` (submodule) | `robot_navigation`, Nav2 (`navigation2`, nested submodule), `spatio_temporal_voxel_layer`, `openvdb_vendor` | Nav2 configuration, localization, SLAM, maps |
| `perception/` | `walkie_perception` | Object detection, 3D pose, and grasp pose services on the ZED camera; see [perception/README.md](perception/README.md) |
| `simulation/gazebo/` | `robot_simulation` | Gazebo worlds, launch files, ros_gz bridge configuration |
| `tools/` | `joy_interface`, `walkie_tf`, `walkie_tf_interfaces`, `twist_stamper` (imported) | Joystick teleoperation, TF lookup service, twist stamping |
| `scripts/` | none | `bringup/` launch wrappers and Zenoh router configuration, `setup_device/` udev and CAN setup, `map/` mapping bag recording |
| `docker/` | none | Dockerfiles, compose files, `justfile` |

Packages marked "imported" are cloned by `pixi run import` from `walkie_external.repos`.
