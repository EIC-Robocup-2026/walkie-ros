# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

Full documentation lives in `docs/` (MkDocs, published at https://eic-robocup-2026.github.io/walkie-ros/). Read the relevant page before changing a subsystem; the Migration Log (`docs/migrations/`) records why things are the way they are. When behavior or commands change, update the matching docs page in the same change.

## Environment

The workspace is managed by pixi + RoboStack Jazzy (`pixi.toml`, `pixi.lock`). The repository root is the colcon workspace. Never source `/opt/ros/*/setup.bash`: it shadows the pixi environment. Run ROS commands through `pixi run` (or inside `pixi shell`); activation sources `install/setup.bash`.

`pixi.toml` pins `ros2-distro-mutex = "0.19.*"` (RoboStack build set `_25`) so that `pixi add`/`pixi update` cannot move every ROS package to another build set. Changing the pin requires `pixi install` and `pixi run clean && pixi run build` (`build-pkg` is not enough); see `docs/guides/pixi.md`. Do not `pip install` into the pixi env.

## Build

```bash
pixi install                          # create .pixi/envs/default from pixi.lock
pixi run build-setup [workers]        # initial setup build (default: 1 worker; e.g. pixi run build-setup 3)
pixi run build                        # setup + colcon build --symlink-install
pixi run setup-real                   # pull real-robot hardware repos (ZED wrapper) from walkie_real.repos
pixi run build-pkg <pkg> [<pkg>...]   # rebuild selected packages
pixi run clean                        # remove build/ install/ log/
```

Add dependencies with `pixi add ros-jazzy-<name>` and commit `pixi.toml` and `pixi.lock` together. `pixi.lock` is `merge=binary`: resolve lock conflicts by re-locking, not by hand. Real-robot hardware packages (e.g. ZED wrapper) are managed via `walkie_real.repos` (`pixi run setup-real`).

## Running

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp            # every shell that runs nodes
pixi run ros2 run rmw_zenoh_cpp rmw_zenohd          # one Zenoh router per machine

pixi run ros2 launch robot_simulation gzsim_omnibot.launch.py     # Gazebo
pixi run ros2 launch robot_bringup nav_sim_omnibot.launch.py      # Gazebo + Nav2
pixi run ros2 launch robot_bringup mujoco_omnibot.launch.py       # MuJoCo + controllers + RViz [headless:=true]
pixi run ros2 launch robot_bringup nav_mujoco_omnibot.launch.py   # MuJoCo + Nav2 (blank map, odometry-only, no AMCL)
pixi run ros2 launch robot_bringup real_omnibot.launch.py         # real robot
pixi run ros2 launch robot_bringup lift_homing.launch.py [mock:=true]

pixi run mjcf          # regenerate description/mujoco/walkie.xml from the xacro
pixi run mjcf-view     # open description/mujoco/scene.xml in the MuJoCo viewer
```

Real robot CAN interfaces are set up persistently by `scripts/setup_device/` (udev rules + systemd units): `can2` = CubeMars wheels and the lift motor, `can0`/`can1` = OpenArm (CAN FD). The lift node needs the uv venv in `bringup/` (`pixi run uv venv --python python3.12 --system-site-packages && pixi run uv sync`); `lift_homing.launch.py` adds it to `PYTHONPATH`.

## Tests

```bash
pixi run test --packages-select <pkg>
pixi run colcon test-result --verbose
pixi run python -m pytest description/test/test_description.py description/test/test_mjcf.py   # after a build
```

pytest is pinned below 9 (pytest 9 rejects the Jazzy launch_testing plugins). `description/test/test_mjcf.py` fails when `walkie.xml` is stale: rerun `pixi run mjcf` after changing the xacro and commit the regenerated file. `test_description.py` checks every ros2_control mode against its controller YAML (`CONTROLLER_PAIRINGS`); known mismatches are strict xfails.

## Docs

```bash
pixi run docs          # serve http://127.0.0.1:8000 (separate `docs` environment, no ROS)
pixi run docs-build    # mkdocs build --strict; broken links and anchors fail
```

New pages must be added to `nav` in `mkdocs.yml`. Admonition bodies are indented 4 spaces. Pages are answer-first: commands, required steps, and breaking warnings stay visible; background, internals, and long tables go in closed `??? info "Descriptive title"` blocks (see "Writing conventions" in `docs/migrations/03-documentation.md`). Keep linked headings outside collapsibles; when renaming one, keep its anchor with `{ #old-anchor }`.

## Architecture

### Packages

| Directory | Package(s) | Role |
|-----------|-----------|------|
| `bringup/` | `robot_bringup` | Top-level launch files and config; lift homing node (TMotorCANControl), ZED watchdog |
| `description/` (submodule) | `walkie_description` | URDF/Xacro, meshes (Git LFS), ros2_control controller YAMLs, MuJoCo model and generator |
| `hardware/actuator/` | `cubemars_hardware`, `dynamixel_hardware_interface`, `dynamixel_interfaces`, `dynamixel_sdk`, `openarm_hardware`, `openarm_can` (submodule), `topic_based_ros2_control` (submodule) | ros2_control hardware plugins and drivers |
| `hardware/lidar/` | `urg_node2`, `lakibeam1`, `dual_laser_merger`, `robot_self_filter`, `unitree_lidar_ros2` | Hokuyo (front) + Lakibeam (rear) merged into `/scan`; point cloud self-filter |
| `manipulate/` | `openarm_bimanual_moveit_config`, `openarm_bimanual_commander_cpp`, `my_robot_interfaces` | MoveIt for the bimanual OpenArm + lift; commander action/service node |
| `navigation/` (submodule) | `robot_navigation`, Nav2 (`navigation/navigation2`, upstream main), `spatio_temporal_voxel_layer`, `openvdb_vendor` | Nav2 config, localization, SLAM, maps |
| `perception/` | `walkie_perception` | Detection, 3D pose and grasp services on the ZED camera (own venv, see `perception/README.md`) |
| `simulation/gazebo/` | `robot_simulation` | Gazebo worlds, launch, ros_gz bridge config |
| `tools/` | `joy_interface`, `walkie_tf`, `walkie_tf_interfaces` | Joystick teleop, TF lookup service |

### Key design points

**ros2_control**: All actuators go through ros2_control. The mode is the xacro arg `ros2_control` in `description/robots/gz_walkie.urdf.xacro` (`real_robot`, `mock`, `gazebo`, `topic_base`, `mujoco`); each mode pairs with a YAML in `description/config/ros2_controller/` (`real_controllers.yaml`, `sim_controllers.yaml`, `isaac_controllers.yaml`, `mujoco_controllers.yaml`). The joint state broadcaster is named `joint_broad`. Omni base: `omni_wheel_drive_controller`, `robot_radius` 0.185.

**MuJoCo**: `description/scripts/build_mjcf.py` expands the xacro with `ros2_control:=mujoco` and applies an MjSpec overlay. Omni wheels are capsules with explicit `<pair>` contacts against a plane floor named `floor` (anisotropic friction only works capsule-vs-plane). Collisions are off per geom via contype/conaffinity classes; never use the global `contact="disable"` flag (it also disables the wheel pairs). The ros2_control runtime, `mujoco_ros2_control` 0.1.2, is in the pixi lock (`mujoco = "3.15.*"` must match its `mujoco_vendor`); it runs as its own `ros2_control_node` (package `mujoco_ros2_control`), started by `bringup/launch/mujoco_omnibot.launch.py` (Nav2: `nav_mujoco_omnibot.launch.py`). Camera and lidar plugin parameters are in `bringup/config/mujoco/mujoco_plugins.yaml`; `mujoco_omnibot.launch.py` sets the `dual_laser_merger` parameter `publish_on_empty_cloud: True` so `/scan` publishes in the empty scene (the real robot keeps the default `false`). The head 3D lidar (Unitree 4D L2) is the sensor `unitree_l2` of the MuJoCo extension `mujoco.plugin.lidar` (`ros-jazzy-mujoco-3d-lidar`, in `pixi.toml`), written into `walkie.xml` by the generator, so every load of `walkie.xml` needs the extension (use pixi's `mujoco`, not one from pip). `Mujoco3dLidarPlugin` publishes it on `/unilidar/cloud` (organized, NaN for no hit, includes the robot); the launch runs `robot_self_filter` to `/unilidar/cloud/filtered`. Nav2 in simulation does not read it. The 2D lidars stay on `RangefinderLidarPlugin` because the extension cannot exclude the robot's own geoms. See `docs/migrations/02-mujoco/`.

**Nav2**: `navigation/navigation2` is upstream main. Every MPPI params file sets `FollowPath.Omni.plugin: "mppi::OmniMotionModel"` (required since the bump; new params files need it too). nav2_bringup's `navigation_launch.py` and `localization_launch.py` start no lifecycle manager, so the walkie launch files that include them start `lifecycle_manager_nav2` themselves. Robot launches wrap the Nav2 include in `SetRemap(cmd_vel -> cmd_vel_controller)`, so commands pass `velocity_smoother` and `collision_monitor` before `twist_mux` (`cmd_vel_nav`); joystick and keyboard bypass the monitor. Use `use_composition:=False` (composed `bringup_launch.py` can deadlock on Jazzy). The monitor's stop is `VelocityPolygonStop` (eight direction bands plus a rotation polygon, in the sim, real and restaurant params), so Nav2 can drive away after a stop. `FootprintApproach` uses the static physical footprint (the padded `published_footprint` froze Nav2). The progress checker allows 8 s for 0.5 m because `PolygonSlow` caps Nav2 at 0.12 m/s. Under Fast DDS the real and restaurant params fail to configure `controller_server` (`voxel_layer` and STVL both publish `local_costmap/voxel_grid`); they start under `rmw_zenoh_cpp`. `nav2_loopback.launch.py` activates only after an initial pose on `/initialpose`. See `docs/migrations/05-nav2-repair.md` and `docs/guides/navigation.md`.

**Lift homing**: `bringup/scripts/lift_homing_node.py` homes against the upper hard stop (current spike), then shapes commands with a software trapezoidal trajectory. `lift_homing_mock_node.py` provides the topics without CAN but not the `follow_joint_trajectory` action.

**Middleware**: `rmw_zenoh_cpp`. Team router config: `scripts/bringup/WALKIE_RMW_ZENOH_ROUTER_CONFIG.json5`, started by `scripts/bringup/connect_walkie_zenoh_router.sh`.

**Submodules**: `description/` and `navigation/` (with nested `navigation2` and `map`) use SSH URLs. Clone with `--recurse-submodules`, then `git submodule foreach --recursive git lfs pull`.

**CI**: `.github/workflows/ci.yml` runs on pushes to `main` and on every pull request. It checks out all submodules with their LFS files, installs the pixi environment from `pixi.lock` (`--locked`), builds the whole workspace including Nav2 (ccache, `BUILD_TESTING=OFF` except for the tested packages), and runs the tests of `walkie_description`, `openarm_bimanual_moveit_config` and `topic_based_ros2_control` without their lint tests. Lint tests of the first-party packages run but do not fail the job. `.github/workflows/docs.yml` publishes the docs site from `main`. CI no longer builds the Docker images in `docker/`. See `docs/guides/testing.md#ci`.

### Lift homing launch arguments

| Argument | Default | Notes |
|----------|---------|-------|
| `mock` | `false` | `true` = no CAN, instant homing |
| `motor_id` | `16` | CAN node ID |
| `homing_erpm` | `-8500` | ERPM toward upper hard stop |
| `lift_m_per_rad` | `-0.00012406` | Signed kinematic ratio, must match the hardware |
| `topic_position_commands` | `lift/cmd` | Input (`Float64MultiArray`: `[pos_cm, vel_cm_s, acc_cm_s2]`) |
| `topic_joint_states` | `lift/joint_states` | Output state topic |
