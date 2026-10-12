# Project Structure

**Goal:** find where each package lives, see which code the team wrote, and
see which code comes from other projects.

The repository is one colcon workspace. `pixi run build` runs colcon from the
repository root, so colcon finds every package below it. Most packages have a
`package.xml`. colcon also builds a plain CMake folder without one
(`unitree_lidar_sdk`). Folders with a `COLCON_IGNORE` file are skipped. See
[Working with Pixi](../guides/pixi.md) for the build tasks.

## Top-level layout

| Path | Contents |
| --- | --- |
| `bringup/` | `robot_bringup`: launch files and config that start the real robot and other setups |
| `description/` | Git submodule. `walkie_description`: the robot model, controller configs, and the MuJoCo model |
| `docker/` | Dockerfiles and compose files for the simulation and physical images |
| `docs/` | This documentation site |
| `hardware/` | Drivers: ros2_control plugins for the actuators, lidar drivers, one camera config |
| `manipulate/` | MoveIt config, arm commander, and interfaces for the two OpenArm arms |
| `navigation/` | Git submodule. Nav2 built from source, the Walkie navigation package, and maps |
| `perception/` | `walkie_perception`: object detection and grasp pose services |
| `scripts/` | Shell and Python helpers: bringup shortcuts, device setup, mapping |
| `simulation/gazebo/` | `robot_simulation`: Gazebo worlds and launch files |
| `tools/` | Teleoperation, a TF lookup service, and one imported tool |

These folders are created by tools and are not in git:

| Path | Created by | Notes |
| --- | --- | --- |
| `build/`, `install/`, `log/` | colcon | `pixi run clean` deletes all three |
| `.pixi/` | pixi | The environment with ROS 2 and every dependency |
| `hardware/lidar/laser_proc/`, `tools/twist_stamper/` | `pixi run import` | See [External repositories](#external-repositories) |

## Packages

The build type comes from `<build_type>` in each `package.xml`. Many
`ament_cmake` packages here contain only Python launch files and scripts.
CMake installs them, so the build type does not tell you the language.

### bringup/

| Path | Package | Build type | Purpose |
| --- | --- | --- | --- |
| `bringup/` | `robot_bringup` | ament_cmake | Launch files for the real robot (`real_omnibot.launch.py`), Isaac Sim (`isaac_omnibot.launch.py`), Gazebo with Nav2 (`nav_sim_omnibot.launch.py`), lift homing, the dual lidar, OpenArm, octomap mapping, and a ZED test (`test_zed.launch.py`). Python nodes in `scripts/`: lift homing (real and mock), ZED watchdog, head touch beep, OpenArm joint state reader. Config in `config/` for cameras, lidars, OpenArm, RViz, and twist_mux. A udev rule for ZED USB reset in `udev/`. |

`bringup/pyproject.toml` and `bringup/uv.lock` describe a separate uv
environment for the lift motor (TMotorCANControl, python-can). It is not part of
the pixi environment. See [Real Robot](../guides/real-robot.md).

### description/

| Path | Package | Build type | Purpose |
| --- | --- | --- | --- |
| `description/` | `walkie_description` | ament_cmake | URDF/Xacro model, meshes, ros2_control controller configs, the robot_state_publisher launch, and the generated MuJoCo model. See [Robot Description](robot-description.md). |

### hardware/

| Path | Package | Build type | Purpose |
| --- | --- | --- | --- |
| `hardware/actuator/cubemars_hardware_interface/` | `cubemars_hardware` | ament_cmake | ros2_control plugin for CubeMars AK motors over CAN. Drives the four wheels on the real robot. |
| `hardware/actuator/dynamixel/dynamixel_hardware_interface/` | `dynamixel_hardware_interface` | ament_cmake | ros2_control plugin for Dynamixel servos. Drives the head servo. |
| `hardware/actuator/dynamixel/dynamixel_interfaces/` | `dynamixel_interfaces` | ament_cmake | Messages and services for Dynamixel control |
| `hardware/actuator/dynamixel/DynamixelSDK/ros/dynamixel_sdk/` | `dynamixel_sdk` | ament_cmake | ROBOTIS Dynamixel SDK for ROS 2 |
| `hardware/actuator/dynamixel/DynamixelSDK/ros/dynamixel_sdk_custom_interfaces/` | `dynamixel_sdk_custom_interfaces` | ament_cmake | SDK example interfaces |
| `hardware/actuator/dynamixel/DynamixelSDK/ros/dynamixel_sdk_examples/` | `dynamixel_sdk_examples` | ament_cmake | SDK examples |
| `hardware/actuator/openarm/openarm_can/` | `openarm_can` | cmake | Git submodule. SocketCAN library for the OpenArm motors. Plain CMake, not ament. |
| `hardware/actuator/openarm/openarm_hardware/` | `openarm_hardware` | ament_cmake | ros2_control plugin `openarm_hardware/OpenArm_v10HW` for the arms |
| `hardware/actuator/topic_based_ros2_control/` | `topic_based_ros2_control` | ament_cmake | Git submodule. ros2_control plugin that sends commands and reads states over topics. Used with Isaac Sim. |
| `hardware/lidar/urg_node2/` | `urg_node2` | ament_cmake | Hokuyo driver. Walkie uses it for the front lidar. |
| `hardware/lidar/Lakibeam_ROS2_Driver/` | `lakibeam1` | ament_cmake | Lakibeam driver. Walkie uses it for the back lidar. |
| `hardware/lidar/dual_laser_merger/` | `dual_laser_merger` | ament_cmake | Merges the front and back scans into one scan |
| `hardware/lidar/robot_self_filter/` | `robot_self_filter` | ament_cmake | Removes the robot body from point clouds |
| `hardware/lidar/unilidar_sdk2/unitree_lidar_ros2/src/unitree_lidar_ros2/` | `unitree_lidar_ros2` | ament_cmake | Unitree lidar driver (GPLv3) |
| `hardware/lidar/unilidar_sdk2/unitree_lidar_sdk/` | `unitree_lidar_sdk` | cmake (no `package.xml`) | Unitree SDK example programs (`set_ip_address`, `set_to_udp_mode`, `set_to_serial_mode`, `example_lidar_udp`, `example_lidar_serial`), linked against the prebuilt `libunilidar_sdk2.a`. The build writes them to `bin/` in the source folder, which Git ignores. |
| `hardware/lidar/laser_proc/` | `laser_proc` | ament_cmake | Not in git. Imported by `pixi run import`. `urg_node2` depends on it. |
| `hardware/camera/realsense/config/` | none | none | One RealSense JSON preset. No package. |

The front and back lidar assignment comes from
`bringup/config/lidar/dual_lidar_config.yml`: `urg_node2` publishes
`front_lidar` in frame `front_lidar_link`, and the Lakibeam driver publishes
`back_lidar` in frame `back_lidar_link`.

### manipulate/

| Path | Package | Build type | Purpose |
| --- | --- | --- | --- |
| `manipulate/my_robot_interfaces/` | `my_robot_interfaces` | ament_cmake | Actions (`GoToPose`, `GoToHome`, `ControlGripper`, `SetJointPosition`, and others), the `PoseCommand` message, and services (`GetEEPose`, `GetJointStates`, and others) |
| `manipulate/openarm_bimanual_commander_cpp/` | `openarm_bimanual_commander_cpp` | ament_cmake | C++ commander for the left and right arms, built on the MoveIt planning interface |
| `manipulate/openarm_bimanual_moveit_config/` | `openarm_bimanual_moveit_config` | ament_cmake | MoveIt config for the two arms. Generated by the MoveIt Setup Assistant (Enactic), then changed for Walkie. |

See [Manipulation](../guides/manipulation.md).

### navigation/

`navigation/` is a submodule. It holds three groups of packages:

| Path | Package | Build type | Purpose |
| --- | --- | --- | --- |
| `navigation/walkie-navigation/` | `robot_navigation` | ament_cmake | Nav2 parameters, maps, localization, mapping and SLAM launch files, and helper nodes such as the human follower |
| `navigation/spatio_temporal_voxel_layer/spatio_temporal_voxel_layer/` | `spatio_temporal_voxel_layer` | ament_cmake | 3D obstacle costmap layer for Nav2 (LGPL v2.1) |
| `navigation/spatio_temporal_voxel_layer/openvdb_vendor/` | `openvdb_vendor` | ament_cmake | Builds OpenVDB from source if the system does not have it |
| `navigation/navigation2/` | 46 packages: 34 `nav2_*` packages, the DWB and `nav_2d` packages, `costmap_queue`, the `opennav_*` docking and following packages, and the `navigation2` metapackage | ament_cmake (`nav2_simple_commander` is ament_python) | Nested submodule: the Nav2 stack, built from source |

Because Nav2 is built from source, a full build and a full `pixi run test` take
much longer than the Walkie packages alone. See
[Navigation](../guides/navigation.md).

### perception/

| Path | Package | Build type | Purpose |
| --- | --- | --- | --- |
| `perception/` | `walkie_perception` | ament_cmake | YOLO and ByteTrack detection, 3D bounding box estimation, and GraspNet grasp services (`GetObPose`, `GetObjectCloud`, `GraspFromCloud`, `GraspFromMask`, `GraspPos`) |

`perception/pyproject.toml` lists the Python packages for detection and
grasping (PyTorch, Ultralytics, Open3D). See
[Perception](../guides/perception.md).

### simulation/gazebo/

| Path | Package | Build type | Purpose |
| --- | --- | --- | --- |
| `simulation/gazebo/` | `robot_simulation` | ament_cmake | Gazebo launch files (`gzsim_omnibot.launch.py`, `world_preview_gz.launch.py`, `rviz.launch.py`) and worlds (`PlainWorld`, `aws_small_house`, `obstacle_gz`, `turtlebot3_world_gz`) |

See [Gazebo Simulation](../guides/simulation-gazebo.md). The MuJoCo model
lives in `description/mujoco/`, not here.

### tools/

| Path | Package | Build type | Purpose |
| --- | --- | --- | --- |
| `tools/joy_interface/` | `joy_interface` | ament_python | Joystick teleoperation: executables `joy_teleop` and `joy_teleop_smooth` |
| `tools/walkie_tf/` | `walkie_tf` | ament_python | TF lookup service node: executable `tf_server` |
| `tools/walkie_tf_interfaces/` | `walkie_tf_interfaces` | ament_cmake | The `GetTransform` service used by `walkie_tf` |
| `tools/twist_stamper/` | `twist_stamper` | ament_python | Not in git. Imported by `pixi run import`. |

`tf_server` (node name `walkie_tf_server`) answers the `get_transform`
service (`walkie_tf_interfaces/srv/GetTransform`). It returns the latest pose
of `target_frame` expressed in `source_frame` (the node calls
`lookup_transform(source_frame, target_frame)`, and tf2 takes the target
frame first), waiting up to `timeout_sec` (1.0 s when the value is 0 or
negative). In the example below, that is the robot's pose on the map. The response has `success`, an error
`message`, the translation `x`, `y`, `z`, and the rotation quaternion `qx`,
`qy`, `qz`, `qw`. It is a quick way for scripts that do not keep a TF
listener to read one transform:

```bash
pixi run ros2 service call /get_transform walkie_tf_interfaces/srv/GetTransform \
  "{source_frame: map, target_frame: base_link, timeout_sec: 1.0}"
```

## Third-party code

A lot of the code in this repository was written by other projects. There are
three ways it gets here, and each one is updated in a different way.

| How | Where | How to update |
| --- | --- | --- |
| Copied into the repository (vendored) | `hardware/actuator/cubemars_hardware_interface/`, `hardware/actuator/dynamixel/` (ROBOTIS), `hardware/actuator/openarm/openarm_hardware/` (Enactic), `hardware/lidar/urg_node2/` (Hokuyo, with `urg_library`), `hardware/lidar/Lakibeam_ROS2_Driver/`, `hardware/lidar/dual_laser_merger/`, `hardware/lidar/robot_self_filter/` (ETH Zurich), `hardware/lidar/unilidar_sdk2/` (Unitree), `navigation/spatio_temporal_voxel_layer/` | Edit in place. Changes are committed in this repository (or in the `navigation` submodule for the voxel layer). Nothing tracks the upstream version. |
| Git submodule | See [Git submodules](#git-submodules) | `git submodule update` |
| vcstool import | See [External repositories](#external-repositories) | `pixi run import` |

Notes on the vendored code:

- `hardware/actuator/dynamixel/DynamixelSDK/` also contains the C, C++ and
  Python SDKs. Their folders have `COLCON_IGNORE` or `AMENT_IGNORE`, so colcon
  builds only the packages under `DynamixelSDK/ros/`.
- `manipulate/openarm_bimanual_moveit_config/` started as Enactic's generated
  MoveIt config. The team has changed it for Walkie, so treat it as team code.

## Git submodules

Submodules are separate git repositories with a pinned commit. The parent
repository stores only the commit hash. Clone with `--recurse-submodules`, or
run the update command after a normal clone:

```bash
git clone --recurse-submodules git@github.com:EIC-Robocup-2026/walkie-ros.git
git submodule update --init --recursive
```

Top-level submodules, from `.gitmodules`:

| Path | URL | Branch |
| --- | --- | --- |
| `navigation` | `git@github.com:EIC-Robocup-2026/navigation.git` | not set |
| `description` | `git@github.com:EIC-Robocup-2026/robot-description.git` | not set |
| `hardware/actuator/topic_based_ros2_control` | `https://github.com/PickNikRobotics/topic_based_ros2_control.git` | not set |
| `hardware/actuator/openarm/openarm_can` | `https://github.com/enactic/openarm_can.git` | not set |

Nested submodules, from `navigation/.gitmodules`:

| Path | URL | Branch |
| --- | --- | --- |
| `navigation/navigation2` | `https://github.com/ros-navigation/navigation2` | not set |
| `navigation/walkie-navigation/map` | `git@github.com:EIC-Robocup-2026/map-collection.git` | `main` |

The two team submodules and the map collection use SSH URLs. You need SSH
access to the EIC-Robocup-2026 organization to clone them.

The branch setting only matters for `git submodule update --remote`, which
moves the submodule to the newest commit of that branch. A plain
`git submodule update` always checks out the commit pinned by the parent.

!!! warning "Commit inside a submodule first"
    When you change files in `description/` or `navigation/`, commit and push
    inside the submodule first. Then commit the new submodule commit in the
    parent repository. If you only commit in the parent, other people get a
    commit hash they cannot fetch.

The description meshes in `.dae` format are stored in Git LFS. A plain
submodule update does not always download them. This command fetches the LFS
files in every submodule (the Docker workflow uses the same command):

```bash
git submodule foreach --recursive git lfs pull
```

## External repositories

`walkie_external.repos` lists packages that are cloned with vcstool (the `vcs`
command) instead of being stored in the repository. `pixi run import` runs `vcs import < walkie_external.repos`.
`pixi run build` runs `import` first. `pixi run build-pkg` does not.

| Path | URL | Version | Why it is imported |
| --- | --- | --- | --- |
| `hardware/lidar/laser_proc` | `https://github.com/ros-perception/laser_proc.git` | commit `4acb5601383d1fb25690377a683603ba35a4973c` | Needed by `urg_node2` |
| `tools/twist_stamper` | `https://github.com/joshnewans/twist_stamper.git` | tag `0.0.5` | `ros-jazzy-twist-stamper` is not built on the robostack-jazzy channel |

Both paths are in `.gitignore`. Do not commit changes inside them. They are
overwritten the next time someone runs `pixi run import`.

## Key top-level files

| File | Purpose |
| --- | --- |
| `pixi.toml` | Workspace manifest: conda channels, dependencies, tasks, and activation. See [Working with Pixi](../guides/pixi.md). |
| `pixi.lock` | Exact versions of every package in the environment. Commit it together with `pixi.toml`. |
| `walkie_external.repos` | vcstool list for `pixi run import` |
| `.gitmodules` | Submodule paths and URLs |
| `.gitattributes` | Marks `pixi.lock` as generated and merges it as a binary file, so git does not try a line-by-line merge |
| `.gitignore` | Ignores build output, `.pixi/`, and the imported repositories |
| `mkdocs.yml` | Configuration of this documentation site |
| `README.md` | Short introduction that points to this site |
| `LICENSE` | MIT license |
| `CLAUDE.md` | Notes for the Claude Code assistant: environment rules, the main pixi commands, and a short architecture summary. It points to this site for details. |
| `.tmuxp.yaml` | tmuxp layout for a tmux session named `walkie` |
| `.github/workflows/build-docker.yml` | Builds and pushes the Docker images. See [Docker Images](../guides/docker.md). |
| `.github/workflows/docs.yml` | Builds this site with `pixi run -e docs docs-build` and publishes it to GitHub Pages |

## scripts/

| Path | Contents |
| --- | --- |
| `scripts/bringup/` | `sim_nav_gazebo.sh`, `real_slam_nav.sh`, `isaac_sim_bringup.sh`, `connect_walkie_zenoh_router.sh`, and the Zenoh router config `WALKIE_RMW_ZENOH_ROUTER_CONFIG.json5`. See [Middleware (Zenoh)](../guides/middleware.md). |
| `scripts/setup_device/` | Interactive scripts that install udev rules and systemd units for the Dynamixel adapter (`/dev/ttyDynamixel`), the wheel CAN adapter (`can2`), and the OpenArm CAN FD adapter |
| `scripts/map/` | `record_mapping_bag.sh` records a bag during mapping for an offline rebuild of the maps. Also `odom_to_tf.py`. |
| `scripts/save_octomap_for_editor.py` | Saves the slam_toolbox 2D map and the OctoMap into one folder for the Walkie map editor |
| `scripts/setup_env.sh` | Sources `install/local_setup.sh`. Not used by pixi: `pixi.toml` sources `install/setup.bash` directly. |
| `scripts/tmux_walkie.sh` | Builds the same `walkie` tmux session as `.tmuxp.yaml` |

## Known issues

| Issue | Effect | Workaround |
| --- | --- | --- |
| `.tmuxp.yaml` and `scripts/tmux_walkie.sh` use the fixed path `~/robocup2026/walkie-ws` and plain `ros2 ...` commands without `pixi run` | On another machine the panes start in a folder that does not exist, and the typed `ros2` commands fail outside a pixi shell | Change the path, and add `pixi run` to the commands, before you use them |
| `hardware/lidar/urg_node2/.gitmodules` names `urg_library` as a submodule, but `urg_library/` is plain files | None for the build. The file is left over from upstream. | Ignore it |
