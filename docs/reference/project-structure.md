# Project Structure

Use this page to find where a package lives and whether the team wrote it or it
comes from another project. Start with the top-level layout, then look up the
folder under [Packages](#packages).

The repository root is one colcon workspace. `pixi run build` runs colcon from
the root, so colcon finds every package below it. See
[Working with Pixi](../guides/pixi.md) for the build tasks.

??? info "How colcon decides what to build"
    - Most packages have a `package.xml`.
    - colcon also builds one plain CMake folder without a `package.xml`:
      `unitree_lidar_sdk`.
    - colcon skips folders that contain a `COLCON_IGNORE` file.

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
| `tools/` | Teleoperation and TF lookup service |

??? info "Folders created by tools (not in git)"
    | Path | Created by | Notes |
    | --- | --- | --- |
    | `build/`, `install/`, `log/` | colcon | `pixi run clean` deletes all three |
    | `.pixi/` | pixi | The environment with ROS 2 and every dependency |
    | `hardware/camera/zed-ros2-wrapper/` | `pixi run setup-real` | See [Real-robot hardware repositories](#external-repositories) |

## Packages

The build type is `<build_type>` from each `package.xml`. It does not tell you
the language. Many `ament_cmake` packages here contain only Python launch files
and scripts, which CMake installs.

### bringup/

| Path | Package | Build type | Purpose |
| --- | --- | --- | --- |
| `bringup/` | `robot_bringup` | ament_cmake | Launch files (real robot, Isaac Sim, Gazebo with Nav2, subsystems), Python nodes, config, and a udev rule |

??? info "Launch files, nodes, and config in robot_bringup"
    - Launch files: the real robot (`real_omnibot.launch.py`), Isaac Sim
      (`isaac_omnibot.launch.py`), Gazebo with Nav2
      (`nav_sim_omnibot.launch.py`), lift homing, the dual lidar, OpenArm,
      octomap mapping, and a ZED test (`test_zed.launch.py`).
    - Python nodes in `scripts/`: lift homing (real and mock), ZED watchdog,
      head touch beep, OpenArm joint state reader.
    - Config in `config/`: cameras, lidars, OpenArm, RViz, and twist_mux.
    - `udev/`: a udev rule for ZED USB reset.

The lift motor uses a separate uv environment (TMotorCANControl, python-can),
described by `bringup/pyproject.toml` and `bringup/uv.lock`. It is not part of
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
| `hardware/lidar/unilidar_sdk2/unitree_lidar_sdk/` | `unitree_lidar_sdk` | cmake (no `package.xml`) | Unitree SDK example programs, linked against the prebuilt `libunilidar_sdk2.a` |
| `hardware/camera/realsense/config/` | none | none | One RealSense JSON preset. No package. |

??? info "Front and back lidar topics and frames"
    `bringup/config/lidar/dual_lidar_config.yml` sets which driver is front
    and which is back:

    | Driver | Publishes | Frame |
    | --- | --- | --- |
    | `urg_node2` (Hokuyo) | `front_lidar` | `front_lidar_link` |
    | Lakibeam driver (`lakibeam1`) | `back_lidar` | `back_lidar_link` |

??? info "Unitree SDK example programs"
    `unitree_lidar_sdk` builds `set_ip_address`, `set_to_udp_mode`,
    `set_to_serial_mode`, `example_lidar_udp`, and `example_lidar_serial`. The
    build writes them to `bin/` in the source folder, which Git ignores.

### manipulate/

| Path | Package | Build type | Purpose |
| --- | --- | --- | --- |
| `manipulate/my_robot_interfaces/` | `my_robot_interfaces` | ament_cmake | Actions (`GoToPose`, `GoToHome`, `ControlGripper`, `SetJointPosition`, and others), the `PoseCommand` message, and services (`GetEEPose`, `GetJointStates`, and others) |
| `manipulate/openarm_bimanual_commander_cpp/` | `openarm_bimanual_commander_cpp` | ament_cmake | C++ commander for the left and right arms, built on the MoveIt planning interface |
| `manipulate/openarm_bimanual_moveit_config/` | `openarm_bimanual_moveit_config` | ament_cmake | MoveIt config for the two arms. Generated by the MoveIt Setup Assistant (Enactic), then changed for Walkie. |

See [Manipulation](../guides/manipulation.md).

### navigation/

`navigation/` is a submodule with three groups of packages:

| Path | Package | Build type | Purpose |
| --- | --- | --- | --- |
| `navigation/walkie-navigation/` | `robot_navigation` | ament_cmake | Nav2 parameters, maps, localization, mapping and SLAM launch files, and helper nodes such as the human follower |
| `navigation/spatio_temporal_voxel_layer/spatio_temporal_voxel_layer/` | `spatio_temporal_voxel_layer` | ament_cmake | 3D obstacle costmap layer for Nav2 (LGPL v2.1) |
| `navigation/spatio_temporal_voxel_layer/openvdb_vendor/` | `openvdb_vendor` | ament_cmake | Builds OpenVDB from source if the system does not have it |
| `navigation/navigation2/` | 46 Nav2 packages | ament_cmake (one ament_python) | Nested submodule: the Nav2 stack, built from source |

??? info "The 46 packages in navigation/navigation2"
    - 34 `nav2_*` packages
    - the DWB and `nav_2d` packages
    - `costmap_queue`
    - the `opennav_*` docking and following packages
    - the `navigation2` metapackage

    All are ament_cmake except `nav2_simple_commander`, which is ament_python.

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

`tf_server` answers the `get_transform` service with the latest pose of
`target_frame` expressed in `source_frame`. Scripts that do not keep a TF
listener can use it to read one transform. This call returns the robot's pose
on the map:

```bash
pixi run ros2 service call /get_transform walkie_tf_interfaces/srv/GetTransform \
  "{source_frame: map, target_frame: base_link, timeout_sec: 1.0}"
```

??? info "get_transform request and response details"
    - Node name: `walkie_tf_server`. Service type:
      `walkie_tf_interfaces/srv/GetTransform`.
    - Frame order: the node calls `lookup_transform(source_frame,
      target_frame)`, and tf2 takes the target frame first. That is why the
      result is the pose of `target_frame` in `source_frame`.
    - Timeout: the node waits up to `timeout_sec`. A value of 0 or less means
      1.0 s.
    - Response: `success`, an error `message`, the translation `x`, `y`, `z`,
      and the rotation quaternion `qx`, `qy`, `qz`, `qw`.

## Third-party code

Code from other projects reaches this repository in three ways. Each one is
updated differently.

| How | Where | How to update |
| --- | --- | --- |
| Copied into the repository (vendored) | [Vendored code](#vendored-code) | Edit in place and commit |
| Git submodule | [Git submodules](#git-submodules) | `git submodule update` |
| vcstool import | [External repositories](#external-repositories) | `pixi run import` |

### Vendored code

Edit vendored code in place. Commit the changes in this repository (or in the
`navigation` submodule for the voxel layer). Nothing tracks the upstream
version.

- `hardware/actuator/cubemars_hardware_interface/`
- `hardware/actuator/dynamixel/` (ROBOTIS)
- `hardware/actuator/openarm/openarm_hardware/` (Enactic)
- `hardware/lidar/urg_node2/` (Hokuyo, with `urg_library`)
- `hardware/lidar/Lakibeam_ROS2_Driver/`
- `hardware/lidar/dual_laser_merger/`
- `hardware/lidar/robot_self_filter/` (ETH Zurich)
- `hardware/lidar/unilidar_sdk2/` (Unitree)
- `navigation/spatio_temporal_voxel_layer/`

`manipulate/openarm_bimanual_moveit_config/` started as Enactic's generated
MoveIt config. The team has changed it for Walkie, so treat it as team code.

??? info "Why colcon builds only part of DynamixelSDK"
    `hardware/actuator/dynamixel/DynamixelSDK/` also contains the C, C++ and
    Python SDKs. Their folders have `COLCON_IGNORE` or `AMENT_IGNORE`, so
    colcon builds only the packages under `DynamixelSDK/ros/`.

## Git submodules

The two team submodules and the map collection use SSH URLs. You need SSH
access to the EIC-Robocup-2026 organization to clone them.

1. Clone with `--recurse-submodules`:

    ```bash
    git clone --recurse-submodules git@github.com:EIC-Robocup-2026/walkie-ros.git
    ```

    After a normal clone, run this in the repository instead:

    ```bash
    git submodule update --init --recursive
    ```

2. Fetch the Git LFS files in every submodule. The description meshes in
    `.dae` format are stored in Git LFS, and a plain submodule update does not
    always download them. The Docker workflow uses the same command.

    ```bash
    git submodule foreach --recursive git lfs pull
    ```

| Path | URL | Branch | Listed in |
| --- | --- | --- | --- |
| `navigation` | `git@github.com:EIC-Robocup-2026/navigation.git` | not set | `.gitmodules` |
| `description` | `git@github.com:EIC-Robocup-2026/robot-description.git` | not set | `.gitmodules` |
| `hardware/actuator/topic_based_ros2_control` | `https://github.com/PickNikRobotics/topic_based_ros2_control.git` | not set | `.gitmodules` |
| `hardware/actuator/openarm/openarm_can` | `https://github.com/enactic/openarm_can.git` | not set | `.gitmodules` |
| `navigation/navigation2` | `https://github.com/ros-navigation/navigation2` | not set | `navigation/.gitmodules` (nested) |
| `navigation/walkie-navigation/map` | `git@github.com:EIC-Robocup-2026/map-collection.git` | `main` | `navigation/.gitmodules` (nested) |

!!! warning "Commit inside a submodule first"
    When you change files in `description/` or `navigation/`, commit and push
    inside the submodule first. Then commit the new submodule commit in the
    parent repository. If you only commit in the parent, other people get a
    commit hash they cannot fetch.

??? info "How submodules are pinned, and what the Branch column changes"
    A submodule is a separate git repository. The parent repository stores
    only its pinned commit hash.

    The branch setting only matters for `git submodule update --remote`, which
    moves the submodule to the newest commit of that branch. A plain
    `git submodule update` always checks out the commit pinned by the parent.

## Real-robot hardware repositories { #external-repositories }

`walkie_real.repos` lists hardware packages that are cloned with vcstool (the `vcs`
command) when preparing a physical robot. `pixi run setup-real` runs
`vcs import < walkie_real.repos`.

| Path | URL | Version | Why it is imported |
| --- | --- | --- | --- |
| `hardware/camera/zed-ros2-wrapper` | `https://github.com/stereolabs/zed-ros2-wrapper.git` | tag `v5.5.0` | Real robot ZED camera driver |
| `hardware/camera/zed-ros2-wrapper/zed-ros2-interfaces` | `https://github.com/stereolabs/zed-ros2-interfaces.git` | tag `5.5.0` | ZED ROS 2 message definitions |
| `hardware/camera/zed-ros2-wrapper/zed-ros2-description` | `https://github.com/stereolabs/zed-ros2-description.git` | tag `0.2.0` | ZED URDF and meshes |
| `hardware/camera/zed-ros2-wrapper/zed-ros2-examples` | `https://github.com/stereolabs/zed-ros2-examples.git` | tag `v5.5.0` | ZED examples |

!!! warning "Do not commit changes inside imported folders"
    `hardware/camera/zed-ros2-wrapper/` is in `.gitignore`. It is kept outside the git repository so non-CUDA machines can build the workspace without CUDA or ZED SDK dependencies.

## Key top-level files

| File | Purpose |
| --- | --- |
| `pixi.toml` | Workspace manifest: conda channels, dependencies, tasks, and activation. See [Working with Pixi](../guides/pixi.md). |
| `pixi.lock` | Exact versions of every package in the environment. Commit it together with `pixi.toml`. |
| `walkie_real.repos` | vcstool list for `pixi run setup-real` |
| `.gitmodules` | Submodule paths and URLs |
| `.gitattributes` | Marks `pixi.lock` as generated and merges it as a binary file, so git does not try a line-by-line merge |
| `.gitignore` | Ignores build output, `.pixi/`, and the imported repositories |
| `mkdocs.yml` | Configuration of this documentation site |
| `README.md` | Short introduction that points to this site |
| `LICENSE` | MIT license |
| `CLAUDE.md` | Notes for the Claude Code assistant (environment rules, main pixi commands, short architecture summary). Points to this site for details. |
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
| `.tmuxp.yaml` and `scripts/tmux_walkie.sh` use the fixed path `~/robocup2026/walkie-ws` and plain `ros2 ...` commands without `pixi run` | On another machine the panes start in a missing folder, and the `ros2` commands fail outside a pixi shell | Change the path and add `pixi run` to the commands before you use them |
| `hardware/lidar/urg_node2/.gitmodules` names `urg_library` as a submodule, but `urg_library/` is plain files | None for the build. The file is left over from upstream. | Ignore it |
