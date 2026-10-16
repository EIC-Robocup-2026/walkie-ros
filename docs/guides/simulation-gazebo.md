# Gazebo Simulation

Run Walkie in Gazebo Harmonic, with or without Nav2, and drive it. For the
newer MuJoCo simulation, see [MuJoCo Simulation](simulation-mujoco.md).

## Quick start

Build the workspace first (see [Getting Started](../getting-started.md)). Then
start Gazebo with the robot, its controllers, and RViz:

```bash
pixi run ros2 launch robot_simulation gzsim_omnibot.launch.py
```

- The command works with any middleware. To use Zenoh like the real robot,
  start a router and export `RMW_IMPLEMENTATION` first. See
  [Middleware (Zenoh)](middleware.md#simulation-on-one-machine).
- Closing Gazebo or RViz stops the whole launch. Use `use_rviz:=false` if you
  want to restart RViz on its own.
- To add Nav2, see [Run Gazebo with Nav2](#gazebo-with-nav2).

??? info "What `gzsim_omnibot.launch.py` starts"
    1. Gazebo (`gz sim -r -v4 <world>`) through `ros_gz_sim`'s
       `gz_sim.launch.py`. `-r` starts the simulation running instead of
       paused, and `-v4` sets the most detailed log level. Closing Gazebo stops
       the whole launch.
    2. `ros_gz_bridge`'s `parameter_bridge` with `gz_bridge.yaml`.
    3. `robot_state_publisher`, through `walkie_description`'s
       `robot_state_publisher.launch.py`, with `use_zed:=false`.
    4. `ros_gz_sim create`, which spawns the robot from the
       `/robot_description` topic as the model `walkie_bot` at height 0.1 m.
    5. The controller spawners. They start only after `create` exits, so the
       controller manager inside Gazebo exists before they run.
    6. `twist_mux` (see [Drive the robot](#driving-the-robot)).
    7. `foxglove_bridge` with a 50 MB send buffer.
    8. RViz, through `rviz.launch.py`.
    9. `walkie_tf`'s `tf_server`.
    10. `walkie_perception`'s `cloud_to_map` (reads
        `/zed_head/zed_node/point_cloud/cloud_registered`) and
        `ob_pose_service_cpp`. See [Perception](perception.md).

??? info "Where the Gazebo files live"
    | Path | Package | Contents |
    | --- | --- | --- |
    | `simulation/gazebo/launch/` | `robot_simulation` | `gzsim_omnibot.launch.py`, `world_preview_gz.launch.py`, `rviz.launch.py` |
    | `simulation/gazebo/worlds/` | `robot_simulation` | Four world files |
    | `simulation/gazebo/models/` | `robot_simulation` | AWS RoboMaker residential models and `turtlebot3_world` |
    | `simulation/gazebo/config/gazebo/gz_bridge.yaml` | `robot_simulation` | Gazebo to ROS topic bridge |
    | `simulation/gazebo/rviz/` | `robot_simulation` | `turtlebot3_rviz.rviz`, `display.rviz` |
    | `bringup/launch/nav_sim_omnibot.launch.py` | `robot_bringup` | Gazebo plus Nav2 plus rosbridge |
    | `bringup/config/twist_mux/twist_mux.yml` | `robot_bringup` | Velocity command priorities |
    | `description/robots/gz_walkie.urdf.xacro` | `walkie_description` | Robot model used in simulation |
    | `description/urdf/walkie/ros2_control.xacro` | `walkie_description` | `ros2_control` hardware blocks, including the Gazebo block |
    | `description/config/ros2_controller/sim_controllers.yaml` | `walkie_description` | Controller settings for simulation |
    | `tools/joy_interface/` | `joy_interface` | Joystick teleoperation |

## Change the world, spawn pose, or RViz

Pass launch arguments to `gzsim_omnibot.launch.py`. This example starts in the
obstacle world without RViz. Run it from the repository root so that `$PWD` is
correct:

```bash
pixi run ros2 launch robot_simulation gzsim_omnibot.launch.py \
  world:=$PWD/simulation/gazebo/worlds/obstacle_gz.world use_rviz:=false
```

| Argument | Default | Meaning |
| --- | --- | --- |
| `world` | `<robot_simulation share>/worlds/aws_small_house.world` | Full path to the world file |
| `x_pose` | `-2.0` | Spawn position, x (m) |
| `y_pose` | `1.0` | Spawn position, y (m) |
| `use_rviz` | `True` | Start RViz |
| `dual_lidar` | `False` | Simulate two lidars instead of one. Keep the default, see [Lidar](#lidar). |

??? info "Other launch arguments"
    | Argument | Default | Meaning |
    | --- | --- | --- |
    | `use_sim_time` | `true` | Passed to `robot_state_publisher`, `foxglove_bridge`, and the perception nodes |
    | `robot_model` | `<walkie_description share>/robots/gz_walkie.urdf.xacro` | Xacro file of the robot |
    | `ros2_control` | `gazebo` | Hardware type passed to the xacro |
    | `rviz_config_file` | `<robot_simulation share>/rviz/turtlebot3_rviz.rviz` | RViz config |

??? info "Environment variables set by the launch file"
    | Variable | Value | Why |
    | --- | --- | --- |
    | `GZ_SIM_RESOURCE_PATH` | `<robot_simulation share>/models` and the `share` directory that contains `walkie_description` | Gazebo searches these directories for `model://` URIs |
    | `GZ_SIM_SYSTEM_PLUGIN_PATH` | `/opt/ros/jazzy/lib/` | Gazebo searches this directory for system plugins before the others |

    The plugin path points outside the pixi environment. Since RoboStack
    build set `_25`, `pixi run` already sets `GZ_SIM_SYSTEM_PLUGIN_PATH` to
    the `lib` folder of the pixi environment, through an environment hook of
    `gz_ros2_control`. The launch file replaces that value. See
    [Known issues](#known-issues).

### Worlds

All world files are in `simulation/gazebo/worlds/`.

| File | World name | Notes |
| --- | --- | --- |
| `aws_small_house.world` | `default` | Default. AWS RoboMaker small house with furniture from `models/`. |
| `obstacle_gz.world` | `empty` | Simple world with obstacles. No IMU system, so `/imu` gets no data. |
| `PlainWorld.world` | `default` | No system plugins. No Sensors system, so the lidar and cameras publish no data. |
| `turtlebot3_world_gz.world` | `default` | Uses `model://turtlebot3_world` from `models/` |

The world name matters. The bottom camera topics in `gz_bridge.yaml` contain
`/world/default/...`, so they get no data in `obstacle_gz.world`.

To open a world without the robot, use `world_preview_gz.launch.py`. It takes
the same `world` argument with the same default:

```bash
pixi run ros2 launch robot_simulation world_preview_gz.launch.py \
  world:=$PWD/simulation/gazebo/worlds/turtlebot3_world_gz.world
```

## Drive the robot { #driving-the-robot }

Publish `TwistStamped` on `/cmd_vel` at 10 Hz. Stop with Ctrl+C.

```bash
pixi run ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_footprint}, twist: {linear: {x: 0.2}}}"
```

- Every input must be `geometry_msgs/msg/TwistStamped`. Plain `Twist`
  messages are not used (`twist_mux` has `use_stamped: true`).
- Keep publishing. `twist_mux` drops an input that sends nothing for 0.5 s,
  and the wheel controller stops the robot if no command arrives for 0.5 s
  (`cmd_vel_timeout: 0.5`).
- The robot is omnidirectional. Set `linear.y` to move sideways and
  `angular.z` to turn.
- Odometry is open loop (`open_loop: true`). It is computed from the commands,
  not from wheel feedback, so wheel slip in Gazebo does not show up in
  `/omni_wheel_drive_controller/odom`.

The full controller parameter list is in
[Omni wheel controller parameters](../reference/robot-description.md#omni-wheel-controller-parameters).

### Which velocity source wins

`twist_mux` forwards the highest-priority active input to the wheel
controller. Its config is `bringup/config/twist_mux/twist_mux.yml`.

```mermaid
flowchart LR
    A["/cmd_vel<br/>priority 5"] --> M[twist_mux]
    B["/cmd_vel_nav<br/>priority 10"] --> M
    C["/cmd_vel_joy<br/>priority 100"] --> M
    D["/cmd_vel_key<br/>priority 100"] --> M
    M --> W["/omni_wheel_drive_controller/cmd_vel"]
```

Nav2 publishes on `/cmd_vel_nav`. A joystick on `/cmd_vel_joy` overrides Nav2.
A command on `/cmd_vel` loses to Nav2.

### With a joystick

```bash
pixi run ros2 launch joy_interface joy_control_smooth.launch.py
```

| Launch file | Output topic | Max linear / angular | Notes |
| --- | --- | --- | --- |
| `joy_control_smooth.launch.py` | `/cmd_vel_joy` | 0.45 m/s / 1.0 rad/s | Acceleration limits. Overrides Nav2. Button 0 cancels all goals on `/navigate_to_pose` and `/navigate_through_poses`. |
| `joy_control.launch.py` | `/cmd_vel` | 0.65 m/s / 1.4 rad/s | No smoothing. Lowest priority, so Nav2 overrides it. |

Both start `joy_linux_node`, which reads `/dev/input/jsX` and publishes `/joy`.
The left stick moves forward and sideways (axes 1 and 0). The right stick
turns (axis 3).

### Keyboard

The repository has no keyboard teleop package, and `pixi.toml` does not
install `teleop_twist_keyboard`. `twist_mux` reserves `/cmd_vel_key` for a
keyboard source. A keyboard tool must publish `TwistStamped` on that topic.

## Run Gazebo with Nav2 { #gazebo-with-nav2 }

!!! warning
    The default simulation map is missing. Extract it and pass it with `map:=` as shown in [The simulation map](navigation.md#the-simulation-map).

!!! warning "Nav2 stops during bringup with the current parameter files"
    No MPPI parameter file in `robot_navigation` sets `FollowPath.Omni.plugin`, so
    `controller_server` fails to configure and Nav2 stops. Add the key to
    `nav2_mppi_walkie_sim_params.yaml`, as shown in
    [Navigation: controller_server cannot load the Omni motion model](navigation.md#mppi-omni-motion-model-plugin).

    The composed Nav2 bringup can also hang at
    `Waiting for service smoother_server/get_state...`. Add `use_composition:=False` to the
    command below. See
    [Navigation: Nav2 bringup hangs at smoother_server/get_state](navigation.md#nav2-composition-deadlock).

```bash
pixi run ros2 launch robot_bringup nav_sim_omnibot.launch.py map:=/tmp/sim_map/map.yaml
```

ROS 2 launch arguments are global, so arguments of the included files also
work here. For example, `map:=...` reaches `localization_Nav2_sim.launch.py`,
and `use_rviz:=false` reaches `gzsim_omnibot.launch.py`.

Prefer the pixi commands in
[Middleware (Zenoh)](middleware.md#simulation-on-one-machine) over
`scripts/bringup/sim_nav_gazebo.sh`. The script runs the same launch file and
starts a Zenoh router if none is running, but it has problems. See
[Known issues](#known-issues).

??? info "What `nav_sim_omnibot.launch.py` starts and its arguments"
    `bringup/launch/nav_sim_omnibot.launch.py` starts:

    - `gzsim_omnibot.launch.py`, with `robot_model`, `ros2_control`, and
      `dual_lidar`
    - `robot_navigation`'s `localization_Nav2_sim.launch.py` (map server,
      AMCL, and the Nav2 servers)
    - `rosbridge_server`'s `rosbridge_websocket_launch.xml`
    - `current_pose_publisher.py`, which publishes the `map` to
      `base_footprint` transform on `current_pose` (`nav_msgs/msg/Odometry`)
      at 10 Hz

    | Argument | Default |
    | --- | --- |
    | `robot_model` | `<walkie_description share>/robots/gz_walkie.urdf.xacro` |
    | `ros2_control` | `gazebo` |
    | `dual_lidar` | `False` |

## Move the head and other controllers { #controllers }

The launch file spawns the base, lift, arm, and gripper controllers. MoveIt
uses the arm and gripper controllers (see [Manipulation](manipulation.md)).
The head controller is not spawned. To move the head, spawn it by hand:

```bash
pixi run ros2 run controller_manager spawner head_servo_controller
```

`head_servo_controller` is a `JointGroupPositionController` on
`head_servo_joint`. It listens on `/head_servo_controller/commands`.

??? info "Spawned and unspawned controllers"
    `sim_controllers.yaml` sets the controller manager to 60 Hz. The launch
    file spawns these controllers, each with `--switch-timeout 30.0`:

    | Controller | Type | Joints |
    | --- | --- | --- |
    | `joint_broad` | `JointStateBroadcaster` | All joints |
    | `omni_wheel_drive_controller` | `OmniWheelDriveController` | Four wheel joints |
    | `lift_controller` | `JointTrajectoryController` | `lift_joint` |
    | `left_joint_trajectory_controller` | `JointTrajectoryController` | Left arm |
    | `right_joint_trajectory_controller` | `JointTrajectoryController` | Right arm |
    | `left_gripper_controller` | `GripperActionController` | Left gripper |
    | `right_gripper_controller` | `GripperActionController` | Right gripper |

    The file also defines controllers that are not spawned:
    `left_arm_controller`, `arm_controller`, `right_arm_controller`, the
    forward position and velocity controllers for each arm, and
    `head_servo_controller`.

??? info "How `ros2_control` runs inside Gazebo"
    The launch file builds the robot from `gz_walkie.urdf.xacro` with
    `ros2_control:=gazebo`. This value selects the Gazebo block in
    `ros2_control.xacro`:

    - The hardware plugin is `gz_ros2_control/GazeboSimSystem`. It connects
      the `ros2_control` joints to the simulated joints.
    - The Gazebo plugin `gz_ros2_control::GazeboSimROS2ControlPlugin`
      (`libgz_ros2_control-system.so`) runs the controller manager inside the
      Gazebo process. It loads `sim_controllers.yaml`. No separate
      `ros2_control_node` runs.
    - The wheel joints take velocity commands. The head servo, lift, arm, and
      finger joints take position commands. The lift command interface
      accepts 0.000 to 0.743 m, but the URDF joint limit is 0.08 to 0.7435 m.

    See [Robot Description](../reference/robot-description.md) for the full
    model.

## Sensors and topics

### Lidar

Keep the default `dual_lidar:=False`. The simulation then uses one 360° lidar,
bridged to `/scan`. The real robot has two lidars, but one is faster to
simulate. With `dual_lidar:=True`, no lidar is bridged and Nav2 has no `/scan`
(see [Known issues](#known-issues)).

??? info "Lidar placement and range"
    | `dual_lidar` | Lidar links | Coverage | Range | Result |
    | --- | --- | --- | --- | --- |
    | `False` (default) | `single_lidar_link`, 0.315 m above the center of `base_link` | 360° | 0.6 to 10 m | Bridged to `/scan` |
    | `True` | `front_lidar_link` and `back_lidar_link`, 0.235 m in front of and behind the center | ±120° each | 0.18 to 10 m | Not bridged. No `/scan`. |

    Both variants publish at 40 Hz with 360 samples per scan.

### Bridged topics

All entries in `gz_bridge.yaml` go from Gazebo to ROS. The head camera uses
the same topic names as the real ZED camera, so the perception nodes can
subscribe to the same topics in simulation.

| ROS topic | Type | Gazebo topic | Notes |
| --- | --- | --- | --- |
| `/clock` | `rosgraph_msgs/msg/Clock` | `clock` | Simulation time |
| `/scan` | `sensor_msgs/msg/LaserScan` | `single_lidar` | Only with `dual_lidar:=False` |
| `/imu` | `sensor_msgs/msg/Imu` | `imu` | Lazy: bridged only while someone subscribes |
| `/zed_head/zed_node/rgb/color/rect/image` | `Image` | `/head/rgb/image` | Head color image |
| `/zed_head/zed_node/rgb/color/rect/image/camera_info` | `CameraInfo` | `/head/rgb/camera_info` | |
| `/zed_head/zed_node/depth/depth_registered` | `Image` | `/head/rgbd/depth_image` | Head depth image |
| `/zed_head/zed_node/depth/depth_registered/camera_info` | `CameraInfo` | `/head/rgbd/camera_info` | |
| `/zed_head/zed_node/point_cloud/cloud_registered` | `PointCloud2` | `/head/rgbd/points` | Head point cloud |

One `ob_pose_service_cpp` parameter does not match the bridge. See
[Known issues](#known-issues).

??? info "Bottom camera and ground-truth topics"
    | ROS topic | Type | Gazebo topic | Notes |
    | --- | --- | --- | --- |
    | `/gazebo_ground_truth/odom` | `geometry_msgs/msg/Pose` | `/model/my_bot/pose` | No data. See [Known issues](#known-issues). |
    | `/bottom_rgb/image_raw`, `/bottom_rgb/camera_info` | `Image`, `CameraInfo` | `/world/default/model/walkie_bot/link/base_footprint/sensor/bottom_rgb_sensor/...` | Only in worlds named `default` |
    | `/bottom_depth/image_raw` | `Image` | `/bottom_depth/image` | |
    | `/bottom_depth/points` | `PointCloud2` | `/bottom_depth/image/points` | |
    | `/bottom_depth/camera_info` | `CameraInfo` | `/bottom_depth_sensor/camera_info` | No data. See [Known issues](#known-issues). |
    | `/bottom_left_ir/...`, `/bottom_right_ir/...` | `Image`, `CameraInfo` | `.../bottom_left_ired_sensor/...`, `.../bottom_right_ired_sensor/...` | No data. See [Known issues](#known-issues). |

??? info "Where the IMU and cameras are on the robot"
    | Sensor | Link | Notes |
    | --- | --- | --- |
    | IMU | `top_imu_link` | Gazebo topic `imu` |
    | Bottom RealSense D415 | on `base_link` | Color and depth only (`flag_ir="0"`), updates at 2 Hz |
    | Head RGB-D camera | on the head servo, which rides on the lift | Bridged to the ZED topic names. See [Bridged topics](#bridged-topics). |

## RViz

The default config uses the fixed frame `map`. Without Nav2 there is no `map`
frame, so RViz shows a TF error. Set the fixed frame to `odom` in the Displays
panel.

??? info "RViz configs and tools"
    | Config | Fixed frame | Used by | Shows |
    | --- | --- | --- | --- |
    | `simulation/gazebo/rviz/turtlebot3_rviz.rviz` | `map` | `gzsim_omnibot.launch.py` | Robot model, TF, `/scan`, `/map`, costmaps, `/plan`, head camera image and point cloud, Nav2 panel |
    | `simulation/gazebo/rviz/display.rviz` | `base_link` | Nothing by default | Robot model and TF |
    | `description/rviz/robot_preview.rviz` | `base_link` | The `walkie_description` preview launch files | Robot model |

    - `rviz.launch.py` shuts down the whole launch when RViz exits, so closing
      RViz also closes Gazebo.
    - The config has tools for a Nav2 goal (`nav2_rviz_plugins/GoalTool`),
      initial pose (`/initialpose`), and publish point (`/clicked_point`).

## Preview the robot model without Gazebo

`walkie_description` has two launch files that show the model in RViz with no
physics. See [Preview in RViz](../reference/robot-description.md#preview-in-rviz).

## Isaac Sim

There is an Isaac Sim launch file, `bringup/launch/isaac_omnibot.launch.py`,
and a helper script, `scripts/bringup/isaac_sim_bringup.sh`. The Isaac Sim
scene is not in this repository.

!!! warning
    The Isaac Sim launch file does not start as it is. See [Known issues](#known-issues).

??? info "How the Isaac Sim launch connects to `ros2_control`"
    The launch file uses `ros2_control:=topic_base`, which selects
    `topic_based_ros2_control/TopicBasedSystem`. It exchanges joint commands
    and states with Isaac Sim on `/isaac_joint_commands` and
    `/isaac_joint_states`.

## Known issues

| Effect | Cause | Workaround |
| --- | --- | --- |
| On a computer with a system ROS 2 Jazzy, Gazebo can load a system build of `libgz_ros2_control-system.so` and crash with a segmentation fault | The launch files set the plugin path to `/opt/ros/jazzy/lib/` and replace the value that pixi sets | Remove the `SetEnvironmentVariable` action for `GZ_SIM_SYSTEM_PLUGIN_PATH` from `gzsim_omnibot.launch.py` and `world_preview_gz.launch.py`. See also [Gazebo problems only on computers with a system ROS](../troubleshooting.md#gazebo-problems-only-on-computers-with-a-system-ros). |
| `nav_sim_omnibot.launch.py` cannot load its map | Default map is missing at the pinned map commit | Pass `map:=`. See [The simulation map](navigation.md#the-simulation-map). |
| `controller_server` fails to configure, and Nav2 stops during bringup | The MPPI parameter files do not set `FollowPath.Omni.plugin`, which the Nav2 fork requires | Add `plugin: "mppi::OmniMotionModel"` under `FollowPath.Omni`. See [Navigation](navigation.md#mppi-omni-motion-model-plugin). |
| Nav2 hangs at `Waiting for service smoother_server/get_state...` | `localization_Nav2_sim.launch.py` keeps the composed bringup, which can deadlock on Jazzy | Pass `use_composition:=False`. See [Navigation](navigation.md#nav2-composition-deadlock). |
| No `/scan` with `dual_lidar:=True`, so Nav2 has no laser data | Bridge entries commented out, no laser merger | Keep the default `dual_lidar:=False` |
| `robot_preview_rviz.launch.py` and `robot_preview_lift_rviz.launch.py` fail with `name 'true' is not defined` | `robot_state_publisher.launch.py` defaults `dual_lidar` to `true`, which the xacro cannot evaluate in `gazebo` mode | Pass `dual_lidar:=False` (or `True`) on the command line |
| `/gazebo_ground_truth/odom` gets no data | Bridge reads `/model/my_bot/pose`, model is `walkie_bot` | None in the repository |
| `/bottom_depth/camera_info` gets no data | Bridge reads the wrong Gazebo topic | None in the repository |
| `/bottom_left_ir/...` and `/bottom_right_ir/...` get no data | Camera built with `flag_ir="0"` | None needed unless you use IR |
| `ob_pose_service_cpp` gets no camera info in Gazebo | Its `info_topic` is not bridged | None in the repository |
| One point cloud display in `turtlebot3_rviz.rviz` stays empty | It uses `/head/depth/points`, which is not bridged | Use the display for `/zed_head/zed_node/point_cloud/cloud_registered` |
| `sim_nav_gazebo.sh` mixes system ROS and pixi, nodes may use DDS, a router stays running | Script sources system ROS, does not set `RMW_IMPLEMENTATION`, does not stop its router | Run the pixi commands in [Middleware (Zenoh)](middleware.md#simulation-on-one-machine). Stop a leftover router with `pkill -x rmw_zenohd`. |
| `isaac_omnibot.launch.py` fails at start | Includes a file from the wrong package | None in the repository |

??? info "Details of the causes"
    - **System plugin path.** `gzsim_omnibot.launch.py` and
      `world_preview_gz.launch.py` set `GZ_SIM_SYSTEM_PLUGIN_PATH` to
      `/opt/ros/jazzy/lib/`. Under pixi, `gz_ros2_control` is in
      `.pixi/envs/default/lib`. Since RoboStack build set `_25`,
      `share/gz_ros2_control/environment/gz_ros2_control.dsv` in the pixi
      environment prepends that folder to the variable, and the launch files
      replace the value. On a computer with a system ROS 2 Jazzy, Gazebo then
      searches the system folder and can load the system build of
      `libgz_ros2_control-system.so`. The fix, removing the override, is not
      in the repository yet. With the value that pixi sets, Gazebo with
      `gz_ros2_control` worked headless on build set `_25`, started by a test
      script rather than these launch files.
    - **Simulation map.** The default map of `localization_Nav2_sim.launch.py`
      is missing at the map commit that the `navigation` submodule pins.
    - **Dual lidar.** With `dual_lidar:=True`, the `front_lidar` and
      `back_lidar` entries in `gz_bridge.yaml` are commented out, and no laser
      merger runs in simulation.
    - **`dual_lidar` default of the preview launch files.**
      `robot_state_publisher.launch.py` in `description/launch/` defaults
      `dual_lidar` to `true` (lowercase). When `ros2_control` is `gazebo`,
      `gz_walkie.urdf.xacro` evaluates `not $(arg dual_lidar)` as a Python
      expression, and `true` is not a Python name. The two preview launch
      files pass `ros2_control` (default `gazebo`) but not `dual_lidar`.
      `gzsim_omnibot.launch.py` and `nav_sim_omnibot.launch.py` pass `False`,
      so they are not affected.
    - **Bottom depth camera info.** The `/bottom_depth/camera_info` bridge
      entry reads `/bottom_depth_sensor/camera_info`, but the sensor publishes
      on `bottom_depth/camera_info`.
    - **Bottom IR.** The bottom camera is built with `flag_ir="0"`, but
      `gz_bridge.yaml` still bridges the IR topics.
    - **`ob_pose_service_cpp`.** It is started with
      `info_topic: /zed_head/zed_node/depth/camera_info`. The bridge publishes
      `/zed_head/zed_node/depth/depth_registered/camera_info`.
    - **`sim_nav_gazebo.sh`.** It sources `/opt/ros/$ROS_DISTRO/setup.bash`
      when it exists, does not set `RMW_IMPLEMENTATION`, and leaves the router
      it starts running.
    - **Isaac Sim.** `isaac_omnibot.launch.py` includes
      `robot_state_publisher.launch.py` from `robot_bringup`, but that file
      only exists in `walkie_description`.

## Troubleshooting

| Symptom | Cause | Fix |
| --- | --- | --- |
| RViz shows "Fixed Frame [map] does not exist" | Nav2 is not running | Set the fixed frame to `odom` |
| Closing RViz stops Gazebo | `rviz.launch.py` emits a shutdown when RViz exits | Use `use_rviz:=false` and start RViz separately |
| The robot does not move with `ros2 topic pub` | Message type is `Twist`, or it was published once | Publish `TwistStamped` with `-r 10` |
| The robot ignores `joy_control.launch.py` while Nav2 runs | `/cmd_vel` has a lower priority than `/cmd_vel_nav` | Use `joy_control_smooth.launch.py` |
| `/bottom_rgb/...` topics have no data | World is not named `default` (for example `obstacle_gz.world`) | Use a world named `default` |
| No lidar or camera data in `PlainWorld.world` | The world loads no Sensors system | Use another world |
