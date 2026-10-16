# Navigation

Walkie navigates with Nav2: AMCL localizes on a saved map, SLAM Toolbox builds
new maps, and the MPPI controller drives the omnidirectional base. Use this
guide to run navigation in simulation or on the robot, and to send goals.

!!! note
    `navigation/` and the map collection are Git submodules. Clone with
    `--recurse-submodules`, or run `git submodule update --init --recursive`
    after cloning.

!!! warning "The collision monitor now stops and slows the robot"
    Since [5. Repair Nav2 After the navigation2 Bump](../migrations/05-nav2-repair.md),
    Nav2 velocity commands pass the velocity smoother and the collision monitor
    before they reach `twist_mux`.

    - The robot slows down near obstacles and stops before an obstacle it
      drives toward. Commands that drive away from the obstacle or along it
      pass, so Nav2 can move the robot on.
    - The monitor stops every Nav2 command while one of its sources has no
      data.
    - If Nav2 stays blocked, drive the robot out with the joystick or
      keyboard. They bypass the monitor.

    See [Velocity command path](#velocity-command-path) and
    [Collision monitor](#collision-monitor).

## Navigate in simulation { #localization-and-mapping-in-simulation }

### The simulation map

`localization_Nav2_sim.launch.py` and `nav2_loopback.launch.py` default to the
map `08_23102025_SimWalkie_awssmallhouse_slam_toolbox` in the map submodule, so
no `map:=` argument is needed. To use another map, pass its full path with
`map:=<path>/map.yaml`.

If the launch cannot find the default map, the map submodule is at an older
commit or the map is not installed yet. Update the submodules and reinstall
the maps, from the repository root:

```bash
git submodule update --init --recursive
pixi run build-pkg robot_navigation
```

??? info "Map history and how the map argument works"
    Before [5. Repair Nav2 After the navigation2 Bump](../migrations/05-nav2-repair.md),
    the map submodule was pinned to `94d86cc`, which had removed this map, and
    the launch files named it without the leading zero. The map collection
    commit `c22354c` restored it.

    With `--symlink-install`, `robot_navigation` installs `map/` as one
    symlink per file. A map directory that is new in the submodule therefore
    needs the rebuild above before the launch files can find it.

    The `map` argument given to `nav_sim_omnibot.launch.py` reaches
    `localization_Nav2_sim.launch.py` because ROS 2 launch arguments are
    global.

### Localize and send goals

```bash
pixi run ros2 launch robot_bringup nav_sim_omnibot.launch.py
```

This starts Gazebo plus `localization_Nav2_sim.launch.py` (see
[Gazebo Simulation](simulation-gazebo.md)).

1. AMCL starts at (0, 0, 0) in the map frame. If the laser scan in RViz does
   not line up with the map, set the pose with the **2D Pose Estimate** tool.
2. Send a goal with the **Nav2 Goal** tool.

### Mapping in simulation

There is no simulation mapping launch file. Start Gazebo, then SLAM Toolbox
with the Walkie parameters. Run from the repository root:

```bash
# Terminal 1
pixi run ros2 launch robot_simulation gzsim_omnibot.launch.py
```

```bash
# Terminal 2
pixi run ros2 launch slam_toolbox online_sync_launch.py use_sim_time:=true \
  slam_params_file:=$PWD/navigation/walkie-navigation/config/slam/slam_toolbox_params.yaml
```

Drive the robot around (see
[Gazebo Simulation](simulation-gazebo.md#driving-the-robot)), then save the
map. `map_saver_cli` writes `map.yaml` and `map.pgm` with the given prefix:

```bash
pixi run ros2 run nav2_map_server map_saver_cli -f /tmp/my_map/map
```

!!! warning
    Do not use `slam_toolbox.launch.py`. It does not load the Walkie
    parameters. See [Known issues](#known-issues).

### Test goal logic without physics

```bash
pixi run ros2 launch robot_navigation nav2_loopback.launch.py
```

`nav2_loopback.launch.py` runs Nav2 with `nav2_loopback_sim`, which moves the
robot exactly as commanded and needs no physics or sensors. Use it to test
behavior trees and goal logic quickly. `nav_commander` is commented out in this
file, so `/navigate_to_object` is not available in loopback mode.

1. Set an initial pose with the **2D Pose Estimate** tool in RViz. Until you
   do, the loopback simulator publishes no transforms, and Nav2 does not
   finish activating.
2. Send a goal with the **Nav2 Goal** tool.

Without RViz, publish the initial pose yourself, for example at the map
origin:

```bash
pixi run ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {orientation: {w: 1.0}}}}"
```

??? info "Loopback setup"
    The launch file uses `ros2_control:=mock` for the robot model and
    `nav2_mppi_walkie_loopback_params.yaml`, which uses plain `Twist`
    everywhere. RViz opens by default (`use_rviz:=true`) with `nav2_bringup`'s
    `nav2_default_view.rviz`.

    Two lifecycle managers start the nodes: `lifecycle_manager_map` for
    `map_server`, and `lifecycle_manager_nav2` for the Nav2 servers. The
    reason is the same as for the real-robot launch files (see
    [Localization and mapping on the real robot](#localization-and-mapping-on-the-real-robot)).
    There is no `twist_mux`. The parameter file chains the commands the
    upstream way: `/cmd_vel_nav` to `velocity_smoother`, `/cmd_vel_smoothed` to
    `collision_monitor`, and `/cmd_vel` to `loopback_simulator`.
    `docking_server` and `following_server` publish on `/cmd_vel` directly.

??? info "Why the loopback simulator needs an initial pose and sim time"
    Since the navigation2 bump, `nav2_loopback_sim` is a C++ Nav2 lifecycle
    node (it was a Python node). No lifecycle manager manages it. The launch
    file sets `autostart_node: true`, so it configures and activates itself.
    Before navigation `e5d33c2`, nothing configured it, so it published no
    transforms, and Nav2 never activated.

    It publishes the `map` to `odom` to `base_footprint` transforms only after
    it receives an initial pose on `/initialpose`. Until then,
    `local_costmap` waits for the `odom` to `base_footprint` transform, and
    `lifecycle_manager_nav2` does not finish activating. This is upstream
    behavior.

    The launch file also sets `use_sim_time: true`, as upstream's
    `loopback_simulation.launch.py` does. Without it, the transforms carry
    wall time and the scan carries sim time. `collision_monitor` then logs
    `Failed to get "base_footprint"->"base_footprint" frame transform: Lookup would require extrapolation into the past`
    and stops on an invalid source.

    navigation `e5d33c2` also changed `nav2_mppi_walkie_loopback_params.yaml`:
    it adds `compute_path_heading_spin_bt_node` to `plugin_lib_names`, sets
    `enable_stamped_cmd_vel: false` on `behavior_server`, and adds a
    `following_server` block with the same setting. Before, both servers
    published `TwistStamped` into the plain `Twist` chain, so their commands
    never reached the simulator.

    On 2026-10-14, `lifecycle_manager_nav2` was active after 16 s, once an
    initial pose at (0, 0) was published on `/initialpose`. A goal sent with `/goal_pose` and a `navigate_to_pose`
    action goal were both reached, within 0.04 m and 0.5° by `/odom`.

## Navigate in MuJoCo

```bash
pixi run ros2 launch robot_bringup nav_mujoco_omnibot.launch.py
```

This starts the MuJoCo simulation and Nav2 on a blank 20 m x 20 m map
(`bringup/config/map/empty_floor.yaml`). Send a goal with the **2D Goal Pose**
tool in RViz. The setup, the launch arguments, and custom worlds are in
[MuJoCo Simulation: Drive with Nav2 and RViz](simulation-mujoco.md#drive-with-nav2-and-rviz).

Nav2 drives the robot in the default MuJoCo world (`scene.xml`), which has no
obstacles. `mujoco_omnibot.launch.py` sets the merger parameter
`publish_on_empty_cloud: True`, so `/scan` is published even when neither
lidar sees anything in range. Without it, the merger publishes nothing in the
empty world, and the collision monitor, which is now in the command path,
stops every Nav2 command. The real robot keeps the default `false` (see
[Dual 2D lidar](#dual-2d-lidar)).

- There is no AMCL. A static identity transform from `map` to `odom` makes the
  wheel odometry the robot pose in the map. The robot must start at the world
  origin, and the map origin must match the world origin. The pose drifts as
  the odometry drifts: 0.17 to 0.43 m against the MuJoCo ground truth after
  one or two goals in the 2026-10-14 tests.
- Nav2 uses `nav2_mppi_walkie_sim_params.yaml`, the same file as Gazebo,
  passed through unchanged. Like the Gazebo and real-robot launch files, it
  remaps the Nav2 `cmd_vel` output to `/cmd_vel_controller`, so commands pass
  the velocity smoother and the collision monitor (see
  [Velocity command path](#velocity-command-path)).
- It starts Nav2 with `use_composition:=False` (see
  [Nav2 bringup hangs at smoother_server/get_state](#nav2-composition-deadlock)).
- `nav_commander` is not started, so `/navigate_to_object` and the RViz
  **Publish Point** tool (`/clicked_point`) have no effect. Send goals with
  **2D Goal Pose** or as `NavigateToPose` action goals.

## Localization and mapping on the real robot

Start the robot first with `real_omnibot.launch.py`. It starts the lidars and
the wheel controller. See [Real Robot](real-robot.md).

| Task | Command |
| --- | --- |
| Localize on a saved map | `pixi run ros2 launch robot_navigation localization_Nav2_real.launch.py` |
| Build a new map | `pixi run ros2 launch robot_navigation mapping_Nav2_real.launch.py` |
| Save the map while mapping | `pixi run ros2 run nav2_map_server map_saver_cli -f <map_dir>/map` |

!!! warning "Run these launch files on `rmw_zenoh_cpp`"
    Under Fast DDS (`rmw_fastrtps_cpp`, the ROS 2 default when
    `RMW_IMPLEMENTATION` is not set), `controller_server` fails to configure in
    both launch files, with
    `create_publisher() called for existing topic name rt/local_costmap/voxel_grid with incompatible type`.
    Both start on `rmw_zenoh_cpp`, the team middleware (see
    [Middleware (Zenoh)](middleware.md)). See
    [Known issues](#known-issues).

- Localization defaults to the Arena map (`27_01072026_Arena`) with its keepout
  zones. To use another map, see [Choose a map](#choosing-a-map-on-the-real-robot).
- Nav2 runs during mapping too, so you can send goals while the map grows.
- Both launch files start their own lifecycle manager, `lifecycle_manager_nav2`,
  which configures and activates the Nav2 servers on startup. SLAM Toolbox
  manages its own lifecycle.
- To rebuild maps later without driving again, record a rosbag during mapping
  with `scripts/map/` ([mapping rosbag guide](https://github.com/EIC-Robocup-2026/walkie-ros/blob/main/scripts/map/README.md)).
- Use these launch files, not `scripts/bringup/real_slam_nav.sh`. See
  [Known issues](#known-issues).

??? info "What the real-robot launch files start"
    `localization_Nav2_real.launch.py` includes `nav2_bringup`'s
    `localization_launch.py` (map server and AMCL) and `navigation_launch.py`
    (the Nav2 servers). It also starts:

    - `lifecycle_manager_nav2` for `map_server`, `amcl`, and the Nav2 servers
    - `head_tilt_near_goal_node.py`, which tilts the ZED head down near the
      goal so tabletops stay in view
    - `nav_commander`
    - a keepout mask server, a costmap filter info server, and their lifecycle
      manager (`lifecycle_manager_keepout`), when `use_keepout_zones` is `true`

    `mapping_Nav2_real.launch.py` includes SLAM Toolbox's
    `online_sync_launch.py` with `config/slam/slam_toolbox_params.yaml`, and
    `navigation_launch.py`. It also starts `lifecycle_manager_nav2` for the
    Nav2 servers, `head_tilt_near_goal_node.py`, and `nav_commander`. There is
    no AMCL and no map server: SLAM Toolbox publishes `/map` and the `map` to
    `odom` transform.

    Both launch files wrap the `navigation_launch.py` include with a `cmd_vel`
    remap (see [Velocity command path](#velocity-command-path)).

??? info "Why the launch files start their own lifecycle manager"
    navigation2 commit `a1904cdc` ("Combine lifecycle managers",
    [#6410](https://github.com/ros-navigation/navigation2/pull/6410)), part of
    the bumped navigation2 (`e48c3296`), removed the lifecycle managers from
    `nav2_bringup`'s `navigation_launch.py` and `localization_launch.py`. Only
    `bringup_launch.py` starts one now, named `lifecycle_manager_nav2`.

    `localization_Nav2_real.launch.py`, `mapping_Nav2_real.launch.py`, and
    `nav2_loopback.launch.py` include those files directly. Before
    [5. Repair Nav2 After the navigation2 Bump](../migrations/05-nav2-repair.md),
    their Nav2 nodes started but nothing configured them, and the log showed
    `Waiting on external lifecycle transitions to activate`.

    Each of the three files now starts `lifecycle_manager_nav2` with
    `autostart: true` from an `OpaqueFunction`. The node list comes from
    `nav2_bringup.navigation_launch.get_lifecycle_nodes()`, so it follows the
    Nav2 servers that `navigation_launch.py` starts. The name matches
    `nav2_bringup`'s manager, which the RViz Nav2 panel talks to.

    | Launch file | Lifecycle managers |
    | --- | --- |
    | `localization_Nav2_real.launch.py` | `lifecycle_manager_nav2` (`map_server`, `amcl`, Nav2 servers), `lifecycle_manager_keepout` (keepout servers) |
    | `mapping_Nav2_real.launch.py` | `lifecycle_manager_nav2` (Nav2 servers). SLAM Toolbox manages its own lifecycle. |
    | `nav2_loopback.launch.py` | `lifecycle_manager_nav2` (Nav2 servers), `lifecycle_manager_map` (`map_server`) |
    | `localization_Nav2_sim.launch.py`, `nav_mujoco_omnibot.launch.py` | `lifecycle_manager_nav2` from `bringup_launch.py`. Not affected by the change. |

### Choose a map { #choosing-a-map-on-the-real-robot }

Pass the map and its keepout mask together. Run from the repository root so
`$PWD` is correct:

```bash
MAP_DIR=$PWD/navigation/walkie-navigation/map/22_27062026_AirBnbHouse_4
pixi run ros2 launch robot_navigation localization_Nav2_real.launch.py \
  map:=$MAP_DIR/map.yaml keepout_mask:=$MAP_DIR/map_keepout.yaml
```

For a map without a keepout mask, turn the keepout servers off:

```bash
MAP_DIR=$PWD/navigation/walkie-navigation/map/20_27062026_AirBnbHouse_2
pixi run ros2 launch robot_navigation localization_Nav2_real.launch.py \
  map:=$MAP_DIR/map.yaml use_keepout_zones:=false
```

!!! warning
    If you change only `map`, the Arena keepout mask stays active on a
    different map.

Maps are in `navigation/walkie-navigation/map/`, named
`<number>_<date>_<place>`. The maps of the current robot (from `19` on) each
have `map.yaml` and `map.pgm`. The keepout mask file name is not the same in
every directory, so pass it explicitly:

| Directory | Keepout mask | Other files |
| --- | --- | --- |
| `19_25062026_AirBnbHouse_1` | | |
| `20_27062026_AirBnbHouse_2` | | |
| `21_27062026_AirBnbHouse_3` | `map_keepout.yaml` | Map editor files |
| `22_27062026_AirBnbHouse_4` | `map_keepout.yaml` | Map editor files |
| `23_29062026_AirBnbHouse2_1` | | `map.ot` (OctoMap) |
| `24_30062026_Arena2_1` | | `map.ot` |
| `25_30062026_Arena2_2` | `keepout.yaml` | Map editor files |
| `26_01072026_Arena2_3` | | `map.ot` |
| `27_01072026_Arena` | `keepout.yaml` | Map editor files. Default for the real robot. |
| `28_15072026_sasinHall` | `keepout.yaml` | `map.ot`, `map_og.pgm` |

Directories `01` to `18` hold older maps: TurtleBot3 maps, the simulation map
`08_23102025_SimWalkie_awssmallhouse_slam_toolbox`, and earlier venues. Some of
them name the map file after a time stamp or the place instead of `map.yaml`.

## Send navigation goals { #goal-interfaces }

### `/navigate_to_object` action

`nav_commander.py` turns an object position (`obj_x`, `obj_y` in the `map`
frame) into a Nav2 goal in front of the object. It finds the table edge in the
local costmap and drives to a pose that faces the edge. The localization and
mapping launch files start it, except the loopback one and
`nav_mujoco_omnibot.launch.py`.

```bash
pixi run ros2 action send_goal /navigate_to_object robot_navigation/action/NavigateToObject \
  "{obj_x: 1.0, obj_y: -3.0, align_method: '', standoff: 0.0}" --feedback
```

- You can also use the **Publish Point** tool in RViz. `nav_commander` listens
  on `/clicked_point` and runs the same pipeline.
- Set `nav_debug:=true` on the launch file to publish debug markers.
- Its parameters are in `config/walkie_nav.yaml`. More detail is in
  `navigation/walkie-navigation/README.md`.

??? info "`NavigateToObject` action fields"
    | Field | Type | Meaning |
    | --- | --- | --- |
    | `obj_x`, `obj_y` | `float64` | Object position in the `map` frame |
    | `align_method` | `string` | Empty uses `default_method` (`nearest_edge`) |
    | `standoff` | `float64` | 0 or less uses the `standoff` parameter (0.1 m) |
    | Result `success`, `message` | `bool`, `string` | |
    | Feedback `distance_remaining`, `x`, `y`, `theta` | `float32` | |

### `/move_robot` service

`Sgoal_Service_node` offers a simpler goal service. No launch file starts it,
so start it yourself, then call it:

```bash
pixi run ros2 run robot_navigation Sgoal_Service_node
```

```bash
pixi run ros2 service call /move_robot robot_navigation/srv/MoveRobot \
  "{x: 2.0, y: 1.0, theta: 0.0}"
```

!!! warning
    `success: true` means the goal was sent, not that the robot arrived. The
    node also returns `true` when the Nav2 action server is offline. It only
    logs an error in that case.

??? info "How `/move_robot` picks and sends the goal"
    The request fields `x`, `y`, and `theta` are `float32`. The response has
    `success` (`bool`) and `message` (`string`).

    The node reads `/global_costmap/costmap`. A goal cell counts as free when
    its cost is below 50 and not unknown. If the goal is blocked, the node
    searches up to 2.0 m around it for a free cell and turns the robot to face
    the original goal. Then it sends a `navigate_to_pose` goal and returns
    without waiting for the result.

## Velocity command path

Nav2 commands pass the velocity smoother and the collision monitor, then reach
`twist_mux` on `/cmd_vel_nav` with priority 10. Joystick and keyboard topics
(priority 100) override Nav2 and do not pass the collision monitor. See
[Gazebo Simulation](simulation-gazebo.md#driving-the-robot).

The chain is `/cmd_vel_controller` (Nav2 servers) to `velocity_smoother`,
`/cmd_vel_smoothed` to `collision_monitor`, `/cmd_vel_nav` to `twist_mux`, and
`/omni_wheel_drive_controller/cmd_vel` to the wheel controller. It applies to
`localization_Nav2_sim.launch.py`, `localization_Nav2_real.launch.py`,
`mapping_Nav2_real.launch.py`, and `nav_mujoco_omnibot.launch.py`.

!!! warning "The collision monitor stops and slows the robot"
    Since [5. Repair Nav2 After the navigation2 Bump](../migrations/05-nav2-repair.md),
    the collision monitor acts on every Nav2 command. Joystick and keyboard
    commands bypass it.

    - **Stop:** the stop polygon follows the direction of the command. A
      command toward an obstacle within reach (up to 0.35 m from the base
      center, on the side the robot drives to) is set to zero. A command that
      drives away from the obstacle or along it passes.
    - **Slow down:** with 4 or more points within 0.55 m of the base center,
      the robot moves at 40 % speed. Narrow passages are slow.
    - **Invalid source:** when a source has no new data for 2 s, or its frame
      cannot be transformed to `base_footprint`, the monitor stops every Nav2
      command and logs `Robot to stop due to invalid source`. The causes are
      in [Troubleshooting](../troubleshooting.md#nav2-robot-stops-near-obstacles).
    - **Speed limit:** the velocity smoother caps the output at 0.3 m/s and
      1.0 rad/s, below the MPPI `wz_max` of 1.5 rad/s.

    For the stop polygons, the blind zones, and what can still block Nav2,
    see [Collision monitor](#collision-monitor).

    It has not been tested on the real robot.

To see why the robot stopped or slowed down, check the `collision_monitor` log.
It logs an INFO line each time its action changes, such as
`Robot to stop due to VelocityPolygonStop polygon`. The line does not name the
direction band. In RViz, add the polygon topics `velocity_polygon_stop` and
`polygon_slowdown`. `velocity_polygon_stop` shows only the band selected by
the current command, and it stays empty until the first Nav2 command arrives.
`bringup/config/rviz/default.rviz`, which `real_omnibot.launch.py` opens,
shows both topics. The MuJoCo RViz configurations do not.

??? info "Velocity topic diagram"
    ```mermaid
    flowchart LR
        C["controller_server<br/>behavior_server<br/>docking_server<br/>following_server"] -->|"/cmd_vel_controller"| S[velocity_smoother]
        S -->|"/cmd_vel_smoothed"| CM[collision_monitor]
        CM -->|"/cmd_vel_nav"| M[twist_mux]
        J["joystick, keyboard"] -->|"/cmd_vel_joy, /cmd_vel_key"| M
        M -->|"/omni_wheel_drive_controller/cmd_vel"| W[omni_wheel_drive_controller]
    ```

??? info "How the launch files route Nav2 commands through the collision monitor"
    `nav2_bringup` remaps `cmd_vel` to `cmd_vel_nav` on `controller_server`,
    `behavior_server`, and `velocity_smoother`. The Walkie launch files that
    run Nav2 for the robot wrap the `nav2_bringup` include in a group with a
    global remap:

    ```python
    GroupAction([
        SetRemap(src="cmd_vel", dst="cmd_vel_controller"),
        IncludeLaunchDescription(...),
    ])
    ```

    `launch_ros` puts `SetRemap` rules before each node's own remappings, and
    rcl applies the first rule that matches. So the Nav2 servers publish on
    `/cmd_vel_controller`, and `velocity_smoother` reads its `cmd_vel` input
    from there. No copy of the `nav2_bringup` launch files is needed, so the
    remap survives navigation2 bumps.

    Before entry 5, the Nav2 servers published on `/cmd_vel_nav` straight into
    `twist_mux`. `velocity_smoother` wrote to `/cmd_vel_smoothed`, which had
    no subscriber, and `collision_monitor` read `/cmd_vel_controller`, which
    had no publisher. The parameter files set
    `controller_server.cmd_vel_topic` and the `velocity_smoother` parameters
    `cmd_vel_in_topic` and `cmd_vel_out_topic`, but Nav2 has none of these
    parameters. Entry 5 removed them, and removed
    `docking_server.enable_stamped_cmd_vel: false`, which would publish
    unstamped `Twist` onto the stamped `/cmd_vel_controller`.

    `nav2_loopback.launch.py` does not use the remap. Its parameter file
    chains the commands the upstream way and has no `twist_mux` (see
    [Test goal logic without physics](#test-goal-logic-without-physics)).

### Collision monitor

The collision monitor checks its sources against three zones, all in the
`base_footprint` frame:

- `VelocityPolygonStop`: a direction-aware stop. The monitor picks one stop
  polygon from the direction of the command. Each polygon covers only the
  side of the footprint the robot drives to, up to 0.35 m from the base
  center.
- `PolygonSlow`: a circle of 0.55 m. With 4 or more points in it, the robot
  moves at 40 % speed.
- `FootprintApproach`: the static physical footprint, checked 2 s ahead. It
  slows the robot down as it approaches an obstacle.

Near an obstacle, the robot behaves like this:

- **Driving toward the obstacle:** the robot slows down and stops before it.
  In the MuJoCo test, it stopped 0.045 to 0.053 m before a wall in front, and
  about 0.11 m before a wall at the side.
- **Driving away from the obstacle or along it:** the command passes, so Nav2
  can drive the robot on after a stop.
- **Rotating in place:** blocked only when points are inside the rotation
  polygon, just outside the circle that the footprint sweeps.

What the stop polygons see depends on the data each source delivers:

- **`/scan` in MuJoCo and on the real robot.** The merged scan has no points
  closer than 0.30 m to the base center: the merger uses `range_min: 0.3` in
  `merged_lidar_link`, which sits at the base center. The footprint front is
  at x = 0.30 m, so in front the stop polygon sees `/scan` points only 0 to
  5 cm ahead of the footprint. At the sides, the footprint edge is 0.239 m
  from the center, and obstacles 0.239 to 0.30 m from the center are not in
  `/scan` at all. The rotation polygon sees `/scan` points only 0.30 to
  0.31 m (simulation file) or 0.30 to 0.336 m (real and restaurant files)
  from the center.
- **The front lidar in MuJoCo.** `front_lidar` sits at x = 0.235 m and has a
  `range_min` of 0.18 m (`bringup/config/mujoco/mujoco_plugins.yaml`), so
  straight ahead it sees nothing closer than 0.415 m from the base center. A
  wall is still detected, because its points off to the side are farther
  from the lidar. A narrow obstacle dead ahead (narrower than about 0.28 m)
  closer than 0.415 m is missed. This is a limit of the MuJoCo setup. The
  minimum ranges of the real Hokuyo and Lakibeam were not checked.
- **The 3D lidar on the real robot.** The `pointcloud` source
  (`/unilidar/cloud/filtered`, 0.3 to 1.5 m high) has no 0.30 m limit, so the
  stop polygons also see closer obstacles at that height, such as tabletops.
  This has not been tested on the robot.
- **Gazebo.** The default simulated lidar has a minimum range of 0.6 m, so
  neither the stop polygons nor `PolygonSlow` trigger. Only
  `FootprintApproach` acts, on obstacles that are still farther than 0.6 m.
  This follows from the configuration; Gazebo was not run.
- **`nav2_mppi_walkie_real_restaurant_param.yaml`** (used by
  `mapping_Nav2_real.launch.py`). Its local footprint has the padding built
  in: 0.10 m at the front and sides (front at x = 0.40 m, half width
  0.339 m), 0.02 m at the rear. The stop polygons and `FootprintApproach`
  use the physical footprint instead, so the monitor no longer enforces this
  padding. Only the costmaps keep that distance.

All polygons cover the base only, not the arms or anything the robot carries.

!!! warning "What can still block Nav2 near an obstacle"
    The monitor sets the Nav2 commands to zero, and goals abort with error 105
    (`Failed to make progress`), when:

    - every direction Nav2 tries points at an obstacle in reach, for example
      when the robot is boxed in
    - parts of the robot or sensor noise put 4 or more points into the
      selected band or the rotation polygon
    - a source is invalid (no new data for 2 s, or no transform)

    The behavior tree has no BackUp recovery, so Nav2 does not reverse out by
    itself. It moves away only when MPPI picks a command away from the
    obstacle. To free the robot, drive it with the joystick or keyboard.
    `twist_mux` gives them priority 100, and they bypass the collision
    monitor. See
    [Troubleshooting](../troubleshooting.md#nav2-stuck-after-a-stop).

??? info "How the stop polygon is chosen"
    The monitor picks the sub-polygon from the command that enters it on
    `/cmd_vel_smoothed`, not from odometry. With `holonomic: true`, it
    compares the speed hypot(vx, vy) with `linear_min` and `linear_max`, and
    the direction atan2(vy, vx) with `direction_start_angle` and
    `direction_end_angle`. It uses the first sub-polygon in
    `velocity_polygons` that matches. There is no hysteresis.

    `rotation` is first in the list, so it applies at 0.01 m/s or less. The
    eight translation bands are 45° wide each and cover any speed up to
    10 m/s. Angles are in `base_footprint`: 0° is forward (+x), 90° is left
    (+y).

    | Sub-polygon | Command direction | Polygon in `base_footprint` (m) |
    | --- | --- | --- |
    | `rotation` | Any, at 0.01 m/s or less (rotation in place or standing still) | 12-gon just outside the footprint's rotation sweep |
    | `forward` | -22.5° to 22.5° | Rectangle x 0 to 0.35, y -0.26 to 0.26 |
    | `forward_left` | 22.5° to 67.5° | L-shape: front strip x 0 to 0.35 (y -0.239 to 0.35), plus left strip y 0 to 0.35 (x -0.285 to 0.35) |
    | `left` | 67.5° to 112.5° | Rectangle y 0 to 0.35, x -0.305 to 0.32 |
    | `backward_left` | 112.5° to 157.5° | L-shape: rear strip x -0.35 to 0, plus left strip y 0 to 0.35 |
    | `backward` | 157.5° to -157.5° (through 180°) | Rectangle x -0.35 to 0, y -0.26 to 0.26 |
    | `backward_right` | -157.5° to -112.5° | L-shape: rear strip plus right strip |
    | `right` | -112.5° to -67.5° | Rectangle y -0.35 to 0, x -0.305 to 0.32 |
    | `forward_right` | -67.5° to -22.5° | L-shape: front strip plus right strip |

    Each band polygon covers only the footprint faces that lead in the
    commanded direction, up to 0.35 m from `base_footprint` (the reach of the
    former `PolygonStop` circle). The axis bands widen the leading face by
    0.02 m per side for sideways drift within the band. The diagonal bands
    cover both leading faces and the corner between them. A command that
    drives away from an obstacle selects a polygon that does not contain it,
    so the command passes.

    The bands use the bounding box of the physical footprint (front
    x = 0.30 m, rear x = -0.285 m, half width 0.239 m), which the simulation
    and real footprints share. So all three files have the same band
    polygons. In the restaurant file, they are deliberately based on the
    physical footprint, not on its padded local footprint.

    The `rotation` 12-gon has an apothem of the largest footprint radius plus
    0.01 m:

    | File | Apothem |
    | --- | --- |
    | `nav2_mppi_walkie_sim_params.yaml` | 0.30 + 0.01 = 0.31 m |
    | `nav2_mppi_walkie_real_param.yaml`, `nav2_mppi_walkie_real_restaurant_param.yaml` | 0.326 + 0.01 = 0.336 m |

    Why 8 bands, not 4: a layout with 4 bands (forward, left, backward, right,
    90° each) left a gap. In MuJoCo, a diagonal command toward a wall closed
    the gap to 0.001 m, and the commands away from the wall and along it were
    then blocked completely. With 8 bands, commands within 67.5° of a wall's
    normal are blocked when the wall is in reach. An offline check of every
    22.5° direction against walls on each side and in corners found no
    direction that stayed blocked while pointing away from every wall, and no
    direction within 45° of a wall's normal that passed. The real and
    restaurant files gave the same result.

??? info "Stop and direction tests in MuJoCo (2026-10-14)"
    The tests ran `nav_mujoco_omnibot.launch.py` headless in a walled test
    world that is not in the repository: an 8 m by 8 m room, and a 1.2 m wall
    whose face is at x = 1.45 m. At the start (y = 0), the robot's left side
    was 3.7 m from the north wall. Commands were published on
    `/cmd_vel_controller`, so they passed the velocity smoother and the
    collision monitor. Gaps were
    measured with MuJoCo ground truth, from the footprint bounding box (front
    x = 0.30 m, half width 0.239 m) to the wall face.

    **Forward stop.** A constant forward command of 0.25 m/s toward the wall
    face gave this output on `/cmd_vel_nav`:

    | Time | `/cmd_vel_nav` |
    | --- | --- |
    | Until about 4 s | 0.25 m/s |
    | From about 4.2 s | 0.10 m/s (`PolygonSlow`, 40 %) |
    | Around 6 s | About 0.05 m/s (`FootprintApproach`) |
    | From about 7.1 s | 0 |

    The base stopped at x = 1.105 m, with the footprint front 0.045 m before
    the wall face (x = 1.104 m and 0.046 m in a repeat run). From the repeat
    run's stop, a goal to (1.5, 1.0, 90°), sent as a `navigate_to_pose`
    action goal, succeeded. The
    behavior tree reported success 28.8 s after the start, 0.072 m and 4.0°
    from the goal by ground truth. On the way, `follow_path` aborted twice
    with error 105 (at 16.5 s and 24.6 s), and the behavior tree retried both
    times.

    **Direction test.** Each phase published one command for 2 s or until the
    robot stopped:

    | Command | Result |
    | --- | --- |
    | Toward the wall face: forward, then diagonal (+x, +y) | Stopped 0.048 m and 0.053 m before it |
    | Toward the north wall: left, then diagonal (+x, +y) | Stopped 0.112 m and 0.114 m before it |
    | Away from a wall, four times (straight and diagonal) | Moved 0.155 to 0.292 m |
    | Along the wall, with the wall 0.229 m from the side | Moved 0.161 m |
    | Rotate in place at the wall, +0.5 rad/s, then -0.5 rad/s | Rotated 21.6° in 2 s, then back; not stopped, but slowed to 40 % by `PolygonSlow` |

    Then two goals were sent from beside the north wall. The first,
    (1.5, 1.0, 90°) on `/goal_pose`, was within tolerance after 17.3 s
    (0.058 m and 4.6° off by ground truth), after one `follow_path` abort
    with error 105 during the slow final approach. The second,
    (-1.0, -0.5, 0°) as an action goal, succeeded in 16.5 s. It was 0.098 m
    off by odometry and 0.363 m by ground truth, because the odometry drifts
    and MuJoCo has no AMCL. With an earlier 4-band layout and a 4 s progress
    allowance, the same goals had 4 to 5 aborts with error 105.

    Before the change, with the 0.35 m `PolygonStop` circle, the robot stopped
    with its footprint front at the wall (x = 1.155 m), and two goals sent
    after the stop both aborted with error 105, after about 120 s and 88 s.

??? info "Collision monitor, velocity smoother, and progress checker settings"
    The settings are in the `collision_monitor`, `velocity_smoother`, and
    `controller_server` sections of the parameter files in
    `navigation/walkie-navigation/config/nav2/`.
    `nav2_mppi_walkie_sim_params.yaml`, `nav2_mppi_walkie_real_param.yaml`,
    and `nav2_mppi_walkie_real_restaurant_param.yaml` use the same values,
    except for the observation sources, the size of the `rotation` polygon,
    and the `FootprintApproach` points.

    | Component | Setting |
    | --- | --- |
    | `velocity_smoother` | `max_velocity` [0.3, 0.3, 1.0], `max_accel` [2.5, 2.5, 3.2]. Input `cmd_vel` (remapped to `/cmd_vel_controller`), output `cmd_vel_smoothed`. Both topic names are fixed in code. |
    | `collision_monitor` topics | `cmd_vel_in_topic: "cmd_vel_smoothed"`, `cmd_vel_out_topic: "cmd_vel_nav"` |
    | `polygons` | `["VelocityPolygonStop", "PolygonSlow", "FootprintApproach"]` |
    | Observation sources | Simulation file: `scan`. Real and restaurant files: `scan` and `pointcloud` (`/unilidar/cloud/filtered`, `min_obstacle_height` 0.3 m, `max_obstacle_height` 1.5 m). |
    | `source_timeout` | Not set, so the default of 2.0 s applies |
    | `VelocityPolygonStop` | `velocity_polygon`, `stop`, `min_points: 4`, `holonomic: true`. Nine sub-polygons: `rotation` and eight direction bands. Topic `velocity_polygon_stop`. |
    | `PolygonSlow` | Circle, radius 0.55 m, `slowdown`, `slowdown_ratio: 0.4`, `min_points: 4`. Topic `polygon_slowdown`. |
    | `FootprintApproach` | `approach` with static `points`: the physical footprint without padding (simulation file: the local footprint points; real and restaurant files: the real footprint with notched rear corners). `time_before_collision: 2.0` s, `min_points: 6`. |
    | Progress checker | `progress_checker` in `controller_server`: `nav2_controller::SimpleProgressChecker`, `required_movement_radius: 0.5`, `movement_time_allowance: 8.0` (0.5 m within 8 s) |
    | `twist_mux` | `bringup/config/twist_mux/twist_mux.yml`: `cmd_vel_nav` priority 10, `cmd_vel_joy` and `cmd_vel_key` priority 100 |

    `FootprintApproach` does not use the padded
    `/local_costmap/published_footprint`. With 6 or more points inside its
    polygon, the approach action computes a collision time of 0 and scales
    every command to zero, also commands that drive away. With the padded
    footprint, obstacles inside the padding next to the robot froze Nav2.

    The progress checker allows 8 s, not 4 s: 0.5 m in 4 s needs 0.125 m/s on
    average, more than the `PolygonSlow` limit of 0.4 x 0.3 = 0.12 m/s, so
    goals through the slow zone aborted `follow_path` with error 105. The cost
    is that a robot that is really stuck is detected only after 8 s.

    The other parameter files keep the earlier setup: `PolygonStop`,
    `FootprintApproach` on `/local_costmap/published_footprint`, and
    `movement_time_allowance: 10.0`. Their collision monitors use `scan` only.

    - `nav2_mppi_walkie_loopback_params.yaml`: `PolygonStop` (0.35 m) is
      disabled. `PolygonSlow` has radius 0.40 m and `slowdown_ratio: 0.5`.
      The progress checker needs 0.2 m within 10 s.
    - `nav2_mppi_walkie_real_param_hero_10_04_26.yaml`: `PolygonStop`
      (0.35 m) is active, so after a stop it blocks every Nav2 command, also
      commands that drive away. `PolygonSlow` has radius 0.50 m and
      `slowdown_ratio: 0.5`.
    - `nav2_RPP_*.yaml` (three files, no launch file): `PolygonStop` is
      active.

## Nav2 configuration

To try another parameter file, pass it with `nav2_config:=<full path>`
(`params_file:=<full path>` for `nav_mujoco_omnibot.launch.py`). The files are
in `navigation/walkie-navigation/config/nav2/`.

??? info "Which launch file uses which parameter file"
    | File in `config/nav2/` | Used by |
    | --- | --- |
    | `nav2_mppi_walkie_sim_params.yaml` | `localization_Nav2_sim.launch.py`, `nav_mujoco_omnibot.launch.py` |
    | `nav2_mppi_walkie_real_param.yaml` | `localization_Nav2_real.launch.py` |
    | `nav2_mppi_walkie_real_restaurant_param.yaml` | `mapping_Nav2_real.launch.py` |
    | `nav2_mppi_walkie_loopback_params.yaml` | `nav2_loopback.launch.py` |
    | `nav2_mppi_walkie_real_param_hero_10_04_26.yaml` | No launch file. A snapshot of the real parameters from 2026-04-10. Starts with `localization_Nav2_real.launch.py nav2_config:=<full path>` since navigation `e5d33c2`. |
    | `nav2_RPP_*.yaml` | No launch file |

    Before navigation `e5d33c2`, `bt_navigator` failed to configure with the
    hero file (`Node not recognized: ComputePathHeadingSpin`, then
    `"spin" action server not available`). Its behavior tree,
    `nav_to_pose_real_recovery_bt.xml`, has since gained `ComputePathHeadingSpin`
    and `Spin`. The commit adds the three custom behavior tree plugins to
    `plugin_lib_names` and the `spin` behavior (`nav2_behaviors::Spin`). No
    tuning value changed. The hero file has no STVL layer, so it also starts
    under Fast DDS.

!!! note
    Nav2 uses the behavior tree set in the parameter file
    (`default_nav_to_pose_bt_xml`). The launch files pass a
    `default_nav_to_pose_bt_xml` launch argument, but the included Nav2 launch
    files do not declare it, so it has no effect.

??? info "Main settings shared by the simulation and real files"
    | Component | Setting |
    | --- | --- |
    | AMCL | `nav2_amcl::OmniMotionModel`. `set_initial_pose: true` at (0, 0, 0). |
    | Controller | MPPI with `motion_model: "Omni"` and `Omni.plugin: "mppi::OmniMotionModel"`. `vx_max` and `vy_max` 0.3 m/s, `wz_max` 1.5 rad/s. |
    | Footprint | `nav2_mppi_walkie_sim_params.yaml` and `nav2_mppi_walkie_real_param.yaml`, local and global: polygon from x = -0.285 m to x = 0.30 m, half width 0.239 m (about 0.59 m by 0.48 m). `nav2_mppi_walkie_real_restaurant_param.yaml`, local: larger, with the padding built in, from x = -0.305 m to x = 0.40 m, half width 0.339 m. |
    | Behavior tree | `default_nav_to_pose_bt_xml` is `behavior_tree/nav_to_pose_real_recovery_bt.xml` |
    | BT plugins | `compute_goal_heading_spin_bt_node`, `compute_goal_bearing_spin_bt_node`, `compute_path_heading_spin_bt_node` (built from `robot_navigation`) |

??? info "Costmap layers in simulation and on the real robot"
    | Costmap | Simulation (`nav2_mppi_walkie_sim_params.yaml`) | Real robot (`nav2_mppi_walkie_real_param.yaml`) |
    | --- | --- | --- |
    | Local | Layers `voxel_layer` (`/scan`), `inflation_layer` | Layers `voxel_layer` (`/scan`), `stvl_layer`, `inflation_layer`. Filter `keepout_filter`. |
    | Global | Layers `static_layer`, `obstacle_layer` (`/scan`), `inflation_layer` | Layers `static_layer`, `obstacle_layer` (`/scan`), `inflation_layer`. Filter `keepout_filter`. |
    | Local `inflation_radius` | 0.35 m (circumscribed radius 0.313 m with the default 0.01 m padding) | 0.38 m (circumscribed radius 0.354 m with 0.02 m padding) |
    | Global `inflation_radius` | 0.45 m | 0.45 m |

    The local `inflation_radius` must be at least the circumscribed radius of
    the padded footprint. Otherwise `controller_server` logs
    `[computeCircumscribedCost]: The inflation radius (...) is smaller than the circumscribed radius (...)`
    every cycle, and MPPI checks the full footprint against the costmap
    everywhere. [5. Repair Nav2 After the navigation2 Bump](../migrations/05-nav2-repair.md)
    raised the local values from 0.25 m, and the one in
    `nav2_mppi_walkie_real_restaurant_param.yaml` from 0.30 m to 0.45 m
    (circumscribed radius 0.435 m, no padding). Costs now reach further from
    obstacles, so the robot keeps a little more clearance.

    In `nav2_mppi_walkie_real_param.yaml` and
    `nav2_mppi_walkie_real_restaurant_param.yaml`, `voxel_layer` and
    `stvl_layer` both publish `voxel_grid` with different message types. This
    breaks `controller_server` under Fast DDS (see
    [Known issues](#known-issues)).

## Sensors for navigation

### Dual 2D lidar

AMCL, SLAM Toolbox, and the costmaps use `/scan`. On the real robot, `/scan`
is one 360° scan merged from two 2D lidars: a Hokuyo at the front and a
Lakibeam at the rear. `dual_lidar_hokuyo_lakibeam.launch.py` starts both
drivers and the merger.

!!! warning
    On the real robot, `/scan` stops when one lidar stops publishing, or when
    one lidar has no valid return at all (covered, faulty, or nothing within
    its range in any direction). Two seconds later, the collision monitor
    stops every Nav2 command.

??? info "Why the merger drops the scan when one lidar sees nothing"
    The merger pairs `/front_lidar` and `/back_lidar` by time (approximate
    time synchronization). If one topic is not published at all, there is no
    `/scan`.

    A lidar's cloud is empty when every beam is outside that lidar's range
    limits or invalid. By default, the merger then returns without
    publishing. walkie-ros `ef51b15` added the parameter
    `publish_on_empty_cloud` (bool, default `false`). When it is `true`, the
    merger publishes anyway, and the empty side reads as free space (`inf`
    ranges with `use_inf: true`).

    Only `mujoco_omnibot.launch.py` sets it to `true`, so Nav2 can drive in
    the empty default MuJoCo world. `dual_lidar_hokuyo_lakibeam.launch.py`
    keeps the default on purpose: on the real robot, an empty cloud can mean a
    covered or faulty lidar, and no `/scan` plus a collision monitor stop is
    the safe outcome.

??? info "Lidar driver and merger settings"
    The driver settings are in `bringup/config/lidar/dual_lidar_config.yml`.
    The merger parameters are set in `dual_lidar_hokuyo_lakibeam.launch.py`.

    | Lidar | Position | Driver | Address | Frame | Topic | Field of view |
    | --- | --- | --- | --- | --- | --- | --- |
    | Hokuyo | Front | `urg_node2` | `10.0.0.101:10940` | `front_lidar_link` | `/front_lidar` | ±2.093 rad (±120°) |
    | Lakibeam | Rear | `lakibeam1_scan_node` | Sensor `10.0.0.102`, host `10.0.0.201`, port 2368 | `back_lidar_link` | `/back_lidar` | `scan_range_start: 60`, `scan_range_stop: 300`, `scanfreq: 30` |

    `dual_laser_merger` (`merger_node::MergerNode`) publishes the merged scan
    in `merged_lidar_link`, with ranges from 0.3 to 25 m. `merged_lidar_link`
    is at x = 0, y = 0 of `base_link`, 0.282 m above it, so `/scan` has no points
    closer than 0.30 m to the base center. The merger output is remapped to
    `/scan`.

### Robot self filter

The depth cameras and the 3D lidar see parts of the robot, such as the arms.
`robot_self_filter` removes points that fall inside the robot's collision
geometry from the URDF. `real_omnibot.launch.py` runs up to three instances.

??? info "Self filter instances"
    | Input | Output | Config | Runs when |
    | --- | --- | --- | --- |
    | `/unilidar/cloud` | `/unilidar/cloud/filtered` | `bringup/config/lidar/unitree_self_filter.yaml` | `use_unitree_lidar:=true` (default) |
    | `/camera/camera/depth/color/points` | `/camera/camera/depth/color/points/filtered` | `bringup/config/camera/d415_bottom_self_filter.yaml` | Always |
    | `/zed_head/zed_node/point_cloud/cloud_registered` | `/zed_head/zed_node/point_cloud/cloud_registered/filtered` | `bringup/config/camera/zed2i_self_filter.yaml` | `use_zed:=true` (default) |

### Spatio-Temporal Voxel Layer

The real local costmap uses STVL (`stvl_layer`) to mark obstacles in 3D from
the filtered clouds. Voxels expire after `voxel_decay` seconds, so moving
people do not stay in the costmap. The simulation config does not use STVL.

??? info "STVL settings and the head tilt near the goal"
    STVL stores each voxel with a time stamp. The simulation config has a
    `voxel_layer` fed by `/scan` only.

    Tabletops are above the 2D lidar, so only the cameras and the 3D lidar see
    them. `head_tilt_near_goal_node.py` tilts the head down near the goal so
    the ZED keeps marking the table edge.

    | Setting | Value |
    | --- | --- |
    | `voxel_size` | 0.05 m |
    | `voxel_decay` | 1.0 s |
    | `decay_model` | `0` (linear) |
    | Sources | `/unilidar/cloud/filtered` (0.2 to 1.5 m high), ZED filtered cloud (0.1 to 1.5 m), RealSense filtered cloud (0.1 to 1.5 m) |

### 3D OctoMap

`octomap_3d_mapping.launch.py` builds a 3D OctoMap in the `map` frame while
SLAM Toolbox runs. Its 2D projection on `/projected_map` shows the full
footprint of furniture, which helps when drawing keepout zones.

!!! warning
    This launch file does not start under pixi. See [Known issues](#known-issues).

??? info "Editor export and OctoMap launch arguments"
    `/projected_map` is the occupied space projected to a 2D grid.
    `scripts/save_octomap_for_editor.py` (with the required `--map-dir`
    argument) saves both maps on the same grid for the map editor.

    | Argument | Default |
    | --- | --- |
    | `resolution` | `0.02` |
    | `max_range` | `6.0` |
    | `use_zed` | `true` |
    | `use_realsense` | `true` |
    | `use_unitree` | `false` |
    | `use_sim_time` | `false` |

## Why Nav2 is built from source

`pixi.toml` does not install the Nav2 stack. It is built from
`navigation/navigation2/` (`nav2_bringup` version 1.5.0), pinned to an upstream
`main` commit.

- The first `pixi run build` compiles all of Nav2, so it takes a long time.
- Nav2 behaves like upstream `main`, not like the Jazzy release. When the
  Jazzy documentation and the behavior disagree, check the source in
  `navigation/navigation2/`.
- Nav2 publishes velocity commands as `geometry_msgs/msg/TwistStamped`
  (`enable_stamped_cmd_vel` defaults to `true`). `twist_mux` and the wheel
  controller expect stamped messages too.
- Nav2 links the `behaviortree_cpp` library of the pixi environment. The
  library's SONAME has no version, so Nav2 loads a newer release without an
  error, even when its ABI changed. After a change of the RoboStack build set,
  rebuild the whole workspace with `pixi run clean && pixi run build` (see
  [Changing the build set](pixi.md#changing-the-build-set)).

??? info "Nav2 on behaviortree_cpp 4.10"
    Since [4. Move to RoboStack Build Set _25](../migrations/04-robostack-build-25.md),
    the pixi environment has `behaviortree_cpp` 4.10.0 (it was 4.9.1). All
    Nav2 packages build against it. On 2026-10-13, `bt_navigator` on 4.10.0
    completed navigation goals in MuJoCo (`nav_mujoco_omnibot.launch.py`).

??? info "Why the Jazzy release of Nav2 is not enough"
    The custom behavior tree plugins in `robot_navigation` need this build.
    Their `CMakeLists.txt` requires `nav2_ros_common`, which the Jazzy release
    of Nav2 does not have.

## Where things live

The `robot_navigation` package is in `navigation/walkie-navigation/`. Its
launch files are in `launch/`, the Nav2 parameter files and behavior trees in
`config/nav2/`, and the saved maps in `map/`.

??? info "All navigation paths"
    | Path | Contents |
    | --- | --- |
    | `navigation/` | Git submodule (`EIC-Robocup-2026/navigation`) |
    | `navigation/navigation2/` | Nav2 source, a nested submodule pinned to an upstream `main` commit |
    | `navigation/spatio_temporal_voxel_layer/` | Spatio-Temporal Voxel Layer (STVL) costmap plugin and `openvdb_vendor` |
    | `navigation/walkie-navigation/` | The `robot_navigation` package |
    | `navigation/walkie-navigation/launch/` | Navigation launch files |
    | `navigation/walkie-navigation/config/nav2/` | Nav2 parameter files and behavior trees |
    | `navigation/walkie-navigation/config/slam/` | SLAM Toolbox parameters |
    | `navigation/walkie-navigation/config/walkie_nav.yaml` | `nav_commander` parameters |
    | `navigation/walkie-navigation/map/` | Saved maps, a nested submodule (`EIC-Robocup-2026/map-collection`) |
    | `navigation/walkie-navigation/robot_navigation/` | Nodes and behavior tree plugins |
    | `hardware/lidar/dual_laser_merger/` | Merges the two 2D lidars into `/scan` |
    | `hardware/lidar/robot_self_filter/` | Removes the robot's own body from point clouds |
    | `bringup/launch/dual_lidar_hokuyo_lakibeam.launch.py` | Real lidar drivers and the merger |
    | `bringup/launch/octomap_3d_mapping.launch.py` | 3D OctoMap built next to the 2D SLAM map |
    | `scripts/map/` | Mapping rosbag recorder and replay helper |

## Launch files

`slam_toolbox.launch.py`, `slam_cartographer.launch.py`, and
`octomap_3d_mapping.launch.py` have known problems. See
[Known issues](#known-issues).

??? info "All navigation launch files"
    | Launch file | Package | Purpose |
    | --- | --- | --- |
    | `localization_Nav2_sim.launch.py` | `robot_navigation` | Nav2 with AMCL on a saved map, for Gazebo |
    | `localization_Nav2_real.launch.py` | `robot_navigation` | Nav2 with AMCL on a saved map, plus keepout zones, for the real robot |
    | `mapping_Nav2_real.launch.py` | `robot_navigation` | SLAM Toolbox mapping plus Nav2, for the real robot |
    | `slam_toolbox.launch.py` | `robot_navigation` | SLAM Toolbox only. Does not load the Walkie parameters. |
    | `nav2_loopback.launch.py` | `robot_navigation` | Nav2 with the loopback simulator, no physics |
    | `slam_cartographer.launch.py` | `robot_navigation` | Cartographer. Does not run. |
    | `nav_sim_omnibot.launch.py` | `robot_bringup` | Gazebo plus `localization_Nav2_sim.launch.py` |
    | `nav_mujoco_omnibot.launch.py` | `robot_bringup` | MuJoCo plus Nav2 on a blank map, with odometry-only localization (no AMCL) |
    | `dual_lidar_hokuyo_lakibeam.launch.py` | `robot_bringup` | Real lidar drivers and merger. Started by `real_omnibot.launch.py`. |
    | `octomap_3d_mapping.launch.py` | `robot_bringup` | 3D OctoMap during mapping. Does not start under pixi. |

??? info "Arguments and defaults of the Nav2 launch files"
    `<robot_navigation share>` is the installed share directory of
    `robot_navigation`.

    **`localization_Nav2_sim.launch.py`** includes `nav2_bringup`'s
    `bringup_launch.py` (map server, AMCL, and the Nav2 servers) with
    `use_composition:=False` and starts `nav_commander`.

    | Argument | Default |
    | --- | --- |
    | `use_sim_time` | `true` |
    | `map` | `<robot_navigation share>/map/08_23102025_SimWalkie_awssmallhouse_slam_toolbox/map.yaml` (see [The simulation map](#the-simulation-map)) |
    | `nav2_config` | `<robot_navigation share>/config/nav2/nav2_mppi_walkie_sim_params.yaml` |
    | `nav_debug` | `false` |

    **`localization_Nav2_real.launch.py`**

    | Argument | Default |
    | --- | --- |
    | `use_sim_time` | `false` |
    | `map` | `<robot_navigation share>/map/27_01072026_Arena/map.yaml` |
    | `nav2_config` | `<robot_navigation share>/config/nav2/nav2_mppi_walkie_real_param.yaml` |
    | `nav_debug` | `false` |
    | `use_keepout_zones` | `true` |
    | `keepout_mask` | `<robot_navigation share>/map/27_01072026_Arena/keepout.yaml` |

    **`mapping_Nav2_real.launch.py`**

    | Argument | Default |
    | --- | --- |
    | `use_sim_time` | `false` |
    | `nav2_config` | `<robot_navigation share>/config/nav2/nav2_mppi_walkie_real_restaurant_param.yaml` |
    | `nav_debug` | `false` |

    **`nav2_loopback.launch.py`**

    | Argument | Default |
    | --- | --- |
    | `map` | `<robot_navigation share>/map/08_23102025_SimWalkie_awssmallhouse_slam_toolbox/map.yaml` (see [The simulation map](#the-simulation-map)) |
    | `nav2_config` | `<robot_navigation share>/config/nav2/nav2_mppi_walkie_loopback_params.yaml` |
    | `use_rviz` | `true` (uses `nav2_bringup`'s `nav2_default_view.rviz`) |
    | `nav_debug` | `false` |

??? info "Other nodes in `robot_navigation` and what starts them"
    | Executable | Started by | Purpose |
    | --- | --- | --- |
    | `current_pose_publisher.py` | `nav_sim_omnibot.launch.py`, `real_omnibot.launch.py` | Publishes the `map` to `base_footprint` transform on `current_pose` (`nav_msgs/msg/Odometry`) |
    | `head_tilt_near_goal_node.py` | Real localization and mapping launches | Tilts the head down near the goal |
    | `human_follower_node.py` | Nothing | Sends `/goal_pose` 1.2 m from the `detected_human` TF frame, every 0.5 s |
    | `accel_stamp_node.py` | Nothing (commented out in the Isaac launch) | Velocity ramping and stamping |
    | `pointcloud_self_filter_node.py` | Nothing | Older box-crop filter, replaced by `robot_self_filter` |
    | `stvl_decay_manager_node.py` | Nothing | Older approach that froze STVL near the goal, replaced by head tilting |

## Known issues

[5. Repair Nav2 After the navigation2 Bump](../migrations/05-nav2-repair.md)
fixed the missing MPPI plugin key, the missing lifecycle managers, the bypassed
collision monitor, the missing simulation map, the missing `/scan` in the
empty MuJoCo world, and the loopback and hero configurations.

| Issue | Effect | Workaround |
| --- | --- | --- |
| Near an obstacle, the collision monitor can still block every Nav2 command: every direction Nav2 tries points at an obstacle in reach, robot parts or noise are inside the selected stop polygon, or a source is invalid. The behavior tree has no BackUp recovery. | The robot stays next to the obstacle. Goals abort with error 105. | Drive out with the joystick or keyboard ([Collision monitor](#collision-monitor)) |
| `follow_path` occasionally aborts with error 105 near an obstacle or during the slow final approach | The behavior tree retries. In the MuJoCo tests, the goals were still reached. | None |
| `/scan` has no points closer than 0.30 m to the base center. In MuJoCo, the front lidar sees nothing closer than 0.415 m straight ahead. The stop polygons cover the base only. | In MuJoCo, a narrow obstacle dead ahead is missed. The arms and carried objects are not protected. | None ([Collision monitor](#collision-monitor)) |
| `nav2_mppi_walkie_real_param_hero_10_04_26.yaml` and the `nav2_RPP_*.yaml` files keep an active `PolygonStop` circle | With these files, a stop blocks every Nav2 command, also commands that drive away | Drive out with the joystick or keyboard, or use the default parameter files |
| In `nav2_mppi_walkie_real_param.yaml` and `nav2_mppi_walkie_real_restaurant_param.yaml`, `voxel_layer` and `stvl_layer` both publish `voxel_grid` with different types | Under Fast DDS, `controller_server` fails to configure in `localization_Nav2_real.launch.py` and `mapping_Nav2_real.launch.py` | Use `rmw_zenoh_cpp`, as the team does. Not fixed in the parameter files. |
| The Gazebo lidar has a minimum range of 0.6 m | The collision monitor's stop polygons and slow-down circle never trigger in Gazebo | None. Only `FootprintApproach` acts. |
| After `bt_navigator` rejects a goal, for example with error 9002, it copies that error code into the result of the next goal of the same action that it accepts, also when that goal succeeds (upstream navigation2). `NavigateToPose` and `NavigateThroughPoses` each keep their own error. | A client that reads only `error_code` treats a successful goal as failed | Judge a goal by its status. In an accepted goal, an `error_code` of 9002 (9102 for `NavigateThroughPoses`) can be left over ([Troubleshooting](../troubleshooting.md#nav2-map-frame-does-not-exist)) |
| `nav_mujoco_omnibot.launch.py` has no AMCL | The pose drifts with the odometry. A goal sent while the robot is already at it by odometry aborts with error 103 (`Resulting plan has 0 poses in it`). | None |
| `nav2_bringup`'s `bringup_launch.py` runs all Nav2 servers on one thread when composed (its default) | Bringup can hang at `Waiting for service smoother_server/get_state...` | `use_composition:=False`. `localization_Nav2_sim.launch.py` and `nav_mujoco_omnibot.launch.py` set it. ([details](#nav2-composition-deadlock)) |
| `slam_toolbox.launch.py` ignores the Walkie parameters | SLAM Toolbox runs with its own defaults and `use_sim_time: true` | Run `online_sync_launch.py` with `slam_params_file:=` ([Mapping in simulation](#mapping-in-simulation)) |
| `slam_cartographer.launch.py` needs `cartographer_ros`, which `pixi.toml` does not install | The launch fails | Use SLAM Toolbox |
| `octomap_3d_mapping.launch.py` needs `octomap_server` and `topic_tools`, which are not in `pixi.toml` | The launch fails with "package not found" under pixi | Add both packages to the environment |
| `scripts/bringup/real_slam_nav.sh` is broken | The Nav2 tab fails | Use the launch files in [Localization and mapping on the real robot](#localization-and-mapping-on-the-real-robot) |
| `navigation/README.md` describes an older setup | Its commands fail | Use this page |
| The fixture of the Nav2 test `test_action_follow_path_action` never sets `cancel_timeout` | The test can hang until CTest stops it. `bt_navigator` is not affected. | Skip the test: `pixi run test --packages-select nav2_behavior_tree --ctest-args -E test_action_follow_path_action` |

??? info "Causes of the known issues"
    - **`voxel_grid` under Fast DDS.** In the real and restaurant files, the
      local costmap has `voxel_layer` (publishes `nav2_msgs/VoxelGrid`) and
      `stvl_layer` (publishes `sensor_msgs/PointCloud2`), both with
      `publish_voxel_map: true`. Both publish `voxel_grid` from the same node.
      Fast DDS rejects two types on one topic, so `controller_server` fails
      with `Failed to initialize costmap plugin stvl_layer! could not create publisher: create_publisher() called for existing topic name rt/local_costmap/voxel_grid with incompatible type`.
      `rmw_zenoh_cpp` accepts it: on 2026-10-14, without hardware, both
      launch files started on Zenoh. The problem predates entry 5. A possible
      fix, not applied, is `voxel_layer.publish_voxel_map: False`, which drops
      the `VoxelGrid` visualization. The simulation, loopback, and hero files
      have no STVL layer.
    - **`slam_toolbox.launch.py`.** It passes its parameter file as
      `params_file`, but `online_sync_launch.py` expects `slam_params_file`.
    - **`real_slam_nav.sh`.** It starts `localization_Nav2_stvl_real.launch.py`,
      which does not exist, and starts SLAM Toolbox (`online_async_launch.py`)
      in a second tab. If the Nav2 tab did start, AMCL and SLAM Toolbox would
      both publish the `map` to `odom` transform.
    - **`navigation/README.md`.** It starts `localization_Nav2.launch.py` with a
      config under `./src/navigation/nav2/config/`. Neither exists.
    - **`test_action_follow_path_action`.**
      `nav2_behavior_tree/test/plugins/action/test_follow_path_action.cpp` in
      `navigation/navigation2/` puts `server_timeout`, `bt_loop_duration`, and
      `wait_for_service_timeout` on the blackboard, but not `cancel_timeout`.
      `BtActionNode` reads `cancel_timeout` from the blackboard into a member
      that has no initial value. The hang was seen on RoboStack build sets
      `_21` and `_25`. `bt_navigator` is not affected, because the behavior
      tree action server always sets `cancel_timeout`. The fix is to set
      `cancel_timeout` in the test fixture in `navigation/navigation2/`.

### controller_server cannot load the Omni motion model { #mppi-omni-motion-model-plugin }

`controller_server` fails to configure, `lifecycle_manager_nav2` aborts the
bringup, and the log shows:

```text
Can not get 'plugin' param value for FollowPath.Omni
Failed to load motion model plugin 'Omni'
```

The MPPI parameter file sets `motion_model: "Omni"` without a plugin key. Add
the key under `FollowPath`, as the repository files do:

```yaml title="config/nav2/nav2_mppi_walkie_sim_params.yaml"
controller_server:
  ros__parameters:
    FollowPath:
      motion_model: "Omni"
      Omni:
        plugin: "mppi::OmniMotionModel"
```

- Every MPPI file in `config/nav2/` sets this key since
  [5. Repair Nav2 After the navigation2 Bump](../migrations/05-nav2-repair.md).
- Any custom MPPI parameter file needs it too, for example one passed with
  `nav2_config:=` or `params_file:=`, or one written for the Jazzy release of
  Nav2. The launch files do not add it.

??? info "Why the Omni motion model needs a plugin key"
    The `navigation` submodule moved navigation2 to upstream `main` on
    2026-10-08 (`navigation` commit `02a697a`). Since upstream PR
    [#6076](https://github.com/ros-navigation/navigation2/pull/6076), MPPI
    loads its motion models as plugins: `motion_model` is an instance name,
    and `<name>.plugin` gives its class. The repository parameter files were
    written before this change and set only `motion_model` until entry 5.

### Nav2 bringup hangs at smoother_server/get_state { #nav2-composition-deadlock }

Nav2 never becomes active, and the log repeats:

```text
Waiting for service smoother_server/get_state...
```

Start Nav2 without composition: pass `use_composition:=False` to
`nav2_bringup`'s `bringup_launch.py`. Each Nav2 server then runs as its own
process. `localization_Nav2_sim.launch.py` (included by
`nav_sim_omnibot.launch.py`) and `nav_mujoco_omnibot.launch.py` do this.

- `bringup_launch.py` defaults to `use_composition:=True`. The hang can still
  happen when you run it directly, or from your own launch file, without the
  argument.
- The real robot, mapping, and loopback launches are not affected. They
  include `navigation_launch.py` and `localization_launch.py` instead, which
  default to `use_composition:=False`.
- The hang depends on the order in which the container loads the servers, so
  it does not happen on every start.

??? info "Why the composed bringup hangs"
    `bringup_launch.py` of the bumped navigation2 starts `nav2_container` as
    `component_container --isolated --executor-type single-threaded`. The
    Jazzy `component_container` ignores both flags. They belong to a newer
    `rclcpp_components`; Jazzy has the separate executables
    `component_container_isolated` and `component_container_mt` instead. All
    composed servers then share one executor thread.

    `lifecycle_manager_nav2` is loaded by a separate `LoadComposableNodes`
    action, and it can be loaded before the servers. Its startup blocks that
    one thread, so the container cannot load the remaining servers, and the
    lifecycle manager waits for them forever.

## Troubleshooting

| Symptom | Cause | Fix |
| --- | --- | --- |
| Keepout zones from the Arena appear on another map | Only `map` was changed | Also pass `keepout_mask:=` or `use_keepout_zones:=false` |
| `/scan` stops on the real robot | One lidar stopped publishing, or one lidar has no valid return at all. The merger needs data from both. | Check both `/front_lidar` and `/back_lidar`, and that neither lidar is covered ([Dual 2D lidar](#dual-2d-lidar)) |
| `/move_robot` returns success but the robot does not move | Nav2 is not running, or the goal failed later | Check the node log and the Nav2 logs |
| The robot stops or crawls near obstacles, or does not move under Nav2 at all | A collision monitor zone has enough points, possibly from the robot's own parts, or a source has no data | Check the `collision_monitor` log, and `velocity_polygon_stop` and `polygon_slowdown` in RViz ([Troubleshooting: The robot stops or slows near obstacles](../troubleshooting.md#nav2-robot-stops-near-obstacles)) |
| The robot stays next to an obstacle, and every goal aborts with error 105 | Every direction Nav2 tries is blocked (the robot is boxed in), robot parts or noise are inside a stop polygon, or a source is invalid. The progress checker then aborts `follow_path` with error 105, because the robot moves less than 0.5 m within 8 s. | Check the `collision_monitor` log. Drive out with the joystick or keyboard ([Troubleshooting: Stuck near an obstacle](../troubleshooting.md#nav2-stuck-after-a-stop)). |
| `controller_server` fails with `create_publisher() called for existing topic name rt/local_costmap/voxel_grid with incompatible type` | Fast DDS and the two `voxel_grid` publishers of the real and restaurant files | Use `rmw_zenoh_cpp` ([Known issues](#known-issues)) |
| `nav2_loopback.launch.py`: Nav2 never finishes activating | The loopback simulator has no initial pose yet, so it publishes no transforms | Set one with **2D Pose Estimate** or on `/initialpose` ([Test goal logic without physics](#test-goal-logic-without-physics)) |
| Nav2 nodes stay unconfigured, and the log shows `Waiting on external lifecycle transitions to activate` | A launch file includes `navigation_launch.py` or `localization_launch.py` without starting a lifecycle manager | Start `lifecycle_manager_nav2` as the Walkie launch files do ([Localization and mapping on the real robot](#localization-and-mapping-on-the-real-robot)) |
| `bt_navigator` rejects goals and logs `"map" passed to lookupTransform argument target_frame does not exist` and `Initial robot pose is not available` (error 9002) | `bt_navigator` has no transform from `map` to `odom` | With AMCL or SLAM Toolbox, check that they publish it: `pixi run ros2 run tf2_ros tf2_echo map odom`. With a `static_transform_publisher` that publishes it once, as in `nav_mujoco_omnibot.launch.py`: start a second publisher. Never add one next to AMCL or SLAM Toolbox. ([Troubleshooting](../troubleshooting.md#nav2-map-frame-does-not-exist)) |
