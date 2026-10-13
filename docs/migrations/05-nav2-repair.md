# 5. Repair Nav2 After the navigation2 Bump

**Goal:** make every Nav2 launch file start again on the bumped `navigation2`,
and put the velocity smoother and the collision monitor into the command path
that the parameter files were written for.

This entry records what broke, why, and how each problem was fixed. For
day-to-day use, read [Navigation](../guides/navigation.md). Two of the
problems were first recorded, with workarounds, in
[2.7 Launch Files and Nav2](02-mujoco/07-launch.md#two-nav2-fork-issues).

## In short

- **Changed:** in the `navigation` submodule, the five MPPI parameter files,
  four launch files of `robot_navigation`, and the map submodule pointer. In
  this repository, `hardware/lidar/dual_laser_merger` (new parameter
  `publish_on_empty_cloud`), `bringup/launch/mujoco_omnibot.launch.py`, which
  sets it, and `bringup/launch/nav_mujoco_omnibot.launch.py`.
- **Why:** on 2026-10-08, the `navigation` submodule moved `navigation2` to
  upstream `main` (`e48c3296`). Two upstream changes then broke every Nav2
  launch except the MuJoCo one. Independent of the bump, Nav2 commands
  bypassed the velocity smoother and the collision monitor. Once the monitor
  was in the path, the empty MuJoCo world, which gave no `/scan`, stopped
  every Nav2 command.
- **After pulling:** update the submodules and rebuild `robot_navigation` and
  `dual_laser_merger` ([After pulling this change](#after-pulling-this-change)).
- **Behavior change:** the collision monitor now stops and slows the robot.
  After a stop, it blocks every Nav2 command
  ([Behavior changes](#behavior-changes)).
- **Result:** Nav2 reaches goals in MuJoCo and in the loopback simulator. The
  sim, loopback and hero launches start under Fast DDS. The real-robot and
  mapping launches start under `rmw_zenoh_cpp`; under Fast DDS they fail on a
  separate, older problem. Gazebo and the real robot were not run
  ([Tests](#tests)).

!!! warning "The collision monitor now acts on Nav2 commands"
    Nav2 commands stop when 4 or more points are within 0.35 m of
    `base_footprint`. In the MuJoCo stop test, the robot stopped with its
    footprint front at the wall. Every later Nav2 command stayed blocked,
    including commands away from the wall. Drive out with the joystick or
    keyboard, which the monitor does not check. Nav2 commands also stop when a
    source has no data for 2 s. See [Behavior changes](#behavior-changes).

## After pulling this change

1. Check out the new submodule commits, including the moved map submodule:

    ```bash
    git submodule update --init --recursive
    ```

2. Rebuild `robot_navigation` and `dual_laser_merger`:

    ```bash
    pixi run build-pkg robot_navigation dual_laser_merger
    ```

    With `--symlink-install`, changed files are already linked into
    `install/`, but the restored map directory is new and needs a build to be
    installed. `dual_laser_merger` is C++. An old build ignores the new
    parameter, and MuJoCo then has no `/scan`.

## What changed

| Problem | Effect before the fix | Fix |
| --- | --- | --- |
| [MPPI motion models are plugins](#mppi-motion-models-are-plugins) | `controller_server` failed to configure | `FollowPath.Omni.plugin` in all five MPPI parameter files |
| [No lifecycle manager in navigation_launch.py and localization_launch.py](#no-lifecycle-manager-for-the-included-nav2-files) | Real robot, mapping and loopback: every Nav2 node stayed unconfigured | The three launch files start `lifecycle_manager_nav2` themselves |
| [Composition deadlock](#composition-deadlock-in-the-gazebo-launch) | Gazebo Nav2 could hang during startup | `use_composition:=False` in `localization_Nav2_sim.launch.py` |
| [Velocity smoother and collision monitor bypassed](#velocity-smoother-and-collision-monitor-bypassed) | Nav2 commands went straight to `twist_mux` | `SetRemap` of `cmd_vel` to `cmd_vel_controller` in four launch files, and parameter cleanups |
| [/scan in the empty MuJoCo world](#scan-in-the-empty-mujoco-world) | No `/scan`, so the collision monitor stopped every Nav2 command in MuJoCo | New `dual_laser_merger` parameter `publish_on_empty_cloud`, set by `mujoco_omnibot.launch.py` |
| [Local inflation radius too small](#local-inflation-radius-below-the-circumscribed-radius) | An error on every control cycle, and slower MPPI collision checks | Larger local `inflation_radius` in three parameter files |
| [Simulation map missing](#simulation-map-restored) | Gazebo and loopback Nav2 could not load their default map | Map submodule moved to `c22354c`; the launch defaults point at the restored directory |
| [Loopback simulator not started](#loopback-simulator) | Loopback Nav2 never activated | `loopback_simulator` activates itself, with sim time; stamped command fixes in the loopback parameters |
| [Hero parameters incomplete](#hero-parameters) | `bt_navigator` failed to configure | The custom BT plugins and the `spin` behavior added |

## MPPI motion models are plugins

Since upstream pull request
[#6076](https://github.com/ros-navigation/navigation2/pull/6076), MPPI loads
its motion models as plugins.
`motion_model: "Omni"` is an instance name, and `FollowPath.Omni.plugin` gives
its class. No MPPI parameter file set the key, so `controller_server` failed to
configure. All five files now set it:

```yaml title="navigation/walkie-navigation/config/nav2/nav2_mppi_walkie_sim_params.yaml"
controller_server:
  ros__parameters:
    FollowPath:
      motion_model: "Omni"
      Omni:
        plugin: "mppi::OmniMotionModel"
```

The files are `nav2_mppi_walkie_sim_params.yaml`,
`nav2_mppi_walkie_real_param.yaml`, `nav2_mppi_walkie_real_restaurant_param.yaml`,
`nav2_mppi_walkie_real_param_hero_10_04_26.yaml` and
`nav2_mppi_walkie_loopback_params.yaml`, in
`navigation/walkie-navigation/config/nav2/`.

`nav_mujoco_omnibot.launch.py` no longer needs its `RewrittenYaml` workaround
and passes `params_file` straight to `nav2_bringup`. A custom MPPI parameter
file still needs the key; see
[Navigation: controller_server cannot load the Omni motion model](../guides/navigation.md#mppi-omni-motion-model-plugin).

## No lifecycle manager for the included Nav2 files

Upstream commit `a1904cdc` ("Combine lifecycle managers",
[#6410](https://github.com/ros-navigation/navigation2/pull/6410)), part of the
bumped `navigation2`, removed the lifecycle managers from `nav2_bringup`'s
`navigation_launch.py` and `localization_launch.py`. Only `bringup_launch.py`
starts one now, named `lifecycle_manager_nav2`. Three walkie launch files
include the two files directly, so every Nav2 node they started stayed
unconfigured, and the log showed:

```text
Waiting on external lifecycle transitions to activate
```

These launches never reached the MPPI plugin error. Each of them now starts
its own `lifecycle_manager_nav2` with `autostart`:

| Launch file | Nodes managed by `lifecycle_manager_nav2` | Other lifecycle managers |
| --- | --- | --- |
| `localization_Nav2_real.launch.py` | `map_server`, `amcl` and the navigation nodes | `lifecycle_manager_keepout` for the keepout servers, unchanged |
| `mapping_Nav2_real.launch.py` | The navigation nodes | None. SLAM Toolbox manages its own lifecycle. |
| `nav2_loopback.launch.py` | The navigation nodes | `lifecycle_manager_map` for `map_server`, unchanged. `loopback_simulator` activates itself (see [Loopback simulator](#loopback-simulator)). |

`localization_Nav2_sim.launch.py` and `nav_mujoco_omnibot.launch.py` include
`bringup_launch.py`, so they were not affected.

- **The node list comes from `nav2_bringup`.** The launch files call
  `nav2_bringup.navigation_launch.get_lifecycle_nodes()`, so the list follows
  the next `navigation2` bump without an edit. The function takes the launch
  context, so the manager is created in an `OpaqueFunction`.
- **The name is the same as `nav2_bringup`'s manager.** The RViz Nav2 panel
  talks to `lifecycle_manager_nav2`.

??? info "How the finding was checked, and the managed node list"
    `navigation_launch.py` of the bumped `nav2_bringup`, launched alone,
    started 11 nodes and no lifecycle manager. `get_lifecycle_nodes()` returns
    the same 11: `controller_server`, `smoother_server`, `planner_server`,
    `route_server`, `behavior_server`, `velocity_smoother`,
    `collision_monitor`, `bt_navigator`, `waypoint_follower`,
    `docking_server` and `following_server`.

    The manager in `mapping_Nav2_real.launch.py`:

    ```python title="navigation/walkie-navigation/launch/mapping_Nav2_real.launch.py"
    def launch_lifecycle_manager(context):
        return [
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_nav2",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": True,
                        "node_names": list(get_navigation_nodes(context)),
                    }
                ],
            )
        ]
    ```

    `get_navigation_nodes` is `get_lifecycle_nodes`, imported from
    `nav2_bringup.navigation_launch`. The function is added with
    `ld.add_action(OpaqueFunction(function=launch_lifecycle_manager))`.

## Composition deadlock in the Gazebo launch

`localization_Nav2_sim.launch.py`, which `nav_sim_omnibot.launch.py` includes
for Gazebo, now passes `use_composition:=False` to `bringup_launch.py`.
`nav_mujoco_omnibot.launch.py` already did. Each Nav2 server runs as its own
process.

The deadlock still applies to anyone who runs `nav2_bringup`'s
`bringup_launch.py` directly with its default `use_composition:=True`. See
[Navigation: Nav2 bringup hangs at smoother_server/get_state](../guides/navigation.md#nav2-composition-deadlock).

??? info "Cause of the deadlock"
    The Jazzy `component_container` ignores the `--isolated` flag that
    `bringup_launch.py` passes, so all composed servers share one executor
    thread. If `lifecycle_manager_nav2` is loaded first, its blocking service
    client holds that thread, and `load_node` for the remaining servers never
    runs. [2.7 Launch Files and Nav2](02-mujoco/07-launch.md#composition-deadlock)
    has the full chain.

## Velocity smoother and collision monitor bypassed

Every walkie launch file that runs Nav2 for the robot now wraps the
`nav2_bringup` include in a `SetRemap`, so that Nav2 commands pass the velocity
smoother and the collision monitor before `twist_mux`:

```python title="bringup/launch/nav_mujoco_omnibot.launch.py"
nav2_launch = GroupAction(
    actions=[
        SetRemap(src="cmd_vel", dst="cmd_vel_controller"),
        IncludeLaunchDescription(...),
    ]
)
```

The files are `localization_Nav2_sim.launch.py`,
`localization_Nav2_real.launch.py` and `mapping_Nav2_real.launch.py` in
`robot_navigation`, and `bringup/launch/nav_mujoco_omnibot.launch.py`. A new
launch file that includes `nav2_bringup` for the robot needs the same wrapper.

```mermaid
flowchart LR
    C[controller_server] -->|"/cmd_vel_controller"| S[velocity_smoother]
    B[behavior_server] -->|"/cmd_vel_controller"| S
    D["docking_server<br/>following_server"] -->|"/cmd_vel_controller"| S
    S -->|"/cmd_vel_smoothed"| CM[collision_monitor]
    CM -->|"/cmd_vel_nav"| M["twist_mux<br/>priority 10"]
    J["/cmd_vel_joy, /cmd_vel_key"] -->|"priority 100"| M
    M -->|"/omni_wheel_drive_controller/cmd_vel"| W[omni_wheel_drive_controller]
```

`twist_mux.yml` did not change. `nav2_loopback.launch.py` gets no `SetRemap`:
its parameter file already chains the commands the upstream way, from
`cmd_vel_nav` through the smoother and the collision monitor to `cmd_vel`,
which `loopback_simulator` reads. It has no `twist_mux`. The loopback launch
file did change in other ways: it starts `lifecycle_manager_nav2`, defaults to
the restored map, and starts `loopback_simulator` with `autostart_node` and
`use_sim_time` (see [Loopback simulator](#loopback-simulator)).

??? info "The command path before the fix"
    `controller_server` and `behavior_server` publish `cmd_vel`, which
    `nav2_bringup` remaps to `/cmd_vel_nav`. That topic is the Nav2 input of
    `twist_mux`, so the commands went straight to the wheels.
    `velocity_smoother` read `/cmd_vel_nav` and wrote `/cmd_vel_smoothed`,
    which had no subscriber. `collision_monitor` read `/cmd_vel_controller`,
    which had no publisher.

    ```mermaid
    flowchart LR
        C[controller_server] -->|"/cmd_vel_nav"| M[twist_mux]
        B[behavior_server] -->|"/cmd_vel_nav"| M
        M -->|"/omni_wheel_drive_controller/cmd_vel"| W[omni_wheel_drive_controller]
        C -->|"/cmd_vel_nav"| S[velocity_smoother]
        S -->|"/cmd_vel_smoothed"| U["no subscriber"]
        P["no publisher"] -->|"/cmd_vel_controller"| CM[collision_monitor]
    ```

    The parameter files set `controller_server.cmd_vel_topic` and
    `velocity_smoother.cmd_vel_in_topic` and `cmd_vel_out_topic`, but Nav2 has
    no such parameters. The topic names are fixed in the code.

??? info "Why one SetRemap overrides nav2_bringup's own remap"
    `SetRemap` adds a global remapping rule to the launch context. `launch_ros`
    puts the global rules before each node's own remappings, and `rcl` applies
    the first rule that matches. `cmd_vel` to `cmd_vel_controller` therefore
    wins over `nav2_bringup`'s `cmd_vel` to `cmd_vel_nav`, for every node in
    the group. The velocity smoother's `cmd_vel` input is remapped the same
    way, so it reads `/cmd_vel_controller`.

    The wrapper uses no copied `nav2_bringup` files, so it survives
    `navigation2` bumps.

??? info "Parameter changes in the sim, real, restaurant and hero files"
    | Parameter | Before | After | Why |
    | --- | --- | --- | --- |
    | `controller_server.cmd_vel_topic` | `/cmd_vel_controller` | Removed | Not a Nav2 parameter |
    | `velocity_smoother.cmd_vel_in_topic`, `cmd_vel_out_topic` | `/cmd_vel_controller`, `/cmd_vel_smoothed` | Removed | Not Nav2 parameters. The input is `cmd_vel`, remapped by the launch files; the output is `cmd_vel_smoothed`. |
    | `collision_monitor.cmd_vel_in_topic` | `/cmd_vel_controller` | `cmd_vel_smoothed` | Reads the smoother output |
    | `collision_monitor.cmd_vel_out_topic` | `/cmd_vel_nav` | `cmd_vel_nav` | The Nav2 input of `twist_mux` |
    | `docking_server.enable_stamped_cmd_vel` | `false` | Removed | It would publish unstamped `Twist` messages onto the stamped `/cmd_vel_controller` |

??? info "Alternatives that were rejected"
    | Alternative | Why it was rejected |
    | --- | --- |
    | Copy `nav2_bringup`'s `navigation_launch.py` and `bringup_launch.py` into `robot_navigation` with the remaps edited (an earlier attempt, not merged) | Hundreds of copied lines that go stale on every `navigation2` bump. Such a copy, made before the bump, no longer starts the localization lifecycle. It also left `behavior_server` publishing straight to `cmd_vel_nav`. |
    | Point `collision_monitor` at `/cmd_vel_nav` and move `twist_mux` to another topic | Needs `robot_bringup` changes on every robot launch |
    | Leave the command path to a later change | On the real robot, the collision monitor is the only hard stop for tabletop obstacles |

## /scan in the empty MuJoCo world { #scan-in-the-empty-mujoco-world }

`dual_laser_merger` has a new parameter, `publish_on_empty_cloud` (bool,
default `false`). `bringup/launch/mujoco_omnibot.launch.py` sets it to `true`.
`/scan` is then published in the default `scene.xml`, which has only a floor,
and Nav2 drives there. `nav_mujoco_omnibot.launch.py` includes that launch
file, so it gets the same setting. The real robot keeps the default.

| `publish_on_empty_cloud` | When a lidar's cloud is empty | Used by |
| --- | --- | --- |
| `false` (default) | No `/scan` message | The real robot (`dual_lidar_hokuyo_lakibeam.launch.py`) |
| `true` | `/scan` is published. The empty side reads as free space (`inf` ranges, with `use_inf: true`). | `mujoco_omnibot.launch.py` |

A lidar's cloud is empty when every beam of that lidar is outside its range
limits or invalid. That happens when nothing is within range (open space, the
empty MuJoCo scene), or when the lidar is covered or faulty.

**Why this was needed.** Before the change, the merger returned without
publishing when either cloud was empty. In the default scene no beam hits
anything, so `/scan` never appeared. With the collision monitor in the command
path, a source without data for 2 s stops every Nav2 command:

```text
Robot to stop due to invalid source
```

The robot did not move, and goals aborted with error 105 ([Tests](#tests),
run 1).

**Why the real robot keeps `false`.** On the real robot, an empty cloud can
mean a covered or faulty lidar. Then no `/scan`, and a collision monitor stop,
is the safe outcome. The consequence: if one lidar has no valid return at all
(covered, faulty, or nothing within its range in any direction), `/scan`
stops, and 2 s later the collision monitor stops all Nav2 commands with the
message above.

!!! note "/scan still needs both lidar topics"
    The merger syncs `/front_lidar` and `/back_lidar` with an approximate-time
    policy. If one of the topics is not published at all, there is no `/scan`,
    whatever the parameter says.

??? info "How the merger handles an empty cloud, and the first version"
    The check is in `MergerNode::sub_callback` in
    `hardware/lidar/dual_laser_merger/src/dual_laser_merger.cpp`. With
    `publish_on_empty_cloud`, the two clouds are merged even if one is empty.
    The shadow filter is skipped when the merged cloud has no points, because
    its k-d tree needs points.

    A first version of the change published unconditionally; review changed it
    to a parameter that defaults to `false`, because treating an empty side as
    free space fails open on the real robot.

## Local inflation radius below the circumscribed radius

The local costmap's `inflation_radius` was smaller than the circumscribed
radius of the footprint. `controller_server` logged this error on every cycle,
and MPPI checked the full footprint for collisions everywhere:

```text
[computeCircumscribedCost]: The inflation radius (0.250000) is smaller than the circumscribed radius (0.313249)...
```

The local `inflation_radius` is now at least the circumscribed radius:

| Parameter file | Before (m) | After (m) | Circumscribed radius |
| --- | --- | --- | --- |
| `nav2_mppi_walkie_sim_params.yaml` | 0.25 | 0.35 | 0.313 m, with the default 0.01 m padding |
| `nav2_mppi_walkie_real_param.yaml` | 0.25 | 0.38 | 0.354 m, with 0.02 m padding |
| `nav2_mppi_walkie_real_restaurant_param.yaml` | 0.30 | 0.45 | 0.435 m, no padding |

The hero file (`robot_radius` 0.15, inflation 0.2) and the loopback file (0.355
against 0.265) were already correct. Costs now reach further from obstacles, so
the robot keeps a little more clearance.

??? info "Alternative that was rejected"
    Lowering the footprint below the inflation radius was rejected: the
    footprint is the size of the robot.

## Simulation map restored

The map submodule of `navigation` moved from `94d86cc` to `c22354c`, the
`main` branch of the map collection ("restore all map"). It brings the AWS
small house map back as `08_23102025_SimWalkie_awssmallhouse_slam_toolbox`,
with a leading zero. `localization_Nav2_sim.launch.py` and
`nav2_loopback.launch.py` now default to that directory.

The old workaround, extracting the map from `94d86cc~1` and passing `map:=`, is
no longer needed.

## Loopback simulator

`nav2_loopback.launch.py` reaches an active Nav2 again. Set an initial pose
after the launch, with **2D Pose Estimate** in RViz or on `/initialpose`:

```bash
pixi run ros2 launch robot_navigation nav2_loopback.launch.py
# In a second terminal, unless you use 2D Pose Estimate in RViz:
pixi run ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    "{header: {frame_id: map}, pose: {pose: {orientation: {w: 1.0}}}}"
```

!!! warning "Nav2 does not activate without an initial pose"
    The loopback simulator publishes `map` to `odom` to `base_footprint` only
    after it receives an initial pose. Until then, `local_costmap` waits for
    the transform, and `lifecycle_manager_nav2` does not finish activating.
    This is upstream behavior.

Navigation `e5d33c2` made three changes:

- **The simulator activates itself.** Since the `navigation2` bump,
  `nav2_loopback_sim` is a C++ Nav2 lifecycle node; before, it was a plain
  Python node. Nothing configured it, so it published no clock, TF or scan,
  and `local_costmap` waited for `odom` to `base_footprint` forever. The
  launch file now sets `autostart_node: true`: the node configures and
  activates itself. No lifecycle manager manages it.
- **It uses sim time.** The launch file sets `use_sim_time: true`, as
  upstream `loopback_simulation.launch.py` does. Without it, TF is stamped with
  wall time and the scan with sim time. `collision_monitor` then logs the
  error below and stops on an invalid source.
- **Stamped commands fixed.** `behavior_server` and `following_server`
  published `TwistStamped` into the plain `Twist` loopback chain, so their
  commands never reached the simulator. Both now set
  `enable_stamped_cmd_vel: false`.

```text
Failed to get "base_footprint"->"base_footprint" frame transform: Lookup would require extrapolation into the past
```

Navigation `a28a150` had already added `lifecycle_manager_nav2` and the
restored map default to the launch file.

??? info "Loopback parameter changes and command chain"
    | Parameter in `nav2_mppi_walkie_loopback_params.yaml` | Change | Why |
    | --- | --- | --- |
    | `bt_navigator.plugin_lib_names` | Added `compute_path_heading_spin_bt_node` | `nav_to_pose_real_recovery_bt.xml` uses `ComputePathHeadingSpin` |
    | `behavior_server.enable_stamped_cmd_vel` | Added, `false` | Plain `Twist` chain |
    | `following_server` | New block: `use_sim_time: true`, `enable_stamped_cmd_vel: false` | Plain `Twist` chain |

    The loopback keeps upstream's own chain: `controller_server` and
    `behavior_server` publish `cmd_vel_nav` to `velocity_smoother`, which
    publishes `cmd_vel_smoothed` to `collision_monitor`, which publishes
    `cmd_vel` to `loopback_simulator`. `docking_server` and `following_server`
    publish `cmd_vel` directly. `docking_server` already had
    `enable_stamped_cmd_vel: false`.

## Hero parameters

`nav2_mppi_walkie_real_param_hero_10_04_26.yaml` loads again. No launch file
uses it; pass it with `nav2_config:=`:

```bash
pixi run ros2 launch robot_navigation localization_Nav2_real.launch.py \
    nav2_config:=$PWD/navigation/walkie-navigation/config/nav2/nav2_mppi_walkie_real_param_hero_10_04_26.yaml
```

Before navigation `e5d33c2`, `bt_navigator` failed to configure, first with
`Node not recognized: ComputePathHeadingSpin`, and then with
`"spin" action server not available`. The fix added the three custom BT
plugins to `bt_navigator.plugin_lib_names` and the `spin` behavior
(`nav2_behaviors::Spin`) to `behavior_server`. No tuning value changed.

??? info "Why the hero file broke"
    The file is a snapshot of the real-robot parameters from 2026-04-10, added
    in navigation `e31af55`. It uses `nav_to_pose_real_recovery_bt.xml`, which
    has since gained `ComputePathHeadingSpin` and `Spin` nodes. The added
    plugins are `compute_goal_heading_spin_bt_node`,
    `compute_goal_bearing_spin_bt_node` and
    `compute_path_heading_spin_bt_node`.

    The hero file has no Spatio-Temporal Voxel Layer, so it also starts under
    Fast DDS (see [Still open](#still-open)).

## Behavior changes

The collision monitor and the velocity smoother now act on every Nav2
command. Joystick and keyboard commands do not pass them: they enter
`twist_mux` directly, with priority 100.

!!! warning "After a stop, Nav2 cannot move the robot away"
    `PolygonStop` zeroes every Nav2 command while 4 or more points are inside
    its circle, including commands that drive away from the obstacle. In the
    MuJoCo stop test, the two goals sent after the stop both aborted with
    error 105, `Failed to make progress`. Drive the robot out with the
    joystick or keyboard. Whether to change this is an open decision
    ([Still open](#still-open)).

| Polygon | Shape | Action | Points needed |
| --- | --- | --- | --- |
| `PolygonStop` | Circle, radius 0.35 m | Stop | 4 |
| `PolygonSlow` | Circle, radius 0.55 m | Slow down to 40 % | 4 |
| `FootprintApproach` | The local costmap footprint | Slow down so that a collision is at least 2.0 s away | 6 |

The polygons are the same in the sim, real and restaurant parameter files.
The sim file checks `scan`. The real and restaurant files check `scan` and
`pointcloud` (`/unilidar/cloud/filtered`, points 0.3 to 1.5 m high).
`VelocityPolygonStop`, a direction-aware stop zone, is defined but disabled
and not in the polygon list. All settings, per parameter file, are in
[Navigation: Collision monitor](../guides/navigation.md#collision-monitor).

- **Stop distance.** In the MuJoCo stop test, the robot stopped with its
  footprint front at the wall, with no clearance left. `/scan` has no points
  closer than 0.30 m to the base center, so for `/scan` the stop circle acts
  only on a ring from 0.30 to 0.35 m.
- **Gazebo.** The default simulated lidar of `gzsim_omnibot.launch.py` has a
  minimum range of 0.6 m, so `PolygonStop` and `PolygonSlow` never trigger
  there.
  Only `FootprintApproach` acts, on obstacles farther than 0.6 m. This follows
  from the configuration; Gazebo was not run.
- **Missing data.** When a source has no data for 2 s, or its transform
  fails, the monitor stops every Nav2 command and logs
  `Robot to stop due to invalid source`. On the real robot, this happens when
  a lidar has no valid return
  ([/scan in the empty MuJoCo world](#scan-in-the-empty-mujoco-world)), or
  with `use_unitree_lidar:=false`, which leaves the `pointcloud` source
  without data.
- **Narrow passages.** Within 0.55 m, the robot slows to 40 %. Tight passages
  are slow, and side obstacles 0.30 to 0.35 m from the center stop the robot.
- **Self-detection.** If a lidar or the point cloud sees parts of the robot
  inside the stop circle, the robot does not move at all.
- **Speed limits.** The smoother limits the output to `max_velocity`
  [0.3, 0.3, 1.0] and `max_accel` [2.5, 2.5, 3.2]. Rotation is capped at
  1.0 rad/s, although MPPI's `wz_max` is 1.5 rad/s.
- **Clearance.** The larger local inflation radius makes the robot keep a
  little more distance from obstacles.

To see what the monitor does, watch its log and the polygons in RViz. It logs
an INFO line such as `Robot to stop due to PolygonStop polygon` when its action
changes, and it publishes the polygons on `polygon_stop` and
`polygon_slowdown`.

!!! warning "The real robot does not have these changes yet"
    The robot runs the Docker image built from `main`. None of these changes
    reach it until they are merged there.

??? info "Why the stop circle leaves no clearance"
    `dual_laser_merger` publishes `/scan` in `merged_lidar_link`, which is at
    x = 0, y = 0 of `base_link`, with `range_min` 0.3 (in
    `mujoco_omnibot.launch.py` and in the real
    `dual_lidar_hokuyo_lakibeam.launch.py`). So `/scan` has no points closer
    than 0.30 m to the base center, and `PolygonStop` sees `/scan` points only
    0.30 to 0.35 m from the center.

    | Parameter file | Local footprint | Effect on the stop ring |
    | --- | --- | --- |
    | `nav2_mppi_walkie_sim_params.yaml`, `nav2_mppi_walkie_real_param.yaml` | Front at x = 0.30 m, half width 0.239 m | In front, the ring is 0 to 5 cm ahead of the footprint. At the sides, obstacles 0.239 to 0.30 m from the center are not in `/scan` at all. |
    | `nav2_mppi_walkie_real_restaurant_param.yaml` | Front at x = 0.40 m (padding included) | In front, the 0.35 m stop circle lies inside the footprint |

    On the real robot, the Unitree point cloud has no 0.30 m limit.

    The stop test ran in a walled MuJoCo test world. A constant 0.25 m/s
    forward command on `/cmd_vel_controller` drove the robot from x = 0 toward
    a wall face at x = 1.45 m. `velocity_smoother` passed 0.25 m/s.
    `collision_monitor` output on `/cmd_vel_nav`:

    | Time | Output | Cause |
    | --- | --- | --- |
    | Until about 4 s | 0.25 m/s | No polygon triggered |
    | From about 4 s | 0.10 m/s | `PolygonSlow` (40 %), wall about 0.52 m from the center |
    | Later | About 0.045, then 0.0375 m/s | `FootprintApproach` |
    | About 8.6 s | 0 | `PolygonStop` |

    The base stopped at x = 1.155 m, 0.295 m from the wall face. That puts
    the footprint front (x = 0.30 m) at the wall.

    In Gazebo, `gzsim_omnibot.launch.py`, which `nav_sim_omnibot.launch.py`
    includes, uses the single lidar by default (`dual_lidar:=False`). Its
    `min_range` is 0.6 m in `description/robots/gz_walkie.urdf.xacro`.

??? info "The hero parameter file"
    `nav2_mppi_walkie_real_param_hero_10_04_26.yaml`, which no launch file
    uses, has a `PolygonSlow` radius of 0.50 m with a slowdown ratio of 0.5,
    and checks only `scan`.

## Tests

All runs were made on 2026-10-14 on one workstation, headless, without
hardware. The middleware was `rmw_fastrtps_cpp` unless noted.

- **MuJoCo:** Nav2 reaches goals in the default scene and in a walled test
  world. Before the merger change it did not move.
- **Sim, loopback and hero launches:** start. `localization_Nav2_sim.launch.py`
  ran with the MuJoCo base instead of Gazebo.
- **Real-robot and mapping launches:** start under `rmw_zenoh_cpp`, the team
  middleware. Under Fast DDS, `controller_server` fails to configure (a
  pre-existing problem; see [Still open](#still-open)).
- **Not run:** Gazebo and the real robot.

??? info "Test runs and results"
    | # | Run | Result |
    | --- | --- | --- |
    | 1 | `nav_mujoco_omnibot.launch.py`, default `scene.xml`, before the merger change | Nav2 active after 10 s, command chain correct, but no `/scan`. `collision_monitor`: `Robot to stop due to invalid source`. The robot did not move; goals aborted with 105. |
    | 2 | Same, with the first version of the merger change (always publishing) | `/scan` at about 27 Hz. Goal (1.5, 1.0, 90°) on `/goal_pose` reached in 11 s, 0.14 m and 3.7° off by ground truth. Goal (-1.0, -0.5, 0°) through the `navigate_to_pose` action SUCCEEDED in 13 s, 0.39 m off by ground truth (odometry drift, no AMCL). |
    | 3 | Same, with the final change (`publish_on_empty_cloud: True` in `mujoco_omnibot.launch.py`) | Nav2 active after 10 s. Command chain `TwistStamped` end to end, Nav2 servers to `/cmd_vel_controller`, `velocity_smoother`, `/cmd_vel_smoothed`, `collision_monitor`, `/cmd_vel_nav`, `twist_mux`. Goal (1.5, 1.0, 90°) on `/goal_pose` reached in 10 s, 0.14 m and 3.8° off by ground truth (odometry 0.10 m from the goal). Goal (-1.0, -0.5, 0°) through the action SUCCEEDED in 14 s, 0.15 m and 1.3° off (odometry 0.02 m from the goal, drift 0.17 m). During goal 1 the progress checker aborted `follow_path` once with 105 in the slow final approach; the behavior tree retried and reached the goal 0.3 s later. |
    | 4 | Walled test world: 8 m x 8 m room, a 1.2 m wall at x = 1.5 m between the start and (3, 0). Kept in `/tmp`, not in the repository. | Goal (3.0, 0, 0°) behind the wall reached in 25 s; odometry 0.06 m from the goal, ground truth 0.24 m (drift 0.28 m). The next goal, (-1.0, -0.5, 0°), aborted with 103 `Resulting plan has 0 poses in it` while odometry was 0.002 m from the goal; ground truth was 0.43 m away. |
    | 5 | Stop test in the walled world (see [Behavior changes](#behavior-changes)) | Stopped at x = 1.155 m, footprint front at the wall. Two later goals, one beyond the wall and one back to (-1.0, -0.5), both aborted with 105, after about 120 s and 88 s. |
    | 6 | `localization_Nav2_sim.launch.py` with `mujoco_omnibot.launch.py` as the base | `lifecycle_manager_nav2` active after 23 s, no errors |
    | 7a | `localization_Nav2_real.launch.py` and `mapping_Nav2_real.launch.py`, static `map` to `odom` to `base_footprint` TF, Fast DDS | Both fail. `controller_server` cannot configure: `Failed to initialize costmap plugin stvl_layer! ... create_publisher() called for existing topic name rt/local_costmap/voxel_grid with incompatible type`. Pre-existing. |
    | 7b | The same two launches, `rmw_zenoh_cpp` with a local `rmw_zenohd` | Both start: real active after 7 s, mapping after 6 s. Command chain `TwistStamped` end to end to `/cmd_vel_nav`. `nav_commander.py` and `head_tilt_near_goal_node.py` print a traceback at shutdown only (double `rclpy` shutdown on Ctrl-C); harmless. |
    | 8 | `localization_Nav2_real.launch.py` with the hero file | Before `e5d33c2`, `bt_navigator` failed to configure. After: active after 8 s, no errors. |
    | 9 | `nav2_loopback.launch.py` | Before `e5d33c2`, Nav2 never activated. After: active after 16 s, once an initial pose (0, 0) was published on `/initialpose`. Goal (1.0, 0.5, 90°) on `/goal_pose` reached in 10 s, 0.04 m and 0.5° off by `/odom`. Goal (0, 0, 0°) through the action SUCCEEDED in 20 s, 0.03 m and 0.5° off. One 105 abort of `follow_path` in the final approach of goal 2; the behavior tree retried and reached the goal 1.8 s later. Command chain plain `Twist`. |

## Commits

| Repository | Commit | Subject |
| --- | --- | --- |
| walkie-ros | `ef51b15` | feat: add publish_on_empty_cloud to dual_laser_merger for MuJoCo |
| walkie-ros | `53579dd` | fix: update navigation and route MuJoCo Nav2 commands through the smoother |
| navigation | `a28a150` | fix: restore Nav2 launches after the navigation2 bump |
| navigation | `e5d33c2` | fix: make the loopback and hero configurations start |

`53579dd` adds the `SetRemap` to `nav_mujoco_omnibot.launch.py`, removes its
`RewrittenYaml` workaround, and moves the `navigation` gitlink from `02a697a`
to `e5d33c2`. The documentation for this entry is a separate walkie-ros
commit. `c26237a` (description at `main`) is on `feat/mujoco-launch` and is
not part of this entry.

??? info "Branches"
    The walkie-ros commits are on `fix/nav2-compat`, stacked on
    `feat/mujoco-launch`, which is on `feat/Pixi`. The navigation commits are
    on `fix/nav2-compat` in `EIC-Robocup-2026/navigation`, on `feat/pixi`
    (`02a697a`).

## Still open

- **Stop-circle deadlock (open decision).** After a `PolygonStop` stop, the
  monitor blocks every Nav2 command, also commands away from the obstacle, and
  goals abort with 105. The robot stops with its footprint front at the
  obstacle. The team has not chosen between switching to the direction-aware
  `VelocityPolygonStop`, dropping `PolygonStop` and relying on
  `FootprintApproach`, or keeping it.
- **The `/scan` blind ring.** `/scan` has no points closer than 0.30 m to the
  base center. The stop circle sees `/scan` points only 0.30 to 0.35 m from the
  center, and at the sides, obstacles 0.239 to 0.30 m from the center are not
  in `/scan`. In Gazebo, the lidar's 0.6 m minimum range keeps both circles
  from triggering.
- **Fast DDS and the Spatio-Temporal Voxel Layer.** In
  `nav2_mppi_walkie_real_param.yaml` and
  `nav2_mppi_walkie_real_restaurant_param.yaml`, the local costmap has
  `voxel_layer` (publishes `nav2_msgs/VoxelGrid`) and `stvl_layer` (publishes
  `sensor_msgs/PointCloud2`), both with `publish_voxel_map: true`. Both publish
  `local_costmap/voxel_grid` from one node. Fast DDS rejects two types on one
  topic, so `controller_server` fails to configure; `rmw_zenoh_cpp` starts.
  Not caused by this entry. A possible fix, not applied, is
  `voxel_layer.publish_voxel_map: False`, which loses the `VoxelGrid`
  visualization. The sim, loopback and hero files have no STVL.
- **No AMCL in MuJoCo.** `nav_mujoco_omnibot.launch.py` localizes with
  odometry only. The measured drift against MuJoCo ground truth was 0.17 to
  0.43 m after one or two goals (runs 3 and 4).
- **Error 103 at the goal.** A goal sent while the robot is already at it by
  odometry aborts with 103, `Resulting plan has 0 poses in it`.
- **Real robot without the 3D lidar.** With `use_unitree_lidar:=false`, the
  `pointcloud` source has no data, so the collision monitor stops every Nav2
  command.
- **Progress checker.** The progress checker (0.5 m within 4 s) is unchanged.
  It aborted `follow_path` once with 105 during a slow final approach in runs
  3 and 9; the behavior tree retried and the goal was reached. The occasional
  `Failed to make progress` abort after the initial spin also remains.
- The wheel radius mismatch is unchanged: the MJCF capsule has radius 0.075 m,
  the controller uses 0.076 m. The odometry scale error is about 1.5 %.
- The smoother caps rotation at 1.0 rad/s, while MPPI's `wz_max` is 1.5 rad/s.
- **Real robot not verified.** No hardware run was made with these changes.
  The Docker images build from `main`, so the robot gets these changes only
  after the merge.
