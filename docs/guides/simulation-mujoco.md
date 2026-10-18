# MuJoCo Simulation

This guide shows how to run Walkie in MuJoCo with ROS 2 and Nav2, how to view and
regenerate the MuJoCo model, and how to build your own world around it. Two launch files
in `robot_bringup` start the simulation: `mujoco_omnibot.launch.py` with the controllers
and RViz, and `nav_mujoco_omnibot.launch.py` with Nav2 on top. MoveIt with MuJoCo is not
tested yet.

## Quick start { #what-works-now }

| Task | Command |
| --- | --- |
| Run MuJoCo with ROS 2, the controllers and RViz | `pixi run ros2 launch robot_bringup mujoco_omnibot.launch.py` |
| The same, plus Nav2: send goals from RViz | `pixi run ros2 launch robot_bringup nav_mujoco_omnibot.launch.py` |
| Open the robot on a floor, in the `home` pose | `pixi run mjcf-view` |
| Open any world file, with a working **Reload** button | `pixi run python -m mujoco.viewer --mjcf=description/mujoco/scene.xml` |
| Regenerate the robot model after a description change | `pixi run mjcf` |
| Test the model and the default world | `pixi run python -m pytest description/test/test_mjcf.py` |

The two launch files need a build and a Zenoh router first. See
[Run with ROS 2](#run-with-ros-2). The other commands need no colcon build: the generator
and `test_mjcf.py` read the xacro from the source tree, not from `install/`.

## The files

- `description/scripts/build_mjcf.py`: the generator. It expands
  `robots/gz_walkie.urdf.xacro` with `ros2_control:=mujoco` and converts it to MJCF.
- `description/mujoco/walkie.xml`: the generated robot model. It is checked in and has no
  floor.
- `description/mujoco/scene.xml`: the default world with floor, light, sky, and the robot.
  Load this file, not `walkie.xml`.

`description/` is a git submodule, so all three files live in the description repository.
[2.1 Generate the Model](../migrations/02-mujoco/01-model-generation.md) explains how the
generator works.

## Open a world in the viewer { #the-viewer }

| Command | Starts in | **Reload** |
| --- | --- | --- |
| `pixi run mjcf-view` | `home` | Does not re-read the file. The task passes a model that is already loaded. |
| `pixi run python -m mujoco.viewer --mjcf=<file>` | Joint positions at 0 | Re-reads the file from disk |

- Use the second form while you edit a world. After each save, click **Reload**.
- To get the `home` pose there, set **Key** to 0 in the left panel and click **Load key**.
  Without the keyframe, every hinge and slide joint starts at 0, so the lift starts below
  its lower limit of 0.08 m.
- You can also drag an XML file onto the viewer window to load it.

??? info "What the `home` keyframe contains"
    `walkie.xml` contains one keyframe, `home`. Its joint positions equal the
    `initial_value` entries of the `mujoco` ros2_control block, and a test keeps the two in
    step.

    | Joint | Position |
    | --- | --- |
    | `floating_base_joint` | z = -0.0062 m, no rotation |
    | `lift_joint` | 0.51 m |
    | `openarm_left_joint1` | 0.2618 rad (15 degrees) |
    | `openarm_left_joint4` | 0.2618 rad |
    | `openarm_right_joint1` | -0.2618 rad |
    | `openarm_right_joint4` | 0.2618 rad |
    | All other joints | 0 |

    The actuator targets in the keyframe equal these positions, so the robot holds the
    pose. The base starts 6.2 mm below the world origin because, in the URDF, the wheel
    bottoms are 6.2 mm above `base_footprint`. Lowering the root by that amount puts the
    wheels on the floor, so the robot starts at rest instead of dropping.

### Drive the robot in the viewer

Press ++space++ to run or pause. To drive the robot, use the **Control** panel on the
right: one slider per actuator. A slider sets the target of its actuator: a speed for the
four wheels, a position for the lift, head and arm joints. This is the way to drive it
without ROS 2. Press ++f1++ for the full list of shortcuts.

??? info "All viewer controls and panels"
    | Input | Action |
    | --- | --- |
    | ++space++ | Run or pause |
    | ++plus++ / ++minus++ | Speed up or slow down |
    | ++arrow-left++ / ++arrow-right++ | Step back or forward |
    | ++tab++ / ++shift+tab++ | Show or hide the left or right panel |
    | ++bracket-left++ / ++bracket-right++ | Cycle through the model cameras: `head_camera`, `bottom_camera` |
    | ++esc++ | Back to the free camera |
    | Left drag | Orbit the view |
    | Right drag, or ++shift++ + right drag | Pan the view |
    | Mouse wheel, or middle drag | Zoom |
    | Double-click | Select a body |
    | ++page-up++ | Select the parent of the selected body |
    | Right double-click | Center the view on the clicked point |
    | ++ctrl++ + right double-click | Make the camera track the clicked body |
    | ++ctrl++ + left drag | Rotate the selected body (applies a torque) |
    | ++ctrl++ + right drag | Pull the selected body (applies a force) |
    | ++f1++ | Help, with the full list of shortcuts |
    | ++f2++ / ++f3++ / ++f4++ | Info, profiler and sensor plots |
    | ++f5++ | Full screen |

    Hold the right mouse button over a panel to show its keyboard shortcuts. Double-click
    a section title to expand or collapse all sections.

    | Panel | Use |
    | --- | --- |
    | **Control** (right panel) | One slider per actuator: a speed for the four wheels, a position for the lift, head and arm joints |
    | **Joint** (right panel) | Current joint positions |
    | **Group enable**, Geom groups | Group 2 holds the visual geometry and is shown by default. Enable group 3 to see the collision geometry. |
    | **Group enable**, Site groups | Enable group 4 to see the URDF frames, group 5 for the lidar beam origins |

    The lidar beams themselves are not drawn.
    [2.4 Sensors](../migrations/02-mujoco/04-sensors.md) explains why.

## Edit the robot

!!! warning "Never edit `walkie.xml` by hand"
    It is generated from the xacro. Its header says so, the next `pixi run mjcf`
    overwrites it, and `test_mjcf.py` fails when the checked-in file differs from what the
    generator writes.

1. Change the xacro under `description/`.
2. Regenerate the model:

    ```bash
    pixi run mjcf
    ```

    It prints one line that starts with `wrote`, with the model sizes (`nq`, `nu`,
    `ngeom` and others) and the total mass. Compare it with the previous run to spot an
    unexpected change. If it prints `error:` lines or stops with a `ValueError`, see
    [Troubleshooting](#troubleshooting).

    Always run the generator through pixi. Another MuJoCo version writes a different
    `walkie.xml`, and a MuJoCo without the head 3D lidar extension
    (`ros-jazzy-mujoco-3d-lidar`) cannot write it at all.

3. Look at the result with `pixi run mjcf-view`, then run
   `pixi run python -m pytest description/test/test_mjcf.py`.
4. Commit the xacro change and the new `walkie.xml` together, inside the `description`
   submodule. Then commit the updated submodule pointer in walkie-ros.

??? info "Why the MuJoCo version is pinned"
    `pixi.toml` pins `mujoco` to `3.15.*`, the libmujoco version that `mujoco_vendor`
    0.1.0, and with it `mujoco_ros2_control`, is built against. The MjSpec serializer
    differs between versions: MuJoCo 3.10 writes a different `walkie.xml` than the
    checked-in 3.15 output.

### Change a MuJoCo-only value { #mujoco-only-values }

Some values exist only in the MuJoCo model, for example actuator gains and the inertia
estimates for `base_link` and `lift_link`. They are constants at the top of
`description/scripts/build_mjcf.py`. Change the constant and run `pixi run mjcf`. The URDF
is not changed.

??? info "MuJoCo-only constants and why some are estimates"
    | Constant | Value | Meaning |
    | --- | --- | --- |
    | `WHEEL_KV` | 20.0 | Velocity gain of the wheel actuators |
    | `POSITION_SATURATION_ERROR` | 0.05 | Position error (rad or m) at which a position actuator reaches its effort limit; sets kp |
    | `LIFT_FORCE_LIMIT_ESTIMATE` | 600.0 N | Lift actuator force limit |
    | `LIFT_POSITION_KP_ESTIMATE` | 5.0e4 N/m | Lift position gain |
    | `LIFT_JOINT_DAMPING_ESTIMATE` | 6000.0 N s/m | Lift joint damping |
    | `LIFT_LINK_MASS_ESTIMATE` | 3.0 kg | Mass used for `lift_link` |

    The URDF inertials of `base_link` and `lift_link` are placeholders, so the generator
    uses its own estimates. The lift and inertia values are marked as estimates in the
    code: they wait for lift drive data and CAD. The estimates are also listed in the
    header comment of `walkie.xml`.
    [2.5 ros2_control](../migrations/02-mujoco/05-ros2-control.md) explains the gain rules.

## Build a custom world

`description/mujoco/scene.xml` is the template. Its header lists the rules a world must
follow. The steps below build an example: a table with a cube on it, 1.2 m in front of
the robot.

1. **Create the file next to `scene.xml`**, here `description/mujoco/table_world.xml`. The
   world file must sit in a directory directly under `description/`. Anywhere else, it
   fails to load with an `Error opening file` message for a robot mesh.
2. **Include the robot first.** Put `<include file="walkie.xml"/>` before any other body
   that has joints. Otherwise the `home` keyframe lands on the wrong joints.
3. **Add the floor.** It must be a `type="plane"` geom named `floor`, in the FLOOR class:
   `contype="1" conaffinity="32"`. The wheel contact pairs refer to it by name.
4. **Copy the four wheel-floor `<pair>` elements unchanged from `scene.xml`.** They are
   the only contacts of the wheels.
5. **Keep the camera near plane.** Keep `<statistic center="0 0 0.7" extent="2.2"/>` and
   `<visual><map znear="0.025"/></visual>`, or the head camera renders the inside of its
   housing.
6. **Tag every other geom with a collision class**, FURN or OBJ. See
   [Collision classes](#collision-classes).
7. **Open the world.** See [Open the world](#open-the-world).

??? info "Why each rule exists"
    - **File location:** `walkie.xml` finds its meshes with `meshdir="../"`, and MuJoCo
      resolves that path from the directory of the top-level file.
    - **Robot first:** the robot's degrees of freedom then come first in `qpos`, and the
      `home` keyframe still lands on the right joints. A movable object declared before
      the include would shift every robot joint in `qpos`.
    - **Wheel pairs:** the wheel capsules belong to no collision class, so the four
      `<pair>` elements are their only contacts. The first friction value (0.005) acts
      along the wheel axle and lets the omni rollers slide sideways. The second (1) acts
      in the rolling direction. This only works against a plane.
      [2.2 Omni Wheels](../migrations/02-mujoco/02-omni-wheels.md) explains the model.
    - **Near plane:** the near clipping plane of every camera is `znear` times `extent`,
      here 5.5 cm. It must stay above about 3 cm, or the head camera renders the inside of
      the ZED housing it sits in.

### Collision classes

| Class | `contype` | `conaffinity` | Use for | Collides with |
| --- | --- | --- | --- | --- |
| FURN | 64 | 56 | Tables, shelves, walls: anything that never moves | Arms, grippers, OBJ |
| OBJ | 32 | 121 | Objects the arms push or grasp | Floor, arms, grippers, OBJ, FURN |

- **Furniture must be static.** FURN does not collide with the floor or with other
  furniture. Put it in a body without a joint. A FURN body with a joint falls through the
  floor.
- **Anything that moves is OBJ.** Give the body a `<freejoint/>` and a mass.
- **The base drives through everything.** The base and the robot body (lift, head, arm
  mounts) never collide with world geometry. This is deliberate: navigation relies on the
  lidars. Check clearance with the lidar readings, not with contacts.
- **Always set the bits.** An untagged geom gets the MuJoCo default,
  `contype="1" conaffinity="1"`. It then collides with the floor and with OBJ, but the arms
  and grippers pass through it.

[2.3 Collisions](../migrations/02-mujoco/03-collisions.md) has the full collision matrix.

### The complete example

```xml
<mujoco model="walkie_table_world">
  <!-- Step 2: the robot first, so its degrees of freedom come first in qpos. -->
  <include file="walkie.xml"/>

  <!-- Step 5: keep these two values. They set the camera near plane (5.5 cm). -->
  <statistic center="0 0 0.7" extent="2.2"/>
  <visual>
    <map znear="0.025"/>
  </visual>

  <worldbody>
    <light pos="0 0 3.5" dir="0 0 -1" directional="true"/>

    <!-- Step 3: the floor, a plane named "floor" in the FLOOR class. -->
    <geom name="floor" type="plane" size="0 0 0.05" rgba="0.3 0.3 0.3 1"
          contype="1" conaffinity="32"/>

    <!-- Step 6, FURN: a static table centred 1.2 m in front of the robot (no joint). -->
    <body name="table" pos="1.2 0 0.375">
      <geom name="table" type="box" size="0.4 0.6 0.375" rgba="0.6 0.45 0.3 1"
            contype="64" conaffinity="56"/>
    </body>

    <!-- Step 6, OBJ: a 5 cm cube on the table. It moves, so it has a free joint. -->
    <body name="cube" pos="0.9 0 0.78">
      <freejoint/>
      <geom name="cube" type="box" size="0.025 0.025 0.025" mass="0.1" rgba="0.8 0.1 0.1 1"
            contype="32" conaffinity="121"/>
    </body>
  </worldbody>

  <!-- Step 4: the four wheel-floor pairs, copied from scene.xml. -->
  <contact>
    <pair geom1="fl_wheel_contact" geom2="floor" condim="3" friction="0.005 1 0.005 0.0001 0.0001"/>
    <pair geom1="fr_wheel_contact" geom2="floor" condim="3" friction="0.005 1 0.005 0.0001 0.0001"/>
    <pair geom1="bl_wheel_contact" geom2="floor" condim="3" friction="0.005 1 0.005 0.0001 0.0001"/>
    <pair geom1="br_wheel_contact" geom2="floor" condim="3" friction="0.005 1 0.005 0.0001 0.0001"/>
  </contact>
</mujoco>
```

??? info "What to expect from the example world"
    - The table spans x = 0.8 to 1.6 m and is 0.75 m high.
    - The cube starts 5 mm above the table top and settles on it.
    - With the robot in `home`, the center beam of the front lidar, `front_lidar-120`,
      reads about 0.565 m to the table edge.
    - The skybox, checker floor and headlight settings of `scene.xml` are cosmetic and are
      left out here.
    - The cube's free joint makes `nq` grow from 31 to 38. The `home` keyframe still
      works: MuJoCo fills the extra values with the cube's starting pose.

### Open the world

To edit and reload, open it with `mujoco.viewer`:

```bash
pixi run python -m mujoco.viewer --mjcf=description/mujoco/table_world.xml
```

Then set **Key** to 0 and click **Load key** to get the `home` pose.

To start directly in `home`, run the `mjcf-view` command with your file:

```bash
pixi run python -c "import mujoco, mujoco.viewer; m = mujoco.MjModel.from_xml_path('description/mujoco/table_world.xml'); d = mujoco.MjData(m); mujoco.mj_resetDataKeyframe(m, d, m.key('home').id); mujoco.viewer.launch(m, d)"
```

## Script the model without the viewer

The MuJoCo Python bindings are in the pixi environment. They load the head 3D lidar
extension on `import mujoco`, so use them and not a MuJoCo from `pip`. This script loads
the example world, starts from `home`, simulates 2 s and prints the base position, the
cube position and the front lidar's center beam:

```python
import mujoco

model = mujoco.MjModel.from_xml_path('description/mujoco/table_world.xml')
data = mujoco.MjData(model)
mujoco.mj_resetDataKeyframe(model, data, model.key('home').id)

for _ in range(int(2.0 / model.opt.timestep)):
    mujoco.mj_step(model, data)

cube = model.body('cube')
cube_address = model.jnt_qposadr[model.body_jntadr[cube.id]]
print('base', data.qpos[:3])
print('cube', data.qpos[cube_address:cube_address + 3])
print('front_lidar-120', data.sensor('front_lidar-120').data)
```

Save it outside `description/` and run it from the repository root with
`pixi run python <script>`. The time step is 0.002 s, so 2 s is 1000 steps.

## Run with ROS 2

`mujoco_omnibot.launch.py` starts MuJoCo, the controllers and RViz.
`nav_mujoco_omnibot.launch.py` starts the same, plus Nav2. Both are in `robot_bringup`.
Run the commands from the repository root.

1. Build the workspace with `pixi run build`, as described in
   [Getting Started](../getting-started.md#5-build-the-workspace). The launch files also
   need `dual_laser_merger`, `walkie_tf`, `robot_self_filter`, `robot_navigation` and Nav2
   (`navigation/navigation2`). If your workspace was built before the MuJoCo launch files
   or the head 3D lidar existed, run `pixi install` (it adds the lidar extension), then
   rebuild the packages that hold the launch files, the default world and the self filter:

    ```bash
    pixi install
    pixi run build-pkg robot_bringup walkie_description robot_self_filter
    ```

    The default world loads from the installed `walkie_description`. Build again after you
    add a world file to `description/mujoco/` that you want to load from `install/` (see
    [What needs a rebuild](pixi.md#what-needs-a-rebuild)).

2. Start one Zenoh router per machine, in its own terminal:

    ```bash
    pixi run ros2 run rmw_zenoh_cpp rmw_zenohd
    ```

3. In a second terminal, start a launch file:

    ```bash
    pixi run ros2 launch robot_bringup mujoco_omnibot.launch.py
    ```

    For Nav2, start `nav_mujoco_omnibot.launch.py` instead. It includes
    `mujoco_omnibot.launch.py`. See [Drive with Nav2 and RViz](#drive-with-nav2-and-rviz).

- `pixi run` selects Zenoh (`RMW_IMPLEMENTATION=rmw_zenoh_cpp`) for every ROS 2 command.
  See [Middleware (Zenoh)](middleware.md#simulation-on-one-machine).
- The MuJoCo viewer window and RViz open. Closing the MuJoCo window stops the whole
  launch. Add `headless:=true` to run without the window.
- Every node runs on simulated time from `/clock`, which the simulator publishes.
- With `nav_mujoco_omnibot.launch.py`, Ctrl+C takes about 15 s and prints red error
  lines. They are harmless. See [Troubleshooting](#troubleshooting).

??? info "What the MuJoCo launch files start"
    `bringup/launch/mujoco_omnibot.launch.py` starts:

    1. `robot_state_publisher`, with the URDF of `description/robots/gz_walkie.urdf.xacro`
       expanded inside the launch file with `ros2_control:=mujoco`,
       `mujoco_model:=<mujoco_model>` and `mujoco_headless:=<headless>`.
       `description/launch/robot_state_publisher.launch.py` is not used, because it does
       not forward `mujoco_model` or `mujoco_headless`.
    2. `ros2_control_node` of the package `mujoco_ros2_control`, not the one of
       `controller_manager`. It steps the physics, runs the controller manager in the same
       process, publishes `/clock`, and reads the URDF from `/robot_description`. Its
       parameters are `use_sim_time: true`,
       `description/config/ros2_controller/mujoco_controllers.yaml` and
       `bringup/config/mujoco/mujoco_plugins.yaml`. It remaps
       `/imu_sensor_broadcaster/imu` to `/imu`. When it exits, for example because the
       MuJoCo window was closed, the whole launch shuts down.
    3. Two spawners, both with `--param-file` set to `mujoco_controllers.yaml`. The first
       spawns `joint_broad`, `omni_wheel_drive_controller` and `imu_sensor_broadcaster`.
       The second spawns `lift_controller`, `head_servo_controller`,
       `left/right_joint_trajectory_controller` and `left/right_gripper_controller`.
    4. `twist_mux` with `bringup/config/twist_mux/twist_mux.yml`, the same file as Gazebo
       and the real robot. Its output `/cmd_vel_out` is remapped to
       `/omni_wheel_drive_controller/cmd_vel`.
    5. `dual_laser_merger_node` (node name `dual_laser_merger`) with the parameters of the
       real robot, plus `publish_on_empty_cloud: True`. It merges `/front_lidar` and
       `/back_lidar` into `/scan`, in the frame `merged_lidar_link`. With
       `publish_on_empty_cloud`, it also publishes when a lidar has no return in range,
       as in the empty default world; that side reads as free space. The real robot keeps
       the default, `false`. See
       [5. Repair Nav2 After the navigation2 Bump](../migrations/05-nav2-repair.md#scan-in-the-empty-mujoco-world).
    6. `robot_self_filter`'s `self_filter.launch.py` (node name `self_filter`) with
       `bringup/config/lidar/unitree_self_filter.yaml`, the configuration of the real
       robot. It reads the head 3D lidar cloud `/unilidar/cloud` and publishes
       `/unilidar/cloud/filtered` without the points on the robot.
    7. `walkie_tf`'s `tf_server` (node name `walkie_tf_server`).
    8. RViz with `rviz_config`, when `use_rviz` is `true`.

    `bringup/launch/nav_mujoco_omnibot.launch.py` starts:

    1. `mujoco_omnibot.launch.py`, with `mujoco_model`, `headless`, `use_rviz` and
       `rviz_config`.
    2. `static_transform_publisher` (node name `map_to_odom`): an identity transform from
       `map` to `odom`.
    3. `nav2_bringup`'s `bringup_launch.py` with `use_sim_time:=True`,
       `use_localization:=False` (no AMCL), `serve_static_map:=True` (`map_server` serves
       `map`), `use_keepout_zones:=False`, `use_speed_zones:=False`, `autostart:=True`
       and `use_composition:=False`. Each Nav2 server runs as its own process.
       `params_file` is passed through unchanged. The include sits in a `GroupAction`
       with `SetRemap(src="cmd_vel", dst="cmd_vel_controller")`, so Nav2 commands pass
       `velocity_smoother` and `collision_monitor` before they reach `twist_mux` on
       `/cmd_vel_nav`.

    `robot_bringup` does not declare `mujoco_ros2_control` or `mujoco_3d_lidar` in its
    `package.xml`. The pixi environment provides them, and declaring them would make rosdep
    install MuJoCo into the `physical` Docker image. [2.7 Launch Files and Nav2](../migrations/02-mujoco/07-launch.md)
    explains the Nav2 settings, and
    [5. Repair Nav2 After the navigation2 Bump](../migrations/05-nav2-repair.md) the
    `cmd_vel` remap.

??? info "Configuration files of the MuJoCo launch files"
    | Path | Contents |
    | --- | --- |
    | `bringup/config/mujoco/mujoco_plugins.yaml` | Cameras and lidars. See [The mujoco_plugins parameter file](#the-mujoco_plugins-parameter-file). |
    | `bringup/config/lidar/unitree_self_filter.yaml` | Self filter of the head 3D lidar cloud, shared with the real robot |
    | `bringup/config/map/empty_floor.yaml`, `empty_floor.pgm` | Blank map for Nav2: 20 m by 20 m, 0.05 m cells (400 by 400 pixels, all free), origin `[-10, -10, 0]`, so it is centered on the world origin |
    | `bringup/config/rviz/mujoco.rviz` | Fixed frame `odom`, orbit view around `base_footprint`. Shows the robot model, `/scan`, the filtered head 3D lidar cloud (**Head 3D lidar**, colored by height) and the head camera image. The front lidar, back lidar, raw 3D lidar cloud, bottom camera and TF displays exist but are off. |
    | `bringup/config/rviz/mujoco_nav.rviz` | Fixed frame `map`, top-down view. **Navigation 2** panel. Shows the map, the global and local costmaps, the robot model, the footprint, the global plan `/plan`, `/scan` and the filtered head 3D lidar cloud. Tools include **2D Goal Pose** and **Nav2 Goal**. |
    | `bringup/config/twist_mux/twist_mux.yml` | Velocity input priorities |
    | `description/config/ros2_controller/mujoco_controllers.yaml` | Controller manager and controller parameters |

### Launch arguments

Pass them as `name:=value`. This example runs Nav2 without the MuJoCo window and without
RViz:

```bash
pixi run ros2 launch robot_bringup nav_mujoco_omnibot.launch.py headless:=true use_rviz:=false
```

| Argument | Launch file | Default | Meaning |
| --- | --- | --- | --- |
| `mujoco_model` | Both | `<walkie_description share>/mujoco/scene.xml` | Absolute path of the MuJoCo world. See [Use a custom world with ROS 2](#use-a-custom-world-with-ros-2). |
| `headless` | Both | `false` | `true` runs MuJoCo without the viewer window |
| `use_rviz` | Both | `true` | `false` starts no RViz |
| `rviz_config` | Both | `<robot_bringup share>/config/rviz/mujoco.rviz`, or `mujoco_nav.rviz` for Nav2 | RViz config file |
| `map` | `nav_mujoco_omnibot.launch.py` | `<robot_bringup share>/config/map/empty_floor.yaml` | Map for `map_server`. Its origin must match the MuJoCo world origin. |
| `params_file` | `nav_mujoco_omnibot.launch.py` | `<robot_navigation share>/config/nav2/nav2_mppi_walkie_sim_params.yaml` | Nav2 parameter file |

### Topics

The base follows `TwistStamped` messages on `/cmd_vel`, through `twist_mux`, as in Gazebo.
Publish at 10 Hz and stop with Ctrl+C:

```bash
pixi run ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_footprint}, twist: {linear: {x: 0.2}}}"
```

[Gazebo Simulation: Drive the robot](simulation-gazebo.md#driving-the-robot) explains the
priorities and timeouts.

| Topic | Type | Notes |
| --- | --- | --- |
| `/cmd_vel` (priority 5), `/cmd_vel_nav` (10, Nav2), `/cmd_vel_joy` and `/cmd_vel_key` (100) | `geometry_msgs/TwistStamped` | Inputs of `twist_mux`. The highest-priority active input wins. `collision_monitor` publishes `/cmd_vel_nav`. |
| `/cmd_vel_controller`, `/cmd_vel_smoothed` | `geometry_msgs/TwistStamped` | Nav2 only: output of the Nav2 servers, input of `velocity_smoother`; output of `velocity_smoother`, input of `collision_monitor` |
| `/omni_wheel_drive_controller/cmd_vel` | `geometry_msgs/TwistStamped` | Output of `twist_mux`. Publish here to bypass `twist_mux`. |
| `/omni_wheel_drive_controller/odom` | `nav_msgs/Odometry` | Wheel odometry in the frame `odom`. The controller also publishes the `odom` to `base_footprint` transform. |
| `/scan` | `sensor_msgs/LaserScan` | `/front_lidar` and `/back_lidar` merged, frame `merged_lidar_link`. Also published in the empty default world: a lidar with no return in range reads as free space. Needs both lidar topics. |
| `/front_lidar`, `/back_lidar` | `sensor_msgs/LaserScan` | Frames `front_lidar_link` and `back_lidar_link`. About 28 Hz. |
| `/unilidar/cloud/filtered` | `sensor_msgs/PointCloud2` | Head 3D lidar without the points on the robot, as on the real robot. Frame `unitree4d_l2_imu_initial`, unorganized, `x`, `y`, `z` only. About 11.7 clouds per simulated second (12 Hz configured). Best effort. |
| `/unilidar/cloud` | `sensor_msgs/PointCloud2` | Head 3D lidar as simulated: organized, 180 x 32 rays, NaN where a ray hits nothing, includes points on the robot. Same frame and rate. |
| `/imu` | `sensor_msgs/Imu` | Frame `top_imu_link` |
| `/zed_head/zed_node/rgb/color/rect/image`, `/zed_head/zed_node/depth/depth_registered` | `sensor_msgs/Image` | Head camera, color and depth. The topic names of the real ZED camera. |
| `/zed_head/zed_node/depth/camera_info` | `sensor_msgs/CameraInfo` | Head camera, frame `zed_head_left_camera_frame_optical` |
| `/bottom_camera/image_raw`, `/bottom_camera/depth/image_raw` | `sensor_msgs/Image` | Bottom camera, color and depth |
| `/bottom_camera/camera_info` | `sensor_msgs/CameraInfo` | Bottom camera, frame `bottom_color_optical_frame` |
| `/mujoco_ground_truth/odom` | `nav_msgs/Odometry` | True base pose from the simulator, without TF. Gazebo uses `/gazebo_ground_truth/odom`. |
| `/clock` | `rosgraph_msgs/Clock` | Simulated time. About 340 messages per second since the head 3D lidar (about 500 before it). |

??? info "What the launch files measured on 2026-10-13"
    Measured headless, with `rmw_zenoh_cpp`.

    | Item | Result |
    | --- | --- |
    | Controllers | The nine spawned controllers (of the 14 in `mujoco_controllers.yaml`) became active |
    | Real-time factor | 1.00, with `/clock` at about 500 Hz |
    | `/front_lidar`, `/back_lidar`, `/scan` | About 27.8 Hz, not the configured 40 Hz. See [Troubleshooting](#troubleshooting). |
    | Frames | `/imu` in `top_imu_link`, `/front_lidar` in `front_lidar_link`, `/back_lidar` in `back_lidar_link`, `/scan` in `merged_lidar_link`, `/omni_wheel_drive_controller/odom` in `odom` |
    | Cameras | Published on the topics and frames listed above |

??? info "What the head 3D lidar costs (measured 2026-10-16)"
    Each cloud stalls the simulation for about 53 ms of wall time, every 84 ms of simulated
    time. `mujoco_omnibot.launch.py` still ran at a real-time factor of 1.00 and
    `nav_mujoco_omnibot.launch.py` at 0.94 to 0.99 (1.00 without the lidar).
    `ros2_control_node` used about 115 % CPU instead of 56 to 66 %, and `/clock` dropped
    from about 500 to about 340 messages per second (about 270 with Nav2). On a slower
    machine the simulation falls further below real time; fewer rays help, see
    [2.8 Head 3D Lidar](../migrations/02-mujoco/08-3d-lidar.md#resolution-and-rate).

### Drive with Nav2 and RViz

```bash
pixi run ros2 launch robot_bringup nav_mujoco_omnibot.launch.py
```

This loads the default world, which has only a floor. Nav2 drives the robot there. Its
collision monitor needs `/scan`, and the launch publishes `/scan` although the lidars see
nothing in range (see [Topics](#topics)). To test obstacle avoidance, add
`mujoco_model:=` with a world that has walls or objects. See
[Use a custom world with ROS 2](#use-a-custom-world-with-ros-2).

Wait until the log shows `Managed nodes are active`, about 10 s after the start. Then send
a goal in RViz:

1. Click **2D Goal Pose** in the toolbar.
2. Click the goal position on the map, drag in the direction the robot should face, and
   release.
3. Nav2 plans a path, drawn as the global plan, and the robot drives to the goal.

The **Navigation 2** panel and the **Nav2 Goal** tool use the same `NavigateToPose`
action, so they should work; they have not been checked interactively. From a script,
publish a `geometry_msgs/PoseStamped` on `/goal_pose`, or send a `NavigateToPose` action
goal.

Nav2 commands pass `velocity_smoother` and `collision_monitor` before `twist_mux`. The
smoother caps the speed at 0.3 m/s and 1.0 rad/s. The collision monitor slows the robot
near obstacles and stops it before obstacles in the direction of travel. Commands that
drive away from the obstacle pass, so Nav2 can move the robot on after a stop. In a
MuJoCo test, the robot stopped 0.045 to 0.053 m before a wall in front and about 0.11 m
before a wall at the side.
[Navigation: Collision monitor](navigation.md#collision-monitor) has the zones and
settings.

!!! warning "The collision monitor misses narrow obstacles straight ahead"
    Straight ahead, the front lidar sees nothing closer than 0.415 m to the base center:
    it sits 0.235 m in front of the center and has a minimum range of 0.18 m. A wall is
    still detected, because its points off to the side are farther from the lidar. A
    narrow obstacle dead ahead, narrower than about 0.28 m and closer than 0.415 m, is not
    in `/scan`, and the collision monitor does not stop for it. This is a limit of the
    MuJoCo sensor setup; the minimum ranges of the real lidars were not checked.

If the robot is boxed in and Nav2 cannot find a free direction, drive it out on
`/cmd_vel_key` or `/cmd_vel_joy`: `twist_mux` ranks them above Nav2, and the monitor does
not check them. This command backs the robot away from an obstacle in front; stop it with
Ctrl+C:

```bash
pixi run ros2 topic pub -r 10 /cmd_vel_key geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_footprint}, twist: {linear: {x: -0.2}}}"
```

!!! warning "Localization is odometry only"
    No AMCL runs. A static identity transform from `map` to `odom` places the robot in the
    map by its wheel odometry alone.

    - **The robot must start at the MuJoCo world origin.** The `home` keyframe does.
    - **The map origin must match the world origin.** The default map,
      `empty_floor.yaml`, is a blank 20 m by 20 m map centered on it.
    - **The pose drifts.** On 2026-10-14, the odometry pose was 0.17 m from the ground
      truth after two goals in the default world. In a walled test world, it was 0.28 m
      after one goal and 0.43 m after two. RViz can show the robot at the goal while the ground truth
      on `/mujoco_ground_truth/odom` is off by that much.

The Nav2 log can show errors that do not come from MuJoCo. See
[Troubleshooting](#troubleshooting).

??? info "What the Nav2 tests measured on 2026-10-14"
    Measured headless, with `rmw_fastrtps_cpp`, on the command path of
    [5. Repair Nav2 After the navigation2 Bump](../migrations/05-nav2-repair.md):
    Nav2 servers to `/cmd_vel_controller`, `velocity_smoother`, `/cmd_vel_smoothed`,
    `collision_monitor`, `/cmd_vel_nav`, `twist_mux`, all `TwistStamped`. Localization is
    odometry only, so the ground truth error includes the odometry drift.

    | Test | Result |
    | --- | --- |
    | Default world, new command path, merger without `publish_on_empty_cloud` (before the merger change) | No `/scan`. `collision_monitor` logged `Robot to stop due to invalid source`, the robot did not move, and goals aborted with error 105 (`Failed to make progress`). |
    | Default world, an earlier version of the merger change that always published | `/scan` at about 27 Hz. Goal (1.5, 1.0, 90 degrees) on `/goal_pose` reached in 11 s, 0.14 m and 3.7 degrees off in the ground truth. Goal (-1.0, -0.5, 0) as a `NavigateToPose` action goal `SUCCEEDED` in 13 s, 0.39 m off in the ground truth. |
    | Default world, `publish_on_empty_cloud: True` in `mujoco_omnibot.launch.py` (the current state) | Nav2 active after 10 s. Goal (1.5, 1.0, 90 degrees) on `/goal_pose` reached in 10 s: 0.10 m from the goal by odometry, 0.14 m and 3.8 degrees off in the ground truth. Goal (-1.0, -0.5, 0) as an action goal `SUCCEEDED` in 14 s: 0.02 m from the goal by odometry, 0.15 m and 1.3 degrees off in the ground truth (odometry drift 0.17 m). |
    | Progress checker, same run, with the earlier 4 s allowance | During the slow final approach to the first goal, `controller_server` aborted `follow_path` once with error 105. The behavior tree retried, and the goal was reached 0.3 s later. |
    | Progress checker with 8 s allowance, walled test world, two goals from beside the room wall | One abort with error 105, at the end of the slow final approach to the first goal; the goal was then within tolerance. With the 4 s allowance and an earlier stop layout, the same goals had 4 to 5 aborts. |
    | Walled test world (the room described below), goal (3.0, 0, 0) behind the wall | Reached in 25 s: 0.06 m from the goal by odometry, 0.24 m in the ground truth (drift 0.28 m) |
    | Walled test world, next goal (-1.0, -0.5, 0) | Aborted with error 103 (`Resulting plan has 0 poses in it`) while the odometry pose was 0.002 m from the goal. The ground truth was 0.43 m away. |
    | Walled test world, constant 0.25 m/s toward the wall, with the earlier 0.35 m `PolygonStop` circle | The collision monitor stopped the robot with the front of its footprint at the wall face. Two goals after the stop aborted with error 105. |
    | Walled test world, constant 0.25 m/s toward the wall, with the direction-aware `VelocityPolygonStop` | Stopped with the footprint front 0.045 m before the wall face (repeat run: 0.046 m). From there, goal (1.5, 1.0, 90 degrees) as an action goal `SUCCEEDED` after 28.8 s, 0.072 m and 4.0 degrees off in the ground truth. `follow_path` aborted twice with error 105 on the way; the behavior tree retried both times. |
    | Walled test world, single commands toward, away from, and along the walls, with `VelocityPolygonStop` | Toward the wall face: stopped 0.048 m before it going forward, 0.053 m going diagonally. Toward the room wall at the robot's left: stopped 0.112 m before it going left, 0.114 m going diagonally. Rotation in place at the wall turned the robot, and every command away from or along a wall moved it. |

??? info "What the Nav2 tests measured on 2026-10-13"
    Measured headless, with `rmw_zenoh_cpp`, before
    [5. Repair Nav2 After the navigation2 Bump](../migrations/05-nav2-repair.md) put
    `velocity_smoother` and `collision_monitor` into the command path and raised the local
    costmap `inflation_radius` to 0.35 m. The results below describe the old command path,
    on which the collision monitor did not act, so Nav2 also drove in the default world
    without `/scan`.

    | Test | Result |
    | --- | --- |
    | Default world, goal on `/goal_pose` (x 1.5 m, y 1.0 m, yaw 90 degrees), as RViz **2D Goal Pose** sends it | `bt_navigator` logged `Goal succeeded` |
    | Default world, `NavigateToPose` action goal (x -1.0 m, y -0.5 m, yaw 0) | `SUCCEEDED`, 0.04 m from the target in the ground truth |
    | Wall world: an 8 m by 8 m room with a 1.2 m wall segment at x = 1.5 m, between the start and the goal (3, 0) | `/scan` at about 27.8 Hz. The robot drove around the wall and reached the goal. The wall reaches 0.6 m to each side of the x axis; while passing it, the robot was 0.83 to 0.99 m from the x axis in the ground truth. Same result with `rmw_fastrtps_cpp`. After this run, the pose in the map was off by 0.22 m and 4.7 degrees. |

    Odometry against ground truth, with fixed twists on
    `/omni_wheel_drive_controller/cmd_vel`:

    | Motion | Ground truth / odometry |
    | --- | --- |
    | Forward | 0.985 |
    | Sideways | 0.986 |
    | Spin in place | 0.978 |
    | Arc | 0.969 |

    The odometry is open loop (`open_loop: true`): it is computed from the commands. The
    MJCF wheel capsule radius comes from the URDF collision cylinder, 0.075 m, while
    `wheel_radius` in `mujoco_controllers.yaml` is 0.076 m. That ratio, 0.075 / 0.076,
    explains the translation error. The rest of the turning error is wheel slip.

    RViz was not used interactively in these tests. Every RViz plugin class in both
    configs exists, and every topic they subscribe to exists in the running stack.

### Use a custom world with ROS 2

Pass the world file with `mujoco_model`. Replace `<world>` with the file name:

```bash
pixi run ros2 launch robot_bringup mujoco_omnibot.launch.py mujoco_model:=$PWD/description/mujoco/<world>.xml
```

- The path must be absolute. Run the command from the repository root, so that `$PWD`
  expands to it.
- A world in the source tree needs no rebuild. MuJoCo finds `walkie.xml` and the meshes
  next to it.
- The rules of [Build a custom world](#build-a-custom-world) still apply: the file sits in
  a directory directly under `description/`, includes the robot first, has a plane named
  `floor`, and has the four wheel-floor pairs.
- `/scan` is published in any world, also when one lidar sees nothing in range, as
  behind the robot in the table example. That side reads as free space.

For Nav2, pass the same argument to `nav_mujoco_omnibot.launch.py`. The blank default map
works, because the costmaps get the obstacles from `/scan`. A map of your own, passed with
`map:=`, must have its origin at the world origin: the robot starts there, and
localization is odometry only.

### The mujoco_plugins parameter file

`bringup/config/mujoco/mujoco_plugins.yaml` configures the cameras and lidars.
`mujoco_omnibot.launch.py` passes it to `ros2_control_node`. Without it, the lidars do not
publish, and the cameras register themselves under `/mujoco_camera_plugin/...`.

| Plugin instance | MuJoCo sensor | Topics | Frame |
| --- | --- | --- | --- |
| `mujoco_camera_plugin` (`CameraPlugin`, 5 Hz) | `head_camera` | `/zed_head/zed_node/rgb/color/rect/image`, `/zed_head/zed_node/depth/depth_registered`, `/zed_head/zed_node/depth/camera_info` | `zed_head_left_camera_frame_optical` |
| | `bottom_camera` | `/bottom_camera/image_raw`, `/bottom_camera/depth/image_raw`, `/bottom_camera/camera_info` | `bottom_color_optical_frame` |
| `rangefinder_lidar_plugin` (`RangefinderLidarPlugin`, 40 Hz configured) | `front_lidar` | `/front_lidar` | `front_lidar_link` |
| | `back_lidar` | `/back_lidar` | `back_lidar_link` |
| `mujoco_3d_lidar_plugin` (`Mujoco3dLidarPlugin`, 12 Hz set in `walkie.xml`) | `unitree_l2` | `/unilidar/cloud` | `unitree4d_l2_imu_initial` |

Each 2D lidar is one entry under the plugin instance:

```yaml title="bringup/config/mujoco/mujoco_plugins.yaml (excerpt)"
/**:
  ros__parameters:
    mujoco_plugins:
      rangefinder_lidar_plugin:
        type: "mujoco_ros2_control_plugins/RangefinderLidarPlugin"
        lidar_publish_rate: 40.0
        front_lidar:
          frame_name: front_lidar_link
          angle_increment: 0.017444444416
          min_angle: -2.09333333
          max_angle: 2.09333333
          range_min: 0.18
          range_max: 10.0
          laserscan_topic: /front_lidar
```

- **Name the 2D lidar instance exactly `rangefinder_lidar_plugin`.** The plugin reads its
  publish rate from the fixed key
  `mujoco_plugins.rangefinder_lidar_plugin.lidar_publish_rate`.
- The camera and lidar names (`head_camera`, `front_lidar`, `unitree_l2` and so on) must
  match the names in `walkie.xml`. Each `frame_name` must be a frame of the URDF.
- The 3D lidar entry sets only `frame_name` and `topic`. Its rays, range and rate are in
  `walkie.xml`: change `LIDAR_3D_RESOLUTION`, `LIDAR_3D_RANGE` or `LIDAR_3D_UPDATE_RATE` in
  `build_mjcf.py` and run `pixi run mjcf` ([Change a MuJoCo-only value](#mujoco-only-values)).
- The head camera publishes its images and camera info on the topic names of the real ZED
  camera, so nodes that read them need no remaps.
- No ZED point cloud is published. Nodes that read
  `/zed_head/zed_node/point_cloud/cloud_registered` or its filtered copy, such as the
  grasp node of `walkie_perception`, get no cloud. MoveIt with `hardware_type:=mujoco`
  runs without an octomap.

??? info "How the file maps to the model"
    - One lidar instance serves both lidars. `front_lidar` and `back_lidar` are the sensor
      name prefixes in `walkie.xml` (`front_lidar-0` and so on). Each `frame_name` is the
      URDF frame of the lidar.
    - The beam values come from the `walkie.xml` header: 241 beams from -2.0933 to
      2.0933 rad, range 0.18 to 10 m.
    - A beam without a return reads -1.0. `dual_laser_merger` builds `/scan` from the two
      topics.
    - The camera `frame_name` values are the URDF optical frames. One MuJoCo camera renders
      both head camera images, so the depth image is registered to the color image.
    - `unitree_l2` is the name of a sensor of the MuJoCo extension `mujoco.plugin.lidar`
      (package `ros-jazzy-mujoco-3d-lidar`) in `walkie.xml`. `Mujoco3dLidarPlugin`
      publishes every sensor of that extension; the entry under the instance only sets its
      frame and topic. The frame is the one the real L2 driver stamps its cloud with.
    - `RangefinderLidarPlugin` is deprecated upstream in favor of `Mujoco3dLidarPlugin`.
      The 2D lidars keep it because the extension cannot skip the robot's own geoms, and
      every 2D beam would end on `base_link`. See
      [2.8 Head 3D Lidar](../migrations/02-mujoco/08-3d-lidar.md).

## Tests

Run the tests after every model change. Add `-k lidar` to run only the tests whose name
contains `lidar`.

```bash
pixi run python -m pytest description/test/test_mjcf.py
```

- The tests use `scene.xml` and do not load custom worlds.
- If `mujoco` is not installed, they are skipped instead of failing.

[2.6 Tests](../migrations/02-mujoco/06-tests.md) lists every test and its tolerance.

## Not available yet

| Feature | Status | Details |
| --- | --- | --- |
| MoveIt with `hardware_type:=mujoco` | The launch option exists. MoveIt has not been tested with MuJoCo. | [2.5 ros2_control](../migrations/02-mujoco/05-ros2-control.md) |
| Robot base collisions with the world | Left out by design. Navigation relies on the lidars. | [2.3 Collisions](../migrations/02-mujoco/03-collisions.md) |
| Localization in the map | Odometry only: a static `map` to `odom` transform. No AMCL and no localization from the ground truth, so the pose drifts. | [Drive with Nav2 and RViz](#drive-with-nav2-and-rviz) |
| Interactive use of RViz | Not yet checked by a person. The plugin classes and topics of both RViz configs were checked. | [Drive with Nav2 and RViz](#drive-with-nav2-and-rviz) |
| Nav2 with the head 3D lidar | The cloud is published and filtered, but the simulation Nav2 parameter file reads only `/scan`. The file is shared with Gazebo, which has no 3D lidar, and `collision_monitor` stops every Nav2 command while a source has no data. | [2.8 Head 3D Lidar](../migrations/02-mujoco/08-3d-lidar.md#still-open) |
| `intensity`, `ring` and `time` in the 3D lidar cloud | The simulated cloud has `x`, `y` and `z` only | [2.8 Head 3D Lidar](../migrations/02-mujoco/08-3d-lidar.md#real-and-simulated) |

## Troubleshooting

| Symptom | Cause | Fix |
| --- | --- | --- |
| `pixi run mjcf` stops with `ValueError: link '<name>' has collision geometry but no collision class; add it to LINK_COLLISION_CLASSES` | A new or renamed link has collision geometry | Add a pattern for the link to `LINK_COLLISION_CLASSES` in `build_mjcf.py`. [2.3 Collisions](../migrations/02-mujoco/03-collisions.md) explains the classes. |
| `pixi run mjcf` stops with `ValueError: wheel link '<name>' needs one collision cylinder` | A wheel link has no collision, several, or a shape other than a cylinder | Give each wheel link exactly one cylinder collision. The generator turns it into the contact capsule. |
| `pixi run mjcf` prints `error: ros2_control joint '<name>' has no actuator` or `error: actuator '<name>' has no ros2_control joint` and exits with code 1 | The `mujoco` ros2_control block and the generated actuators disagree | Fix the `mujoco` branch of the ros2_control xacro. The file is still written, so do not commit it until the error is gone. |
| `test_generated_model_is_up_to_date` fails with `mujoco/walkie.xml is out of date` | The xacro changed, but `walkie.xml` was not regenerated | Run `pixi run mjcf` and commit `walkie.xml` |
| **Reload** shows the old world after an edit | `pixi run mjcf-view` passes a model that is already loaded | Open the file with `pixi run python -m mujoco.viewer --mjcf=<file>` |
| The robot falls through the floor | The four wheel-floor pairs are missing | Copy the four `<pair>` elements from `scene.xml` |
| Loading fails with `geom 'floor' not found in collision 0` | The pairs refer to `floor`, but the plane has another name | Name the plane `floor` |
| The robot starts in a strange pose in a custom world | A body with joints comes before `<include file="walkie.xml"/>`, so the keyframe values land on the wrong joints | Move the include to the top |
| `Error opening file` for a robot mesh | The world file is not in a directory directly under `description/` | Move it next to `scene.xml` |
| A table or shelf falls through the floor | It is FURN but has a joint | Remove the joint. If it must move, make it OBJ. |
| The arms pass through a world object | The geom has no collision bits, or the wrong ones | Set FURN or OBJ bits |
| The robot drives through a wall | By design: the base never collides with world geometry | Use the lidar readings for clearance |
| The head camera shows the inside of its housing | `znear` or `extent` was changed | Restore `<statistic extent="2.2"/>` and `<map znear="0.025"/>` |
| The lift stops near 0.48 m when you lower it from `home` | At `home`, the fingers hit `base_link`. This is a model issue: it happens with and without ROS 2. | None yet. See [Known issues](../migrations/02-mujoco/index.md#known-issues). |
| `ros2_control_node` of `mujoco_ros2_control` cannot load the model | `install/` has no `mujoco` folder, or not your world file | `pixi run build-pkg walkie_description`, or pass the source-tree path with `mujoco_model:=$PWD/description/mujoco/<world>.xml` |
| The MuJoCo window opens, but you want none | `headless` defaults to `false` | Add `headless:=true` |
| The whole launch stops when the MuJoCo window is closed | `ros2_control_node` ends the launch when it exits | Expected. Use `headless:=true` to run without the window. |
| `/front_lidar` and `/back_lidar` do not exist | `ros2_control_node` was started without `bringup/config/mujoco/mujoco_plugins.yaml`, for example by hand | Use `mujoco_omnibot.launch.py`, or pass the file. See [The mujoco_plugins parameter file](#the-mujoco_plugins-parameter-file). |
| Loading `walkie.xml` or a world fails with `XML Error: plugin mujoco.plugin.lidar not found` | The MuJoCo in use does not have the head 3D lidar extension, for example a MuJoCo from `pip` outside pixi, or a pixi environment created before the extension was added | Run `pixi install`, and load the model through `pixi run`. See [2.8 Head 3D Lidar](../migrations/02-mujoco/08-3d-lidar.md#packaged-extension). |
| `pixi run mjcf` stops with `RuntimeError: MuJoCo extension 'mujoco.plugin.lidar' is not available` | Same cause, found by the generator before it writes anything | `pixi install` |
| `/unilidar/cloud` does not exist | `ros2_control_node` was started without the `mujoco_3d_lidar_plugin` entry of `mujoco_plugins.yaml`, or `walkie.xml` predates the 3D lidar | Use `mujoco_omnibot.launch.py`. Rebuild `walkie_description` after updating the `description` submodule. |
| The launch stops at the start with `PackageNotFoundError` for `robot_self_filter` | `robot_self_filter` is not built in this workspace | `pixi run build-pkg robot_self_filter` |
| RViz shows nothing under **Head 3D lidar** | The display does not use **Best Effort**, or `/unilidar/cloud` is missing (see the row above) | Set the display's reliability policy to **Best Effort**. Check `pixi run ros2 topic hz /unilidar/cloud/filtered`. |
| `/front_lidar`, `/back_lidar` and `/scan` publish at about 28 Hz, not 40 Hz | Not known yet. [2.7 Launch Files and Nav2](../migrations/02-mujoco/07-launch.md) records a likely cause. | None yet |
| Camera topics appear under `/mujoco_camera_plugin/...`, and their frames are not in TF | No `CameraPlugin` is configured, so the cameras register themselves (deprecated) | Use `mujoco_omnibot.launch.py`, or add the `mujoco_camera_plugin` entry of the parameter file |
| No `/scan` | `dual_laser_merger` pairs `/front_lidar` and `/back_lidar` by time stamp and publishes nothing while one of them is missing. It also publishes nothing when it cannot transform a scan into `merged_lidar_link` (`Transform failure` in its log). A merger started outside `mujoco_omnibot.launch.py` without `publish_on_empty_cloud: True` publishes nothing while a lidar has no return in range, as in the default world. | Check both lidar topics with `pixi run ros2 topic hz /front_lidar` and `/back_lidar`; if one is missing, see the row on missing lidar topics above. Start the merger through `mujoco_omnibot.launch.py`. |
| The robot does not move under Nav2. `collision_monitor` logs `Robot to stop due to invalid source`, and `controller_server` repeats `Failed to make progress`. | `collision_monitor` stops every Nav2 command while `/scan` has had no data for 2 s or its transform fails | Get `/scan` publishing again. See the row above. |
| The robot stays stopped next to an obstacle under Nav2. Goals abort with `Failed to make progress`. | The robot is boxed in, so the stop polygon of every direction Nav2 tries holds points, or `/scan` has points of the robot or noise inside a stop polygon | Drive the robot out on `/cmd_vel_key` or `/cmd_vel_joy`. See [Drive with Nav2 and RViz](#drive-with-nav2-and-rviz) and [Troubleshooting](../troubleshooting.md#nav2-stuck-after-a-stop). |
| With `nav_mujoco_omnibot.launch.py`, Ctrl+C takes about 15 s and prints red `failed to terminate` and `process has died ... exit code -9` lines | `planner_server` and `controller_server`, sometimes `smoother_server`, do not exit on SIGINT or SIGTERM, so launch kills them with SIGKILL. Same with Fast DDS. | Harmless. Wait for the launch to exit. |
| `walkie_tf_server` prints a traceback with `RCLError` and `rcl_shutdown already called` on Ctrl+C | `tools/walkie_tf/walkie_tf/tf_server_node.py` calls `rclpy.shutdown()` after the context is already shut down | Harmless. The bug predates the MuJoCo launch files. |
| Nav2 hangs at `Waiting for service smoother_server/get_state...` | The composed Nav2 bringup can deadlock on Jazzy | Use `nav_mujoco_omnibot.launch.py`, which runs each server as its own process. With `nav2_bringup` directly, pass `use_composition:=False`. See [Navigation: Nav2 bringup hangs at smoother_server/get_state](navigation.md#nav2-composition-deadlock). |
| `controller_server` logs `Can not get 'plugin' param value for FollowPath.Omni` and `Failed to load motion model plugin 'Omni'`, and `lifecycle_manager_nav2` aborts the bringup | The file passed with `params_file:=` sets `motion_model: "Omni"` without `FollowPath.Omni.plugin`. navigation2 (upstream main) loads MPPI motion models as plugins. The `robot_navigation` parameter files set the key. | Add `Omni: plugin: "mppi::OmniMotionModel"` under `FollowPath` in your file. See [Navigation](navigation.md#mppi-omni-motion-model-plugin). |
| Nav2 repeats `The inflation radius (...) is smaller than the circumscribed radius (...)` errors | The local costmap `inflation_radius` in the file passed with `params_file:=` is smaller than the circumscribed radius of the footprint. Not a MuJoCo issue. | Set it to at least the circumscribed radius. `nav2_mppi_walkie_sim_params.yaml` uses 0.35 m (circumscribed radius 0.313 m). |
| A goal logs `Failed to make progress` once or twice, for example during the slow final approach, and is still reached | The progress checker wants 0.5 m of movement within 8 s | None needed. The behavior tree retries, and the goal is reached. |
| A goal aborts with error 103, `Resulting plan has 0 poses in it` | The robot is already at the goal by odometry | None needed. The ground truth pose can still be off by the odometry drift. |
| `bt_navigator` rejects goals and logs `"map" passed to lookupTransform argument target_frame does not exist` and `Initial robot pose is not available` (error 9002) | `bt_navigator` has no transform from `map` to `odom`, which the launch publishes only once. Seen once; the cause is not known. | Start a second publisher. See [Troubleshooting](../troubleshooting.md#nav2-map-frame-does-not-exist). |
| A goal ends `SUCCEEDED`, but its result has `error_code` 9002 | The error is left over from an earlier goal that `bt_navigator` rejected | Judge goals by their status. See [Troubleshooting](../troubleshooting.md#nav2-map-frame-does-not-exist). |
| RViz shows the robot at the goal, but `/mujoco_ground_truth/odom` is off, for example by 0.17 to 0.43 m after one or two goals | Odometry-only localization drifts | Expected. See [Drive with Nav2 and RViz](#drive-with-nav2-and-rviz). |
