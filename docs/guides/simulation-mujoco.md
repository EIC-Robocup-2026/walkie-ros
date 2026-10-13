# MuJoCo Simulation

This guide shows how to view and regenerate the MuJoCo model of Walkie, and how to build
your own world around it.

!!! warning "The ROS 2 runtime is not wired up yet"
    You can load, view, script and test the MuJoCo model today. No launch file starts it
    with ROS 2 yet, so MuJoCo publishes no topics and accepts no `/cmd_vel`. The
    `mujoco_ros2_control` package is not in the pixi environment either. Use Gazebo for
    anything that needs ROS 2. [2.5 ros2_control](../migrations/02-mujoco/05-ros2-control.md)
    lists what is still missing.

## Quick start { #what-works-now }

| Task | Command |
| --- | --- |
| Open the robot on a floor, in the `home` pose | `pixi run mjcf-view` |
| Open any world file, with a working **Reload** button | `pixi run python -m mujoco.viewer --mjcf=description/mujoco/scene.xml` |
| Regenerate the robot model after a description change | `pixi run mjcf` |
| Test the model and the default world | `pixi run python -m pytest description/test/test_mjcf.py` |

None of these needs a colcon build. The generator and `test_mjcf.py` read the xacro from
the source tree, not from `install/`.

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
four wheels, a position for the lift, head and arm joints. This is the way to drive it until the ROS 2 side
exists. Press ++f1++ for the full list of shortcuts.

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

3. Look at the result with `pixi run mjcf-view`, then run
   `pixi run python -m pytest description/test/test_mjcf.py`.
4. Commit the xacro change and the new `walkie.xml` together, inside the `description`
   submodule. Then commit the updated submodule pointer in walkie-ros.

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

The MuJoCo Python bindings are in the pixi environment. This script loads the example
world, starts from `home`, simulates 2 s and prints the base position, the cube position
and the front lidar's center beam:

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
| Launch file that runs MuJoCo with ROS 2 | Not written | [2.5 ros2_control](../migrations/02-mujoco/05-ros2-control.md) |
| `mujoco_ros2_control` in the pixi environment | The 0.1.2 build that the xacro targets needs the RoboStack `_25` build set. The lock file has `_21`. | [2.5 ros2_control](../migrations/02-mujoco/05-ros2-control.md#packaging-status) |
| `/cmd_vel`, `/joint_states` and the controllers in `mujoco_controllers.yaml` | Configured, but nothing loads them | [2.5 ros2_control](../migrations/02-mujoco/05-ros2-control.md) |
| `/scan` from the emulated lidars | The beams exist in the model; the ROS plugin is not set up | [2.4 Sensors](../migrations/02-mujoco/04-sensors.md) |
| Camera and IMU topics | The sensors exist in the model; nothing publishes them | [2.4 Sensors](../migrations/02-mujoco/04-sensors.md) |
| MoveIt with `hardware_type:=mujoco` | The launch option exists, but there is no MuJoCo controller manager to talk to | [2.5 ros2_control](../migrations/02-mujoco/05-ros2-control.md) |
| Nav2 in MuJoCo | Needs `/scan` and the drive controller first | [2. Add MuJoCo Simulation](../migrations/02-mujoco/index.md) |
| Robot base collisions with the world | Left out by design | [2.3 Collisions](../migrations/02-mujoco/03-collisions.md) |

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
