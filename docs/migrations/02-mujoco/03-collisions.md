# 2.3 Collisions

**Goal:** keep contact work low while the robot drives, let the arms collide with the body and
with objects during manipulation, and have both at the same time without switching modes.

The answer is a fixed set of collision classes, written into the model by the generator.
This page explains MuJoCo's filtering rules, the classes, and what they do and do not affect.

## The requirements

Three of the requirements in [Why MuJoCo](index.md#why-mujoco) are about collisions:

1. **All collisions off by default**, to reduce the simulation workload.
2. **Collisions for the arms and the robot body during manipulation.**
3. **Ideally both at the same time.**

"Off" cannot mean everything. The wheels must always touch the floor, or the robot cannot
drive. So the design turns contacts off per geometry, and keeps the wheel contacts as
explicit pairs that are always active.

## How MuJoCo decides which geoms collide

MuJoCo checks several filters before it computes a contact between two geoms:

| Mechanism | Rule | Applies to explicit `<pair>` contacts |
| --- | --- | --- |
| Bitmask filter | Geoms A and B may collide if `A.contype & B.conaffinity` or `B.contype & A.conaffinity` is not zero | No |
| Same body or rigidly attached bodies | Geoms in one weld group (bodies with no joint between them) never collide | No |
| Parent and child | A body does not collide with its parent body (unless the parent is the world) | No |
| `<exclude>` | Removes one body pair | No |
| Global `<flag contact="disable"/>` | Turns off all contacts | **Yes, pairs are turned off too** |

`contype` and `conaffinity` are 32-bit masks. A geom with both set to 0 never collides
through the bitmask filter.

An explicit `<pair>` skips all of the filters except the global flag. In the MuJoCo source
(`src/engine/engine_collision_driver.c`), the bitmask filter runs only for candidates that do
not come from a pair. The global flag makes `mj_collision` return before any pair is
processed. This is why Walkie's wheel capsules have `contype="0" conaffinity="0"` and still
touch the floor through their pairs, and why the global flag cannot be used: the robot falls
through the floor. See [Omni wheels](02-omni-wheels.md).

## The collision classes

`description/scripts/build_mjcf.py` defines seven classes. Every collision geom carries
exactly one class bit as `contype`, and the union of the classes it may touch as
`conaffinity`.

| Class | Bit | `contype` | `conaffinity` | Contents |
| --- | --- | --- | --- | --- |
| FLOOR | 1 << 0 | 1 | 32 | The world plane named `floor` |
| BASE | 1 << 1 | 2 | 24 | Mobile base and everything rigidly attached to it |
| BODY | 1 << 2 | 4 | 24 | Lift, head and arm mounts |
| ARM | 1 << 3 | 8 | 126 | Arm links 1 to 7 |
| GRIP | 1 << 4 | 16 | 126 | Hands, fingers and wrist camera brackets |
| OBJ | 1 << 5 | 32 | 121 | Movable world objects (graspable items) |
| FURN | 1 << 6 | 64 | 56 | Static world geometry other than the floor (furniture, walls) |

The matrix is symmetric: if class X lists class Y, class Y also lists class X. So both halves
of MuJoCo's OR rule give the same answer, and two geoms collide exactly when their classes
are paired.

### Collision matrix

| | FLOOR | BASE | BODY | ARM | GRIP | OBJ | FURN |
| --- | --- | --- | --- | --- | --- | --- | --- |
| **FLOOR** | - | - | - | - | - | yes | - |
| **BASE** | - | - | - | yes | yes | - | - |
| **BODY** | - | - | - | yes | yes | - | - |
| **ARM** | - | yes | yes | yes | yes | yes | yes |
| **GRIP** | - | yes | yes | yes | yes | yes | yes |
| **OBJ** | yes | - | - | yes | yes | yes | yes |
| **FURN** | - | - | - | yes | yes | yes | - |

The wheel capsules are in no class (`contype="0" conaffinity="0"`). They touch only the floor,
through the four pairs in the world file.

## Which robot geoms get which class

The generator assigns the class from the link name, using the full-match regular expressions
in `LINK_COLLISION_CLASSES`. If a link has collision geometry but matches no pattern,
generation fails, so a new link cannot silently end up without a class.

| Class | Links | Geoms in the current model |
| --- | --- | --- |
| BASE | `base_link`, `bottom_link` | `base_link_collision_0` (cylinder), `base_link_collision_1` and `_2` (boxes), `bottom_link_collision` |
| BODY | `lift_link`, `head_servo_link`, `zed_head_camera_center`, `openarm_left_link0`, `openarm_right_link0` | `top_bar`, `stem`, `head_servo_link_collision`, `zed_head_camera_center_collision`, `openarm_left_link0_collision`, `openarm_right_link0_collision` |
| ARM | `openarm_left_link1` to `link7`, `openarm_right_link1` to `link7` | 14 geoms, `openarm_left_link1_collision` to `openarm_right_link7_collision` |
| GRIP | `openarm_*_hand`, `openarm_*_left_finger`, `openarm_*_right_finger`, `openarm_*_wristcam_bracket` | 8 geoms: hand, both fingers and wrist camera bracket on each side |
| none | wheel links | `fl_wheel_contact`, `fr_wheel_contact`, `bl_wheel_contact`, `br_wheel_contact` |

Visual geoms always have `contype="0" conaffinity="0"`. They never collide.

## How the design meets the requirements

| Requirement | How it is met |
| --- | --- |
| Off by default | BASE and BODY never touch the floor, furniture or objects. The floor touches only OBJ. The wheels touch only the floor, through pairs. A robot driving through an empty world has 8 contacts: two per wheel. |
| Arm and body collisions | ARM and GRIP collide with BASE, BODY, ARM, GRIP, OBJ and FURN. The arms cannot pass through the robot body, each other, objects or furniture. |
| Both at the same time | The classes are fixed in the model. Nothing switches at runtime, so driving and manipulation work together. |

Inside the robot, the usual MuJoCo filters still apply. A link never collides with its parent
link, and four body pairs whose meshes overlap at the zero pose are excluded (see
[Model generation](01-model-generation.md)). All other arm contacts with the body and with
the other arm stay active.

!!! warning "The base drives through furniture and walls"
    BASE and BODY do not collide with FURN or OBJ, by design. Navigation relies on the
    lidars and the Nav2 costmaps to keep the robot away from obstacles, so the simulated
    base does not need contacts. The consequence: in MuJoCo, a navigation bug that drives the
    robot into a wall does not show up as a crash. The robot passes through the wall. Check
    the costmaps and the planned path, not the physics, when you test navigation.

## Objects and furniture in a custom world

Every geom you add to a world needs one of these settings. The FURN and OBJ rules are in the
header comment of `description/mujoco/scene.xml`:

| Class | Use for | Attributes | Collides with |
| --- | --- | --- | --- |
| FURN | Furniture, walls, shelves: anything that never moves | `contype="64" conaffinity="56"` | OBJ, ARM, GRIP |
| OBJ | Items the arms pick up or push | `contype="32" conaffinity="121"` | FLOOR, FURN, OBJ, ARM, GRIP |
| none | Decoration that nothing should touch | `contype="0" conaffinity="0"` | nothing |

FURN does not collide with FLOOR or with other FURN geoms. A FURN body with a joint would fall
through the floor, so furniture must be a static body (no joint). Anything that moves must be
OBJ. [MuJoCo simulation: Build a custom world](../../guides/simulation-mujoco.md#build-a-custom-world)
has a complete world with a FURN table and an OBJ cube.

!!! warning "Do not leave out the attributes"
    A geom without `contype` and `conaffinity` gets MuJoCo's defaults,
    `contype="1" conaffinity="1"`. In Walkie's scheme that is the FLOOR bit. Such a geom
    collides with the floor and with OBJ, but the arms pass through it, because ARM and GRIP
    do not list FLOOR.

## Performance findings

The main reason for requirement 1 was speed. Measurements during the migration showed that
collision filtering is not the biggest cost. The lidar rays are.

The numbers below come from `description/mujoco/scene.xml` with the robot at rest in the
empty world, stepping physics only (no rendering), on an Intel Core i7-10750H. Each value is
the best of three 4-second runs. Absolute numbers vary between runs and machines, so compare
the rows with each other.

| Configuration | Speed (simulated time / wall time) | Contacts |
| --- | --- | --- |
| Model as generated | about 18x | 8 |
| Every collision geom (group 3, except the wheels) set to collide with each other and the floor | about 16x | 8 |
| Visual geoms also set to collide in the same way | about 5x | 24 |
| Lidar rangefinders removed | about 24x | 8 |
| Rangefinders evaluated every physics step | about 5x | 8 |

What this means:

- **Collision filtering saved little at rest.** MuJoCo runs the detailed collision test only
  for geoms whose bounding boxes overlap. Geoms that are far apart cost little whether they
  are filtered or not.
- **Detailed meshes in contact are expensive.** When the visual meshes were allowed to collide,
  16 extra contacts appeared and the speed dropped from about 16x to about 5x. Keep visual
  geoms at `contype="0" conaffinity="0"`, and keep collision meshes simple.
- **The lidars are the largest cost.** The model has 482 rangefinder rays (two fans of 241).
  The generator sets each rangefinder's `interval` to the scan period (1/40 s), so the rays
  are cast once per scan instead of every 2 ms step. Without that, the speed drops from about
  18x to about 5x.

Camera rendering cost was not measured.

## What this means for MoveIt

MoveIt never sees MuJoCo's contact settings. It runs its own collision checks while it plans,
using:

- the `<collision>` shapes of the URDF,
- minus the link pairs disabled in the SRDF,
- plus the octomap, when one is configured.

So the MuJoCo collision classes do not make MoveIt planning faster or slower. With
`hardware_type:=mujoco`, `moveit_demo.launch.py` and `moveit_only.launch.py` also remove the
`sensors_3d` (octomap) configuration, as they do for every hardware type except `gazebo` and
`real_robot`. MoveIt then does not see objects in the MuJoCo world unless they are added to
its planning scene.

The settings that do change MoveIt's planning cost (planning time and attempts, collision
mesh size, segment check density, disabled pairs, octomap) are listed in
[Manipulation: Why MuJoCo collision settings do not affect MoveIt](../../guides/manipulation.md#why-mujoco-collision-settings-do-not-affect-moveit).

!!! danger "Do not strip collision elements to lighten the simulation"
    The URDF `<collision>` elements are shared. MoveIt plans with them, the lidar
    `robot_self_filter` (`hardware/lidar/robot_self_filter`) uses them to remove the robot from
    sensor data, and the MJCF generator turns them into MuJoCo collision geoms and visual
    fallbacks. Removing them breaks all three. To make the simulation cheaper, change the
    collision class or simplify the mesh instead.

## Known issues

| Issue | Effect | Workaround |
| --- | --- | --- |
| `link3_symp.stl` is not simplified | The arm link 3 collision mesh has 72,160 triangles, the same count as the unsimplified `visual/link3.stl`. (The URDF visual uses `visual/link3.dae`.) Every MoveIt collision check against link 3 pays for this detail. | Simplify the mesh and keep its file name. |
