# 2.3 Collisions

**Goal:** keep contact work low while the robot drives, let the arms collide with the body and
with objects during manipulation, and have both at the same time without switching modes.

The answer is a fixed set of collision classes, written into the model by the generator.

## In short

- **What changed:** `description/scripts/build_mjcf.py` defines seven fixed collision classes and
  writes one into every robot collision geom. The wheel capsules have no class and touch the
  floor through explicit pairs. Floor, furniture and object geoms get their class in the world
  file.
- **Key decisions:** contacts are off per geom, never through the global contact flag, because
  that flag also turns off the wheel pairs. The base and body never touch the floor, objects or
  furniture. The arms and grippers touch the base, the body, each other, objects and furniture.
- **Result:** driving and manipulation work at the same time, with no runtime switch. The lidar
  rays, not collisions, turned out to be the largest simulation cost. MoveIt does not see these
  settings.

## The requirements

Three of the requirements in [Why MuJoCo](index.md#why-mujoco) are about collisions:

1. **All collisions off by default**, to reduce the simulation workload.
2. **Collisions for the arms and the robot body during manipulation.**
3. **Ideally both at the same time.**

"Off" cannot mean everything. The wheels must always touch the floor, or the robot cannot
drive. So the design turns contacts off per geometry, and keeps the wheel contacts as explicit
pairs that are always active.

## How MuJoCo decides which geoms collide

Two geoms A and B may collide when `A.contype & B.conaffinity` or `B.contype & A.conaffinity`
is not zero. A geom with both set to 0 never collides through this bitmask filter. An explicit
`<pair>` skips the bitmask and every other filter except the global contact flag. This is why
Walkie's wheel capsules have `contype="0" conaffinity="0"` and still touch the floor through
their pairs (see [2.2 Omni Wheels](02-omni-wheels.md)).

!!! warning "Do not use the global contact flag"
    `<flag contact="disable"/>` also turns off explicit pairs. The wheels lose the floor and the
    robot falls through it. Turn contacts off per geom instead.

??? info "All of MuJoCo's collision filters, and where the source shows it"
    `contype` and `conaffinity` are 32-bit masks. MuJoCo checks several filters before it
    computes a contact between two geoms:

    | Mechanism | Rule | Applies to explicit `<pair>` contacts |
    | --- | --- | --- |
    | Bitmask filter | Geoms A and B may collide if `A.contype & B.conaffinity` or `B.contype & A.conaffinity` is not zero | No |
    | Same body or rigidly attached bodies | Geoms in one weld group (bodies with no joint between them) never collide | No |
    | Parent and child | A body does not collide with its parent body (unless the parent is the world) | No |
    | `<exclude>` | Removes one body pair | No |
    | Global `<flag contact="disable"/>` | Turns off all contacts | **Yes, pairs are turned off too** |

    In the MuJoCo source (`src/engine/engine_collision_driver.c`), the bitmask filter runs only
    for candidates that do not come from a pair. The global flag makes `mj_collision` return
    before any pair is processed.

## The collision classes

`description/scripts/build_mjcf.py` defines seven classes. Every collision geom carries exactly
one class bit as `contype`, and the union of the classes it may touch as `conaffinity`.

| Class | Contents | Collides with |
| --- | --- | --- |
| FLOOR | The world plane named `floor` | OBJ |
| BASE | Mobile base and everything rigidly attached to it | ARM, GRIP |
| BODY | Lift, head and arm mounts | ARM, GRIP |
| ARM | Arm links 1 to 7 | every class except FLOOR |
| GRIP | Hands, fingers and wrist camera brackets | every class except FLOOR |
| OBJ | Movable world objects (graspable items) | FLOOR, ARM, GRIP, OBJ, FURN |
| FURN | Static world geometry other than the floor (furniture, walls) | ARM, GRIP, OBJ |
| none | Wheel capsules (`contype="0" conaffinity="0"`) | only the floor, through the four pairs in the world file |

The pairing is symmetric, so two geoms collide exactly when their classes are paired.

??? info "Collision matrix, class bits and mask values"
    | | FLOOR | BASE | BODY | ARM | GRIP | OBJ | FURN |
    | --- | --- | --- | --- | --- | --- | --- | --- |
    | **FLOOR** | - | - | - | - | - | yes | - |
    | **BASE** | - | - | - | yes | yes | - | - |
    | **BODY** | - | - | - | yes | yes | - | - |
    | **ARM** | - | yes | yes | yes | yes | yes | yes |
    | **GRIP** | - | yes | yes | yes | yes | yes | yes |
    | **OBJ** | yes | - | - | yes | yes | yes | yes |
    | **FURN** | - | - | - | yes | yes | yes | - |

    | Class | Bit | `contype` | `conaffinity` |
    | --- | --- | --- | --- |
    | FLOOR | 1 << 0 | 1 | 32 |
    | BASE | 1 << 1 | 2 | 24 |
    | BODY | 1 << 2 | 4 | 24 |
    | ARM | 1 << 3 | 8 | 126 |
    | GRIP | 1 << 4 | 16 | 126 |
    | OBJ | 1 << 5 | 32 | 121 |
    | FURN | 1 << 6 | 64 | 56 |

    If class X lists class Y, class Y also lists class X. So both halves of MuJoCo's OR rule
    give the same answer.

## Which robot geoms get which class

The generator assigns the class from the link name, using the full-match regular expressions in
`LINK_COLLISION_CLASSES`. If a link has collision geometry but matches no pattern, generation
fails, so a new link cannot silently end up without a class.

Visual geoms always have `contype="0" conaffinity="0"`. They never collide.

??? info "Links and geoms per class in the current model"
    | Class | Links | Geoms in the current model |
    | --- | --- | --- |
    | BASE | `base_link`, `bottom_link` | `base_link_collision_0` (cylinder), `base_link_collision_1` and `_2` (boxes), `bottom_link_collision` |
    | BODY | `lift_link`, `head_servo_link`, `zed_head_camera_center`, `openarm_left_link0`, `openarm_right_link0` | `top_bar`, `stem`, `head_servo_link_collision`, `zed_head_camera_center_collision`, `openarm_left_link0_collision`, `openarm_right_link0_collision` |
    | ARM | `openarm_left_link1` to `link7`, `openarm_right_link1` to `link7` | 14 geoms, `openarm_left_link1_collision` to `openarm_right_link7_collision` |
    | GRIP | `openarm_*_hand`, `openarm_*_left_finger`, `openarm_*_right_finger`, `openarm_*_wristcam_bracket` | 8 geoms: hand, both fingers and wrist camera bracket on each side |
    | none | wheel links | `fl_wheel_contact`, `fr_wheel_contact`, `bl_wheel_contact`, `br_wheel_contact` |

## How the design meets the requirements

- **Off by default:** BASE and BODY never touch the floor, furniture or objects. The floor
  touches only OBJ. A robot driving through an empty world has 8 contacts: two per wheel.
- **Arm and body collisions:** the arms cannot pass through the robot body, each other,
  objects or furniture.
- **Both at the same time:** the classes are fixed in the model. Nothing switches at runtime,
  so driving and manipulation work together.

??? info "Which contacts inside the robot are still filtered"
    Inside the robot, the usual MuJoCo filters still apply. A link never collides with its
    parent link, and four body pairs whose meshes overlap at the zero pose are excluded (see
    [2.1 Generate the Model](01-model-generation.md)). All other arm contacts with the body and with
    the other arm stay active.

!!! warning "The base drives through furniture and walls"
    BASE and BODY do not collide with FURN or OBJ, by design. Navigation relies on the lidars
    and the Nav2 costmaps to keep the robot away from obstacles, so the simulated base does not
    need contacts. In MuJoCo, a navigation bug that drives the robot into a wall does not show
    up as a crash: the robot passes through the wall. Check the costmaps and the planned path,
    not the physics, when you test navigation.

## Objects and furniture in a custom world

Give every geom you add to a world one of these settings:

| Class | Use for | Attributes | Collides with |
| --- | --- | --- | --- |
| FURN | Furniture, walls, shelves: anything that never moves | `contype="64" conaffinity="56"` | OBJ, ARM, GRIP |
| OBJ | Items the arms pick up or push | `contype="32" conaffinity="121"` | FLOOR, FURN, OBJ, ARM, GRIP |
| none | Decoration that nothing should touch | `contype="0" conaffinity="0"` | nothing |

- FURN does not collide with FLOOR or with other FURN geoms. A FURN body with a joint would fall
  through the floor, so furniture must be a static body (no joint).
- Anything that moves must be OBJ.

The FURN and OBJ rules are in the header comment of `description/mujoco/scene.xml`.
[MuJoCo Simulation: Build a custom world](../../guides/simulation-mujoco.md#build-a-custom-world)
has a complete world with a FURN table and an OBJ cube.

!!! warning "Do not leave out the attributes"
    A geom without `contype` and `conaffinity` gets MuJoCo's defaults,
    `contype="1" conaffinity="1"`. In Walkie's scheme that is the FLOOR bit. Such a geom
    collides with the floor and with OBJ, but the arms pass through it, because ARM and GRIP
    do not list FLOOR.

## Performance findings

Requirement 1 was mainly about speed. Measurements during the migration showed that collision
filtering saves little at rest. The lidar rays are the biggest cost.

- The generator sets each rangefinder's `interval` to the scan period (1/40 s), so the rays are
  cast once per scan instead of every 2 ms step.
- Detailed meshes in contact are expensive. Keep visual geoms at `contype="0" conaffinity="0"`,
  and keep collision meshes simple.

??? info "Measurement setup, results and analysis"
    The numbers come from `description/mujoco/scene.xml` with the robot at rest in the empty
    world, stepping physics only (no rendering), on an Intel Core i7-10750H. Each value is the
    best of three 4-second runs. Absolute numbers vary between runs and machines, so compare
    the rows with each other.

    | Configuration | Speed (simulated time / wall time) | Contacts |
    | --- | --- | --- |
    | Model as generated | about 18x | 8 |
    | Every collision geom (group 3, except the wheels) set to collide with each other and the floor | about 16x | 8 |
    | Visual geoms also set to collide in the same way | about 5x | 24 |
    | Lidar rangefinders removed | about 24x | 8 |
    | Rangefinders evaluated every physics step | about 5x | 8 |

    - Letting every collision geom collide changed the speed only slightly (about 18x to about
      16x).
    - Collision filtering saved little at rest because MuJoCo runs the detailed collision test
      only for geoms whose bounding boxes overlap. Geoms that are far apart cost little whether
      they are filtered or not.
    - When the visual meshes were allowed to collide, 16 extra contacts appeared and the speed
      dropped from about 16x to about 5x.
    - The model has 482 rangefinder rays (two fans of 241). Without the scan-period `interval`,
      the speed drops from about 18x to about 5x.
    - Camera rendering cost was not measured.

## What this means for MoveIt

MoveIt never sees MuJoCo's contact settings, so the collision classes do not make MoveIt
planning faster or slower. While it plans, MoveIt checks the `<collision>` shapes of the URDF,
minus the link pairs disabled in the SRDF, plus the octomap when one is configured.

With `hardware_type:=mujoco`, MoveIt does not see objects in the MuJoCo world unless they are
added to its planning scene. The reason: `moveit_demo.launch.py` and `moveit_only.launch.py`
remove the `sensors_3d` (octomap) configuration, as they do for every hardware type except
`gazebo` and `real_robot`.

The settings that do change MoveIt's planning cost (planning time and attempts, collision mesh
size, segment check density, disabled pairs, octomap) are in
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
| `link3_symp.stl` is not simplified | Every MoveIt collision check against arm link 3 pays for 72,160 triangles | Simplify the mesh and keep its file name |

The 72,160 triangles are the same count as the unsimplified `visual/link3.stl`. The URDF visual
uses `visual/link3.dae`.
