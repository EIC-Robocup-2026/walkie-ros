# 2.2 Omni Wheels

**Goal:** make Walkie's four omni wheels drive in MuJoCo the way the real wheels do: grip in
the rolling direction, slide freely along the axle, in every direction the robot faces.

This was the hardest part of the migration. The working solution depends on a detail of
MuJoCo's contact code, so this entry records the reasoning as well as the settings.

## In short

- Each wheel is a thin capsule that touches a `type="plane"` floor through an explicit
  `<pair>`. Only capsule-on-plane contacts align MuJoCo's friction directions with the axle.
- The pair friction is `0.005 1`: low along the axle, high in the rolling direction. This is
  the reverse of Gazebo's order.
- The Gazebo wheel joint damping and friction are replaced. They cost about 22 % of the linear
  speed and 23 % of the turn rate.
- The global `contact="disable"` flag is never used. It also turns off the wheel pairs.
- Result: the base drives the same at every heading. A wheel radius mismatch leaves it about
  1.5 % slow (2.4 % when turning). `description/test/test_mjcf.py` checks the driving.

## Wheel layout

Four omni wheels sit on the corners of a square, 0.18518 m from the center. Every axle points
radially outward. In the `base_footprint` frame (x forward, y left):

```text
                 +x (forward)
                      ^
          fl  45°     |     fr  315°
                      |
  +y (left) <---------+
                      |
          bl  135°    |     br  225°
```

??? info "Wheel positions, controller indices and URDF source"
    | Wheel | Joint | Position in `base_footprint` (m) | Angle from +x | Index in `wheel_names` |
    | --- | --- | --- | --- | --- |
    | Front left | `fl_wheel_joint` | (+0.131, +0.131) | 45° | 0 |
    | Back left | `bl_wheel_joint` | (-0.131, +0.131) | 135° | 1 |
    | Back right | `br_wheel_joint` | (-0.131, -0.131) | 225° | 2 |
    | Front right | `fr_wheel_joint` | (+0.131, -0.131) | 315° | 3 |

    The URDF places the wheels in the `base_link` frame, which is rotated 90 degrees (yaw)
    from `base_footprint`. The wheels are placed in `description/robots/gz_walkie.urdf.xacro`
    (positions ±0.130943 m in `base_link`, height 0.05 m, joint axis `0 1 0` in each wheel
    frame) with the `wheel_v0` macro from
    `description/urdf/walkie/omni_wheel/gz_omni_wheel.urdf.xacro` (radius 0.075 m, thickness
    0.05 m).

    !!! note
        Since description `main` was merged (description `f6649ee`), `base_link` is aligned
        with `base_footprint`, so the wheel origins in `base_link` match the table above. The
        wheel positions in the world are unchanged. See
        [Robot Description: Frames and joints](../../reference/robot-description.md#frames-and-joints).

## Why the friction must follow the axle

An omni wheel grips in its rolling direction and slides almost freely along its axle, because
the rollers on its rim spin. Both directions are fixed to the wheel, not to the world. When the
robot turns 90 degrees, every axle turns with it, and the low-friction direction must turn too.

A simulator that ties the friction directions to world axes is correct at one heading only.
After a turn, the wheels grip where they should slide and slide where they should grip.

??? info "How a real omni wheel works"
    An omni wheel has small rollers around its rim. The roller axes are at right angles to the
    wheel axle. When the motor turns the wheel, the rim pushes against the floor like a normal
    wheel. When something pushes the wheel sideways, along its axle, the rollers turn and the
    wheel slides with almost no resistance.

    | Direction | Behavior | Friction |
    | --- | --- | --- |
    | Rolling direction (perpendicular to the axle) | Grips, transmits motor torque | High |
    | Along the axle | Rollers spin freely | Almost zero |

    Four such wheels at 45, 135, 225 and 315 degrees let the base move in any direction and
    turn at the same time. Each wheel adds its push in its own rolling direction and slides
    along its axle when the other wheels push the base that way.

## The capsule-on-plane trick

Each wheel collision geom is a capsule, and the floor is a `type="plane"` geom. This is the
only combination where MuJoCo aligns the contact frame with the wheel axle.

In an explicit contact `<pair>`, the first friction value acts along tangent 1 of the contact
frame and the second along tangent 2. Axle-aligned friction needs tangent 1 along the axle. The
plane-capsule collider sets tangent 1 to the capsule axis, which for a wheel is the axle. The
other colliders that were checked fall back to the world Y axis.

!!! warning "Floors must be planes"
    The floor of any MuJoCo world for Walkie must be a `type="plane"` geom. A floor made of a
    box or a mesh breaks the wheel friction, even if it looks flat. The header of
    `description/mujoco/scene.xml` states this rule.

??? info "MuJoCo source reading and the experiment that confirmed it"
    Every MuJoCo contact has its own frame: a normal and two tangent directions. Reading the
    MuJoCo source (version 3.15.0) shows which collision functions set tangent 1:

    - `mjc_PlaneCapsule` in `src/engine/engine_collision_primitive.c` aligns the contact frame
      with the capsule axis ("align contact frames with capsule axis"). For a wheel capsule,
      the capsule axis is the axle.
    - The other colliders checked (plane-cylinder, plane-sphere, box, convex and mesh
      collisions, SDF) leave tangent 1 empty. `mju_makeFrame` in
      `src/engine/engine_util_spatial.c` then fills it with the world Y axis projected onto the
      contact plane.

    So the friction follows the axle only when the wheel geom is a capsule and the floor is a
    plane. A cylinder wheel, or a capsule on a box or mesh floor, ties the friction to world Y.

    An experiment with Walkie's model during the migration confirmed this. For the capsule on
    the plane, tangent 1 lay along the axle at every heading. For a cylinder on the plane, and
    for a capsule on a box floor, it was always world Y.

## Geometry and contact parameters

The generator turns each wheel's URDF collision cylinder into a capsule with `contype` and
`conaffinity` 0, so it touches nothing through the normal collision filter. The capsules touch
the floor only through explicit pairs in the world file:

```xml
<!-- friction: along the axle, along the rolling direction, torsional, rolling (x2) -->
<pair geom1="fl_wheel_contact" geom2="floor" condim="3" friction="0.005 1 0.005 0.0001 0.0001"/>
```

!!! warning "The friction order is the reverse of Gazebo's"
    In MuJoCo the low value comes first because tangent 1 is the axle. The Gazebo wheel
    (`description/urdf/walkie/omni_wheel/omni_wheel.gazebo.xacro`) sets `mu1` 1.0 along
    `fdir1` (the rolling direction) and `mu2` 0.001. Copying Gazebo's order into the pair makes
    the wheels slide in the rolling direction and grip along the axle.

??? info "Generated wheel capsule and its shape"
    ```xml
    <geom name="fl_wheel_contact" size="0.075 0.005" quat="0.707107 0.707107 0 0"
          type="capsule" contype="0" conaffinity="0" group="3"/>
    ```

    | Property | Value | Source |
    | --- | --- | --- |
    | Name | `<wheel>_wheel_contact` | `wheel_contact_geom()` in `build_mjcf.py` |
    | Radius | 0.075 m | URDF collision cylinder |
    | Half-length | 0.005 m | `WHEEL_CAPSULE_HALF_LENGTH` |
    | `contype`, `conaffinity` | 0, 0 | The capsule touches nothing through the normal collision filter |
    | Group | 3 | Collision group, hidden by default |

    The capsule is a sphere of radius 0.075 m stretched by 1 cm along the axle. Against a
    plane, only its bottom line touches, so it acts like a thin disc. MuJoCo creates two
    contacts per wheel, one at each end of that line.

??? info "Pair attributes and solver settings"
    | Attribute | Value | Meaning |
    | --- | --- | --- |
    | `condim` | 3 | Normal force plus sliding friction in two tangent directions. Torsional and rolling friction are not used, so the last three friction values have no effect. |
    | `friction[0]` | 0.005 | Along the axle (tangent 1): the rollers slide |
    | `friction[1]` | 1 | Along the rolling direction (tangent 2): the wheel grips |
    | `solref`, `solimp` | Not set | MuJoCo defaults: `solref` 0.02 1, `solimp` 0.9 0.95 0.001 0.5 2 |
    | Friction cone | `elliptic` | Set in `<option>` by the generator |

## Wheel joint damping and friction

The generator overrides the wheel joint dynamics that the URDF took over from the Gazebo setup.
With the Gazebo values, the robot lost about 22 % of its commanded linear speed.

| Setting | URDF value | MuJoCo value | Constant |
| --- | --- | --- | --- |
| `damping` | 5.0 | 0.001 | `WHEEL_JOINT_DAMPING` |
| `frictionloss` | 0.5 | 0 | set directly |
| `armature` | none | 0.01 | `WHEEL_JOINT_ARMATURE` |

??? info "Measured speed loss with the URDF values"
    The URDF wheel joint has `<dynamics damping="5.0" friction="0.5"/>`. MuJoCo's URDF importer
    copies these into the joint as `damping` and `frictionloss`. With them, a forward command
    of 0.2 m/s gave only about 0.156 m/s, and a rotation of 0.5 rad/s gave about 0.383 rad/s.
    The robot lost about 22 % of its linear speed and 23 % of its turn rate.

## Velocity actuators

Each wheel joint has a velocity actuator with the same name as the joint. The torque is
`kv × (command - wheel speed)`, with `kv` = 20 N m per rad/s (`WHEEL_KV`), limited to ±100 N m.
The actuator accepts ±100 rad/s, but ros2_control declares only ±3.14 rad/s, see
[Known issues](#known-issues).

??? info "Generated actuator XML and where the limits come from"
    This is how `MjSpec` writes a velocity actuator:

    ```xml
    <general name="fl_wheel_joint" joint="fl_wheel_joint" ctrlrange="-100 100" ctrllimited="true"
             forcelimited="true" forcerange="-100 100" biastype="affine" gainprm="20" biasprm="0 0 -20"/>
    ```

    | Limit | Value | Source |
    | --- | --- | --- |
    | Actuator control range | ±100 rad/s | URDF `<limit velocity="100"/>` |
    | Actuator torque range | ±100 N m | URDF `<limit effort="100"/>` |
    | ros2_control command range | ±3.14 rad/s | `mujoco_wheel_joint` entries in `description/urdf/walkie/ros2_control.xacro` |

## Mapping to omni_wheel_drive_controller

The base is driven by `omni_wheel_drive_controller`, configured in
`description/config/ros2_controller/mujoco_controllers.yaml`. `wheel_offset` (π/4) puts the
first wheel (`fl`) at 45° from +x, and the other `wheel_names` follow counterclockwise, 90° apart.
`robot_radius` (0.185 m) is the distance from the center to each wheel. `wheel_radius` is
0.076 m, but the capsule radius is 0.075 m, see [Known issues](#known-issues).

??? info "Wheel parameters in the MuJoCo controller YAML"
    ```yaml
    omni_wheel_drive_controller:
      ros__parameters:
        wheel_offset: 0.7853981633974485
        wheel_names:
          ["fl_wheel_joint", "bl_wheel_joint", "br_wheel_joint", "fr_wheel_joint"]
        robot_radius: 0.185
        wheel_radius: 0.076
    ```

For a body velocity command (vx, vy, wz), the wheel at angle θ gets the speed:

```text
wheel speed = (sin θ · vx - cos θ · vy - wz · robot_radius) / wheel_radius
```

This assumes the joint axis of each wheel points outward, as in the [layout](#wheel-layout).
A wheel mounted the other way round would need the opposite sign, and the robot would drive
wrong.

## How driving accuracy is tested

`description/test/test_mjcf.py` checks the wheels directly in MuJoCo, without ROS:

- `test_floor_and_wheel_pairs`: the floor is a plane, and each wheel is a capsule with a floor
  pair and low axle friction.
- `test_wheel_layout_matches_controller`: the wheel angles, distances and axles match the
  controller parameters.
- `test_drive_follows_omni_kinematics`: wheel speeds from the formula above give the commanded
  body velocity.

`description/test/test_description.py` also checks every controller file against the URDF.

??? info "Exact checks, drive commands and tolerances"
    | Test | What it checks |
    | --- | --- |
    | `test_floor_and_wheel_pairs` | The floor is a plane. Each wheel capsule is a capsule with `contype` and `conaffinity` 0. Each wheel has a pair with the floor, `condim` 3, and axle friction below 10 % of the rolling friction. |
    | `test_wheel_layout_matches_controller` | Each wheel sits at `wheel_offset` + index × 90°, within 0.005 m of `robot_radius`, with its axle pointing outward. |
    | `test_drive_follows_omni_kinematics` | Sets the wheel speeds with the formula above, lets the robot settle for 1.5 s, then compares the average body velocity over 0.5 s with the command. |

    The drive test uses four commands:

    | Name | vx (m/s) | vy (m/s) | wz (rad/s) |
    | --- | --- | --- | --- |
    | `forward` | 0.3 | 0 | 0 |
    | `sideways` | 0 | 0.3 | 0 |
    | `rotation` | 0 | 0 | 0.5 |
    | `combined` | 0.2 | -0.15 | 0.4 |

    The tolerance is 3 % + 0.002 m/s for the linear velocity and 4 % + 0.005 rad/s for the
    angular velocity. That is tight enough to catch a few percent of error in `wheel_radius` or
    `robot_radius`, and loose enough for the current wheel radius mismatch (see
    [Known issues](#known-issues)).

    `test_description.py` checks `robot_radius` within 0.001 m, `wheel_radius` within 0.002 m,
    and the wheel angles within 1 degree.

## Alternatives that failed

A cylinder wheel (the Gazebo wheel shape) and a capsule on a box floor both tie the friction to
world Y instead of each axle. The robot reached about half the commanded forward speed, and its
behavior changed with its heading. The capsule on a plane gives the same result at every
heading.

??? info "Measured results for each wheel and floor combination"
    These were measured during the migration with the same method as the drive test: wheel
    speeds from the formula above, 2 s to settle, then the average body velocity over 0.5 s.
    The commands were 0.2 m/s forward, 0.2 m/s sideways (to the left) and 0.5 rad/s rotation.
    "Turned 90°" means the robot started at a yaw of 90 degrees and got the same command in its
    own frame.

    | Wheel and floor | Forward, yaw 0 | Forward, turned 90° | Sideways, yaw 0 | Rotation |
    | --- | --- | --- | --- | --- |
    | Capsule on a plane (the current model) | 0.197 m/s | 0.197 m/s | 0.197 m/s | 0.489 rad/s |
    | Cylinder on a plane (the Gazebo wheel shape) | 0.099 m/s | about 0 m/s | about 0.02 m/s, spinning at 0.28 rad/s | 0.383 rad/s |
    | Capsule on a box floor | 0.099 m/s | about 0 m/s, spinning at 0.07 rad/s | about 0.04 m/s, spinning at 0.21 rad/s | 0.489 rad/s |

    The cylinder and the box floor fail in the same way. The wheels get a mix of grip and
    slide. After a 90 degree turn, a forward command barely moves the robot, while a sideways
    command now gives about half speed.

    For the capsule on a plane, the remaining 1.5 % comes from the wheel radius mismatch.

!!! danger "Do not use the global contact flag"
    Turning off all collisions with `<option><flag contact="disable"/></option>` also turns off
    the explicit wheel pairs: MuJoCo's collision code returns on this flag before it processes
    pairs. In a test the robot fell 4.9 m in one second, which is free fall. Switch collisions
    off per geom instead, see [2.3 Collisions](03-collisions.md).

## Known issues

| Issue | Effect | Workaround |
| --- | --- | --- |
| Wheel radius mismatch | Controller files use `wheel_radius: 0.076`, but the wheel capsules (the URDF wheel cylinder) are 0.075 m. The robot drives about 1.5 % slower and turns about 2.4 % slower than commanded. | None. The drive test tolerances allow for it. |
| Wheel speed limits disagree | The MuJoCo ros2_control block declares ±3.14 rad/s (about 0.24 m/s at the rim with `wheel_radius` 0.076 m). The MJCF actuators accept ±100 rad/s from the URDF placeholder `velocity="100"`. | Replace the URDF placeholder with the real motor limit, then run `pixi run mjcf`. |
