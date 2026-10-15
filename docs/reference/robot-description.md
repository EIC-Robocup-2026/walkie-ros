# Robot Description

Look up how the Walkie model is built: its files, xacro arguments,
ros2_control modes, controllers, frames, and joints. Secondary tables and
background are in the collapsed blocks.

The model is the `walkie_description` package in `description/`. This folder
is a git submodule, so commit changes there first (see
[Project Structure](project-structure.md#git-submodules)).

## Files

| Path (in `description/`) | Contents |
| --- | --- |
| `robots/gz_walkie.urdf.xacro` | The only entry point for the full robot. Robot name `walkie_bot`. |
| `urdf/walkie/` | Xacro macros for the base, omni wheels and their rollers, lift, head servo, lidars, IMU, cameras, and `ros2_control.xacro` |
| `urdf/openarm/` | Xacro macros for the two OpenArm v10 arms and hands, and their ros2_control blocks |
| `config/ros2_controller/` | Controller manager parameter files, one per ros2_control mode |
| `config/openarm/` | Arm, arm body, and hand parameters (`arm/`, `body/`, `hand/`): joint limits, gains, inertials, kinematics. Loaded with `xacro.load_yaml` by `gz_walkie.urdf.xacro` and `urdf/openarm/robot/openarm_robot.xacro`. |
| `config/hardware_layout.yaml` | Per-joint hardware list. Not used. See [Known issues](#known-issues). |
| `meshes/` | Visual and collision meshes. The `.dae` files are in Git LFS. |
| `launch/` | `robot_state_publisher.launch.py`, two RViz preview launches, and `rviz.launch.py` |
| `rviz/robot_preview.rviz` | RViz config for the previews |
| `mujoco/` | `walkie.xml` (generated) and `scene.xml` (hand-written world) |
| `scripts/build_mjcf.py` | Generates `mujoco/walkie.xml` from the xacro |
| `scripts/create_gz_walkie_urdf.sh` | Writes expanded URDF files to `robots/`. See [Export a plain URDF](#export-a-plain-urdf). |
| `readme.md` | Why the mirrored OpenArm links use pre-mirrored meshes instead of a negative scale |
| `test/` | Model tests. See [Tests](../guides/testing.md). |

## Entry point and xacro arguments

The Gazebo, MuJoCo, real robot, and Isaac launch files expand
`robots/gz_walkie.urdf.xacro` directly. To expand the model by hand and check
it:

```bash
pixi run xacro description/robots/gz_walkie.urdf.xacro ros2_control:=mock > /tmp/walkie.urdf
pixi run check_urdf /tmp/walkie.urdf
```

| Argument | Default | Effect |
| --- | --- | --- |
| `ros2_control` | `gazebo` | Which ros2_control hardware to declare. See [ros2_control modes](#ros2_control-modes). |
| `dual_lidar` | `True` | Front and back lidars. `False` gives one 360 degree `single_lidar_link`, only in `gazebo` mode. Every other mode always has two lidars. |
| `roller` | `false` | `true` replaces the collision cylinder of each omni wheel with 28 free-spinning roller links. Not supported in `mujoco` mode. |
| `lift_at_top` | `false` | `true` mounts `lift_joint` at the top of its travel and changes its range to -0.7435 to 0 m, so 0 is the top. Not supported in `mujoco` mode. See [Known issues](#known-issues). |
| `use_arm` | `True` | Declared, but has no effect. The arms are always included. |
| `use_fake_arm_hardware` | `True` | Default for the two arguments below. `robot_state_publisher.launch.py` uses another default, see [Known issues](#known-issues). |
| `left_use_fake_arm_hardware` | value of `use_fake_arm_hardware` | `real_robot` mode only: `True` uses `mock_components/GenericSystem` for the left arm, `False` uses the OpenArm CAN plugin |
| `right_use_fake_arm_hardware` | value of `use_fake_arm_hardware` | Same for the right arm |
| `right_joint2_fixed` | `False` | Makes `openarm_right_joint2` a fixed joint and removes it from ros2_control |
| `sim_controllers_yaml` | `$(find walkie_description)/config/ros2_controller/sim_controllers.yaml` | `gazebo` mode only: controller file read by the Gazebo ros2_control plugin |
| `left_can_interface` | `can1` | CAN interface of the left arm (`real_robot` with real arm hardware) |
| `right_can_interface` | `can0` | CAN interface of the right arm |
| `mujoco_model` | `$(find walkie_description)/mujoco/scene.xml` | `mujoco` mode only: scene loaded by the MuJoCo hardware plugin |
| `mujoco_headless` | `false` | `mujoco` mode only: run without the MuJoCo window |

??? info "Arguments declared by included files"
    `urdf/walkie/head/servo_camera.urdf.xacro` declares a few more arguments.
    The useful ones:

    | Argument | Default |
    | --- | --- |
    | `camera_name` | `zed_head` (from `zed_${prefix}`) |
    | `camera_model` | `zed2i` |
    | `use_zed_localization` | `false` |

    `urdf/walkie/omni_wheel/gz_omni_wheel.urdf.xacro` declares `roller`
    (default `false`) a second time, and `standalone` (default `false`),
    `prefix`, and `parent`. The last three only matter when the file is
    expanded alone to test one wheel.

??? info "What `roller` and `lift_at_top` change"
    - `roller:=true`: each wheel gets 28 links `roller_<wheel>_<n>_link` (for
      example `roller_fl_1_link`) on continuous joints
      `roller_<wheel>_<n>_joint`. Each roller collides as four spheres of
      about 0.009 m radius. The wheel link loses its collision cylinder, so the
      model has 168 links instead of 56. The anisotropic wheel friction of
      `wheel_gazebo_v0` (`mu1` 1.0, `mu2` 0.001) then has no collision to act
      on. `urdf/walkie/roller/roller.gazebo.xacro` defines
      `roller_gazebo_v0`, but nothing calls it, so the rollers use the Gazebo
      default surface parameters.
    - `lift_at_top:=true`: the `lift_joint` origin moves from z = 0.3985 m to
      z = 1.1420 m in `base_link` (0.3985 + 0.7435), and the URDF range
      becomes -0.7435 to 0 m. The `mock`, `gazebo`, `topic_base`, and
      `real_robot` ros2_control blocks use the same range. `mock` and
      `real_robot` start at 0 (the top). `gazebo` still starts at 0.51.
    - `mujoco` mode: `build_mjcf.py` expands the xacro with only
      `ros2_control:=mujoco`, so `walkie.xml` always has the plain wheels and
      the lift mounted at the bottom. The `mujoco` ros2_control block keeps
      fixed lift limits of 0 to 0.743 m and an initial position of 0.51 m.
      `mujoco_omnibot.launch.py` passes neither argument.
    - No launch file in `robot_bringup` sets either argument.
      `robot_state_publisher.launch.py` forwards both to xacro.

??? info "Why `sim_controllers_yaml` exists"
    Gazebo reads the controller file from a path inside the URDF, not from a
    launch argument. For example, with `hardware_type:=gazebo` and
    `right_joint2_fixed:=true`, `moveit_demo.launch.py` writes a copy of
    `sim_controllers.yaml` without `openarm_right_joint2` and passes its path
    through this argument.

??? info "Other top-level xacro files"
    - `manipulate/openarm_bimanual_moveit_config/config/openarm_bimanual.urdf.xacro`
      (in the MoveIt package) includes `gz_walkie.urdf.xacro`. It only changes
      the default of `ros2_control` to `mock` for MoveIt.
    - `urdf/openarm/robot/v10.urdf.xacro` describes the OpenArm alone. It is
      not part of the Walkie model.

### Export a plain URDF

`scripts/create_gz_walkie_urdf.sh` expands the xacro with the default
`ros2_control:=gazebo` and writes URDF files to `description/robots/`, for
tools that cannot run xacro. It needs a build, because the xacro files use
`$(find walkie_description)`.

```bash
pixi run description/scripts/create_gz_walkie_urdf.sh [all|simple|roller] [true|false]
```

The first argument picks the variants (default `all`). The second one is
`lift_at_top` (default `false`).

| Variant | `roller` | Files |
| --- | --- | --- |
| `simple` | `false` | `gz_walkie_simple.urdf`, `gz_walkie_simple_absolute_path.urdf` |
| `roller` | `true` | `gz_walkie_roller.urdf`, `gz_walkie_roller_absolute_path.urdf` |

The `*_absolute_path.urdf` files replace `package://walkie_description` with a
`file://` path to the `description/` source folder. `description/.gitignore`
ignores `*.urdf`, so the files are not committed.

## ros2_control modes

`urdf/walkie/ros2_control.xacro` declares different hardware for each value of
`ros2_control`. A value that is not in this table gives no error: the model
then has no ros2_control block at all.

| Mode | Used for | ros2_control block | Plugin |
| --- | --- | --- | --- |
| `mock` | Tests without hardware or simulator | `MockSystem` | `mock_components/GenericSystem` |
| `gazebo` | Gazebo simulation | `GazeboSimSystem` | `gz_ros2_control/GazeboSimSystem`, loaded by the `<gazebo>` plugin `gz_ros2_control::GazeboSimROS2ControlPlugin` (`libgz_ros2_control-system.so`) |
| `mujoco` | MuJoCo simulation (in progress) | `MujocoSystem` | `mujoco_ros2_control/MujocoSystemInterface` |
| `topic_base` | Isaac Sim | `TopicBaseSystem` | `topic_based_ros2_control/TopicBasedSystem`, commands on `/isaac_joint_commands`, states from `/isaac_joint_states` |
| `real_robot` | The real robot | `CubeMarsSystem` (wheels) | `cubemars_hardware/CubeMarsSystemHardware` on `can2` |
| | | `HeadServoSystem` (head) | `dynamixel_hardware_interface/DynamixelHardware` on `/dev/ttyDynamixel`, 57600 baud, servo ID 2 |
| | | `LiftSystem` (lift) | `mock_components/GenericSystem`. The real lift motor is controlled outside ros2_control by `lift_homing_node.py` in `robot_bringup`. |
| | | `openarm_left_hardware_interface`, `openarm_right_hardware_interface` | `openarm_hardware/OpenArm_v10HW`, or `mock_components/GenericSystem` when the fake arm argument is true |

- In the four non-real modes, one block holds all joints: wheels, lift, head,
  both arms, and both grippers.
- In `real_robot` mode the arms get their own blocks from
  `urdf/openarm/ros2_control/openarm.bimanual.ros2_control.xacro`.
- Wheel CAN IDs in `real_robot` mode: `fl_wheel_joint` 1, `br_wheel_joint` 2,
  `fr_wheel_joint` 3, `bl_wheel_joint` 4.

`bringup/launch/mujoco_omnibot.launch.py` in `robot_bringup` starts the
`mujoco` mode. See
[MuJoCo Simulation: Run with ROS 2](../guides/simulation-mujoco.md#run-with-ros-2).

??? info "Joint limits that differ between modes"
    | Mode | Wheel velocity limit (rad/s) | `lift_joint` command range (m) | `lift_joint` initial (m) | `head_servo_joint` command range (rad) |
    | --- | --- | --- | --- | --- |
    | `mock` | ±3.14 | 0.000 to 0.7435 | 0.7435 | ±1.54 |
    | `gazebo` | ±3.14 | 0.000 to 0.7435 | 0.51 | ±1.54 |
    | `mujoco` | ±3.14 | 0.000 to 0.743 | 0.51 | ±1.54 |
    | `topic_base` | ±16 | 0.000 to 0.7435 | not set | ±1.54 |
    | `real_robot` | ±16 | 0.000 to 0.7435 | 0.7435 | ±0.7854 |

    The lift values are for the default `lift_at_top:=false`. With
    `lift_at_top:=true`, every mode except `mujoco` uses -0.7435 to 0 m, and
    `mock` and `real_robot` start at 0. `gazebo` keeps 0.51 and `mujoco`
    keeps its values.

    The URDF joint limits are `lift_joint` 0 to 0.7435 m (-0.7435 to 0 m with
    `lift_at_top:=true`) and `head_servo_joint` -0.785 to 1.047 rad. Every mode
    except `real_robot` allows a wider `head_servo_joint` range than the URDF.

??? info "`mujoco` mode interfaces and IMU"
    In `mujoco` mode the wheels have only a velocity command interface and
    every other joint has only a position command interface. The block also
    declares the IMU sensor `top_imu`. Its plugin parameters
    `odom_free_joint_name` (`floating_base_joint`) and `odom_topic`
    (`/mujoco_ground_truth/odom`) ask for ground-truth odometry of the base.

## Controller files

Each file in `config/ros2_controller/` is loaded together with one ros2_control
mode. `CONTROLLER_PAIRINGS` in `test/test_description.py` records this pairing
and checks every file against its mode.

| Mode | Controller file | Loaded by |
| --- | --- | --- |
| `gazebo` | `sim_controllers.yaml` | The Gazebo plugin, through the `sim_controllers_yaml` argument. Started from `simulation/gazebo/launch/gzsim_omnibot.launch.py`. |
| `mujoco` | `mujoco_controllers.yaml` | `ros2_control_node` of `mujoco_ros2_control`. Started from `bringup/launch/mujoco_omnibot.launch.py`. |
| `topic_base` | `isaac_controllers.yaml` | `bringup/launch/isaac_omnibot.launch.py` |
| `real_robot` | `real_controllers.yaml` | `bringup/launch/real_omnibot.launch.py` |
| `mock` | `real_controllers.yaml` | `bringup/launch/test_zed.launch.py` |

!!! note
    The `mock` + `real_controllers.yaml` pairing does not work. See [Known issues](#known-issues).

??? info "Controller manager settings per file"
    | File | `update_rate` (Hz) | `use_sim_time` |
    | --- | --- | --- |
    | `sim_controllers.yaml` | 60 | true |
    | `mujoco_controllers.yaml` | 60 | true |
    | `isaac_controllers.yaml` | 60 | false |
    | `real_controllers.yaml` | 50 | false |

### Controllers in each file

A controller is only active after a spawner starts it.

| Controller | Type | Joints | sim | mujoco | isaac | real |
| --- | --- | --- | --- | --- | --- | --- |
| `joint_broad` | `joint_state_broadcaster/JointStateBroadcaster` | all (real: all except `lift_joint`) | yes | yes | yes | yes |
| `omni_wheel_drive_controller` | `omni_wheel_drive_controller/OmniWheelDriveController` | four wheels | yes | yes | yes | yes |
| `lift_controller` | `joint_trajectory_controller/JointTrajectoryController` | `lift_joint` | yes | yes | yes | yes |
| `head_servo_controller` | `position_controllers/JointGroupPositionController` | `head_servo_joint` | yes | yes | yes | yes |
| `left_joint_trajectory_controller`, `right_joint_trajectory_controller` | `joint_trajectory_controller/JointTrajectoryController` | joints 1 to 7 of one arm | yes | yes | yes | yes |
| `left_gripper_controller`, `right_gripper_controller` | `position_controllers/GripperActionController` | `openarm_*_finger_joint1` | yes | yes | yes | yes |
| `left_forward_position_controller`, `right_forward_position_controller` | `forward_command_controller/ForwardCommandController` (position) | joints 1 to 7 of one arm | yes | yes | yes | yes |
| `left_forward_velocity_controller`, `right_forward_velocity_controller` | `forward_command_controller/ForwardCommandController` (velocity) | joints 1 to 7 of one arm | yes | no | yes | yes |
| `left_arm_controller`, `right_arm_controller` | `joint_trajectory_controller/JointTrajectoryController` | joints 1 to 7 of one arm | yes | yes | yes | no |
| `arm_controller` | `joint_trajectory_controller/JointTrajectoryController` | both arms and both `finger_joint1` | yes | yes | yes | no |
| `imu_sensor_broadcaster` | `imu_sensor_broadcaster/IMUSensorBroadcaster` | sensor `top_imu`, frame `top_imu_link` | no | yes | no | no |

??? info "Spawners, lift state relay, MuJoCo differences, and unused parameters"
    - Spawners: for example, `gzsim_omnibot.launch.py` spawns
      `omni_wheel_drive_controller`, `joint_broad`, `lift_controller`, the two
      `*_joint_trajectory_controller`, and the two `*_gripper_controller`.
    - In `real_controllers.yaml`, `joint_broad` lists its joints and leaves out
      `lift_joint`. The lift state comes from the lift homing node through a
      `topic_tools` relay in `real_omnibot.launch.py`. The comment in the file
      explains why: a second publisher would make RViz flicker.
    - `mujoco_omnibot.launch.py` spawns `joint_broad`,
      `omni_wheel_drive_controller`, `imu_sensor_broadcaster`,
      `lift_controller`, `head_servo_controller`, the two
      `*_joint_trajectory_controller`, and the two `*_gripper_controller`.
    - `mujoco_controllers.yaml` has no forward velocity controllers, because
      the MuJoCo arm joints have only a position command interface. Its header
      says that `imu_sensor_broadcaster` publishes on `~/imu`, so a launch file
      must remap it. `mujoco_omnibot.launch.py` remaps it to `/imu`.
    - `sim_controllers.yaml`, `isaac_controllers.yaml`, and
      `real_controllers.yaml` contain parameters for a
      `forward_velocity_controller` that no controller manager entry registers.
      These parameters are unused.

### Omni wheel controller parameters

All four files use the same `omni_wheel_drive_controller` parameters, except
`robot_radius`. The values must match the URDF.

| Parameter | Value | Meaning |
| --- | --- | --- |
| `wheel_names` | `[fl_wheel_joint, bl_wheel_joint, br_wheel_joint, fr_wheel_joint]` | Wheels in counterclockwise order, starting at front left |
| `wheel_offset` | 0.7853981633974485 (pi/4) | Angle of the first wheel. Each next wheel is 90 degrees further. |
| `robot_radius` | 0.185 (sim, mujoco, isaac), 0.18518136649782 (real) | Distance from the center of `base_footprint` to each wheel, in m |
| `wheel_radius` | 0.076 | Wheel radius in m |
| `odom_frame_id` | `odom` | Parent frame of the odometry TF |
| `base_frame_id` | `base_footprint` | Child frame of the odometry TF |
| `open_loop` | true | Odometry comes from the commanded velocity, not from wheel feedback |
| `position_feedback` | false | |
| `enable_odom_tf` | true | The controller publishes `odom` to `base_footprint` |
| `cmd_vel_timeout` | 0.5 | Seconds without a command before the wheels stop |
| `publish_limited_velocity` | true | |

??? info "How the values follow from the URDF, and the test tolerances"
    The wheels sit at (±0.130943, ±0.130943) m in `base_link`, so the distance
    to each wheel is 0.130943 × √2 ≈ 0.18518 m. The wheel collision cylinder
    has radius 0.075 m.

    `test_omni_wheel_geometry_matches_urdf` allows 0.001 m for
    `robot_radius`, 1 degree for the wheel angles, and 0.002 m for
    `wheel_radius`.

## Frames and joints

`base_footprint` is the root link. It sits on the floor. `base_link` is
0.0312 m above it (`base_link_joint`) with the same orientation, so forward
is +x in both frames.

!!! warning "Forward is -y in `lift_link` and `head_servo_link`"
    `lift_joint` is rotated 90 degrees about z, so the x axis of `lift_link`
    and `head_servo_link` points to the robot's left. `zed_head_camera_link`
    and the arm `link0` frames have x forward again.

```mermaid
graph TD
    odom -.->|omni_wheel_drive_controller| base_footprint
    base_footprint --> base_link
    base_link --> wheels["fl / fr / bl / br _wheel_link"]
    wheels --> rollers["roller_*_*_link, 28 per wheel (roller:=true only)"]
    base_link --> lift_link
    base_link --> front_lidar_link
    base_link --> back_lidar_link
    base_link --> merged_lidar_link
    base_link --> top_imu_link
    base_link --> bottom["bottom_* (RealSense D415)"]
    base_link --> unitree4d_l2_imu_initial
    lift_link --> head_servo_link
    head_servo_link --> zed["zed_head_* (ZED 2i)"]
    lift_link --> openarm_left_link0
    lift_link --> openarm_right_link0
    openarm_left_link0 --> left_hand["... openarm_left_hand, openarm_left_hand_tcp"]
    openarm_right_link0 --> right_hand["... openarm_right_hand, openarm_right_hand_tcp"]
```

??? info "`base_link` was rotated 90 degrees before description f6649ee"
    Until the description `main` branch was merged (description `f6649ee`),
    `base_link` was rotated 90 degrees about z, and forward was -y in
    `base_link`. The merge removed the rotation and re-expressed the base
    geometry and the wheel, lidar, camera, and lift origins, so their poses
    in the world are unchanged. Only `base_link` and the two frames fixed to
    it without a rotation of their own, `merged_lidar_link` and
    `top_imu_link`, turned by 90 degrees. Data in these frames, such as the
    merged scan and the IMU, is now x forward. Code that uses TF is not
    affected. Code that assumed the old axes is.

### Main frames

With `ros2_control:=gazebo`, the model has 56 links (168 with
`roller:=true`). The main ones:

| Frame | Parent | Notes |
| --- | --- | --- |
| `base_footprint` | none (root) | Robot origin on the floor. MuJoCo puts its free joint `floating_base_joint` here. |
| `base_link` | `base_footprint` | 0.0312 m above `base_footprint`, same orientation |
| `fl_wheel_link`, `fr_wheel_link`, `bl_wheel_link`, `br_wheel_link` | `base_link` | Omni wheels, at 45, 315, 135, and 225 degrees around `base_footprint` |
| `roller_<wheel>_<n>_link` (`n` 1 to 28) | `<wheel>_wheel_link` | Only with `roller:=true` |
| `lift_link` | `base_link` | Moves up and down with `lift_joint`. Rotated 90 degrees about z: x points left. |
| `front_lidar_link`, `back_lidar_link` | `base_link` | 0.235 m in front of and behind the center, 0.28232 m above `base_link`. `back_lidar_link` faces backward. Field of view ±2.09333333 rad, range 0.18 to 10 m in Gazebo. |
| `merged_lidar_link` | `base_link` | Frame of the merged scan, at the center and the lidar height, same orientation as `base_link` |
| `single_lidar_link` | `base_link` | Only with `ros2_control:=gazebo dual_lidar:=False` |
| `top_imu_link` | `base_link` | IMU, 0.17 m above `base_link`, same orientation |
| `bottom_link`, `bottom_color_optical_frame`, `bottom_depth_optical_frame` | `base_link`, via `bottom_bottom_screw_frame` | RealSense D415 camera (prefix `bottom`) at the front of the base, about 0.22 m above the floor. In the URDF its optical axis points forward and 50 degrees up. |
| `unitree4d_l2_imu_initial`, `unitree4d_l2_imu` | `base_link`, then `unitree4d_l2_imu_initial` | Unitree lidar frames, near the top of the robot |
| `head_servo_link` | `lift_link` | Turns with `head_servo_joint`. Rotated like `lift_link`: x points left. Collision mesh: `HeadCameraMount_collision.stl`. |
| `zed_head_camera_link`, `zed_head_left_camera_frame_optical` | `head_servo_link` (the optical frame via `zed_head_camera_link`) | ZED 2i head camera. The ZED links have no collision geometry. |
| `openarm_left_link0` to `openarm_left_link7`, `openarm_right_link0` to `openarm_right_link7` | `lift_link` | The two arms. `link0` is fixed to the lift. |
| `openarm_left_hand_tcp`, `openarm_right_hand_tcp` | `openarm_*_hand` | Tool center points, 0.08 m from the hand |

### Joints that ros2_control drives

| Joint | Type | Limits | Notes |
| --- | --- | --- | --- |
| `fl_wheel_joint`, `fr_wheel_joint`, `bl_wheel_joint`, `br_wheel_joint` | continuous | none | Velocity command |
| `lift_joint` | prismatic | 0 to 0.7435 m (-0.7435 to 0 m with `lift_at_top:=true`), 0.1 m/s | Position command. 0 is the bottom, or the top with `lift_at_top:=true`. |
| `head_servo_joint` | revolute | -0.785398163 to 1.04719755 rad | Position command |
| `openarm_left_joint1` to `openarm_left_joint7`, `openarm_right_joint1` to `openarm_right_joint7` | revolute | from `config/openarm/` | Position command. `openarm_right_joint2` is left out when `right_joint2_fixed` is true. |
| `openarm_left_finger_joint1`, `openarm_right_finger_joint1` | prismatic | 0 to 0.044 m | Gripper. `openarm_*_finger_joint2` copies it as a mimic joint and is not commanded. |

??? info "Initial arm positions in the `mock`, `gazebo`, `mujoco`, and `topic_base` modes"
    - 0.2618 rad: `openarm_left_joint1`, `openarm_left_joint4`, and
      `openarm_right_joint4`
    - -0.2618 rad: `openarm_right_joint1`
    - All other arm joints start at 0.

## Meshes and Git LFS

The `.dae` meshes are stored in Git LFS (`description/.gitattributes` has
`*.dae filter=lfs`). STL meshes are normal git files.

If git-lfs was not installed when you cloned, the `.dae` files are small text
pointers instead of meshes, and RViz and Gazebo cannot load them. Fetch the
real files:

```bash
git submodule foreach --recursive git lfs pull
```

??? info "The 23 files in Git LFS"
    | Path (in `description/`) | Used for |
    | --- | --- |
    | `meshes/openarm/arm/v10/visual/link0.dae` to `link7.dae` | Right arm visuals, and `link4` of the left arm |
    | `meshes/openarm/arm/v10/visual/link0_inv.dae` to `link7_inv.dae` | Mirrored left arm visuals. `link4_inv.dae` is not used. |
    | `meshes/openarm/arm/v10/visual/link3_original.dae` | Not used by the model |
    | `meshes/openarm/body/v10/visual/body_link0.dae` | Not used by the Walkie model, which has no OpenArm body link |
    | `meshes/openarm/ee/openarm_hand/visual/hand.dae`, `finger.dae`, `finger_inv.dae` | Hand visuals. `finger_inv.dae` is the `right_finger` of each hand. |
    | `meshes/walkie/omni_wheel_optimize.dae` | Omni wheel visual |
    | `urdf/walkie/depth_camera/d415/visual/d435.dae` | Not used by the model. The D415 macro uses `d415.stl`. |

??? info "Mirrored OpenArm meshes and simplified collision meshes"
    The mirrored OpenArm links use pre-mirrored meshes with a positive scale:
    `*_inv.dae` for visuals, `*_symp_inv.stl` for collisions, and
    `finger_inv.*` for the right finger of each hand. The model has no
    negative mesh scale. PhysX-based simulators (Isaac Sim, SAPIEN, O3DE) fail
    to cook meshes with a negative scale, and MuJoCo's URDF importer rejects
    them. `description/readme.md` explains this in detail.

    The collision meshes of the arm links, the hands, the wrist camera
    brackets, and the head mount (`HeadCameraMount_collision.stl`) are
    low-poly meshes of fewer than 100 triangles each. The ZED camera links
    have no collision geometry.

    Both changes came with the merge of description `main` (description
    `f6649ee`).

## Launch files

### robot_state_publisher.launch.py

`launch/robot_state_publisher.launch.py` runs `xacro` on `robot_model` and
starts `robot_state_publisher`.

| Argument | Default | Passed to xacro |
| --- | --- | --- |
| `robot_model` | `share/walkie_description/robots/gz_walkie.urdf.xacro` | It is the file |
| `ros2_control` | `gazebo` | yes |
| `dual_lidar` | `true` | yes |
| `use_zed` | `false` | yes, but no xacro file declares it, so it has no effect |
| `use_arm` | `true` | yes, but it has no effect in the xacro |
| `use_fake_arm_hardware` | `false` | yes. The xacro default is `True`. See [Known issues](#known-issues). |
| `left_can_interface` | `can1` | yes |
| `right_can_interface` | `can0` | yes |
| `right_joint2_fixed` | `false` | yes |
| `roller` | `false` | yes |
| `lift_at_top` | `false` | yes |
| `use_sim_time` | `false` | no, it is a robot_state_publisher parameter |

It does not pass `mujoco_model`, `mujoco_headless`, or `sim_controllers_yaml`,
so the xacro defaults apply. For this reason,
`bringup/launch/mujoco_omnibot.launch.py` expands the xacro itself.

??? info "Which launch files include it"
    The Gazebo, real robot, Isaac, `test_zed`, OpenArm joint state reader, Nav2
    loopback, and the two preview launches include it. Four launches start
    their own `robot_state_publisher` node instead: `moveit_demo.launch.py`,
    `moveit_demo_with_commander.launch.py`,
    `bringup/launch/openarm.bimanual.launch.py`, and
    `bringup/launch/mujoco_omnibot.launch.py`.

### Preview in RViz

Show the model in RViz without a simulator:

```bash
pixi run ros2 launch walkie_description robot_preview_rviz.launch.py dual_lidar:=False
```

With the default `ros2_control:=gazebo`, pass `dual_lidar:=False` or
`dual_lidar:=True`. Without it, xacro fails with `name 'true' is not defined`
(see [Gazebo Simulation: Known issues](../guides/simulation-gazebo.md#known-issues)).

| Launch file | What it starts |
| --- | --- |
| `robot_preview_rviz.launch.py` | robot_state_publisher, `joint_state_publisher_gui` (sliders for every joint), and RViz with `rviz/robot_preview.rviz` |
| `robot_preview_lift_rviz.launch.py` | robot_state_publisher, `joint_state_publisher` that reads `/lift/joint_states` at 30 Hz with `lift_joint` starting at 0.7435, and RViz |

Both declare `rviz_config_file` (default `rviz/robot_preview.rviz`) and
`use_rviz` (default `True`). They also pass `robot_model`, `ros2_control`
(default `gazebo`), and `use_sim_time` to `robot_state_publisher.launch.py`,
so you can set these on the command line, for example `ros2_control:=mock`.
`use_sim_time` defaults to `true` in `robot_preview_rviz.launch.py` and to
`false` in `robot_preview_lift_rviz.launch.py`.

??? info "Why `--show-args` shows other defaults"
    `--show-args` also prints the arguments of the included launch files, with
    their own defaults (for example `use_sim_time` `false` from
    `robot_state_publisher.launch.py`). The preview launch files pass their
    own values to the include, so those are the values that apply.

`launch/rviz.launch.py` starts RViz only. The preview launches pass
`robot_preview.rviz` to it. Do not start it alone with its default config. See
[Known issues](#known-issues).

## MuJoCo model

The MuJoCo model is generated from the same xacro, so the URDF stays the
single source of truth.

```bash
pixi run mjcf        # regenerate walkie.xml after a change to the description
pixi run mjcf-view   # open scene.xml in the MuJoCo viewer at the home keyframe
```

Neither command needs a colcon build. `build_mjcf.py` reads the xacro files
from the source tree, not from `install/`, so an old install cannot leak into
the model. To run the model with ros2_control, use `mujoco_omnibot.launch.py`
in `robot_bringup` (see
[MuJoCo Simulation: Run with ROS 2](../guides/simulation-mujoco.md#run-with-ros-2)).

!!! warning "Regenerate after every description change"
    `walkie.xml` is checked in. `test_generated_model_is_up_to_date` in
    `test/test_mjcf.py` fails when the file differs from a fresh generation.
    Run `pixi run mjcf` and commit the new `walkie.xml` together with the
    xacro change.

| File | Edit it? | Contents |
| --- | --- | --- |
| `scripts/build_mjcf.py` | yes | Expands `gz_walkie.urdf.xacro` with `ros2_control:=mujoco` from the source tree, then adds what URDF cannot describe. All other arguments keep their defaults, so the model has no rollers and the lift is mounted at the bottom (`roller:=false`, `lift_at_top:=false`). |
| `mujoco/walkie.xml` | no, generated | The robot. It has no floor. |
| `mujoco/scene.xml` | yes | Default world: floor, light, and the four wheel-floor contact pairs |

??? info "What the generator adds, and what the file headers document"
    - `build_mjcf.py` adds the free joint, wheel contact capsules, actuators,
      IMU, cameras, lidars, collision classes, and the `home` keyframe.
    - The header comment of `walkie.xml` lists the collision classes, lidars,
      cameras, and IMU.
    - The header of `scene.xml` explains how to write a custom world.

The full story of the MuJoCo model is in
[2. Add MuJoCo Simulation](../migrations/02-mujoco/index.md).

## Known issues

| Issue | Effect | Workaround |
| --- | --- | --- |
| `joint_broad` in `real_controllers.yaml` asks for states the `mock` block does not declare | `test_zed.launch.py` cannot activate `joint_broad` | None yet. When you fix it, remove the entry from `KNOWN_PAIRING_MISMATCHES`. |
| `use_fake_arm_hardware` defaults to `false` in `robot_state_publisher.launch.py` and to `True` in the xacro | With `ros2_control:=real_robot`, the launch file uses the real OpenArm CAN plugin. A plain `xacro` call uses mock arms. | Always pass `use_fake_arm_hardware` explicitly |
| `use_arm` is declared in the xacro but not used. No xacro file declares `use_zed`. | Setting either changes nothing. The arms are always in the model. | None |
| `launch/rviz.launch.py` defaults to `rviz/turtlebot3_rviz.rviz`, which does not exist in this package | Started alone, RViz cannot load its config | Pass `rviz_config:=<path>`, or use a preview launch |
| `config/hardware_layout.yaml` uses old joint names (`left_joint1`, ...) | None. No launch file or xacro reads it. | Ignore it. See [ros2_control modes](#ros2_control-modes). |
| `lift_at_top:=true` is only partly supported | `gazebo` starts `lift_joint` at 0.51, outside its -0.7435 to 0 m range. `lift_homing_node.py` and `robot_preview_lift_rviz.launch.py` still use 0 = bottom and 0.7435 = top. | Keep `lift_at_top:=false` |
| `roller` and `lift_at_top` are not supported in `mujoco` mode | `walkie.xml` is generated with the defaults, and the `mujoco` ros2_control block has fixed lift limits (0 to 0.743 m, initial 0.51 m). Setting either argument makes the URDF disagree with the MuJoCo model. | Keep both at `false` in `mujoco` mode. `mujoco_omnibot.launch.py` does not pass them. |

??? info "Causes and test details for these issues"
    - `joint_broad` and `mock`: `joint_broad` in `real_controllers.yaml` asks
      for wheel effort states and `head_servo_joint` velocity and effort
      states. The `mock` block does not declare them. This breaks
      `test_zed.launch.py`, which pairs `mock` with `real_controllers.yaml`.
      The pairing is listed in `KNOWN_PAIRING_MISMATCHES`, and its test is a
      strict expected failure. See [Tests](../guides/testing.md).
    - `hardware_layout.yaml`: the hardware of each joint is set in the
      ros2_control xacro files instead.
    - `lift_at_top`: `urdf/walkie/ros2_control.xacro` derives `lift_min`,
      `lift_max`, and `lift_init` from `lift_at_top`. The `gazebo` block
      still sets the initial value 0.51, and the `mujoco` block sets fixed
      values. `bringup/scripts/lift_homing_node.py` homes against the upper
      hard stop and publishes `lift_max_cm` / 100 (0.7435 m) there.
