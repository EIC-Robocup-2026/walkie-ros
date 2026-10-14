# Real Robot

Prepare the robot computer once, then start the lift, the hardware drivers,
and the rest of the stack on the physical robot.

You need a workspace built with `pixi run build`
([Getting Started](../getting-started.md)). Read
[Middleware (Zenoh)](middleware.md) first: every terminal on the robot needs
the Zenoh settings. On a new robot computer, do the one-time steps below
first.

## Safety notes

!!! warning "Before you power the robot"
    - **No emergency stop in software.** No node in this workspace
      implements an emergency stop.
    - **The lift moves at launch.** Homing starts as soon as
      `lift_homing.launch.py` runs.
    - **`lift_m_per_rad` must match the hardware.** A wrong sign or value
      moves the lift the wrong way or the wrong distance.
    - **The arms move at startup.** With `use_arm:=true`, the arms go to the
      home pose over 5 s after their controllers start. The head goes to
      0.25 rad.
    - **The joystick overrides Nav2.** Priority 100 beats Nav2's 10. Button 0
      cancels Nav2 goals.

    The lift has more limits that affect safety. See
    [Known issues](#known-issues).

## Set up the devices (once) { #device-setup }

USB adapters can get a different name after each boot or replug. The setup
script gives each device a fixed name. For CAN adapters, it also sets the
bitrate and brings the interface up every time the adapter is plugged in.
Run it once per robot computer, from the system shell with `sudo`, not
through pixi. Use only `setup_all_device.sh`: the older single-device scripts
in `scripts/setup_device/` conflict with it (see [Known issues](#known-issues)).

1. Unplug each device before you start.
2. Run the script. In the menu, enter device numbers from the table below
   (for example `1 3`) or `all`. `all` still asks `Proceed? [Y/n]` before it
   starts.

    ```bash
    sudo ./scripts/setup_device/setup_all_device.sh          # interactive menu
    sudo ./scripts/setup_device/setup_all_device.sh all      # all devices
    ```

3. Plug in each device when the script asks. The script finds it by comparing
   the devices before and after. It waits up to 30 seconds for a new device,
   then asks `Install? [Y/n]`.
4. Check a device. `status` does not need root:

    ```bash
    ./scripts/setup_device/setup_device.sh status --alias can2
    ```

| Number | Device | Type | Name after setup | Settings |
| --- | --- | --- | --- | --- |
| 1 | Dynamixel USB-to-serial | serial | `/dev/ttyDynamixel` | USB latency timer 1 ms |
| 2 | CubeMars USB-to-CAN | can | `can2` | 1 Mbit/s |
| 3 | OpenArm USB CAN FD | canfd | `can0` (first channel), `can1` (second channel) | 1 Mbit/s nominal, 5 Mbit/s data |
| 4 | Left wrist camera | camera | `/dev/left_wrist_cam` | none |
| 5 | Right wrist camera | camera | `/dev/right_wrist_cam` | none |

??? info "Call `setup_device.sh` directly, and what it installs"
    `setup_all_device.sh` calls `scripts/setup_device/setup_device.sh` for
    each device. You can also call it directly:

    ```bash
    sudo ./scripts/setup_device/setup_device.sh install --type can --alias can2 --bitrate 1000000
    sudo ./scripts/setup_device/setup_device.sh uninstall --alias can2
    ```

    | Type | Files | Effect |
    | --- | --- | --- |
    | `serial` | `/etc/udev/rules.d/99-<alias>.rules` | Symlink `/dev/<alias>`, mode 0666, optional latency timer |
    | `camera` | `/etc/udev/rules.d/99-<alias>.rules` | Symlink `/dev/<alias>`, group `video` |
    | `can`, `canfd` | `/etc/systemd/network/70-<interface>.link` | Fixed interface name per channel |
    | `can`, `canfd` | `/etc/systemd/system/<alias>-up.service` | Sets the bitrate and brings the interface up when the adapter appears |

??? info "Bring a CAN interface up by hand (current session only)"
    ```bash
    sudo ip link set can2 up type can bitrate 1000000
    sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on
    ```

### ZED USB reset rule

The [ZED watchdog](#zed-watchdog) can reset the camera's USB port. That needs
write access to the USB device. Install the rule once, then unplug and replug
the camera:

```bash
sudo cp bringup/udev/99-zed-usb-reset.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### Set up the ZED camera (ZED SDK 5.5 & CUDA 13.2) { #zed-setup }

The robot uses a Stereolabs ZED 2i stereo camera on the head (`zed_head`). Building and running the ZED camera driver requires NVIDIA CUDA 13.2 and the Stereolabs ZED SDK 5.5 on the host computer.

Machines without an NVIDIA GPU or CUDA can skip this section and run simulations or navigation without camera hardware (`use_zed:=false`).

#### 1. Install NVIDIA Driver and CUDA 13.2

Ensure the proprietary NVIDIA display driver and CUDA 13.2 toolkit are installed on Ubuntu:

```bash
# Verify NVIDIA driver is loaded and GPU is accessible
nvidia-smi

# Check CUDA compiler version (CUDA 13.2)
nvcc --version
```

If CUDA 13.2 is not yet installed, follow NVIDIA's official instructions for Ubuntu:
```bash
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update
sudo apt install -y cuda-toolkit-13-2
```
Ensure `/usr/local/cuda-13.2/bin` is in your `PATH` and `/usr/local/cuda-13.2/lib64` is in `LD_LIBRARY_PATH`.

#### 2. Install ZED SDK 5.5

Download the ZED SDK 5.5 installer for Ubuntu 24.04 and CUDA 13.2 from the [Stereolabs Download Page](https://www.stereolabs.com/developers/release):

```bash
# Make the installer executable
chmod +x ZED_SDK_Linux_Ubuntu24_cuda13.2_v5.5.0.run

# Run the installer (silent mode bypasses interactive prompts)
./ZED_SDK_Linux_Ubuntu24_cuda13.2_v5.5.0.run -- silent
```

Verify the SDK installation with the ZED Diagnostic tool:
```bash
/usr/local/zed/tools/ZED_Diagnostic
```

#### 3. Pull the ZED ROS 2 Wrapper Repositories

From the `walkie-ros` repository root, pull `hardware/camera/zed-ros2-wrapper` and its submodules (`zed-ros2-interfaces`, `zed-ros2-description`, `zed-ros2-examples`) pinned to version `v5.5.0`:

```bash
pixi run setup-real
```

This clones the repositories defined in `walkie_real.repos` into `hardware/camera/zed-ros2-wrapper/`. The directory is listed in `.gitignore` so your Git status stays clean.

#### 4. Build the ZED ROS 2 Wrapper Packages

Build the camera driver packages against the active pixi environment:

```bash
pixi run build-pkg zed_interfaces zed_description zed_components zed_wrapper
```
*(Or run `pixi run build` to build the full workspace with the newly added packages).*

#### 5. Verify and Launch the ZED Camera

1. **Standalone camera test**:
   ```bash
   pixi run ros2 launch robot_bringup test_zed.launch.py
   ```
2. **Real robot bringup with ZED enabled**:
   ```bash
   pixi run ros2 launch robot_bringup real_omnibot.launch.py use_zed:=true
   ```
   The camera will be supervised by `bringup/scripts/zed_watchdog.py`, which monitors the stream and automatically recovers from USB or frame-stall faults.

## Create the lift Python environment (once) { #python-environment }

The lift node needs TMotorCANControl and python-can. They are not in the pixi
environment, so they live in a uv virtual environment at `bringup/.venv`.

1. Install [uv](https://docs.astral.sh/uv/getting-started/installation/) on
   the robot computer.
2. From the repository root, create the environment:

    ```bash
    cd bringup
    pixi run uv venv --python python3.12 --system-site-packages
    pixi run uv sync
    ```

Run uv through `pixi run`, so the venv uses the pixi Python that also runs the
lift node ([why](pixi.md#python-virtual-environments-next-to-pixi)). You do
not activate the venv: `lift_homing.launch.py` adds it to the node's
`PYTHONPATH`. This works by default, because `pixi run build` uses
`--symlink-install`. If you build without it, set the path yourself:

```bash
export ROBOT_BRINGUP_VENV_SITE=$PWD/bringup/.venv/lib/python3.12/site-packages
```

??? info "How the launch file finds the venv, and what `uv sync` installs"
    `lift_homing.launch.py` adds `bringup/.venv/lib/python3.12/site-packages`
    to the node's `PYTHONPATH`. It finds the source folder by following the
    symlink that `--symlink-install` creates for the launch file.

    `uv sync` installs the packages locked in `bringup/uv.lock`: the team fork
    of TMotorCANControl, python-can, numpy, pyserial, and pyyaml.

## Start the robot { #bringup-order }

In every terminal, export the Zenoh settings first (see
[Middleware (Zenoh)](middleware.md)):

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=23
```

Then start each step in its own terminal, in this order:

| Step | Command | Why here |
| --- | --- | --- |
| 1 | `pixi run ros2 run rmw_zenoh_cpp rmw_zenohd` | Nodes cannot find each other without the router |
| 2 | `pixi run ros2 launch robot_bringup lift_homing.launch.py` | Resets `can2` and homes the lift. Wait for `Homing Successful`. |
| 3 | `pixi run ros2 launch robot_bringup real_omnibot.launch.py` | Hardware, sensors, and controllers |
| 4 | `pixi run ros2 launch robot_navigation localization_Nav2_real.launch.py` | Localization and Nav2. See [Navigation](navigation.md). |
| 5 | `pixi run ros2 launch openarm_bimanual_moveit_config moveit_only.launch.py hardware_type:=real_robot` | MoveIt. See [Manipulation](manipulation.md). |
| 6 | `pixi run ros2 launch openarm_bimanual_commander_cpp commander.launch.py` | Arm commander |
| 7 | `pixi run ros2 launch joy_interface joy_control_smooth.launch.py` | Joystick teleoperation |

- `moveit_only.launch.py` also has a `right_joint2_fixed` argument. Give it
  the same value as in `real_omnibot.launch.py`.
- Under pixi, step 3 stops with `package not found`. See
  [Packages that pixi does not provide](#packages-that-pixi-does-not-provide).

??? info "Which bringup step needs which"
    ```mermaid
    flowchart LR
        Z["1. Zenoh router"] --> L["2. Lift homing"]
        L --> R["3. real_omnibot"]
        R --> N["4. Nav2"]
        R --> M["5. MoveIt"]
        M --> C["6. Commander"]
        R --> J["7. Joystick"]
    ```

## Run and command the lift { #lift-node }

The lift is not controlled through ros2_control.
`bringup/scripts/lift_homing_node.py` drives the real motor with the
TMotorCANControl Python library.

??? info "How the lift still appears in the robot model"
    In the `real_robot` URDF, `lift_joint` uses a
    `mock_components/GenericSystem` placeholder. `real_omnibot.launch.py`
    relays `lift/joint_states` into `/joint_states`, so RViz and
    `robot_state_publisher` still show the lift.

### Start the lift

```bash
pixi run ros2 launch robot_bringup lift_homing.launch.py
pixi run ros2 launch robot_bringup lift_homing.launch.py mock:=true   # no hardware
```

!!! warning "The lift moves when the node starts"
    Homing starts inside the node's constructor. The lift moves as soon as
    you launch it. Keep hands and objects clear of the lift. The node also
    resets the `can2` bus that the wheels use (see [Known issues](#known-issues)).

The node homes against the upper hard stop. It detects the stop by a rise in
motor current.

- **Success:** the log shows `Homing Successful. Origin set.`
- **Failure:** the log shows an error that starts with `Homing failed` or
  `Backoff aborted`. The motor goes idle, and the node keeps running. It
  ignores `lift/cmd` without a message and aborts action goals with the
  warning `not in CONTROL state`. Fix the cause and restart the node.

??? info "Homing sequence step by step"
    1. Measure the idle current for 0.9 s. Warn if it is already above
       `current_diff_limit`.
    2. Back away from the top at `-homing_erpm` for `homing_backoff_time_s`.
       If the current goes above `abs_current_cap`, homing stops.
    3. Drive toward the top at `homing_erpm`. After a short settle, measure
       the baseline current for 0.5 s. Then watch for a current rise above
       `current_diff_limit`. This step has no timeout. Only
       `abs_current_cap` stops it.
    4. At the hard stop: brake briefly, wait, and save the motor's origin
       permanently. The lift is now at `lift_max_cm`.
    5. Switch to position-velocity control. The node now accepts commands.

### Launch arguments

All tuning values are launch arguments. Always start the node through the
launch file: several of the node's own defaults differ from the launch
defaults, and its default `lift_m_per_rad` of `0.0` does not work.

??? info "All lift launch arguments"
    | Argument | Default | Meaning |
    | --- | --- | --- |
    | `mock` | `false` | `true` runs the mock node: no CAN, no venv |
    | `motor_id` | `16` | CAN ID of the lift motor |
    | `motor_type` | `AK45-10` | TMotor model name passed to TMotorCANControl |
    | `homing_erpm` | `-8500` | Speed and direction toward the upper hard stop |
    | `current_diff_limit` | `0.85` | Current rise (A) above the baseline that counts as the hard stop |
    | `abs_current_cap` | `4.0` | Absolute current (A) that aborts homing |
    | `control_frequency` | `100.0` | Control loop rate (Hz) |
    | `max_velocity_cm_s` | `5.0` | Soft velocity limit (cm/s) |
    | `max_acceleration_cm_s2` | `2.0` | Acceleration limit (cm/s²) |
    | `homing_backoff_time_s` | `3.0` | Time to back away from the top before homing |
    | `hardware_max_cm_s` | `3.0` | Hard velocity ceiling (cm/s) |
    | `lift_m_per_rad` | `-0.00012406` | Signed ratio between motor angle and lift height. Calibrated on the robot. |
    | `lift_min_cm` | `8.0` | Lowest allowed position (cm) |
    | `lift_max_cm` | `74.35` | Highest position (cm). This is the homed top. |
    | `joint_name` | `lift_joint` | Joint name in published messages |
    | `topic_joint_states` | `lift/joint_states` | Output joint state topic |
    | `topic_lift_controller_commands` | `lift_controller/commands` | Output position topic |
    | `topic_position_commands` | `lift/cmd` | Input command topic |

    `lift_m_per_rad` is negative because a positive motor rotation moves the
    lift down. With the node's own default of `0.0`, the node logs an error,
    rejects `lift/cmd`, and its control loop divides by zero after homing.

### Move the lift { #commands-and-interfaces }

Publish `[pos_cm, vel_cm_s, acc_cm_s2]` on `lift/cmd` (velocity and
acceleration are optional). Example: move to 40 cm at 3 cm/s with 2 cm/s²
acceleration.

```bash
pixi run ros2 topic pub --once /lift/cmd std_msgs/msg/Float64MultiArray "{data: [40.0, 3.0, 2.0]}"
```

- `lift/cmd` uses centimeters. `lift/joint_states` and the action use meters
  (0.08 to 0.7435), the same as the `lift_joint` limits in the URDF.
- On `lift/cmd`, a position outside `lift_min_cm` to `lift_max_cm` is
  rejected with an error. The action clamps its goal to this range instead.
- The velocity is limited to `max_velocity_cm_s` and `hardware_max_cm_s`.
  With the launch defaults, the lift never moves faster than 3 cm/s or
  accelerates faster than 2 cm/s².

MoveIt moves the lift through the `lift_controller/follow_joint_trajectory`
action, which the lift node serves.

??? info "Lift topics, action server, and motion profile"
    | Name | Kind | Type | Content |
    | --- | --- | --- | --- |
    | `lift/cmd` | Subscribed topic | `std_msgs/Float64MultiArray` | `[pos_cm, vel_cm_s, acc_cm_s2]`. Velocity and acceleration are optional. |
    | `lift/joint_states` | Published topic | `sensor_msgs/JointState` | Position (m) and velocity of the software trajectory, effort from the motor |
    | `lift_controller/commands` | Published topic | `std_msgs/Float64MultiArray` | `[pos_m]` |
    | `lift_controller/follow_joint_trajectory` | Action server | `control_msgs/FollowJointTrajectory` | Used by MoveIt. Only the last waypoint is used. |

    The node shapes each move with a trapezoidal velocity profile and sends
    the profile to the motor point by point. This keeps the motor from
    getting sudden large steps.

    The action server waits up to 60 s for the lift to reach the goal within
    1 cm, then reports success (see [Known issues](#known-issues)).

### Mock mode

`mock:=true` starts `lift_homing_mock_node.py`. It needs no CAN bus and no
venv. It starts at the top (`lift_max_cm`) without homing. It has the same
topics as the real node, with two differences:

- It has no `follow_joint_trajectory` action server. MoveIt cannot move the
  lift against the mock.
- The effort in `lift/joint_states` is always 0.

## Main bringup: `real_omnibot.launch.py`

`bringup/launch/real_omnibot.launch.py` starts ros2_control with the real
hardware plugins, the sensors, and the support nodes.

```bash
pixi run ros2 launch robot_bringup real_omnibot.launch.py
pixi run ros2 launch robot_bringup real_omnibot.launch.py use_zed:=false use_rviz:=false
```

| Argument | Default | Meaning |
| --- | --- | --- |
| `use_zed` | `true` | Start the ZED head camera and the nodes that use it |
| `use_rviz` | `true` | Start RViz with `bringup/config/rviz/default.rviz` |
| `use_arm` | `true` | Spawn the arm controllers and send the arms to the home pose |
| `use_fake_arm_hardware` | `false` | Use mock arm hardware instead of the OpenArm CAN plugin |
| `right_joint2_fixed` | `false` | Lock the right arm's joint 2 and remove it from the right arm controllers |
| `use_unitree_lidar` | `true` | Start the Unitree 4D L2 driver and its self filter |

!!! note
    Without the arms connected, pass `use_fake_arm_hardware:=true` as well as `use_arm:=false` (see [Known issues](#known-issues)).

??? info "Other arguments: ZED watchdog, head beep, CAN interfaces, Unitree frames"
    | Argument | Default | Meaning |
    | --- | --- | --- |
    | `use_zed_watchdog` | `true` | Run the ZED under `zed_watchdog.py` instead of launching it directly |
    | `use_head_beep` | `true` | Start the head touch beep node |
    | `left_can_interface` | `can1` | CAN interface of the left arm |
    | `right_can_interface` | `can0` | CAN interface of the right arm |
    | `unitree_cloud_frame` | `unitree4d_l2_imu_initial` | Frame of the Unitree point cloud |
    | `unitree_imu_frame` | `unitree4d_l2_imu` | Frame of the Unitree IMU |

??? info "Everything `real_omnibot.launch.py` starts"
    | Component | Package | Condition |
    | --- | --- | --- |
    | `robot_state_publisher` | `walkie_description` launch file | always |
    | `ros2_control_node` with `real_controllers.yaml` | `controller_manager` | always |
    | Spawners: `omni_wheel_drive_controller`, `joint_broad`, `head_servo_controller` | `controller_manager` | always |
    | Spawners: left and right joint trajectory and gripper controllers | `controller_manager` | `use_arm` |
    | Arm home pose, sent once after the trajectory controllers start | `ros2 topic pub` | `use_arm` |
    | Head start position 0.25 rad, sent once | `ros2 topic pub` | always |
    | `twist_mux`, output to `/omni_wheel_drive_controller/cmd_vel` | `twist_mux` | always |
    | Dual 2D lidar and merger, output `/scan` | `bringup/launch/dual_lidar_hokuyo_lakibeam.launch.py` | always |
    | Unitree driver and self filter, output `/unilidar/cloud/filtered` | `unitree_lidar_ros2`, `robot_self_filter` | `use_unitree_lidar` |
    | RealSense `bottom` camera and self filter | `realsense2_camera`, `robot_self_filter` | always |
    | ZED `zed_head` camera, directly or under the watchdog | `zed_wrapper`, `robot_bringup` | `use_zed` |
    | ZED self filter, `cloud_to_map`, `ob_pose_service_cpp` | `robot_self_filter`, `walkie_perception` | `use_zed` |
    | Relay `lift/joint_states` to `/joint_states` | `topic_tools` | always |
    | Current pose publisher (`current_pose`, `map` to `base_footprint`) | `robot_navigation` | always |
    | Head touch beep node | `robot_bringup` | `use_head_beep` |
    | TF server | `walkie_tf` | always |
    | rosbridge WebSocket server | `rosbridge_server` | always |
    | Foxglove bridge on `0.0.0.0` | `foxglove_bridge` | always |
    | RViz | `rviz2` | `use_rviz` |

    The lift controller is not spawned on purpose. The lift node serves the
    `lift_controller/follow_joint_trajectory` action itself.

    All spawners use `--switch-timeout 30`. The controllers can take some
    seconds to become active after the hardware connects.

### Packages that pixi does not provide

| Package | Used for | Effect under pixi |
| --- | --- | --- |
| `zed_wrapper` | ZED camera | Fetched via `pixi run setup-real`; requires ZED SDK 5.5 and CUDA 13.2 (see [Set up the ZED camera](#zed-setup)). Pass `use_zed:=false` if working without CUDA. |
| `realsense2_camera` | Bottom camera | Always started. Without it, the launch stops with `package not found`. |
| `topic_tools` | Lift joint state relay | Always started. Without it, the launch stops with `package not found`. |

The physical Docker image installs all three (see [Docker Images](docker.md)).
For the fix under pixi, see
[real_omnibot.launch.py stops with package not found](../troubleshooting.md#real_omnibotlaunchpy-stops-with-package-not-found).

### Velocity commands

`real_omnibot.launch.py` starts `twist_mux` with the same
`bringup/config/twist_mux/twist_mux.yml` as the simulation. The input
priorities, timeouts, and joystick controls are in
[Gazebo Simulation: Drive the robot](simulation-gazebo.md#driving-the-robot).

## ZED watchdog

After some USB faults, the ZED node keeps sending frames, but every frame is
plain green and the depth is all NaN. The ZED wrapper does not detect this.
`bringup/scripts/zed_watchdog.py` restarts the camera when it sees a fault.
It runs by default.

- Every decision goes to `~/.ros/zed_watchdog.log`.
- To launch the ZED without the watchdog, pass `use_zed_watchdog:=false`.
- After repeated restarts, the watchdog resets the ZED's USB port. That needs
  the [ZED USB reset rule](#zed-usb-reset-rule).

??? info "Fault limits and restart behavior"
    The watchdog owns the ZED launch process and watches
    `/zed_head/zed_node/rgb/color/rect/image`.

    | Fault | Default limit |
    | --- | --- |
    | Frames have one uniform color (the green fault) | 4 s |
    | No new frame (stall) | 8 s |
    | No first frame after a (re)start | 45 s |
    | The ZED launch process exits | immediately |

    After a restart, the watchdog waits 30 s before it checks again. After 5
    restarts within 300 s, it switches to a slow retry every 60 s and resets
    the ZED's USB port before each retry.

## Head touch beep

`bringup/scripts/head_touch_beep_node.py` detects a press on the head from the
servo's position error or load change. It plays a short chirp with `aplay` and
publishes `std_msgs/Empty` on `~/touched` (node name `head_touch_beep`).
Disable it with `use_head_beep:=false`.

## Arm joint state reader

```bash
pixi run ros2 launch robot_bringup openarm_joint_state_reader.launch.py
```

`bringup/launch/openarm_joint_state_reader.launch.py` reads the OpenArm joint
positions directly from the CAN bus. By default it disables the motors at
start (`disable_on_start:=true`), so you can move the arms by hand and read
the positions.

!!! warning
    It uses the same CAN interfaces as `real_omnibot.launch.py`. Do not run
    both at the same time.

??? info "Joint state reader launch arguments"
    | Argument | Default |
    | --- | --- |
    | `arms` | `both` |
    | `left_can_interface` | `can1` |
    | `right_can_interface` | `can0` |
    | `hand` | `true` |
    | `rate_hz` | `50.0` |
    | `disable_on_start` | `true` |
    | `print_throttle_s` | `2.0` |
    | `use_rsp` | `true` |
    | `rviz` | `true` |
    | `lift_position` | `0.7435` |

## Helper scripts

`scripts/bringup/connect_walkie_zenoh_router.sh` starts a router with the team
config (see [The connect script](middleware.md#the-connect-script)). The other
helper scripts are out of date or unused (see [Known issues](#known-issues)).
Run the [bringup steps](#bringup-order) by hand instead.

??? info "All helper scripts"
    | Script | What it does |
    | --- | --- |
    | `scripts/bringup/connect_walkie_zenoh_router.sh` | Starts a router with the team config |
    | `scripts/tmux_walkie.sh` | Builds a tmux session `walkie`. Window `ros-ws` has one pane per bringup step, with the command typed but not run. `WALKIE_TMUX_NO_ATTACH=1` builds it without attaching. |
    | `.tmuxp.yaml` | The same session for `tmuxp` |
    | `scripts/bringup/real_slam_nav.sh` | Opens `gnome-terminal` tabs for Nav2 and SLAM Toolbox |
    | `scripts/setup_env.sh` | Old pixi activation hook |

    The last four are out of date or unused.

## Hardware overview

| Part | Hardware | Connection | Driver |
| --- | --- | --- | --- |
| Base | 4 CubeMars motors with omni wheels | `can2`, CAN 1 Mbit/s | `cubemars_hardware` (ros2_control) |
| Lift | TMotor AK45-10, CAN ID 16 | `can2`, shared with the base | `lift_homing_node.py` (outside ros2_control) |
| Arms | Bimanual OpenArm with Damiao motors | `can0` right arm, `can1` left arm, CAN FD | `openarm_hardware` (ros2_control) |
| Head | Dynamixel servo, ID 2 | `/dev/ttyDynamixel`, 57600 baud | `dynamixel_hardware_interface` (ros2_control) |
| Front 2D lidar | Hokuyo URG | Ethernet, `10.0.0.101` | `urg_node2` |
| Back 2D lidar | Lakibeam | Ethernet, `10.0.0.102` | `lakibeam1` |
| 3D lidar | Unitree 4D L2 | Ethernet (UDP), `10.0.0.103` | `unitree_lidar_ros2` |
| Head camera | ZED 2i, camera name `zed_head` | USB | `zed_wrapper`, supervised by `zed_watchdog.py` |
| Bottom camera | RealSense D415, camera name `bottom` | USB | `realsense2_camera` |
| Wrist cameras | USB cameras | `/dev/left_wrist_cam`, `/dev/right_wrist_cam` | Device names only. No launch file uses them yet. |

The lidar configuration expects the robot computer at `10.0.0.201` on the
lidar network. The Lakibeam `hostip` in
`bringup/config/lidar/dual_lidar_config.yml` and the Unitree `local_ip` in
`bringup/launch/real_omnibot.launch.py` both use this address.

??? info "Wheel CAN IDs and OpenArm motor models"
    CubeMars wheel motor CAN IDs: front left 1, front right 3, back right 2,
    back left 4.

    The OpenArm motors per joint (from
    `bringup/scripts/openarm_joint_state_reader.py`):

    | Joints | Motor |
    | --- | --- |
    | Joint 1 and 2 | DM8009 |
    | Joint 3 and 4 | DM4340 |
    | Joint 5 to 7, gripper | DM4310 |

??? info "Connection diagram"
    ```mermaid
    flowchart LR
        PC["Robot computer"]
        PC ---|"can2, CAN 1 Mbit/s"| BASE["4 CubeMars wheel motors"]
        PC ---|"can2"| LIFT["TMotor lift motor"]
        PC ---|"can0, CAN FD"| RARM["Right OpenArm"]
        PC ---|"can1, CAN FD"| LARM["Left OpenArm"]
        PC ---|"USB serial"| HEAD["Dynamixel head servo"]
        PC ---|"Ethernet"| LIDAR["Hokuyo, Lakibeam, Unitree L2"]
        PC ---|"USB"| CAM["ZED 2i, RealSense D415"]
    ```

## Known issues

| Issue | Effect | Workaround |
| --- | --- | --- |
| The TMotorCANControl fork resets `can2` when the lift node starts and takes it down when the node exits. | The wheels share `can2`. Starting or stopping the lift node cuts the base off its bus. | Start the lift node before `real_omnibot.launch.py`. Do not restart it while the base runs. |
| The homing search for the upper hard stop has no timeout. | If the node never detects the stop, the motor keeps driving. Only `abs_current_cap` stops it. | Stay at the robot during homing, ready to cut power. |
| `lift/joint_states` publishes the software trajectory, not a measured position. | If the motor stalls, the published position still reaches the goal. | Check the lift on the robot. No node publishes the measured position. |
| The lift action reports success after its 60 s wait even if the lift did not arrive. | MoveIt treats a stalled lift move as done. | Do not treat success as proof that the lift arrived. |
| `use_arm` in `description/robots/gz_walkie.urdf.xacro` is declared but not used. | `use_arm:=false` only skips the arm controllers. The OpenArm CAN plugin still loads. | Without the arms, also pass `use_fake_arm_hardware:=true`. |
| `pixi.toml` does not include `realsense2_camera` or `topic_tools`. | `real_omnibot.launch.py` stops with `package not found` under pixi. | See [Troubleshooting](../troubleshooting.md#real_omnibotlaunchpy-stops-with-package-not-found). |
| [`bringup/scripts/readme.md`](https://github.com/EIC-Robocup-2026/walkie-ros/blob/main/bringup/scripts/readme.md) is out of date. | Its setup steps and topic do not match the code. Its example `[0.0, 1.0]` is rejected, because the bottom is 8 cm. | Follow [Run and command the lift](#lift-node) on this page. |
| `scripts/setup_device/` still has the older single-device setup scripts. | They and `setup_device.sh` overwrite or uninstall each other's files. | Use only `setup_all_device.sh`. |
| `scripts/tmux_walkie.sh` and `.tmuxp.yaml` are out of date. | The pane commands run outside pixi and without the Zenoh settings. | Run the commands from [Start the robot](#bringup-order) by hand. |
| `scripts/bringup/real_slam_nav.sh` launches a file that does not exist. | The script fails. | Use `localization_Nav2_real.launch.py`. See [Navigation](navigation.md). |
| `scripts/setup_env.sh` is an old pixi activation hook. | Nothing uses it. `pixi.toml` activates `install/setup.bash`. | Ignore it. |

??? info "Why the lift node takes `can2` down"
    The team fork of TMotorCANControl hard-codes `can2`. When the lift node
    starts, the library runs `sudo /sbin/ip link set can2 down` and brings
    `can2` up again at 1 Mbit/s. It takes `can2` down when the node exits.

??? info "What is out of date in the old scripts and the lift readme"
    - `bringup/scripts/readme.md` uses `/home/ryu` paths, `/opt/ros/jazzy`,
      the old input topic `/forward_position_controller/commands`, and the
      node defaults instead of the launch defaults. It treats 0 cm as the
      bottom and does not mention the action.
    - The older single-device scripts are `setup_can2_udev.sh`,
      `setup_dynamixel_udev.sh`, and `setup_openarm_can_udev.sh`.
      `setup_can2_udev.sh` writes `70-can2.link` and `can2-up.service`, the
      same file names that `setup_device.sh` uses for `can2`.
    - `scripts/tmux_walkie.sh` and `.tmuxp.yaml` use
      `~/robocup2026/walkie-ws` and have no `pixi run` or Zenoh exports.
    - `scripts/bringup/real_slam_nav.sh` sources `/opt/ros` and launches
      `localization_Nav2_stvl_real.launch.py`, which does not exist.

For errors during bringup, such as
[The lift node cannot import TMotorCANControl](../troubleshooting.md#the-lift-node-cannot-import-tmotorcancontrol),
see [Troubleshooting](../troubleshooting.md).
