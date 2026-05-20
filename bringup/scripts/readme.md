# Lift Homing Node

ROS 2 bridge for the AK45-10 lift motor (TMotor servo CAN). Runs a current-rise
homing sequence at startup, then shapes incoming position commands with a SW
trapezoidal trajectory and feeds them to the motor.

Source: `lift_homing_node.py` (node name: `tmotor_ros2_bridge`).

## Position convention

Matches the URDF (`lift.urdf.xacro`: prismatic, `lower=0.000m`, `upper=0.7435m`):

| Position (m) | Meaning                |
|--------------|------------------------|
| `0.0`        | bottom of travel       |
| `0.7435`     | top of travel (homed)  |

The motor is homed against the **upper** hard stop, so `motor_rad = 0`
corresponds to the lift at the top. `lift_m_per_rad` is signed (negative)
because commanding the motor positive drives the lift downward.

## Setup (Python deps via `uv`)

The node depends on `TMotorCANControl` and `python-can`, declared in
`bringup/pyproject.toml` and pinned in `bringup/uv.lock`. The venv must inherit
the system `site-packages` so `rclpy` / `sensor_msgs` / `std_msgs` from
`/opt/ros/jazzy` remain importable.

```bash
cd ~/walkie_ws/walkie-ros/bringup
uv sync --system-site-packages
```

Activate the venv (must be sourced in every shell that runs the node):

```bash
source /home/ryu/walkie_ws/walkie-ros/bringup/.venv/bin/activate
```

Then build the workspace and source it:

```bash
cd ~/walkie_ws
colcon build --packages-select robot_bringup --symlink-install
source install/setup.bash
```

## Run

```bash
source /home/ryu/walkie_ws/walkie-ros/bringup/.venv/bin/activate
source /home/ryu/walkie_ws/install/setup.bash
ros2 launch robot_bringup lift_homing.launch.py
```

Override params on the CLI, e.g.:

```bash
ros2 launch robot_bringup lift_homing.launch.py \
  hardware_max_cm_s:=2.5 \
  lift_m_per_rad:=-0.00012406
```

The homing sequence runs synchronously inside `__init__`. The node enters
`CONTROL` state once homing succeeds; commands sent before then are ignored.

## Topics

### Subscribed

| Topic                                       | Type                          | Notes                                              |
|---------------------------------------------|-------------------------------|----------------------------------------------------|
| `/forward_position_controller/commands`     | `std_msgs/Float64MultiArray`  | `[pos_cm, vel_cm_s, acc_cm_s2]` (vel/acc optional) |

### Published (every loop tick @ `control_frequency` Hz)

| Topic                          | Type                          | Notes                                                |
|--------------------------------|-------------------------------|------------------------------------------------------|
| `/lift/joint_states`           | `sensor_msgs/JointState`      | Feeds the preview launch's `joint_state_publisher`.   |
| `/lift_controller/commands`    | `std_msgs/Float64MultiArray`  | Single-element `[traj_current_pos_m]` for the ros2_control `lift_controller`. |

`/lift_controller/commands` flows into
`position_controllers/JointGroupPositionController` on `lift_joint`. With
`mock_components/GenericSystem` the command loops back through
`joint_state_broadcaster` → `/joint_states` → RViz.

## Commands

`std_msgs/Float64MultiArray.data = [pos_cm, vel_cm_s?, acc_cm_s2?]`

- `pos_cm` must lie in `[lift_min_cm, lift_max_cm]` — out-of-range is rejected.
- `vel_cm_s` is clamped by `min(max_velocity_cm_s, hardware_max_cm_s)`.
- `acc_cm_s2` is clamped by `max_acceleration_cm_s2`.

Examples:

```bash
# Go to the bottom at 1 cm/s
ros2 topic pub --once /forward_position_controller/commands \
  std_msgs/msg/Float64MultiArray "data: [0.0, 1.0]"

# Go to the top at 2 cm/s
ros2 topic pub --once /forward_position_controller/commands \
  std_msgs/msg/Float64MultiArray "data: [74.35, 2.0, 0.5]"
```

## Parameters

| Name                     | Default        | Notes                                                                 |
|--------------------------|----------------|-----------------------------------------------------------------------|
| `motor_id`               | `16`           | CAN ID                                                                |
| `motor_type`             | `AK45-10`      | TMotor model                                                          |
| `homing_erpm`            | `-8000`        | Drive direction toward the upper hard stop                            |
| `homing_backoff_time_s`  | `1.0`          | Time to back off before driving toward the stop                       |
| `current_diff_limit`     | `0.35` A       | Δ-current above baseline that flags a hard stop                       |
| `abs_current_cap`        | `4.0` A        | Hard safety cap during homing                                         |
| `control_frequency`      | `100.0` Hz     | Timer rate for trajectory advance + publishing                        |
| `max_velocity_cm_s`      | `5.0`          | User-facing nominal max velocity                                      |
| `max_acceleration_cm_s2` | `20.0`         | User-facing nominal max acceleration                                  |
| `hardware_max_cm_s`      | `3.0`          | Hard ceiling — what the motor can physically sustain. Caps both the explicit and fallback vel so RViz (which publishes the trajectory) cannot outrun reality. |
| `lift_m_per_rad`         | `0.0`          | Signed m / motor-rad. **Must be non-zero** or commands are rejected.  |
| `lift_min_cm`            | `0.0`          | Lower position limit (cm) — matches URDF lower limit                  |
| `lift_max_cm`            | `74.35`        | Upper position limit (cm) — matches URDF upper limit (= top, homed)   |
| `joint_name`             | `lift_joint`   | Joint name used in published `JointState`                             |
