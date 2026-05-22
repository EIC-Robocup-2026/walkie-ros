# openarm_bimanual_commander_cpp

MoveIt2-based motion planning commander for the Walkie bimanual robot arms. Exposes ROS 2 action servers and services that wrap MoveIt `MoveGroupInterface`, with TF2-aware frame handling and full collision checking on every motion.

---

## Architecture

```
Client (Python / C++ / CLI)
        │
        ▼ action / service call
┌────────────────────────┐
│  Commander node (C++)  │
│  ├── MoveGroupInterface│  ← MoveIt planning + collision check
│  ├── tf2_ros Buffer    │  ← frame transforms
│  └── GripperCommand    │  ← GripperActionController clients
└────────────────────────┘
        │
        ▼ JointTrajectory
  ros2_control JTC
        │
        ▼
    Hardware / sim
```

---

## MoveIt Groups

| Group name | Joints | Notes |
|---|---|---|
| `left_arm` | 7 (openarm_left_joint1–7) | Left arm only |
| `right_arm` | 7 (openarm_right_joint1–7) | Right arm only |
| `left_arm_lift` | 8 (left arm + lift_joint) | Left arm + lift column |
| `right_arm_lift` | 8 (right arm + lift_joint) | Right arm + lift column |
| `both_arms` | 14 (both arms) | Bimanual planning |
| `both_arms_lift` | 15 (both arms + lift_joint) | Bimanual + lift |
| `left_gripper` | 1 | Gripper open/close (use `control_gripper`) |
| `right_gripper` | 1 | Gripper open/close (use `control_gripper`) |

All arm groups run with `maxVelocityScalingFactor=1.0`, `planningTime=1.0s`, `numPlanningAttempts=10000`.

---

## Launch

```bash
# Requires openarm_bimanual_moveit_config to be sourced
ros2 launch openarm_bimanual_commander_cpp commander.launch.py
```

---

## Action Interfaces

All actions are in the `my_robot_interfaces` package.

### `go_to_pose` — Absolute EEF pose (Euler)

```
string group_name       # arm group (not gripper)
float64 x, y, z        # position (m)
float64 roll, pitch, yaw  # orientation (rad, RPY)
string frame_id         # reference frame (default: base_footprint)
bool cartesian_path     # true = straight-line Cartesian path
---
bool success
string status
```

```bash
ros2 action send_goal /go_to_pose my_robot_interfaces/action/GoToPose \
  "{group_name: right_arm, x: 0.4, y: -0.2, z: 0.5, roll: 0.0, pitch: 1.57, yaw: 0.0}"
```

---

### `go_to_pose_quat` — Absolute EEF pose (quaternion)

```
string group_name
float64 x, y, z
float64 qx, qy, qz, qw
string frame_id
bool cartesian_path
---
bool success
string status
```

---

### `go_to_pose_relative` — Relative EEF displacement

```
string group_name
float64 x, y, z        # positional offset (m)
float64 roll, pitch, yaw  # orientation delta (rad)
string frame_id         # reference frame for offset (default: base_footprint)
bool cartesian_path
bool ee_frame           # offset reference:
                        #   false → world/frame_id axes
                        #   true  → EEF-local axes
---
bool success
string status
```

**Frame semantics:**

| `ee_frame` | Position offset axes | Orientation composition |
|---|---|---|
| `false` | `frame_id` world axes | `q_delta × q_current` (world-fixed rotation) |
| `true` | EEF-local axes | `q_current × q_delta` (body-fixed rotation) |

Example — move +5 cm along the EEF's own Z axis:
```bash
ros2 action send_goal /go_to_pose_relative my_robot_interfaces/action/GoToPoseRelative \
  "{group_name: right_arm, z: 0.05, ee_frame: true}"
```

---

### `go_to_home` — Named home position

```
string group_name   # arm groups only (not gripper)
---
bool success
string status
```

```bash
ros2 action send_goal /go_to_home my_robot_interfaces/action/GoToHome \
  "{group_name: both_arms}"
```

---

### `control_gripper` — Open/close gripper

```
string group_name   # left_gripper or right_gripper
float64 position    # meters: 0.0 = fully closed
---
bool success
string status
```

```bash
# Open right gripper
ros2 action send_goal /control_gripper my_robot_interfaces/action/ControlGripper \
  "{group_name: right_gripper, position: 0.04}"
```

---

### `set_joint_position` — Direct joint angles

```
string group_name
float64[] joint_positions   # rad, length must match group DOF
---
bool success
string status
```

MoveIt plans a collision-free path to the target joint configuration. Not suitable for high-rate streaming (see Benchmark below).

```bash
ros2 action send_goal /set_joint_position my_robot_interfaces/action/SetJointPosition \
  "{group_name: right_arm, joint_positions: [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0]}"
```

---

## Service Interfaces

### `get_ee_pose` — Query EEF position

```
string group_name
string frame_id     # default: base_footprint
---
float64 x, y, z
float64 qx, qy, qz, qw
string frame_id
bool success
string status
```

```bash
ros2 service call /get_ee_pose my_robot_interfaces/srv/GetEEPose \
  "{group_name: right_arm, frame_id: base_footprint}"
```

---

### `get_joint_states` — Query current joint positions

```
string group_name
---
string[] joint_names
float64[] joint_positions   # rad
bool success
string status
```

```bash
ros2 service call /get_joint_states my_robot_interfaces/srv/GetJointStates \
  "{group_name: right_arm}"
```

---

## Included Tools

### Interactive CLI — `commander_interactive`

Terminal UI for manually testing all interfaces.

```bash
ros2 run openarm_bimanual_commander_cpp commander_interactive
```

Menu options:

| Key | Action |
|---|---|
| 1 | `go_to_pose` — Euler RPY input |
| 2 | `go_to_pose_relative` — with EE-frame toggle |
| 3 | `go_to_pose_quat` — Euler or quaternion input |
| 4 | `go_to_home` |
| 5 | `control_gripper` |
| 6 | `set_joint_position` |
| 7 | `get_ee_pose` — shows position + quaternion + euler (rad & deg) |
| 8 | `get_joint_states` — shows each joint in rad and degrees |
| q | quit |

---

### Benchmark — `commander_benchmark`

Characterizes commander throughput and planning latency.

```bash
ros2 run openarm_bimanual_commander_cpp commander_benchmark [options]
```

**Arguments:**

| Argument | Default | Description |
|---|---|---|
| `--group` | `right_arm_lift` | MoveIt group to use |
| `--benchmark` | `all` | `all` \| `get_joint_states` \| `sequential` \| `stress` \| `jtc` |
| `--n` | 1000 | Calls for `get_joint_states` benchmark |
| `--n-seq` | 10 | Goals for sequential benchmark |
| `--duration` | 5.0 | Seconds for stress / JTC benchmarks |
| `--hz` | 100.0 | Target rate for stress / JTC benchmarks |
| `--amplitude` | 0.05 | Sine amplitude (rad, ≈ 3°) |
| `--freq` | 0.5 | Sine trajectory frequency (Hz) |

**Benchmark descriptions:**

| Name | What it measures |
|---|---|
| `get_joint_states` | Service latency (mean / min / max / p95 / p99) at N calls tight-loop |
| `sequential` | Round-trip time per `set_joint_position` goal (plan + execute) |
| `stress` | `set_joint_position` at `--hz` — preemptive cancel mode and fire-and-forget mode, tracks accepted / rejected / middleware-dropped / completed |
| `jtc` | Raw `JointTrajectory` topic streaming to `/right_joint_trajectory_controller/joint_trajectory` at `--hz`, measures publish rate jitter and tracking error vs `/joint_states` |

**Example — service throughput only:**
```bash
ros2 run openarm_bimanual_commander_cpp commander_benchmark \
  --group right_arm --benchmark get_joint_states --n 1000
```

**Example — JTC streaming at 500 Hz:**
```bash
ros2 run openarm_bimanual_commander_cpp commander_benchmark \
  --group right_arm --benchmark jtc --hz 500 --duration 5 --amplitude 0.05
```

---

## Performance Characteristics

| Interface | Achieved rate | Notes |
|---|---|---|
| `get_joint_states` service | ~100 Hz | Ceiling set by `joint_state_broadcaster` `update_rate` (500 Hz in current config) |
| `set_joint_position` action | ~0.4 Hz | MoveIt plan + execute per goal |
| `set_joint_position` stress (100 Hz) | ~0 accepted | Zenoh middleware queue (depth 10) overflows; use JTC for streaming |
| JTC topic streaming | 500+ Hz achievable | No collision check; use for VLA joint streaming |

For high-rate joint position streaming **with collision checking**, see `walkie_ik_servo` (Pinocchio-based, Cartesian input) or MoveIt Servo (`left_arm_moveit_config/config/servo.yaml`).

---

## Dependencies

- `rclcpp`, `rclcpp_action`
- `moveit_ros_planning_interface`, `moveit_core`
- `my_robot_interfaces` (actions + services)
- `tf2_ros`, `tf2_geometry_msgs`
- `control_msgs` (GripperCommand)
- `openarm_bimanual_moveit_config` (SRDF, kinematics, joint limits)
