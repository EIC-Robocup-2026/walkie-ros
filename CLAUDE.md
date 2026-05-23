# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

```bash
# Full build (use --symlink-install so edits to Python/launch files take effect without rebuilding)
colcon build --symlink-install

# Single package rebuild
colcon build --packages-select <package_name> --symlink-install

# Source after build
source install/setup.bash
```

## Running

```bash
# Gazebo simulation + Nav2 (one command, handles sourcing and zenoh router)
./scripts/sim_nav_gazebo.sh

# Gazebo only
ros2 launch robot_simulation gzsim_omnibot.launch.py

# Full hardware bringup
ros2 launch robot_bringup real_omnibot.launch.py

# Lift homing (hardware)
ros2 launch robot_bringup lift_homing.launch.py

# Lift homing (no hardware — instant, no CAN bus)
ros2 launch robot_bringup lift_homing.launch.py mock:=true
```

## CAN bus (real robot)

```bash
# Bring CAN interface up (required before any TMotor/CubeMars hardware)
./scripts/can_setup.sh up can0 1000000

# Bring down
./scripts/can_setup.sh down
```

## Python venv (bringup package)

The `bringup` package has extra Python dependencies (TMotorCANControl, python-can, etc.) managed by `uv`. The venv must exist before running on real hardware; the launch file injects it into `PYTHONPATH` automatically via `additional_env` so no manual sourcing is needed.

```bash
cd bringup
uv venv --python python3.12 --system-site-packages
uv sync
```

`--system-site-packages` keeps `/opt/ros/jazzy` visible inside the venv.

## Tests

```bash
# Run all tests
colcon test
colcon test-result --verbose

# Single package
colcon test --packages-select <package_name>
```

Notable test files:
- `description/test/test_xacro.py` — validates all URDF/Xacro files
- `manipulate/walkie_ik_servo/test/` — IK and self-collision tests

## Architecture

### Packages

| Directory | Package(s) | Language | Role |
|-----------|-----------|----------|------|
| `bringup/` | `robot_bringup` | Python | Top-level launch files and config; lift homing node with TMotorCANControl |
| `description/` | `walkie_description` | Xacro/URDF | Robot model (submodule) |
| `hardware/actuator/` | `cubemars_hardware`, `dynamixel_hardware_interface`, `dynamixel_interfaces` | C++ | ros2_control hardware plugins for wheel and arm actuators |
| `hardware/lidar/` | `dual_laser_merger`, `urg_node2`, `Lakibeam_ROS2_Driver` | C++ | Hokuyo (rear) + Lakibeam (front) drivers, merged into `/scan` |
| `manipulate/` | `left_arm_commander_cpp`, `left_arm_moveit_config`, `walkie_ik_servo`, `my_robot_interfaces` | C++/Python | MoveIt arm planning, Pinocchio-based IK with self-collision |
| `navigation/` | `robot_navigation` + nav2 submodule | C++/Python | Nav2 config, SLAM, human follower, pose publisher |
| `perception/` | `walkie_perception` | C++/Python | YOLOv8 object detection on ZED RGB-D, `GetObPose` service |
| `simulation/gazebo/` | `robot_simulation` | Python | Gazebo worlds, ros_gz bridge config |
| `tools/` | `joy_interface`, `keyboard_teleop` | Python/C++ | Joystick and keyboard teleoperation |

### Key design points

**ros2_control**: All actuators go through ros2_control. Controller configs live in `description/config/ros2_controller/`. Real robot uses `cubemars_hardware` and `dynamixel_hardware_interface` plugins; simulation uses `gz_ros2_control`.

**Dual lidar**: Two physical sensors (Hokuyo URG rear, Lakibeam front) are merged by `dual_laser_merger` into a single `/scan` topic for Nav2.

**Lift homing**: `bringup/scripts/lift_homing_node.py` runs a blocking homing sequence on init (drives to upper hard stop, detects current spike, sets origin), then switches to position-velocity control. A software trapezoidal trajectory shapes motion commands to avoid overwhelming the motor. `lift_homing_mock_node.py` provides the same ROS interface without CAN.

**Middleware**: Uses `rmw_zenoh_cpp` instead of the default DDS. The `sim_nav_gazebo.sh` script auto-starts `rmw_zenohd` if not running. The Zenoh router config is at `scripts/WALKIE_RMW_ZENOH_ROUTER_CONFIG.json5`.

**Submodules**: `description/` and `navigation/` are separate git submodules. Always clone with `--recurse-submodules`.

**Docker/CI**: GitHub Actions builds two Docker images (`simulation` and `physical`) on push to `main`, published to `ghcr.io/eic-robocup-2026/walkie-ros`. Local compose files are in `docker/`.

### Lift homing launch arguments

`lift_homing.launch.py` exposes all tuning parameters as arguments. Key ones:

| Argument | Default | Notes |
|----------|---------|-------|
| `mock` | `false` | `true` = no CAN, instant homing |
| `motor_id` | `16` | CAN node ID |
| `homing_erpm` | `-8500` | ERPM toward upper hard stop |
| `lift_m_per_rad` | `-0.00012406` | Signed kinematic ratio, must be set correctly |
| `topic_position_commands` | `lift/cmd` | Input command topic (`Float64MultiArray`: `[pos_cm, vel_cm_s, acc_cm_s2]`) |
| `topic_joint_states` | `lift/joint_states` | Output state topic |
