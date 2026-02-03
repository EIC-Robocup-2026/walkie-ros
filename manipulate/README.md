# Manipulate
This folder contains packages related to the manipulation capabilities of the robot, specifically for the left arm.

## Packages
### [left_arm_commander_cpp](./left_arm_commander_cpp)
This package contains the commander node for controlling the left arm. It likely interfaces with MoveIt and other ROS 2 control components to send commands to the arm.

### [left_arm_moveit_config](./left_arm_moveit_config)
This package contains the MoveIt 2 configuration files for the left arm. This includes the SRDF, kinematics configuration, joint limits, and launch files for bringing up the MoveIt planning pipeline.

### [my_robot_interfaces](./my_robot_interfaces)
This package defines custom interfaces (messages, services, actions) used by the manipulation packages. It ensures consistent communication types between nodes.

## Installation

### Prerequisites

#### Install MoveIt 2 for ROS 2 Jazzy:
```bash
sudo apt update
sudo apt install ros-jazzy-moveit
```

#### Install Additional MoveIt Dependencies:
You will likely need these common MoveIt controllers and plugins:
```bash
sudo apt install ros-jazzy-moveit-planners \
                 ros-jazzy-moveit-ros-planning-interface \
                 ros-jazzy-moveit-ros-move-group \
                 ros-jazzy-controller-manager \
                 ros-jazzy-ros2-control \
                 ros-jazzy-ros2-controllers \
                 ros-jazzy-trac-ik
```

#### Build walkie_description package:
The `walkie_description` package must be built before building the manipulation packages, as it contains the robot description files required by MoveIt configuration.
```bash
colcon build --packages-select walkie_description
source install/setup.bash
```

### Build
Navigate to your workspace root and build the packages:
```bash
colcon build --packages-select my_robot_interfaces left_arm_commander_cpp left_arm_moveit_config
source install/setup.bash
```

## Configuration

### Hardware Layout Configuration
The `walkie_description/config/hardware_layout.yaml` file defines which hardware interface each joint uses.

#### Mock Mode (Simulation/Testing):
```yaml
hardware_config:
  # Left Arm
  left_joint1: mock
  left_joint2: mock
  left_joint3: mock
  left_joint4: mock
  left_joint5: mock
  left_joint6: mock
  left_joint7: mock
  left_gripper_worm_gear_joint: mock
```

All joints are set to **`mock`**, meaning:
- **Simulation/Testing mode** - no real motors are controlled
- ROS 2 Control uses the `MockHardware` interface
- Useful for development, testing MoveIt configurations, and debugging without hardware
- The robot state updates in RViz, but no physical movement occurs

#### Real Hardware Mode (CubeMars Motors):
To control actual CubeMars motors, change from `mock` to `cubemars`:
```yaml
hardware_config:
  # Left Arm
  left_joint1: cubemars
  left_joint2: cubemars
  left_joint3: cubemars
  left_joint4: cubemars
  left_joint5: cubemars
  left_joint6: cubemars
  left_joint7: cubemars
  left_gripper_worm_gear_joint: cubemars
```

## Usage

### 1. Launch MoveIt & RViz
Use the `test.launch.py` file to start the MoveIt MoveGroup node and open RViz. This visualization allows you to see the robot state and interactive markers.
```bash
ros2 launch left_arm_moveit_config test.launch.py
```

### 2. Launch the Commander Node
In a new terminal, launch the commander node. This node initializes the Action Servers and waits for commands.
```bash
ros2 launch left_arm_commander_cpp commander.launch.py 
```

**Note:** You should see "Commander Node Initialized (Left Arm Only)" in the terminal.

## Action API Reference

The commander node provides a high-level Action Server interface for controlling the Walkie robot using MoveIt 2. It handles inverse kinematics, path planning, and gripper control (converting degrees to radians automatically).

You can control the robot by sending Action Goals from the terminal or another node.

### Understanding Cartesian Path Planning

**Cartesian Path** refers to how the robot's end-effector (gripper) moves through space when executing a motion command.

#### Two Planning Methods:

**1. `cartesian_path: false` (Joint Space Planning)**
- The planner calculates joint angles needed to reach the target pose
- The arm takes the most efficient path in joint space
- **The end-effector path is NOT a straight line** - it may curve or take an unpredictable route
- Faster and more reliable for most movements
- **Good for:** moving to goal positions where the exact path doesn't matter

**2. `cartesian_path: true` (Cartesian Space Planning)**
- The end-effector follows a **straight line** in 3D space from start to goal
- MoveIt computes many intermediate waypoints along this straight path
- More computationally expensive and can fail if obstacles are in the way
- **Good for:** precise tasks like drawing, pouring, inserting objects, or avoiding obstacles along a specific path

### A. Control Gripper (Degrees)
The node accepts degrees and automatically converts them to radians for the controller.
* **Open:** -15.71 rad
* **Close:** 0.7 rad

**Open Gripper:**
```bash
ros2 action send_goal /control_gripper my_robot_interfaces/action/ControlGripper "{group_name: 'left_gripper', position: -15.71}"
```

**Close Gripper:**
```bash
ros2 action send_goal /control_gripper my_robot_interfaces/action/ControlGripper "{group_name: 'left_gripper', position: 0.7}"
```

### B. Go To Pose (Absolute)
Moves the arm to a specific Cartesian coordinate (x, y, z) and orientation (roll, pitch, yaw) relative to the `base_footprint`.

**Coordinate Mapping (Base_Footprint Frame):**
* **X:** (+ FORWARD) / (- BACKWARD)
* **Y:** (+ LEFT) / (- RIGHT)
* **Z:** (+ UP) / (- DOWN)

**Orientation Presets:**
* **Gripper Forward:** `Roll: -1.57, Pitch: 0.0, Yaw: 1.57`
* **Gripper Down:** `Roll: 0.0, Pitch: 0.0, Yaw: 0.0` (Parallel to body)

**Example with Gripper Forward orientation:**
```bash
ros2 action send_goal /go_to_pose my_robot_interfaces/action/GoToPose "
{
  group_name: 'left_arm',
  x: 0.38, 
  y: 0.19, 
  z: 0.58, 
  roll: -1.57, 
  pitch: 0.0, 
  yaw: 1.57, 
  cartesian_path: false
}"
```

**Example with Gripper Down orientation:**
```bash
ros2 action send_goal /go_to_pose my_robot_interfaces/action/GoToPose "
{
  group_name: 'left_arm',
  x: 0.38, 
  y: 0.19, 
  z: 0.58, 
  roll: 0.0, 
  pitch: 0.0, 
  yaw: 0.0, 
  cartesian_path: false
}"
```

### C. Go To Pose (Relative)
Moves the arm relative to its current end-effector position. This is useful for precise adjustments.

**Coordinate Mapping (End-Effector Frame):**
* **X:** (+ Left) / (- Right)
* **Y:** (+ Down) / (- Up)
* **Z:** (+ Backward) / (- Forward)

**Example: Move 5cm DOWN (Y+)**
```bash
# Uses cartesian_path: true to ensure straight line motion
ros2 action send_goal /go_to_pose_relative my_robot_interfaces/action/GoToPoseRelative "
{
  group_name: 'left_arm',
  x: 0.0, 
  y: 0.05, 
  z: 0.0, 
  roll: 0.0, 
  pitch: 0.0, 
  yaw: 0.0,
  cartesian_path: true
}"
```

### D. Go To Home
Moves the arm to the default "all-zeros" joint configuration.
```bash
ros2 action send_goal /go_to_home my_robot_interfaces/action/GoToHome "{group_name: 'left_arm'}"
```

### Inverse Kinematics (IK) Solver Configuration

The IK solver can be configured in `manipulate/left_arm_moveit_config/config/kinematics.yaml`. You can choose between different solvers and optimization strategies.

#### Option 1: KDL Kinematics Plugin (Default)
```yaml
# Uncomment to use KDL solver
# left_arm:
#   kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
#   kinematics_solver_search_resolution: 0.005
#   kinematics_solver_timeout: 0.005
```

#### Option 2: TRAC-IK Kinematics Plugin (Recommended)
```yaml
left_arm:
  kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
  kinematics_solver_timeout: 1000.0
  solve_type: Speed  # Optional: "Speed", "Distance", or "Manip1"
```

**TRAC-IK Solve Types:**
- **`Speed`**: Fastest solution, optimizes for computation time
- **`Distance`**: Optimizes for minimal joint displacement from current position
- **`Manip1`**: Optimizes for manipulability (avoids singularities and joint limits)

## Troubleshooting

### 1. "Action Server not available"
* Make sure the robot's controllers are running. Check with:
```bash
ros2 control list_controllers
```
You should see `left_hand_controller` and `left_arm_controller` as `active`.

### 2. "Planning Failed"
* The target might be out of reach or in collision.
* Try moving the target closer to the center of the workspace.
* If using `cartesian_path: true`, the straight-line path might be blocked or impossible. Try with `cartesian_path: false`.

### 3. "Aborted (Invalid Group)"
* Ensure you are using `group_name: 'left_arm'` for arm movements and `group_name: 'left_gripper'` for gripper commands.

### 4. "No motion with real hardware"
* Verify that `hardware_layout.yaml` has the correct joints set to `cubemars` instead of `mock`.
* Check CAN bus or serial connections to the motors.
* Ensure motor power supply is connected and enabled.
