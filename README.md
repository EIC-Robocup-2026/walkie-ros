# walkie-ros

## Description
This repository contains the ROS 2 software stack for Walkie robot. It provides a comprehensive suite of packages to support both real-world robot operation and simulation.

The project includes:
- **Robot Description**: URDF and Xacro definitions for the robot's kinematics and visualization.
- **Hardware Interfaces**: Drivers for Cubemars actuators, Lidar sensors (Hokuyo, Lakibeam), and other peripherals.
- **Simulation**: Gazebo environments and configurations for testing the robot in a virtual world.
- **Navigation**: Integration with the Nav2 stack for autonomous navigation, localization, and SLAM.
- **Bringup**: Launch files to easily start the robot's hardware and software subsystems.

## Project Structure
```text
walkie-ros
├── bringup
├── description (sub repo)
├── hardware
│   ├── actuator
│   └── lidar
├── navigation (sub repo)
│   └── nav2
├── simulation
│   └── gazebo
└── tools
│   └── joy_interface
└── readme.md
```

- **bringup**: Launch files and config for launching necessary software stacks.
- **description**: Robot URDF and mesh files.
- **hardware**: Contains all hardware drivers and software/packages specific to hardware.
- **navigation**: Contains Nav2 config and launch files, and future custom navigation packages.
- **simulation**: Gazebo launch, world, and model files.
- **tools**: Custom packages / tools for development/deployment.

# Getting Started

## 0. Prerequisites
Ensure you have the following software installed:
- **Ubuntu 24.04** (Native, VM, or WSL2)
- **ROS 2 Jazzy**
- **Git**
- **Git LFS**

## 1. Set up a ROS 2 Workspace
Create a workspace directory structure.
```text
ros2_ws
├── build/    # Build artifacts (generated)
├── install/  # Install artifacts (generated)
├── log/      # Build logs (generated)
└── src/      # Source code directory
    ├── walkie-ros  # This repository
    └── ...         # Other packages
```
Create the directory:
```bash
mkdir -p ~/ros2_ws/src
```

## 2. Clone the Repository
Clone this repository into your workspace `src` folder. Be sure to include submodules.
```bash
cd ~/ros2_ws/src/
git clone --recurse-submodules git@github.com:EIC-Robocup-2026/walkie-ros.git
```

## 3. Install Dependencies
Install the necessary ROS 2 dependencies using `rosdep`.
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
```

## 4. Build and Run
Build the packages and source the workspace.
```bash
colcon build --symlink-install
source install/setup.bash
```

### Run Gazebo Simulation
```bash
ros2 launch robot_simulation gzsim_omnibot.launch.py
```
