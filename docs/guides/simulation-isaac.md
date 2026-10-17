# Isaac Sim Simulation

This guide explains the automated, configuration-driven **NVIDIA Isaac Sim** integration for Walkie Bot (`walkie-isaac`).

## Overview

Unlike manual Omniverse GUI workflows, the simulation workspace is fully automated and version-controlled:

1. **Submodule Architecture**: The simulation package lives in the `walkie-isaac` Git submodule, decoupling large USD assets and simulator scripts from the main ROS 2 autonomy packages.
2. **Automated Action Graph**: At runtime, `launch_sim.py` programmatically constructs the OmniGraph ROS 2 bridge without manual pin wiring.
3. **Environment Isolation**: Isaac Sim's internal Python 3.12 runtime and bundled ROS 2 libraries are isolated from Pixi's ROS 2 runtime.
4. **Configuration-Driven**: Topic names, sensor mounts, and physics rates are loaded from `config/sim_config.yaml`.
5. **Clean Stage Referencing**: The converted USD robot asset is referenced onto a pristine stage (`/World/walkie_bot`) rather than opened directly, preventing corrupted viewport metadata from crashing the simulation.

## Quick Start

### 1. Launch Simulation via Pixi

From the root of `walkie-ros`:

```bash
pixi run sim
```

To run headless in automated testing or CI:

```bash
pixi run sim -- --headless --test --steps 300
```

### 2. Monitor Active Topics

In another terminal:

```bash
pixi run topics
```

Output:
```
/clock
/cmd_vel
/parameter_events
/rosout
/scan
```

### 3. Full ROS 2 Bringup & Navigation

To run autonomous navigation inside Isaac Sim:

```bash
# Terminal 1: Zenoh router (required for rmw_zenoh_cpp)
pixi run zenoh-router

# Terminal 2: Isaac Sim Walkie Bot
pixi run sim

# Terminal 3: Full ROS 2 bringup (RSP, controllers, self-filter, RViz)
pixi run full-isaacbringup

# Terminal 4: Nav2 localization & navigation
pixi run nav-sim
```

Or for online SLAM mapping:
```bash
pixi run slam-sim
```

### 4. Drive the Robot (Teleop)

In a separate terminal:

```bash
pixi run teleop
```

Use standard keyboard controls (`u/i/o/j/k/l/m/,/.` or shift for holonomic omni-drive).

### 5. Joint & Manipulation Control

- **Interactive Joint CLI**: `pixi run test-joints` (controls arm, lift, head, and gripper positions).
- **MoveIt 2 Planning**: `pixi run moveit-sim` (launches MoveIt 2 bimanual motion planning).

> [!NOTE]
> **Lift Joint Direction**: The Isaac Sim USD model defines the torso lift prismatic joint with its origin at the top (extending downward). The simulation bringup sets `lift_at_top:=true` in `robot_state_publisher` so the TF tree correctly aligns the torso, arms, and sensors with the base.

> [!TIP]
> **Build Guideline**: Always build packages with `--parallel-workers 1` (`pixi run build-setup` or `pixi run build-pkg -- <pkg>`) to prevent high RAM usage and system freezes.

## Communication Spec

| Topic | ROS 2 Message Type | Direction | Purpose |
| --- | --- | --- | --- |
| `/clock` | `rosgraph_msgs/msg/Clock` | Isaac Sim → ROS 2 | Simulation time synchronization (`use_sim_time: true`). |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | ROS 2 → Isaac Sim | Velocity commands for omnidirectional base. |
| `/scan` | `sensor_msgs/msg/LaserScan` | Isaac Sim → ROS 2 | 2D planar LiDAR for SLAM and Nav2. |

## Configuration (`config/sim_config.yaml`)

```yaml
simulation:
  headless: false
  physics_dt: 0.016666667   # 60 Hz physics step
  rendering_dt: 0.016666667 # 60 Hz rendering step

robot:
  usd_path: "assets/Collected_gz_walkie_roller/gz_walkie_roller_absolute_path.usda"
  prim_path: "/World/walkie_bot"

sensors:
  lidar:
    mount_prim_path: "/World/walkie_bot/Geometry/base_footprint/base_link/front_lidar_link"
    sensor_prim_path: "/World/walkie_bot/Geometry/base_footprint/base_link/front_lidar_link/RtxLidar"
    frame_id: "front_lidar_link"
    topic_name: "/scan"
    type: "laser_scan"
    draw_debug: false
    horizontal_fov: 360.0
    min_range: 0.05
    max_range: 25.0

ros2:
  domain_id: 0
  clock_topic: "/clock"
  cmd_vel_topic: "/cmd_vel"
  scan_topic: "/scan"
```
