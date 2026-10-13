# Walkie ROS

ROS 2 (Jazzy) software stack for Walkie, the service robot of the EIC RoboCup
2026 team. New to the workspace? Start with
[Getting Started](getting-started.md).

![Walkie at RoboCup@Home 2026, Incheon](https://github.com/user-attachments/assets/4141053d-d985-4ba4-ae8d-bf04ddfef6bd)

## Where to start

| If you want to... | Read |
| --- | --- |
| Install the workspace and run the simulation | [Getting Started](getting-started.md) |
| Do a specific task: simulate, navigate, manipulate, run the real robot | [Guides](guides/index.md) |
| Find a package, a launch argument, or a robot frame | [Project Structure](reference/project-structure.md) and [Robot Description](reference/robot-description.md) |
| Understand why the workspace looks the way it does | [Migration Log](migrations/index.md) |
| Fix an error | [Troubleshooting](troubleshooting.md) |

The workspace is managed with [pixi](https://pixi.sh) and
[RoboStack](https://robostack.github.io). pixi installs ROS 2 and the build
and simulation dependencies into the project folder, so you do not need a
system ROS installation.

??? info "What the workspace contains"
    Everything that runs on the robot or in simulation:

    - the robot description (URDF/Xacro) and the MuJoCo model generated from it;
    - ros2_control hardware plugins for the wheels, arms, and head, a separate
      node for the lift, and the lidar drivers;
    - Gazebo simulation, and MuJoCo simulation in progress;
    - Nav2 navigation, MoveIt manipulation for the bimanual OpenArm, and
      perception;
    - the bringup launch files that tie them together.

??? info "For next generations: this site as a learning guide"
    This site is also a learning guide. The [Migration Log](migrations/index.md)
    records the big changes to the workspace in order: what the team changed,
    why, and, where recorded, which alternatives were considered. Read it when
    you need to change the same parts, or when you want to learn how a ROS 2
    project is put together.

## Contributing to the docs

The pages are Markdown files in the `docs/` folder. Preview them locally at
http://127.0.0.1:8000:

```bash
pixi run docs
```

[3. Add the Documentation Site](migrations/03-documentation.md) explains how the
site is built and how to add a page.
