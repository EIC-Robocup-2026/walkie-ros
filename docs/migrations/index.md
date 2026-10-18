# Migration Log

The big changes to the workspace, in order. Each entry records **what** the
team changed, **why**, and, where recorded, which alternatives were rejected.
The guides tell you how to use the workspace today; the log tells you how it
got that way.

Read an entry before you change the same parts. A setting that looks odd, such
as a capsule-shaped wheel or a version pin, usually has a reason recorded here.

| Entry | What changed |
| --- | --- |
| [1. Move to Pixi](01-pixi.md) | From a system ROS installation (apt, rosdep, colcon) to a project-local pixi environment |
| [2. Add MuJoCo Simulation](02-mujoco/index.md) | A MuJoCo model of Walkie generated from the robot description: omni wheels, collision classes, sensors. The ros2_control runtime has been installed since entry 4. Launch files in `robot_bringup` start it, alone or with Nav2 |
| [3. Add the Documentation Site](03-documentation.md) | This site: MkDocs Material, built by pixi and published to GitHub Pages |
| [4. Move to RoboStack Build Set _25](04-robostack-build-25.md) | From RoboStack build set `_21` to `_25`, pinned with `ros2-distro-mutex`. Installs `mujoco_ros2_control` 0.1.2 and `rmw_zenoh_cpp` 0.2.11, and moves OpenCV to 5.0 |
| [5. Repair Nav2 After the navigation2 Bump](05-nav2-repair.md) | Fixes the Nav2 launches for upstream `navigation2`: MPPI plugin key, lifecycle managers, no composition, loopback and hero configurations. Nav2 commands now pass the velocity smoother and the collision monitor, which stops and slows the robot. The stop is direction-aware, so Nav2 can drive away after a stop. A new `dual_laser_merger` parameter, `publish_on_empty_cloud`, gives `/scan` in the empty MuJoCo world |
| [6. Select Zenoh in Every Pixi Command](06-zenoh-in-pixi.md) | `pixi.toml` sets `RMW_IMPLEMENTATION=rmw_zenoh_cpp`, so every `pixi run` and `pixi shell` uses Zenoh, and an exported value no longer changes it. CI starts a Zenoh router before the tests |

## Adding an entry

Add an entry when you make a change that others will need to understand later,
such as a new simulator, a new build tool, or a new hardware interface.

1. Create `docs/migrations/NN-short-name.md` with the next number.
2. Start with a **Goal:** line, then explain the steps in order and the reason
   for each one.
3. List the commits or pull requests involved, and what is still unfinished.
4. Add the page to this table and to `nav` in `mkdocs.yml`.
