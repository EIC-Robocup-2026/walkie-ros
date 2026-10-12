# Migration Log

This section records the big changes to the workspace, in order. Each entry
explains **what** the team changed, **why**, and, where recorded, which
alternatives were rejected. The guides tell you how to use the workspace today; the log tells
you how it got that way.

Read an entry before you change the same parts. A setting that looks odd, such
as a capsule-shaped wheel or a version pin, usually has a reason recorded here.

| Entry | What changed |
| --- | --- |
| [1. Move to Pixi](01-pixi.md) | The workspace moved from a system ROS installation (apt, rosdep, colcon) to a project-local pixi environment |
| [2. Add MuJoCo Simulation](02-mujoco/index.md) | A MuJoCo model of Walkie, generated from the robot description, with omni wheels, collision classes, and sensors. The ros2_control runtime is not working yet |
| [3. Add the Documentation Site](03-documentation.md) | This site: MkDocs Material, built by pixi and published to GitHub Pages |

## Adding an entry

When you make a change that others will need to understand later, such as a
new simulator, a new build tool, or a new hardware interface, add an entry:

1. Create `docs/migrations/NN-short-name.md` with the next number.
2. Start with a **Goal:** line, then explain the steps in order and the reason
   for each one.
3. List the commits or pull requests involved, and what is still unfinished.
4. Add the page to this table and to `nav` in `mkdocs.yml`.
