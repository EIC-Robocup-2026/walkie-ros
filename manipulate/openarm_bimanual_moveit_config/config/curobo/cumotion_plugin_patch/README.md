# Patched cuMotion MoveIt plugin (isaac_ros_cumotion_moveit 4.4.0)

cuMotion's stock MoveIt plugin can't be driven by standard MoveIt clients
(RViz / `MoveGroupInterface` / the bimanual commander) on this multi-joint robot,
and its trajectories render/validate wrong. These two patched source files fix
both. They are **container-only** (the cuMotion source isn't in this repo's git),
so they're captured here to make another machine reproducible.

These files replace the originals in the Isaac ROS container at:
`/workspaces/isaac_ros-dev/src/isaac_ros_cumotion/isaac_ros_cumotion_moveit/src/`

## What each patch does

- **`cumotion_move_group_client.cpp`** — in `updateGoal`, prunes
  `start_state.joint_state` down to the planning group's active joints. The stock
  plugin forwards the FULL robot state, so cuRobo fails
  `cspace_position [24] must equal [14]` for any client that sends a full
  RobotState (RViz, MoveGroupInterface). Pruning lets them drive cuMotion by just
  selecting the pipeline.
- **`cumotion_interface.cpp`** — seeds the response reference `RobotState` from
  `planning_scene->getCurrentState()` (full robot) instead of a zero state, so
  `ValidateSolution` (base/torso mesh check) doesn't false-reject on zeroed
  non-group joints, and RViz renders the trajectory at the correct lift height.

## How to apply on another machine

```bash
# inside the Isaac ROS container, with the cuMotion source present:
D=/workspaces/isaac_ros-dev/src/isaac_ros_cumotion/isaac_ros_cumotion_moveit/src
cp cumotion_move_group_client.cpp "$D/"
cp cumotion_interface.cpp        "$D/"

cd /workspaces/isaac_ros-dev
colcon build --packages-select isaac_ros_cumotion_moveit   # builds the overlay in install/
# then source install/setup.bash AFTER /opt/ros/jazzy so move_group loads the patched .so
```

Verify the patched `.so` is the one loaded:
`cat /proc/$(pgrep -x move_group)/maps | grep libisaac_ros_cumotion_moveit.so`
should point at `/workspaces/isaac_ros-dev/install/...`, not `/opt/ros/jazzy/lib`.

> Version note: captured against `isaac_ros_cumotion_moveit 4.4.0`. If the
> container's cuMotion version differs, re-apply the two changes by hand (they're
> small and clearly commented in these files) rather than copying verbatim.
