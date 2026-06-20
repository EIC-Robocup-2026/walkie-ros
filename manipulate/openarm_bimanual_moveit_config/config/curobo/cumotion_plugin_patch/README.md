# Patched cuMotion MoveIt plugin (isaac_ros_cumotion_moveit 4.4.0)

cuMotion's stock MoveIt plugin can't be driven by standard MoveIt clients
(RViz / `MoveGroupInterface` / the bimanual commander) on this multi-joint robot,
and its trajectories render/validate wrong. These patched source files fix that.
They are **container-only** (the cuMotion source isn't in this repo's git), so
they're captured here to make another machine reproducible.

`cumotion_move_group_client.cpp` and `cumotion_interface.cpp` belong to the
**`isaac_ros_cumotion_moveit`** package (move_group plugin) and replace the
originals at:
`/workspaces/isaac_ros-dev/src/isaac_ros_cumotion/isaac_ros_cumotion_moveit/src/`

`robot_manager_impl.cpp` belongs to the **`isaac_ros_cumotion`** package (the GPU
planner node) and replaces the original at:
`/workspaces/isaac_ros-dev/src/isaac_ros_cumotion/isaac_ros_cumotion/src/impl/`

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
  Also drops the redundant `result_callback` registration: `getGoal()` already
  pulls the action result via `async_get_result()`'s future on the planning
  thread, so the registered callback was a second writer of `plan_response`
  (a `std::vector` free+realloc) on a multi-threaded-executor thread. On
  successful plans the two raced into a heap double-free and move_group aborted
  with `malloc_consolidate(): invalid chunk size`. (This change is in
  `cumotion_move_group_client.cpp`.)
- **`robot_manager_impl.cpp`** — on a runtime `SetRobotDescription` reload, if the
  latched `tool_frame_` isn't in the new description, re-latch to its first tool
  frame (warn) instead of `FATAL`. `tool_frame_` is never read from a param; it
  latches to the first-loaded description's `tool_frames[0]` (the bimanual xrdf's
  `openarm_left_hand`), so swapping to a single-arm xrdf whose only tool is the
  other hand used to kill the planner with
  `Specified tool frame 'openarm_left_hand' not found`. Now `set_xrdf.py` can swap
  freely across arms.

## How to apply on another machine

```bash
# inside the Isaac ROS container, with the cuMotion source present:
D=/workspaces/isaac_ros-dev/src/isaac_ros_cumotion
cp cumotion_move_group_client.cpp "$D/isaac_ros_cumotion_moveit/src/"
cp cumotion_interface.cpp        "$D/isaac_ros_cumotion_moveit/src/"
cp robot_manager_impl.cpp        "$D/isaac_ros_cumotion/src/impl/"

cd /workspaces/isaac_ros-dev
colcon build --packages-select isaac_ros_cumotion_moveit isaac_ros_cumotion  # overlay in install/
# then source install/setup.bash AFTER /opt/ros/jazzy so move_group + the planner
# node load the patched .so files
```

Verify the patched `.so` is the one loaded:
`cat /proc/$(pgrep -x move_group)/maps | grep libisaac_ros_cumotion_moveit.so`
should point at `/workspaces/isaac_ros-dev/install/...`, not `/opt/ros/jazzy/lib`.

> Version note: captured against `isaac_ros_cumotion_moveit 4.4.0`. If the
> container's cuMotion version differs, re-apply the two changes by hand (they're
> small and clearly commented in these files) rather than copying verbatim.
