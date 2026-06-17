#!/usr/bin/env bash
# Real-robot move_group, GPU planning. Launches move_group (ompl + cumotion) +
# the cuMotion planner INSIDE the Isaac container, under rmw_zenoh so it JOINS
# this host's zenoh router (the container is --network host, so the host
# rmw_zenohd on localhost is reachable). It then shares the ROS graph with the
# host's controllers/commander/joint_states and executes plans on the real arms.
#
# This REPLACES the host `moveit_only.launch.py` — run exactly ONE move_group.
# The commander selects the planner per request:
#   ros2 param set /bimanual_commander planning_pipeline cumotion   # GPU
#   ros2 param set /bimanual_commander planning_pipeline ""         # OMPL fallback
#
# Prereqs: host rmw_zenohd running; container isaac_ros_dev_container up with the
# walkie overlay built (/tmp/wi) and rmw_zenoh installed (it is, in the image).
set -euo pipefail

CONTAINER=isaac_ros_dev_container
INNER='RMW=zenoh /workspaces/isaac_ros-dev/openarm_cumotion/launch_cumotion_stack.sh \
  move_group_cumotion.launch.py \
  cumotion_urdf:=/workspaces/isaac_ros-dev/openarm_cumotion/openarm_bimanual.urdf'

# Use docker directly if the shell has the group, else via `sg docker`.
if docker ps >/dev/null 2>&1; then
  exec docker exec "$CONTAINER" bash -lc "$INNER"
else
  exec sg docker -c "docker exec $CONTAINER bash -lc '$INNER'"
fi
