#!/usr/bin/env bash
# Clean (re)launch of a cuMotion MoveIt stack INSIDE the Isaac ROS container.
#
# Always kills any existing stack FIRST, then sources the three overlays and
# launches exactly one. This prevents the duplicate-move_group trap: a second
# move_group can load the UNPATCHED apt cuMotion plugin, and RViz/clients that
# hit it fail with "cspace_position [24] must equal [14]" (the start state isn't
# pruned to the group's cspace). One patched move_group = full-state requests
# (RViz interactive markers, the commander) plan fine.
#
# Run this from a shell INSIDE the container:
#   docker exec -it isaac_ros_dev_container bash
#   /workspaces/isaac_ros-dev/openarm_cumotion/launch_cumotion_stack.sh [launch_file] [extra ros2 launch args...]
#
# Defaults to the full mock + commander + RViz demo, under FastDDS. Examples:
#   launch_cumotion_stack.sh                                  # demo (mock+commander+rviz)
#   launch_cumotion_stack.sh moveit_only_cumotion.launch.py   # move_group + rviz only
#   launch_cumotion_stack.sh moveit_demo_with_commander_cumotion.launch.py cumotion_xrdf:=/path/left_arm.xrdf
#
# REAL ROBOT (host hardware): run under zenoh so move_group/cuMotion join the
# host's zenoh router and execute on the real controllers (host must be running
# rmw_zenohd; do NOT start a host move_group). Use move_group_cumotion (no mock
# control / no commander -> those live on the host):
#   RMW=zenoh launch_cumotion_stack.sh move_group_cumotion.launch.py \
#       cumotion_urdf:=/workspaces/isaac_ros-dev/openarm_cumotion/openarm_bimanual.urdf
#
# NOTE: no `set -u` — sourcing the ROS env scripts references unbound vars
# (AMENT_TRACE_SETUP_FILES) and would abort the script before it launches.
set -o pipefail

LAUNCH_FILE="${1:-moveit_demo_with_commander_cumotion.launch.py}"
[ "$#" -gt 0 ] && shift   # remaining args ($@) pass through to ros2 launch

# --- kill any existing stack (bracket trick avoids matching this script) ------
echo "[*] killing any existing cuMotion/MoveIt stack..."
for pat in "[r]os2 launch" "[m]ove_group" "[c]umotion_container" "[c]omponent_container" \
           "[r]os2_control_node" "[b]imanual_commander" "[r]viz2" "[s]pawner" \
           "[r]obot_state_publisher" "[s]tatic_transform_publisher"; do
  pkill -9 -f "$pat" 2>/dev/null || true
done

# Count only LIVE (non-zombie) move_groups: this container's PID 1 is
# `sleep infinity`, which never reaps orphans, so killed move_groups linger as
# unreapable ZOMBIES (state Z). Counting them would make this loop time out
# forever and the report misleading, so filter them out.
live_move_group() {
  local n=0 p st
  for p in $(pgrep -x move_group 2>/dev/null); do
    st=$(awk '/^State:/{print $2}' /proc/"$p"/status 2>/dev/null)
    [ "$st" != "Z" ] && n=$((n + 1))
  done
  echo "$n"
}
# wait until the key nodes are gone (live move_group + cumotion container).
for _ in $(seq 1 15); do
  cm=$(pgrep -fc "[c]umotion_container" 2>/dev/null || echo 0)
  [ "$(live_move_group)" = "0" ] && [ "$cm" = "0" ] && break
  sleep 1
done
sleep 1
echo "    live move_group: $(live_move_group)  cumotion_container: $(pgrep -fc '[c]umotion_container' 2>/dev/null || echo 0)  (zombies ignored)"

# --- RMW selection -----------------------------------------------------------
# RMW=zenoh   -> rmw_zenoh_cpp: the stack JOINS the host's zenoh router (the
#               container is --network host, so rmw_zenohd on localhost:7447 is
#               reachable). Use this for the REAL ROBOT so move_group/cuMotion
#               share the graph with the host controllers/commander/joint_states.
#               The host must be running rmw_zenohd. NOTE: do NOT also start a
#               move_group on the host (one move_group only).
# RMW=fastdds -> rmw_fastrtps_cpp (default): self-contained in-container stack
#               (the mock demo). Does NOT interoperate with a host zenoh graph.
case "${RMW:-fastdds}" in
  zenoh)   export RMW_IMPLEMENTATION=rmw_zenoh_cpp ;;
  fastdds) export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ;;
  *)       export RMW_IMPLEMENTATION="${RMW}" ;;   # pass through an explicit value
esac
echo "[*] RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
[ "$RMW_IMPLEMENTATION" = "rmw_zenoh_cpp" ] && \
  echo "    (joining host zenoh router — ensure host rmw_zenohd is up and the host move_group is NOT running)"

# --- source overlays: jazzy -> patched cuMotion plugin -> walkie packages -----
source /opt/ros/jazzy/setup.bash
[ -f /workspaces/isaac_ros-dev/install/setup.bash ] && source /workspaces/isaac_ros-dev/install/setup.bash
[ -f /tmp/wi/setup.bash ] && source /tmp/wi/setup.bash

# confirm the PATCHED plugin will be the one found (overlay before apt)
PLUGIN=$(ros2 pkg prefix isaac_ros_cumotion_moveit 2>/dev/null || true)
echo "[*] isaac_ros_cumotion_moveit prefix: ${PLUGIN:-<not found>}"
case "$PLUGIN" in
  /workspaces/isaac_ros-dev/install/*) echo "    -> PATCHED overlay (good)";;
  *) echo "    WARN: not the patched overlay; full-state/RViz requests may fail [24]!=[14]";;
esac

echo "[*] launching: $LAUNCH_FILE $*"
exec ros2 launch openarm_bimanual_moveit_config "$LAUNCH_FILE" "$@"
