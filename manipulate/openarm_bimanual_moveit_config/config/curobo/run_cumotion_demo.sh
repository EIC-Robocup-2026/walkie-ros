#!/usr/bin/env bash
# Run the cuMotion OpenArm planning demo end-to-end.
#
# Brings up move_group + the cuMotion planner inside the Isaac ROS container,
# publishes a valid arms-raised start pose, and requests a GPU plan for the
# 14-DOF both_arms group.
#
# Usage:
#   ./run_cumotion_demo.sh              # full demo: launch stack + plan both_arms
#   ./run_cumotion_demo.sh plan [grp]   # just request a plan (stack already up)
#   ./run_cumotion_demo.sh stop         # stop the in-container ROS processes
#
# Notes:
#   - Use group "both_arms" (14 joints) — matches the XRDF cspace.
#   - From q=0 the arms fold into the base and the start is (correctly) rejected;
#     pub_js.py publishes the arms-raised home pose cuMotion plans from.
#   - An occasional "trajopt: UNKNOWN_STATUS" is a transient optimizer miss; re-run.
set -euo pipefail

CONTAINER=isaac_ros_dev_container
IMAGE=isaac_ros_cumotion:walkie
WS=/workspaces/isaac_ros-dev
APP="$WS/openarm_cumotion"
HOST_WS="${ISAAC_ROS_WS:-$HOME/workspaces/isaac_ros-dev}"

# Which XRDF the planner loads (selects the planning group). Override with env:
#   CUMOTION_XRDF=right_arm.xrdf ./run_cumotion_demo.sh   -> native right_arm
XRDF_FILE="${CUMOTION_XRDF:-openarm_bimanual.xrdf}"
case "$XRDF_FILE" in
  right_arm.xrdf)      GROUP=right_arm ;;
  left_arm.xrdf)       GROUP=left_arm ;;
  right_arm_lift.xrdf) GROUP=right_arm_lift ;;
  left_arm_lift.xrdf)  GROUP=left_arm_lift ;;
  both_arms_lift.xrdf) GROUP=both_arms_lift ;;
  *)                   GROUP=both_arms ;;
esac

# --- pick a docker invocation that works (direct, else via `sg docker`) ---
if docker ps >/dev/null 2>&1; then
  DOCKER() { docker "$@"; }
else
  DOCKER() { sg docker -c "docker $(printf '%q ' "$@")"; }
fi

# Run a command inside the container under a sourced ROS env. The local overlay
# (if built) is sourced AFTER jazzy so move_group loads the PATCHED
# isaac_ros_cumotion_moveit plugin (start-state pruned to the group's cspace),
# which is what lets RViz / MoveGroupInterface / the commander drive cuMotion
# without hand-building a minimal start state.
SRC='source /opt/ros/jazzy/setup.bash; [ -f /workspaces/isaac_ros-dev/install/setup.bash ] && source /workspaces/isaac_ros-dev/install/setup.bash'
dexec()    { DOCKER exec    "$CONTAINER" bash -lc "$SRC; $1"; }
dexec_bg() { DOCKER exec -d "$CONTAINER" bash -lc "$SRC; $1"; }

ensure_container() {
  if DOCKER ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then return; fi
  if DOCKER ps -a --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    echo "[*] starting existing container..."; DOCKER start "$CONTAINER" >/dev/null; return
  fi
  echo "[*] creating container from $IMAGE ..."
  DOCKER run -d --name "$CONTAINER" --gpus all --runtime nvidia \
    --network host --ipc host \
    -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v "$HOST_WS:$WS" --entrypoint /bin/bash "$IMAGE" -lc 'sleep infinity' >/dev/null
}

stop_stack() {
  echo "[*] stopping in-container ROS processes..."
  # Kill launch + nodes, then WAIT until pub_js is fully gone before returning.
  # Stale /joint_states publishers (orphaned pub_js) flicker the start state and
  # cause spurious INVALID_INITIAL failures, so a clean teardown is essential.
  # NOTE the [b]racket trick: e.g. pattern "[p]ub_js" matches the target procs
  # but NOT this killer shell's own command line (which literally contains
  # "[p]ub_js"), avoiding the pkill self-kill that left orphans behind.
  DOCKER exec "$CONTAINER" bash -lc '
    pkill -f "[r]os2 launch"; pkill -f "[c]omponent_container"; pkill -f "[m]ove_group"; pkill -f "[p]ub_js"
    for i in $(seq 1 10); do
      pgrep -f "[p]ub_js" >/dev/null || break
      sleep 1; pkill -9 -f "[p]ub_js"; pkill -9 -f "[c]omponent_container"
    done
    sleep 1
    echo "  remaining pub_js: $(pgrep -fc "[p]ub_js" 2>/dev/null || echo 0)"' || true
  echo "[*] done."
}

run_plan() {
  local grp="${1:-both_arms}"
  echo "[*] requesting plan for group: $grp"
  # The cuMotion MoveIt plugin can intermittently return PLANNING_FAILED before
  # dispatching a goal (planning-scene current-state race / single-goal action
  # server briefly busy) or hit a transient trajopt UNKNOWN_STATUS. Every request
  # that actually reaches the planner succeeds, so retry until success. We print
  # only the successful result plus how many tries it took.
  local out tries=0
  for attempt in $(seq 1 8); do
    tries=$attempt
    out=$(dexec "cd $WS; python3 openarm_cumotion/plan_test_openarm.py $grp" 2>&1 \
          | grep -iE 'group =|error_code|planning_time|trajectory points|final|NOT available|NO RESPONSE' || true)
    echo "$out" | grep -q 'error_code = 1' && break
    sleep 2
  done
  if echo "$out" | grep -q 'error_code = 1'; then
    echo "$out" | sed 's/^/    /'
    echo "    (succeeded on try $tries/8)"
  else
    echo "[!] no successful plan after 8 tries (see $APP/mg.log)"
  fi
}

launch_stack() {
  echo "[*] launching move_group + cuMotion planner (XRDF=$XRDF_FILE, group=$GROUP)..."
  dexec_bg "export CUMOTION_XRDF=$XRDF_FILE; cd $WS; ros2 launch openarm_cumotion/cumotion_openarm_validate.launch.py > $APP/mg.log 2>&1"
  echo -n "[*] waiting for planner to come up"
  for _ in $(seq 1 30); do
    if dexec "grep -aq 'MotionPlan action server initialized' $APP/mg.log 2>/dev/null"; then
      echo " ready."; break
    fi
    echo -n "."; sleep 5
  done
  if ! dexec "grep -aq 'MotionPlan action server initialized' $APP/mg.log 2>/dev/null"; then
    echo " TIMEOUT. Check $APP/mg.log"; exit 1
  fi
  # /joint_states (arms-raised pose) is published by pub_js, started by the
  # launch itself (respawn=True), so no separate process to babysit.
  echo "[*] waiting for /joint_states + state monitor to sync..."
  sleep 6
}

status_stack() {
  DOCKER exec "$CONTAINER" bash -lc '
    source /opt/ros/jazzy/setup.bash 2>/dev/null
    nodes=$(timeout 8 ros2 node list 2>/dev/null)
    mg=$(echo "$nodes" | grep -c "move_group$")
    cm=$(echo "$nodes" | grep -c cumotion_planner)
    pub=$(timeout 6 ros2 topic info /joint_states 2>/dev/null | grep -o "Publisher count: [0-9]*" | grep -o "[0-9]*")
    if [ "$mg" -ge 1 ] && [ "$cm" -ge 1 ]; then
      echo "STACK: UP  (move_group + cumotion_planner live; /joint_states publishers=${pub:-?})"
      if [ "${pub:-1}" != "1" ]; then
        echo "  WARN: ${pub} /joint_states publishers (expected 1); if planning is flaky: docker restart isaac_ros_dev_container"
      fi
    else
      echo "STACK: DOWN (move_group=$mg cumotion_planner=$cm)  -> start with: ./run_cumotion_demo.sh"
    fi
    true' || echo "STACK: container not reachable"
}

# Drive a plan via the MoveGroup ACTION so it publishes /display_planned_path,
# which RViz animates. (run_plan uses the service, which does not display.)
show_plan() {
  local grp="${1:-both_arms}"
  echo "[*] planning $grp via move_action (animates in RViz /display_planned_path)..."
  for attempt in $(seq 1 8); do
    out=$(dexec "cd $WS; python3 openarm_cumotion/move_action_test.py $grp" 2>&1 \
          | grep -iE 'move_action error_code|REJECTED|NOT available')
    echo "$out" | grep -q 'error_code=1' && { echo "  OK ($out)"; return; }
    sleep 2
  done
  echo "[!] no plan after 8 tries (see $APP/mg.log)"
}

launch_rviz() {
  echo "[*] launching RViz INSIDE the container (renders on your host screen)..."
  DOCKER exec isaac_ros_dev_container bash -lc 'pkill -f "[r]viz2"; true' || true
  dexec_bg "export DISPLAY=:0; cd $WS; ros2 launch openarm_cumotion/view_incontainer.launch.py > $APP/rviz.log 2>&1"
  echo "[*] RViz should appear shortly. Then drive a plan to animate it:"
  echo "      $0 show both_arms"
}

case "${1:-demo}" in
  stop)   ensure_container; stop_stack; DOCKER exec isaac_ros_dev_container bash -lc 'pkill -f "[r]viz2"; true' || true ;;
  status) ensure_container; status_stack ;;
  rviz)   ensure_container; launch_rviz ;;
  plan)   ensure_container; run_plan "${2:-$GROUP}" ;;
  show)   ensure_container; show_plan "${2:-$GROUP}" ;;
  demo|"")
    ensure_container
    stop_stack            # clean slate
    launch_stack
    run_plan "$GROUP"
    echo
    echo "[*] stack is still running."
    echo "    check status : $0 status"
    echo "    re-plan      : $0 plan [group]"
    echo "    show in RViz : $0 show [group]   (animates /display_planned_path)"
    echo "    stop         : $0 stop"
    ;;
  *) echo "usage: $0 [demo|status|rviz|plan [group]|show [group]|stop]"; exit 2 ;;
esac
