#!/bin/bash
# Launch the "walkie" tmux dev session (mirrors the hand-built session 0 layout).
#
#   ros-ws : 4x2 grid, each pane has a command TYPED (not run) — press Enter to run.
#   editor : left idle bash, right runs `lazygit`.
#   claude : runs `claude`.
#
# Works from a bare terminal OR from inside tmux. Re-running just re-attaches.
# Set WALKIE_TMUX_NO_ATTACH=1 to build the session without attaching (for testing).

set -u

S=walkie
WS="$HOME/robocup2026/walkie-ws"
ROS="$WS/src/walkie-ros"

# Row-major 4-col x 2-row layout (pane 1..4 = top row, 5..8 = bottom row).
LAYOUT='0494,184x44,0,0[184x22,0,0{46x22,0,0,0,45x22,47,0,1,45x22,93,0,2,45x22,139,0,3},184x21,0,23{46x21,0,23,4,45x21,47,23,5,45x21,93,23,6,45x21,139,23,7}]'

attach() {
  [ -n "${WALKIE_TMUX_NO_ATTACH:-}" ] && return 0
  if [ -n "${TMUX:-}" ]; then
    tmux switch-client -t "$S"
  else
    tmux attach -t "$S"
  fi
}

# Already running? just attach.
if tmux has-session -t "$S" 2>/dev/null; then
  attach
  exit 0
fi

# Build the whole session. tmux prints a harmless "no current client" to stderr
# when commands run before any client is attached; collect stderr and filter it
# out afterwards (a temp file avoids process-substitution pipe deadlocks).
ERR="$(mktemp)"
{
# --- Window 1: ros-ws (8-pane 4x2 grid) ---
tmux new-session -d -s "$S" -n ros-ws -c "$WS" -x 184 -y 44
for _ in 1 2 3 4 5 6 7; do
  # Re-tile after each split so panes stay evenly sized (avoids "no space for new pane").
  tmux split-window -t "$S:ros-ws" -c "$WS"
  tmux select-layout -t "$S:ros-ws" tiled
done
tmux select-layout -t "$S:ros-ws" "$LAYOUT"

# Pane ids in creation order == reading order after the row-major layout.
mapfile -t P < <(tmux list-panes -t "$S:ros-ws" -F '#{pane_id}')

# Commands TYPED but not run (no Enter). -l sends the string literally.
tmux send-keys -t "${P[0]}" -l 'ros2 run rmw_zenoh_cpp rmw_zenohd'
tmux send-keys -t "${P[1]}" -l 'ros2 launch robot_bringup lift_homing.launch.py'
tmux send-keys -t "${P[2]}" -l 'ros2 launch robot_bringup real_omnibot.launch.py'
tmux send-keys -t "${P[3]}" -l 'ros2 launch robot_navigation localization_Nav2_real.launch.py'
tmux send-keys -t "${P[4]}" -l 'ros2 launch openarm_bimanual_moveit_config moveit_only.launch.py hardware_type:=real_robot'
tmux send-keys -t "${P[5]}" -l 'ros2 launch openarm_bimanual_commander_cpp commander.launch.py'
tmux send-keys -t "${P[6]}" -l 'ros2 launch joy_interface joy_control_smooth.launch.py'
# P[7] left idle.

# --- Window 2: editor (left idle, right runs lazygit) ---
tmux new-window -t "$S" -n editor -c "$ROS"
tmux split-window -h -t "$S:editor" -c "$ROS"
tmux select-layout -t "$S:editor" even-horizontal
mapfile -t E < <(tmux list-panes -t "$S:editor" -F '#{pane_id}')
tmux send-keys -t "${E[1]}" -l 'lazygit'
tmux send-keys -t "${E[1]}" Enter

# --- Window 3: claude ---
tmux new-window -t "$S" -n claude -c "$ROS"
tmux send-keys -t "$S:claude" -l 'claude'
tmux send-keys -t "$S:claude" Enter

tmux select-window -t "$S:ros-ws"
} 2>"$ERR"
grep -v 'no current client' "$ERR" >&2 || true
rm -f "$ERR"

attach
