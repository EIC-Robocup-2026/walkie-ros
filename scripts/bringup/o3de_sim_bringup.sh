#!/bin/bash
# ==============================================================================
# O3DE Simulation Bringup Script (Walkie Robot) - Base Bringup Only
# Launches O3DE ROS 2 interface (RSP, ros2_control, rosbridge) WITHOUT Nav2/MoveIt.
# Allows running Navigation, MoveIt, or custom nodes independently in separate terminals.
# ==============================================================================

set -e

# Ensure DISPLAY and X11/Qt authentication for GUI tools (if rviz is enabled)
export DISPLAY="${DISPLAY:-:0}"
if [ -z "$XAUTHORITY" ] || [ ! -f "$XAUTHORITY" ]; then
    for f in /run/user/$(id -u)/xauth_*; do
        if [ -f "$f" ]; then
            export XAUTHORITY="$f"
            ln -sf "$f" "$HOME/.Xauthority" 2>/dev/null || true
            break
        fi
    done
fi
if [ -z "$XAUTHORITY" ] && [ -f "$HOME/.Xauthority" ]; then
    export XAUTHORITY="$HOME/.Xauthority"
fi
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"

# Get the directory where the script is located
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"

# Find the workspace root by looking for the 'install' directory upwards
WS_ROOT="$SCRIPT_DIR"
while [[ "$WS_ROOT" != "/" && ! -d "$WS_ROOT/install" ]]; do
    WS_ROOT="$(dirname "$WS_ROOT")"
done

if [[ ! -d "$WS_ROOT/install" ]]; then
    echo "Error: Could not find workspace root (containing 'install' folder)."
    exit 1
fi

# 1. Source Pixi / ROS 2 Environment
if [ -f "$WS_ROOT/.pixi/envs/default/setup.bash" ]; then
    source "$WS_ROOT/.pixi/envs/default/setup.bash"
elif [ -f "/home/armmy2530/O3DE/Projects/testROS2/ros2_ws/.pixi/envs/default/setup.bash" ]; then
    source "/home/armmy2530/O3DE/Projects/testROS2/ros2_ws/.pixi/envs/default/setup.bash"
elif [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    source "/opt/ros/$ROS_DISTRO/setup.bash"
fi

# 2. Source workspace overlay
source "$WS_ROOT/install/setup.bash"

# 3. Check and start rmw_zenohd if needed
if ! pgrep -x "rmw_zenohd" > /dev/null; then
    echo "Starting rmw_zenohd router..."
    ros2 run rmw_zenoh_cpp rmw_zenohd >/dev/null 2>&1 &
    sleep 1
fi

USE_RVIZ="false"
for arg in "$@"; do
    case $arg in
        --rviz)
            USE_RVIZ="true"
            ;;
        --no-rviz)
            USE_RVIZ="false"
            ;;
        --help|-h)
            echo "Usage: $0 [options]"
            echo "Options:"
            echo "  --rviz       Launch RViz2 with default config"
            echo "  --no-rviz    Do not launch RViz2 (default)"
            exit 0
            ;;
    esac
done

echo "=========================================================="
echo " Starting O3DE Walkie Base Bringup (Simulation Interface Only)"
echo "   RViz2: $USE_RVIZ"
echo "=========================================================="

ros2 launch robot_bringup o3de_omnibot.launch.py use_rviz:="$USE_RVIZ"
