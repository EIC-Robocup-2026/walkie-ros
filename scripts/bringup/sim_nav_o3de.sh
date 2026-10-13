#!/bin/bash
# ==============================================================================
# O3DE Simulation Bringup Script (Walkie Robot)
# Launches O3DE ROS 2 interface with Nav2 Autonomous Navigation & MoveIt
# ==============================================================================

set -e

# Ensure DISPLAY and X11/Qt authentication for GUI tools (RViz2)
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

# Parse command line flags
USE_NAV="true"
USE_MOVEIT="true"
USE_RVIZ="true"
RVIZ_MODE="nav"

for arg in "$@"; do
    case $arg in
        --no-nav)
            USE_NAV="false"
            ;;
        --no-moveit)
            USE_MOVEIT="false"
            ;;
        --no-rviz)
            USE_RVIZ="false"
            ;;
        --moveit-rviz)
            RVIZ_MODE="moveit"
            ;;
        --nav-rviz)
            RVIZ_MODE="nav"
            ;;
        --base-only|--bringup-only)
            USE_NAV="false"
            USE_MOVEIT="false"
            USE_RVIZ="false"
            ;;
        --nav-only)
            USE_MOVEIT="false"
            RVIZ_MODE="nav"
            ;;
        --moveit-only)
            USE_NAV="false"
            RVIZ_MODE="moveit"
            ;;
        --help|-h)
            echo "Usage: $0 [options]"
            echo "Options:"
            echo "  --moveit-rviz   Launch RViz2 configured with MoveIt MotionPlanning (interactive arm control)"
            echo "  --nav-rviz      Launch RViz2 configured with Nav2 Navigation panel (default)"
            echo "  --bringup-only  Launch ONLY O3DE bridge & ros2_control (no Nav2, MoveIt, or RViz)"
            echo "  --base-only     Alias for --bringup-only"
            echo "  --nav-only      Launch O3DE bridge with Nav2 (no MoveIt)"
            echo "  --moveit-only   Launch O3DE bridge with MoveIt and MoveIt RViz (no Nav2)"
            echo "  --no-nav        Disable Nav2"
            echo "  --no-moveit     Disable MoveIt"
            echo "  --no-rviz       Disable RViz2 GUI"
            exit 0
            ;;
    esac
done

echo "=========================================================="
echo " Starting O3DE Walkie ROS 2 Integration"
echo "   Nav2:      $USE_NAV"
echo "   MoveIt:    $USE_MOVEIT"
echo "   RViz2:     $USE_RVIZ (mode: $RVIZ_MODE)"
echo "=========================================================="

ros2 launch robot_bringup nav_sim_o3de.launch.py \
    use_nav:="$USE_NAV" \
    use_moveit:="$USE_MOVEIT" \
    use_rviz:="$USE_RVIZ" \
    rviz_mode:="$RVIZ_MODE"
