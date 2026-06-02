#!/bin/bash
set -e

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

# Source ROS 2 (prefer existing ROS_DISTRO, otherwise detect)
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    source "/opt/ros/$ROS_DISTRO/setup.bash"
else
    # Fallback/Detect common versions
    for distro in jazzy humble foxy; do
        if [ -f "/opt/ros/$distro/setup.bash" ]; then
            source "/opt/ros/$distro/setup.bash"
            break
        fi
    done
fi

# Source the workspace
source "$WS_ROOT/install/setup.bash"

# Check if rmw_zenohd is already running
if ! pgrep -x "rmw_zenohd" > /dev/null
then
    # Start rmw_zenohd in the background without verbosity
    ros2 run rmw_zenoh_cpp rmw_zenohd >/dev/null 2>&1 &
fi

# Check if Foxglove ROS Bridge is already running
if ! pgrep -f "foxglove_bridge" > /dev/null
then
    # Start the Foxglove ROS Bridge in the background on port 8765
    ros2 run foxglove_bridge foxglove_bridge foxglove_bridge_launch.xml >/dev/null 2>&1 &
fi

ros2 launch robot_bringup isaac_omnibot.launch.py
