#!/bin/bash

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

# Launch the simulation
ros2 launch robot_bringup nav_sim_omnibot.launch.py
