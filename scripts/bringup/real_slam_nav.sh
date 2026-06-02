#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
WS_ROOT="$SCRIPT_DIR"
while [[ "$WS_ROOT" != "/" && ! -d "$WS_ROOT/install" ]]; do
    WS_ROOT="$(dirname "$WS_ROOT")"
done

if [[ ! -d "$WS_ROOT/install" ]]; then
    echo "Error: Could not find workspace root (containing 'install' folder)."
    exit 1
fi

if [ -z "$ROS_DISTRO" ]; then
    for distro in jazzy humble foxy; do
        if [ -d "/opt/ros/$distro" ]; then
            SELECTED_DISTRO=$distro
            break
        fi
    done
else
    SELECTED_DISTRO=$ROS_DISTRO
fi

SOURCE_CMD="source /opt/ros/$SELECTED_DISTRO/setup.bash && source $WS_ROOT/install/setup.bash"

echo "Launching Navigation and SLAM in separate tabs..."
gnome-terminal --tab --title="NAVIGATION" -- bash -c "$SOURCE_CMD && ros2 launch robot_navigation localization_Nav2_stvl_real.launch.py; exec bash"
gnome-terminal --tab --title="SLAM" -- bash -c "sleep 2 && $SOURCE_CMD && ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false; exec bash"