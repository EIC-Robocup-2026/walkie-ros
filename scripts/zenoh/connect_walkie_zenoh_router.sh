#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_JSON="$SCRIPT_DIR/WALKIE_RMW_ZENOH_ROUTER_CONFIG.json5"

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CONFIG_URI="$CONFIG_JSON"
echo "Exported ZENOH_ROUTER_CONFIG_URI=$ZENOH_ROUTER_CONFIG_URI"

TARGET_IP=$1

if [ -n "$TARGET_IP" ]; then
    echo "Starting zenoh router connecting to $TARGET_IP via: ros2 run rmw_zenoh_cpp rmw_zenohd -- -e tcp/$TARGET_IP:7447"
    ros2 run rmw_zenoh_cpp rmw_zenohd -- -e tcp/$TARGET_IP:7447
else
    echo "Starting zenoh router via: ros2 run rmw_zenoh_cpp rmw_zenohd"
    ros2 run rmw_zenoh_cpp rmw_zenohd
fi
