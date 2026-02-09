SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_JSON="$SCRIPT_DIR/WALKIE_RMW_ZENOH_ROUTER_CONFIG.json5"

#!/usr/bin/env bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CONFIG_URI="$CONFIG_JSON"
export ROS_DOMAIN_ID=23
echo "Exported ZENOH_ROUTER_CONFIG_URI=$ZENOH_ROUTER_CONFIG_URI"

echo "Starting zenoh router via: ros2 run rmw_zenoh_cpp rmw_zenohd"
ros2 run rmw_zenoh_cpp rmw_zenohd
