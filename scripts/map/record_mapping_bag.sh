#!/bin/bash
# Record a rosbag while driving during mapping, for OFFLINE rebuild of the
# slam_toolbox 2D map and the octomap 3D map after a match.
#
# Records three things in one bag:
#   * SLAM/octomap INPUTS  -> full re-tuneable rebuild (re-run slam / octomap offline)
#   * live map OUTPUTS     -> pull the map exactly as it was at any time t (no re-run)
#
# See scripts/map/README.md for the record / playback / snapshot recipes.
#
# Usage:
#   ./record_mapping_bag.sh [OUTPUT_DIR]     # default: ~/walkie_mapping_bags/mapping_<timestamp>
#   ./record_mapping_bag.sh --heavy          # also record /octomap_full + voxel-centers cloud
#   ./record_mapping_bag.sh -h               # help
#
# Env (must match the running robot bringup):
#   RMW_IMPLEMENTATION  default rmw_zenoh_cpp
#   ROS_DOMAIN_ID       default 23 (matches connect_walkie_zenoh_router.sh)
set -u

# --- locate + source the workspace (same pattern as scripts/bringup/*.sh) ----
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
WS_ROOT="$SCRIPT_DIR"
while [[ "$WS_ROOT" != "/" && ! -d "$WS_ROOT/install" ]]; do
    WS_ROOT="$(dirname "$WS_ROOT")"
done
if [[ ! -d "$WS_ROOT/install" ]]; then
    echo "Error: could not find workspace root (folder containing 'install')." >&2
    exit 1
fi

# ROS setup.bash trips over `set -u` (unbound AMENT_* vars), so relax it here.
set +u
if [ -f "/opt/ros/${ROS_DISTRO:-}/setup.bash" ]; then
    source "/opt/ros/$ROS_DISTRO/setup.bash"
else
    for distro in jazzy humble foxy; do
        if [ -f "/opt/ros/$distro/setup.bash" ]; then
            source "/opt/ros/$distro/setup.bash"
            break
        fi
    done
fi
source "$WS_ROOT/install/setup.bash"
set -u

# Match the robot's middleware + domain so we actually see its topics.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-23}"

usage() {
    cat <<'EOF'
Record a mapping rosbag (for offline slam + octomap rebuild).

Usage:
  record_mapping_bag.sh [OUTPUT_DIR]   Record to OUTPUT_DIR
                                       (default: ~/walkie_mapping_bags/mapping_<timestamp>)
  record_mapping_bag.sh --heavy        Also record /octomap_full + /octomap_point_cloud_centers
  record_mapping_bag.sh -h | --help    This help

Run the robot bringup + both mapping launches first, then run this and drive.
See the README next to this script for playback / snapshot-at-time-t recipes.
EOF
}

# --- args ------------------------------------------------------------------
HEAVY=0
OUT=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        -h|--help)   usage; exit 0 ;;
        --heavy)     HEAVY=1; shift ;;
        -o|--output) OUT="${2:-}"; shift 2 ;;
        -*)          echo "Unknown option: $1" >&2; usage; exit 2 ;;
        *)           OUT="$1"; shift ;;
    esac
done
: "${OUT:=$HOME/walkie_mapping_bags/mapping_$(date +%Y%m%d_%H%M%S)}"

# --- topics ----------------------------------------------------------------
TOPICS=(
    # transforms (needed by both pipelines)
    /tf /tf_static
    # SLAM inputs
    /scan /front_lidar /back_lidar /omni_wheel_drive_controller/odom
    # Octomap input: the merged, self-filtered cloud (heavy)
    /octomap/cloud_in
    # live map OUTPUTS -- "give me the map at time t"
    /map /map_metadata /octomap_binary /projected_map
)
if [[ $HEAVY -eq 1 ]]; then
    TOPICS+=( /octomap_full /octomap_point_cloud_centers )
fi

# --- preflight: are the key topics actually live? --------------------------
echo "Checking live topics (RMW=$RMW_IMPLEMENTATION, ROS_DOMAIN_ID=$ROS_DOMAIN_ID)..."
LIVE="$(timeout 10 ros2 topic list 2>/dev/null)"
if [[ -z "$LIVE" ]]; then
    echo "WARNING: no topics visible. Is the Zenoh router + robot bringup running," >&2
    echo "         and does ROS_DOMAIN_ID=$ROS_DOMAIN_ID match the robot?" >&2
else
    for t in /scan /octomap/cloud_in /map /octomap_binary; do
        grep -qx "$t" <<<"$LIVE" || echo "WARNING: '$t' is not being published yet." >&2
    done
fi

mkdir -p "$(dirname "$OUT")"
echo
echo "Recording to: $OUT"
echo "Topics:"
printf '  %s\n' "${TOPICS[@]}"
echo
echo "Drive the robot to map the arena. Press Ctrl-C to stop and close the bag."
echo

# ros2 bag record handles SIGINT itself (closes the bag cleanly). Keep this
# script alive through the Ctrl-C so we can print the follow-up hints after.
trap ':' INT
ros2 bag record --storage mcap \
    --compression-mode message --compression-format zstd \
    -o "$OUT" \
    "${TOPICS[@]}"
trap - INT

echo
echo "Bag saved: $OUT"
echo "Inspect:   ros2 bag info \"$OUT\""
echo "Playback / snapshot recipes: $SCRIPT_DIR/README.md"
