#!/usr/bin/env bash
# Open RViz on the HOST to visualize cuMotion plans running in the container.
#
# The container's move_group/cuMotion stack must already be up
# (./run_cumotion_demo.sh). RViz runs on FastDDS to match the container's RMW and
# discovers move_group over the shared network. The robot model is generated here
# with package:// mesh paths so meshes render on the host.
#
#   ./view_cumotion.sh
#
# Then in RViz:
#   1. Fixed Frame = base_footprint
#   2. Add -> MotionPlanning   (auto-loads from the robot_description params)
#   3. MotionPlanning panel -> Planning tab:
#        Planning Group   = both_arms
#        (Context tab) Planning Pipeline = isaac_ros_cumotion   (or ompl)
#   4. Drag the goal markers (or use a named/random goal) -> Plan
#      -> the planned trajectory animates on /display_planned_path.
#   5. (optional) Add -> MarkerArray, topic /cumotion/collision_spheres
#      to see the GPU collision spheres.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
CFG="$(cd "$HERE/.." && pwd)"
REPO="$(cd "$HERE/../../../.." && pwd)"     # .../walkie-ros
WS_ROOT="$(cd "$REPO/../.." && pwd)"        # .../walkie_ws

source /opt/ros/jazzy/setup.bash
[ -f "$WS_ROOT/install/setup.bash" ] && source "$WS_ROOT/install/setup.bash"

# Match the container's RMW (FastDDS) so RViz can see its move_group.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_DOMAIN_ID 2>/dev/null || true   # default domain 0, same as container
# Force UDP transport: FastDDS's default shared-memory transport does NOT work
# across the host<->container boundary (discovery succeeds but topic DATA never
# arrives), so RViz would render the default pose. UDP delivers the live state.
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

echo "[*] generating host URDF..."
xacro "$CFG/openarm_bimanual.urdf.xacro" > /tmp/oa_view.urdf
# walkie_description's meshes are NOT installed to its share/ dir, so package://
# won't resolve in RViz. Point straight at the source meshes instead.
sed -i "s#package://walkie_description/#file://$REPO/description/#g" /tmp/oa_view.urdf
echo "[*] mesh URIs -> file://$REPO/description/"

if ! timeout 8 ros2 node list 2>/dev/null | grep -q move_group; then
  echo "[!] move_group not visible. Start the stack first:"
  echo "      $HERE/run_cumotion_demo.sh"
  echo "    (continuing to open RViz anyway)"
fi

echo "[*] launching RViz (RMW=$RMW_IMPLEMENTATION)..."
ros2 launch "$HERE/view_cumotion_host.launch.py"
