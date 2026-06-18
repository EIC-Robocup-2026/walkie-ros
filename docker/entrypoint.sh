#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash 
cd /ros2_ws   
#  apt update && 
#  rosdep update 
#  rosdep install --from-paths src -y -i  
source /home/zed_deps_ws/install/setup.bash 
colcon build --symlink-install 
source install/setup.bash 
cd src/walkie-ros/bringup
echo '=========== uv sync =================='
uv sync 
cd /ros2_ws  
sleep infinity
