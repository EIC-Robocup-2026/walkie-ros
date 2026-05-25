cmake_minimum_required(VERSION 3.8)
project(openarm_bimanual_commander_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(my_robot_interfaces REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(commander src/commander_template.cpp)
ament_target_dependencies(commander
  rclcpp
  moveit_ros_planning_interface
  example_interfaces
  my_robot_interfaces
  rclcpp_action
  control_msgs
  moveit_msgs
  moveit_core
)

add_executable(pick_and_place_isaac src/pick_and_place_isaac.cpp)
ament_target_dependencies(pick_and_place_isaac
  rclcpp
  rclcpp_action
  tf2_ros
  tf2_msgs
  my_robot_interfaces
  moveit_ros_planning_interface
  moveit_msgs
)

add_executable(pick_and_place_isaac2 src/pick_and_place_isaac2.cpp)
ament_target_dependencies(pick_and_place_isaac2
  rclcpp
  rclcpp_action
  tf2_ros
  tf2_msgs
  my_robot_interfaces
  moveit_ros_planning_interface
  moveit_msgs
  visualization_msgs
)

install(TARGETS commander pick_and_place_isaac pick_and_place_isaac2
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  src/commander_interactive.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME commander_interactive
)

install(PROGRAMS
  src/commander_benchmark.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME commander_benchmark
)

ament_package()
