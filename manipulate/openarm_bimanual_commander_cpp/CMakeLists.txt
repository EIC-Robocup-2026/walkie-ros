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
find_package(moveit_ros_warehouse REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)

add_executable(benchmark_collision src/benchmark_collision.cpp)
ament_target_dependencies(benchmark_collision
  rclcpp
  moveit_core
  moveit_ros_planning
)

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
  action_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Serves /base_controller/follow_joint_trajectory for whole-body plans. Reads the
# multi_dof_trajectory part and drives the base through Nav2's collision monitor.
add_executable(whole_body_base_follower src/whole_body_base_follower.cpp)
ament_target_dependencies(whole_body_base_follower
  rclcpp
  rclcpp_action
  control_msgs
  geometry_msgs
  nav2_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# move_group loads this under base_mode:=whole_body, ahead of time parameterization.
add_library(unwrap_planar_heading SHARED src/unwrap_planar_heading.cpp)
ament_target_dependencies(unwrap_planar_heading moveit_core pluginlib)
pluginlib_export_plugin_description_file(moveit_core planning_response_adapters.xml)
install(TARGETS unwrap_planar_heading
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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

# CLI tool to author + save a named OrientationConstraint into the MoveIt
# warehouse (the half RViz's MotionPlanning panel is missing). Uses the
# warehouse_ros plugin loader; the sqlite backend is loaded at runtime.
add_executable(save_named_constraint src/save_named_constraint.cpp)
ament_target_dependencies(save_named_constraint
  rclcpp
  moveit_ros_warehouse
  warehouse_ros
  moveit_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS commander whole_body_base_follower pick_and_place_isaac pick_and_place_isaac2 save_named_constraint benchmark_collision
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
  src/whole_body_demo.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME whole_body_demo
)

install(PROGRAMS
  src/commander_benchmark.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME commander_benchmark
)

if(BUILD_TESTING)
  find_package(ros_testing REQUIRED)
  # The real move_group and commander against stub controllers. They need a running
  # Zenoh router, see docs/guides/testing.md.
  install(PROGRAMS
    test/whole_body_stub_world.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME whole_body_stub_world
  )
  add_ros_test(test/whole_body_option.test.py TIMEOUT 600)
  add_ros_test(test/whole_body_gate.test.py TIMEOUT 900)
  add_ros_test(test/whole_body_plan_only.test.py TIMEOUT 300)
  add_ros_test(test/whole_body_clock.test.py TIMEOUT 300)
  add_ros_test(test/whole_body_stopping.test.py TIMEOUT 600)
  add_ros_test(test/whole_body_follower.test.py TIMEOUT 180)
endif()

ament_package()
