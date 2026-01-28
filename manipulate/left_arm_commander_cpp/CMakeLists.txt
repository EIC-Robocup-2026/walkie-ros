cmake_minimum_required(VERSION 3.8)
project(left_arm_commander_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(my_robot_interfaces REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_core REQUIRED)
# find_package(moveit_task_constructor_core REQUIRED)

add_executable(commander src/commander_template.cpp)
ament_target_dependencies(commander rclcpp moveit_ros_planning_interface example_interfaces my_robot_interfaces rclcpp_action control_msgs moveit_msgs moveit_core)

add_executable(commander_vision src/commander_vision.cpp)
ament_target_dependencies(commander_vision rclcpp moveit_ros_planning_interface example_interfaces my_robot_interfaces)

add_executable(left_arm_test src/test_moveit.cpp)
ament_target_dependencies(left_arm_test
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_geometry_msgs
  tf2_eigen
  moveit_msgs
  moveit_core
  rclcpp_action
  control_msgs
)

install(TARGETS left_arm_test commander commander_vision
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
