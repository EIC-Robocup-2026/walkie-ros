cmake_minimum_required(VERSION 3.8)
project(perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

install(PROGRAMS
  src/ob_detection.py
  src/ob_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  # launch
  models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()