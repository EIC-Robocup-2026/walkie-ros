cmake_minimum_required(VERSION 3.8)
project(walkie_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

ament_package_xml()
list(APPEND ${PROJECT_NAME}_MEMBER_OF_GROUPS "rosidl_interface_packages")

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetObPose.srv"
  "srv/GetObjectCloud.srv"
  DEPENDENCIES geometry_msgs vision_msgs sensor_msgs
)

# ── C++ node ──────────────────────────────────────────────────────────────────
add_executable(ob_pose_service_cpp src/ob_pose_service.cpp)
ament_target_dependencies(ob_pose_service_cpp
  rclcpp
  sensor_msgs
  vision_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target
  "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(ob_pose_service_cpp "${cpp_typesupport_target}")
install(TARGETS ob_pose_service_cpp DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  perception/ob_detection.py
  perception/ob_pose.py
  perception/yolo_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  perception/obb_3d.py
  RENAME obb3d
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  models
  srv
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
