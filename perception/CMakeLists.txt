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
find_package(tf2_sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

ament_package_xml()
list(APPEND ${PROJECT_NAME}_MEMBER_OF_GROUPS "rosidl_interface_packages")

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetObPose.srv"
  "srv/GetObjectCloud.srv"
  "srv/GraspFromMask.srv"
  "srv/GraspFromCloud.srv"
  "srv/GraspPos.srv"
  DEPENDENCIES geometry_msgs vision_msgs sensor_msgs
)

# ── C++ nodes ─────────────────────────────────────────────────────────────────
add_executable(cloud_to_map src/cloud_to_map_node.cpp)
ament_target_dependencies(cloud_to_map
  rclcpp sensor_msgs tf2_ros tf2_sensor_msgs)
install(TARGETS cloud_to_map DESTINATION lib/${PROJECT_NAME})

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

# ── Python nodes ──────────────────────────────────────────────────────────────
install(PROGRAMS perception/ob_pose.py
  RENAME ob_pose DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS perception/yolo_node.py
  RENAME yolo_node DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS perception/obb_3d.py
  RENAME obb3d DESTINATION lib/${PROJECT_NAME})

# Unified GraspNet server — one model, three services
# (/grasp/from_mask, /grasp/from_cloud, /grasp/pos).
install(PROGRAMS perception/grasp_node.py
  RENAME grasp_server DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS perception/test_grasp_viz.py
  RENAME test_grasp_viz DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY srv
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
