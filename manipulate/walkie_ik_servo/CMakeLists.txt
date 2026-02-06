cmake_minimum_required(VERSION 3.8)
project(walkie_ik_servo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install Python modules
install(DIRECTORY walkie_ik_servo/
  DESTINATION lib/${PROJECT_NAME}/walkie_ik_servo
)

# Install Python nodes (executables)
install(PROGRAMS
  walkie_ik_servo/ik_servo_node.py
  # walkie_ik_servo/test_circle_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install URDF files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
