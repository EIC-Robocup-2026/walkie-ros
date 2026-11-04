# Copyright 2025 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.22)
project(openarm_bimanual_moveit_config)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Checks that every link/joint named in the SRDF exists in the walkie URDF.
  ament_add_pytest_test(test_srdf_urdf_contract test/test_srdf_urdf_contract.py
    TIMEOUT 120)
  # Checks that floor returns reach the octomap updater as rays that mark nothing.
  ament_add_pytest_test(test_cloud_floor_gate test/test_cloud_floor_gate.py
    TIMEOUT 60)
endif()

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/cloud_floor_gate.py
  scripts/octomap_jump_guard.py
  scripts/wait_for_graph.py
  DESTINATION lib/${PROJECT_NAME})
