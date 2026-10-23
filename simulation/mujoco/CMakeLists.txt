cmake_minimum_required(VERSION 3.12)
project(walkie_mujoco_worlds NONE)

find_package(ament_cmake REQUIRED)

# Install only what robot_bringup's world loader reads at launch time:
# the manifest, exported MJCF scenes, their meshes and textures, and the
# occupancy map. Blender projects, authoring data, tools and local reports
# stay in the source checkout.
#
# The files are installed one by one with install(FILES) so that
# --symlink-install links them into the source tree. ament's symlink install
# does not support install(DIRECTORY ... FILES_MATCHING), falls back to a copy
# for it, and ignores a directory PATTERN ... EXCLUDE. CONFIGURE_DEPENDS
# re-runs the glob at every build, so new world files need a rebuild but no
# change here.
file(GLOB_RECURSE world_files CONFIGURE_DEPENDS
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}/worlds/*")
list(FILTER world_files INCLUDE REGEX "(/world\\.toml|\\.(xml|obj|stl|msh|png|yaml|pgm))$")
list(FILTER world_files EXCLUDE REGEX "/(authoring|__pycache__)/")
foreach(world_file IN LISTS world_files)
  get_filename_component(world_file_dir "${world_file}" DIRECTORY)
  install(FILES "${world_file}" DESTINATION "share/${PROJECT_NAME}/${world_file_dir}")
endforeach()

ament_package()
