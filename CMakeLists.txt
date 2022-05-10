cmake_minimum_required(VERSION 3.5)
project(goto_plugins)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  ament_cmake_ros
  pluginlib
  goto_plugin_base
  rclcpp
  rclcpp_action
  nav_msgs
  as2_core
  as2_msgs
  geometry_msgs
  as2_motion_command_handlers
  Eigen3
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(goto_plugins src/goto_plugin_speed.cpp src/goto_plugin_traj.cpp)
target_include_directories(goto_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  goto_plugins
  ${PROJECT_DEPENDENCIES}
)

pluginlib_export_plugin_description_file(goto_plugin_base plugins.xml)

install(
  TARGETS goto_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(
  goto_plugins
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
