cmake_minimum_required(VERSION 3.8)
project(walking_stabilization_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_manager REQUIRED)
find_package(pluginlib REQUIRED)

pluginlib_export_plugin_description_file(robot_manager plugins.xml)

add_library(walking_stabilization_controller
  src/WalkingStabilizationController.cpp
)
ament_target_dependencies(walking_stabilization_controller
  rclcpp
  robot_manager
  pluginlib
)
target_include_directories(walking_stabilization_controller
  PUBLIC
	  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	  $<INSTALL_INTERFACE:include>
  PRIVATE
    include
)

target_compile_definitions(walking_stabilization_controller
  PRIVATE
	"WALKING_STABILIZATION_CONTROLLER_BUILDING_LIBRARY"
)

install(
  DIRECTORY
	include/
  DESTINATION
	include
)
install(TARGETS walking_stabilization_controller
  EXPORT export_walking_stabilization_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  walking_stabilization_controller
)
ament_export_targets(
  export_walking_stabilization_controller
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


