cmake_minimum_required(VERSION 3.8)
project(foot_step_planner)

set(CMAKE_CXX_STANDARD 17)

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

add_library(foot_step_planner
  src/FootStepPlanner.cpp
)
ament_target_dependencies(foot_step_planner
  robot_manager
  rclcpp
  pluginlib
)
target_include_directories(foot_step_planner
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    include
)

target_compile_definitions(foot_step_planner
  PRIVATE
    "FOOT_STEP_PLANNER_BUILDING_LIBRARY"
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(TARGETS foot_step_planner
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  foot_step_planner
)
ament_export_targets(
  export_${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
