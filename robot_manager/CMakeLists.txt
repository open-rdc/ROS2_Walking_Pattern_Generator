cmake_minimum_required(VERSION 3.8)
project(robot_manager)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robot_messages REQUIRED)

add_executable(robot_manager
  src/robot_manager.cpp
  src/robot_manager_main.cpp
)
target_include_directories(robot_manager
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
  # PRIVATE
  #   include
)
target_compile_features(robot_manager
  PUBLIC
    c_std_99
    cxx_std_17
)
ament_target_dependencies(robot_manager
  rclcpp
  pluginlib
  sensor_msgs
  robot_messages
)

ament_export_include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)
install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS robot_manager
  DESTINATION lib/${PROJECT_NAME}
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
