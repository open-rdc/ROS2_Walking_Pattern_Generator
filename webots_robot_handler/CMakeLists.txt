cmake_minimum_required(VERSION 3.8)
project(webots_robot_handler)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# find_package(msgs_package REQUIRED)
# find_package(kinematics REQUIRED)
find_package(robot_messages REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(webots_ros2_driver REQUIRED)
# find_package(webots_ros2_control REQUIRED)
# find_package(controller_manager REQUIRED)
find_package(pluginlib REQUIRED)

pluginlib_export_plugin_description_file(webots_ros2_driver webots_robot_handler.xml)

add_library(${PROJECT_NAME}
  SHARED
  src/WebotsRobotHander.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  pluginlib
  rclcpp
  webots_ros2_driver
  # kinematics
  sensor_msgs
  robot_messages
  # webots_ros2_control
  # controller_manager
  # msgs_package
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    include
)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)
ament_export_libraries(${PROJECT_NAME})

install(DIRECTORY
  launch
  worlds
  resource
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
