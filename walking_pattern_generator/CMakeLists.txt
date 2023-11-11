cmake_minimum_required(VERSION 3.8)
project(walking_pattern_generator)

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

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_manager REQUIRED)
find_package(pluginlib REQUIRED)
# find_package(msgs_package REQUIRED)
# find_package(kinematics REQUIRED)
# find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# add_executable(walking_pattern_generator
#   src/WalkingPatternGenerator.cpp
#   src/WalkingPatternGenerator_main.cpp
# )
# ament_target_dependencies(walking_pattern_generator
#   rclcpp
#   msgs_package
#   kinematics
#   sensor_msgs
# )
# target_include_directories(walking_pattern_generator
#   PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
#     $<INSTALL_INTERFACE:include>
#   PRIVATE
#     include
# )

pluginlib_export_plugin_description_file(robot_manager plugins.xml)

add_library(walking_pattern_generator
  src/LinearInvertedPendulumModel.cpp
)
ament_target_dependencies(walking_pattern_generator
  robot_manager
  rclcpp
  pluginlib
)
target_include_directories(walking_pattern_generator
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    include
)

target_compile_definitions(walking_pattern_generator
  PRIVATE
    "WALKING_PATTERN_GENERATOR_BUILDING_LIBRARY"
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(TARGETS walking_pattern_generator
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  walking_pattern_generator
)
ament_export_targets(
  export_${PROJECT_NAME}
)

# install(TARGETS
#   walking_pattern_generator
#   DESTINATION lib/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
