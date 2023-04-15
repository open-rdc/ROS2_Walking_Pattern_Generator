cmake_minimum_required(VERSION 3.5)
project(walking_pattern_generator)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(msgs_package REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(walking_pattern_generator
  src/WalkingPatternGenerator.cpp
  src/WalkingPatternGenerator_main.cpp
)
ament_target_dependencies(walking_pattern_generator
  rclcpp
  msgs_package
)
target_include_directories(walking_pattern_generator
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    include
)

install(TARGETS
  walking_pattern_generator
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
