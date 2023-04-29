cmake_minimum_required(VERSION 3.5)
project(old_kinematics)

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

add_executable(fk_srv 
  src/FK.cpp
  src/FK_main.cpp
)
ament_target_dependencies(fk_srv
  rclcpp
  msgs_package
)
target_include_directories(fk_srv
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    include
)

add_executable(ik_srv
  src/IK.cpp
  src/IK_main.cpp
)
ament_target_dependencies(ik_srv
  rclcpp
  msgs_package
)
target_include_directories(ik_srv
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    include
)

install(TARGETS
  fk_srv
  ik_srv
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
