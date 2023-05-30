cmake_minimum_required(VERSION 3.5)
project(walking_stabilization_controller)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(msgs_package REQUIRED)
find_package(kinematics REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME}
  src/WalkingStabilizationController.cpp
)
# add_executable(${PROJECT_NAME}
#   src/WalkingStabilizationController.cpp
#   src/WalkingStabilizationController_main.cpp
# )
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  msgs_package
  kinematics
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME}
  PUBLIC
    c_std_99
    cxx_std_17
)
target_compile_definitions(${PROJECT_NAME}
  PRIVATE
    "WALKING_STABILIZATION_CONTROLLER_BUILDING_LIBRARY"
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)

install(
  DIRECTORY
    include/
  DESTINATION
    include
)
install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
