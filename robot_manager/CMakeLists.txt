cmake_minimum_required(VERSION 3.8)
project(robot_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(msgs_package REQUIRED)
find_package(kinematics REQUIRED)
find_package(Eigen3 REQUIRED)

# add_library(robot_manager
#   src/robot_manager.cpp
# )
add_executable(robot_manager
  src/robot_manager.cpp
  src/robot_manager_main.cpp
)
ament_target_dependencies(robot_manager
  rclcpp
  msgs_package
  kinematics
)
target_include_directories(robot_manager 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(robot_manager  # Require C99 and C++17
  PUBLIC 
    c_std_99 
    cxx_std_17
)
target_compile_definitions(robot_manager
  PRIVATE
    "ROBOT_MANAGER_BUILDING_LIBRARY"
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  robot_manager
)
ament_export_targets(
  export_${PROJECT_NAME}
  HAS_LIBRARY_TARGET
)

install(
  DIRECTORY
    include/
  DESTINATION
    include
)
install(TARGETS 
  robot_manager
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # set(ament_cmake_copyright_FOUND TRUE)
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
