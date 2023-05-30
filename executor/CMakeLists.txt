cmake_minimum_required(VERSION 3.8)
project(executor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_manager REQUIRED)
find_package(walking_stabilization_controller REQUIRED)
find_package(walking_pattern_generator REQUIRED)
find_package(kinematics REQUIRED)
find_package(msgs_package REQUIRED)
find_package(Eigen3 REQUIRED)

# add_executable(multi_executor
#   src/multi_executor_main.cpp
# )
# ament_target_dependencies(multi_executor
#   rclcpp
#   walking_stabilization_controller
#   robot_manager
#   msgs_package
# )
# target_include_directories(multi_executor
#   PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
#     $<INSTALL_INTERFACE:include>
# )

# install(TARGETS
#   multi_executor
#   DESTINATION lib/${PROJECT_NAME}
# )

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
