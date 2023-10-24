cmake_minimum_required(VERSION 3.5)
project(msgs_package)

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
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(msgs_package
  "msg/ControlOutput.msg"
  "msg/Feedback.msg"
  "msg/WalkingPattern.msg"
  "srv/StabilizationControl.srv"
  # old_message_file
  # "srv/ToRobotManager.srv"
  # "srv/ToWalkingPatternGenerator.srv"
  # "srv/ToWalkingStabilizationController.srv"
  # "msg/ToWalkingStabilizationControllerMessage.msg"
  # "srv/ToKinematicsMessage.srv"
  # "srv/ToWebotsRobotHandlerMessage.srv"
  # DEPENDENCIES
)

# add_executable(hello_world src/hello_world.cpp)
# target_include_directories(hello_world PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

# install(TARGETS hello_world
#   DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
