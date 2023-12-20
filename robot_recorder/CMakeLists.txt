cmake_minimum_required(VERSION 3.8)
project(robot_recorder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_messages REQUIRED)

add_executable(feedback_recorder
  src/robot_feedback_recorder.cpp
)
target_include_directories(feedback_recorder
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(feedback_recorder
  PUBLIC 
    c_std_99 
    cxx_std_17
)  # Require C99 and C++17
ament_target_dependencies(feedback_recorder
  rclcpp
  robot_messages
)

add_executable(footStep_recorder
  src/robot_footStepPlanner_recorder.cpp
)
target_include_directories(footStep_recorder
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(footStep_recorder
  PUBLIC 
    c_std_99 
    cxx_std_17
)  # Require C99 and C++17
ament_target_dependencies(footStep_recorder
  rclcpp
  robot_messages
)

add_executable(walkingPattern_recorder
  src/robot_walkingPatternGenerator_recorder.cpp
)
target_include_directories(walkingPattern_recorder
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(walkingPattern_recorder
  PUBLIC 
    c_std_99 
    cxx_std_17
)  # Require C99 and C++17
ament_target_dependencies(walkingPattern_recorder
  rclcpp
  robot_messages
)

add_executable(walkingStabilization_recorder
  src/robot_walkingStabilizationController_recorder.cpp
)
target_include_directories(walkingStabilization_recorder
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(walkingStabilization_recorder
  PUBLIC 
    c_std_99 
    cxx_std_17
)  # Require C99 and C++17
ament_target_dependencies(walkingStabilization_recorder
  rclcpp
  robot_messages
)

add_executable(jointState_recorder
  src/robot_jointStates_recorder.cpp
)
target_include_directories(jointState_recorder
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(jointState_recorder
  PUBLIC 
    c_std_99 
    cxx_std_17
)  # Require C99 and C++17
ament_target_dependencies(jointState_recorder
  rclcpp
  robot_messages
)

install(TARGETS feedback_recorder
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS footStep_recorder
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS walkingPattern_recorder
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS walkingStabilization_recorder
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS jointState_recorder
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
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
