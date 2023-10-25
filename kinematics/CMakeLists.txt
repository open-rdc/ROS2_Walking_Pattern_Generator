cmake_minimum_required(VERSION 3.8)
project(kinematics)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_manager REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)

pluginlib_export_plugin_description_file(robot_manager plugins.xml)

add_library(kinematics
  src/ForwardKinematics.cpp
  src/InverseKinematics.cpp
  src/Jacobian.cpp
)
# add_library(InverseKinematics 
#   src/InverseKinematics.cpp
# )
# add_library(Jacobian
#   src/Jacobian.cpp
# )

ament_target_dependencies(kinematics
  rclcpp
  pluginlib
  robot_manager
)
# ament_target_dependencies(InverseKinematics
#   rclcpp
#   pluginlib
#   robot_manager
# )
# ament_target_dependencies(Jacobian
#   rclcpp
#   pluginlib
#   robot_manager
# )

target_compile_features(kinematics  # Require C99 and C++17
  PUBLIC 
    c_std_99 
    cxx_std_17
)  
# target_compile_features(InverseKinematics  # Require C99 and C++17
#   PUBLIC 
#     c_std_99 
#     cxx_std_17
# )
# target_compile_features(Jacobian
#   PUBLIC
#     c_std_99
#     cxx_std_17
# )

target_include_directories(kinematics
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)
# target_include_directories(InverseKinematics 
#   PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
#     $<INSTALL_INTERFACE:include>
# )
# target_include_directories(Jacobian
#   PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
#     $<INSTALL_INTERFACE:include>
# )

target_compile_definitions(kinematics
  PRIVATE 
    "KINEMATICS_BUILDING_LIBRARY"
)
# target_compile_definitions(InverseKinematics
#   PRIVATE 
#     "KINEMATICS_BUILDING_LIBRARY"
# )
# target_compile_definitions(Jacobian
#   PRIVATE 
#     "KINEMATICS_BUILDING_LIBRARY"
# )

install(
  DIRECTORY 
    include/
  DESTINATION 
    include
)
install(TARGETS kinematics
  # ForwardKinematics
  # Jacobian
  # InverseKinematics
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


ament_export_include_directories(
  include
)
ament_export_libraries(
  kinematics
  # ForwardKinematics
  # InverseKinematics
  # Jacobian
)
ament_export_targets(
  export_${PROJECT_NAME}
  HAS_LIBRARY_TARGET
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
