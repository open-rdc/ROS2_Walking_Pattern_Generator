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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(webots_ros2_driver REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)

# 以下、Plugin関連
pluginlib_export_plugin_description_file(webots_ros2_driver walking_pattern_generator.xml)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/WalkingPatternGenerator.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
  PRIVATE
  include
)

ament_target_dependencies(
  ${PROJECT_NAME}
  pluginlib
  rclcpp
  webots_ros2_driver
)
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)
ament_export_libraries(
  ${PROJECT_NAME}
)

# ココで、install/PACKAGE_NAME/share/PACKAGE_NAME/下に置くディレクトリを明示。launchファイルはココからファイルを覗く.
install(DIRECTORY
  launch
  worlds
  resource
  DESTINATION share/${PROJECT_NAME}/
)

# 以下、Test1_Sub関連
# add_executable(test1_sub
#   src/test1_sub.cpp
#   src/test1_sub_main.cpp
# )
# ament_target_dependencies(test1_sub
#   rclcpp
# )
# target_include_directories(test1_sub
#   PRIVATE
#   include
# )
# install(TARGETS
#   test1_sub
#   DESTINATION lib/${PROJECT_NAME}
# )

# 以下、Supervisor_pub関連
# add_executable(supervisor_pub
#   src/Supervisor_pub.cpp
#   src/Supervisor_pub_main.cpp
# )
# ament_target_dependencies(supervisor_pub
#   rclcpp
#   webots_ros2_driver
  
# )
# target_include_directories(supervisor_pub
#   PRIVATE
#   include
# )
# install(TARGETS
#   supervisor_pub
#   DESTINATION lib/${PROJECT_NAME}
# )

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
