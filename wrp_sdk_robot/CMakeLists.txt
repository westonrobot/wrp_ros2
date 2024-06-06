cmake_minimum_required(VERSION 3.8)
project(wrp_sdk_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fno-gnu-unique)

  # -fno-gnu-unique is a hotfix for when compiling on humble
endif()

list(APPEND CMAKE_PREFIX_PATH "/opt/weston_robot/lib/cmake")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(xacro REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# wr packages
find_package(wrp_sdk_msgs REQUIRED)
find_package(wrp_sdk 1.3 REQUIRED)

add_executable(mobile_robot_base_node
  src/mobile_base_node.cpp
)
target_link_libraries(mobile_robot_base_node
  westonrobot::wrp_sdk_robot
)
target_include_directories(mobile_robot_base_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mobile_robot_base_node
  rclcpp
  rclcpp_components
  wrp_sdk_msgs
  std_msgs
  tf2
  tf2_ros
  geometry_msgs
  nav_msgs
  tf2_geometry_msgs
)

install(TARGETS
  mobile_robot_base_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config launch urdf meshes
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
