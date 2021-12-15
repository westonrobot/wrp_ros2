cmake_minimum_required(VERSION 3.5)
project(wrp_ros2 VERSION 0.2.0)

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
find_package(rmw_implementation_cmake REQUIRED)
# ros2
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
# interfaces
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)
# wr packages
find_package(wrp_zbus REQUIRED)
find_package(wrp_sdk REQUIRED)

# generate interfaces
# even though its called sdk_interfaces
# headers generated are still wrp_ros2/msg/...
rosidl_generate_interfaces(sdk_interfaces
    "msg/LiftState.msg"
    "action/LiftGoal.action"
  DEPENDENCIES
    builtin_interfaces
    std_msgs
    action_msgs
    sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

add_library(${PROJECT_NAME}
  # peripheral device support
  src/peripheral/gps_receiver_node.cpp
  src/peripheral/imu_sensor_node.cpp
  src/peripheral/lift_server_node.cpp)
add_dependencies(${PROJECT_NAME} sdk_interfaces)
rosidl_target_interfaces(${PROJECT_NAME} 
  sdk_interfaces "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}
    westonrobot::wrp_sdk
    westonrobot::wrp_zbus)
target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs)

# create executables from library
rclcpp_components_register_node(${PROJECT_NAME} PLUGIN "wrp_ros2::GpsReceiverNode" EXECUTABLE gps_receiver_node)
rclcpp_components_register_node(${PROJECT_NAME} PLUGIN "wrp_ros2::ImuSensorNode" EXECUTABLE imu_sensor_node)
rclcpp_components_register_node(${PROJECT_NAME} PLUGIN "wrp_ros2::LiftServerNode" EXECUTABLE lift_server_node)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
  
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)


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
