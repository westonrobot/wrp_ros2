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

list(APPEND CMAKE_PREFIX_PATH "/opt/weston_robot/lib/cmake")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
# ros2
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# interfaces
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# wr packages
find_package(wrp_sdk REQUIRED)

# set all interface files
set(message_files
# peripheral device
"msg/peripheral/LiftState.msg"
# mobile base
"msg/mobile_base/ActuatorState.msg"
"msg/mobile_base/ActuatorStateArray.msg"
"msg/mobile_base/DriverState.msg"
"msg/mobile_base/LightControlType.msg"
"msg/mobile_base/MotionCommand.msg"
"msg/mobile_base/MotionState.msg"
"msg/mobile_base/MotorState.msg"
"msg/mobile_base/RangeData.msg"
"msg/mobile_base/RangeDataType.msg"
"msg/mobile_base/RcState.msg"
"msg/mobile_base/SystemState.msg")

set(service_files
# mobile base
"srv/mobile_base/AccessControl.srv"
"srv/mobile_base/AssistedModeControl.srv"
"srv/mobile_base/LightControl.srv"
"srv/mobile_base/MotionReset.srv")

set(action_files
# peripheral devices
"action/peripheral/LiftGoal.action")

# generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
${message_files} ${service_files} ${action_files}  
  DEPENDENCIES
    builtin_interfaces
    std_msgs
    action_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs)

ament_export_dependencies(rosidl_default_runtime)

add_library(wrp_sdk_ros2
  #mobile base
  src/mobile_base/mobile_base_node.cpp
  # peripheral device support
  src/peripheral/gps_receiver_node.cpp
  src/peripheral/imu_sensor_node.cpp
  src/peripheral/lift_server_node.cpp
)
add_dependencies(wrp_sdk_ros2 ${PROJECT_NAME})
rosidl_target_interfaces(wrp_sdk_ros2 
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(wrp_sdk_ros2 westonrobot::wrp_sdk)
target_include_directories(wrp_sdk_ros2 PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>)
ament_target_dependencies(wrp_sdk_ros2
  rclcpp
  rclcpp_action
  rclcpp_components
  tf2
  builtin_interfaces
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  action_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs)

# create executables from library
rclcpp_components_register_node(wrp_sdk_ros2 PLUGIN "westonrobot::GpsReceiverNode" EXECUTABLE gps_receiver_node)
rclcpp_components_register_node(wrp_sdk_ros2 PLUGIN "westonrobot::ImuSensorNode" EXECUTABLE imu_sensor_node)
rclcpp_components_register_node(wrp_sdk_ros2 PLUGIN "westonrobot::LiftServerNode" EXECUTABLE lift_server_node)
rclcpp_components_register_node(wrp_sdk_ros2 PLUGIN "westonrobot::MobileBaseNode" EXECUTABLE mobile_base_node)

install(TARGETS wrp_sdk_ros2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY launch config urdf meshes
  DESTINATION share/${PROJECT_NAME}/)

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
