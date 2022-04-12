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
find_package(wrp_sdk 1.0.0 REQUIRED)

# Attempt to find lelycan libraries for peripheral nodes
find_package(PkgConfig REQUIRED)
pkg_check_modules(LelyIO QUIET IMPORTED_TARGET liblely-io2)
pkg_check_modules(LelyCOAPP QUIET IMPORTED_TARGET liblely-coapp)

set(BUILD_PERIPH_MODULE OFF)
if(LelyIO_FOUND AND LelyCOAPP_FOUND)
    message(STATUS "Lely-core libraries found, peripheral module will be included")
    set(BUILD_PERIPH_MODULE ON)
else()
    message(WARNING "CANopen-related dependencies are not found, ROS nodes for peripherals will not be built. Run the following commands to install the missing libraries:
    $ sudo apt-get install -y software-properties-common 
    $ sudo add-apt-repository ppa:lely/ppa && sudo apt-get update
    $ sudo apt-get install -y pkg-config liblely-coapp-dev liblely-co-tools")
endif()

# set all interface files
set(message_files
# mobile base
"msg/mobile_base/ActuatorState.msg"
"msg/mobile_base/ActuatorStateArray.msg"
"msg/mobile_base/DriverState.msg"
"msg/mobile_base/LightControlType.msg"
"msg/mobile_base/MotionCommand.msg"
"msg/mobile_base/MotionState.msg"
"msg/mobile_base/MotorState.msg"
"msg/mobile_base/RangeDataArray.msg"
"msg/mobile_base/RangeData.msg"
"msg/mobile_base/RcState.msg"
"msg/mobile_base/SystemState.msg")

set(service_files
# mobile base
"srv/mobile_base/AccessControl.srv"
"srv/mobile_base/AssistedModeControl.srv"
"srv/mobile_base/LightControl.srv"
"srv/mobile_base/MotionReset.srv")


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

include_directories(
  include
)

add_executable(mobile_base_node src/mobile_base/mobile_base_node.cpp)
add_dependencies(mobile_base_node ${PROJECT_NAME})
rosidl_target_interfaces(mobile_base_node 
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(mobile_base_node westonrobot::wrp_sdk_robot)
ament_target_dependencies(mobile_base_node
  rclcpp
  builtin_interfaces
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs)

install(TARGETS
  mobile_base_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_PERIPH_MODULE)
  add_executable(gps_receiver_node src/peripheral/gps_receiver_node.cpp)
  add_dependencies(gps_receiver_node ${PROJECT_NAME})
  rosidl_target_interfaces(gps_receiver_node 
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(gps_receiver_node westonrobot::wrp_sdk)
  ament_target_dependencies(gps_receiver_node
    rclcpp
    builtin_interfaces
    std_msgs
    sensor_msgs)

  add_executable(imu_sensor_node src/peripheral/imu_sensor_node.cpp)
  add_dependencies(imu_sensor_node ${PROJECT_NAME})
  rosidl_target_interfaces(imu_sensor_node 
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(imu_sensor_node westonrobot::wrp_sdk)
  ament_target_dependencies(imu_sensor_node
    rclcpp
    builtin_interfaces
    std_msgs
    sensor_msgs)

  install(TARGETS
    gps_receiver_node 
    imu_sensor_node
    DESTINATION lib/${PROJECT_NAME})
endif()


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
