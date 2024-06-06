cmake_minimum_required(VERSION 3.8)
project(wrp_sdk_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

set(message_files

  # mobile base
  "msg/mobile_base/ActuatorState.msg"
  "msg/mobile_base/ActuatorStateArray.msg"
  "msg/mobile_base/DriverState.msg"
  "msg/mobile_base/LightControlType.msg"
  "msg/mobile_base/MotionCommand.msg"
  "msg/mobile_base/MotionState.msg"
  "msg/mobile_base/MotorState.msg"
  "msg/mobile_base/RangeData.msg"
  "msg/mobile_base/RangeDataArray.msg"
  "msg/mobile_base/RcState.msg"
  "msg/mobile_base/SystemState.msg"

  # peripheral
  "msg/peripheral/PowerRegulatorChannelState.msg"
  "msg/peripheral/PowerRegulatorDeviceState.msg"
  "msg/peripheral/LiftState.msg"
)

set(service_files

  # mobile base
  "srv/mobile_base/AccessControl.srv"
  "srv/mobile_base/AssistedModeControl.srv"
  "srv/mobile_base/LightControl.srv"
  "srv/mobile_base/MotionReset.srv"

  # peripheral
  "srv/peripheral/PowerRegulatorControl.srv"
  "srv/peripheral/LiftQuery.srv"
)

set(action_files

  # peripheral
  "action/peripheral/LiftControl.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${message_files} ${service_files} ${action_files}
  DEPENDENCIES
  std_msgs
  action_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs)

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

ament_export_dependencies(rosidl_default_runtime)
ament_package()
