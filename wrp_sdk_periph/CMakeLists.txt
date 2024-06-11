cmake_minimum_required(VERSION 3.8)
project(wrp_sdk_periph)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fno-gnu-unique)

  # -fno-gnu-unique is a hotfix for when compiling on humble
endif()

list(APPEND CMAKE_PREFIX_PATH "/opt/weston_robot/lib/cmake")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

# wr packages
find_package(wrp_sdk_msgs REQUIRED)
find_package(wrp_sdk 1.3.0...<1.5.0 REQUIRED)

# Attempt to find lelycan libraries for peripheral nodes
find_package(PkgConfig REQUIRED)
pkg_check_modules(LelyIO QUIET IMPORTED_TARGET liblely-io2)
pkg_check_modules(LelyCOAPP QUIET IMPORTED_TARGET liblely-coapp)

set(BUILD_CANOPEN_MODULE OFF)

if(LelyIO_FOUND AND LelyCOAPP_FOUND)
  message(STATUS "Lely-core libraries found, canopen peripheral modules will be included")
  set(BUILD_CANOPEN_MODULE ON)
else()
  message(WARNING "CANopen-related dependencies are not found, ROS nodes for canopen peripherals will not be built. Run the following commands to install the missing libraries:
    $ sudo apt-get install -y software-properties-common 
    $ sudo add-apt-repository ppa:lely/ppa && sudo apt-get update
    $ sudo apt-get install -y pkg-config liblely-coapp-dev liblely-co-tools")
endif()

include_directories(
  include
)

add_executable(gps_receiver_node
  "src/gps_receiver_node.cpp"
)
target_link_libraries(gps_receiver_node
  westonrobot::wrp_sdk_peripheral
)
target_include_directories(gps_receiver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(gps_receiver_node
  rclcpp
  sensor_msgs
)

add_executable(ultrasonic_sensor_node
  "src/ultrasonic_sensor_node.cpp"
)
target_link_libraries(ultrasonic_sensor_node
  westonrobot::wrp_sdk_peripheral
)
target_include_directories(ultrasonic_sensor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(ultrasonic_sensor_node
  rclcpp
  sensor_msgs
)

add_executable(lift_controller_node
  "src/lift_controller_node.cpp"
)
target_link_libraries(lift_controller_node
  westonrobot::wrp_sdk_peripheral
)
target_include_directories(lift_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(lift_controller_node
  rclcpp
  sensor_msgs
  wrp_sdk_msgs
  rclcpp_action
)

install(TARGETS
  gps_receiver_node
  ultrasonic_sensor_node
  lift_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_CANOPEN_MODULE)
  add_executable(power_regulator_node
    "src/power_regulator_node.cpp"
  )
  target_link_libraries(power_regulator_node
  westonrobot::wrp_sdk_peripheral
    westonrobot::wrp_sdk_canopen
  )
  target_include_directories(power_regulator_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(power_regulator_node
    rclcpp
    std_msgs
    sensor_msgs
    wrp_sdk_msgs
  )

  add_executable(imu_sensor_node
    "src/imu_sensor_node.cpp"
  )
  target_link_libraries(imu_sensor_node
    westonrobot::wrp_sdk_peripheral
    westonrobot::wrp_sdk_canopen
  )
  target_include_directories(imu_sensor_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(imu_sensor_node
    rclcpp
    sensor_msgs
  )

  install(TARGETS
    power_regulator_node
    imu_sensor_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config launch
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
