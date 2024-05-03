cmake_minimum_required(VERSION 3.8)
project(wrp_sdk_periph)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# wr packages
find_package(wrp_sdk 1.3 REQUIRED)
find_package(wrp_sdk_msgs REQUIRED)

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

add_library(wrp_sdk_periph_base SHARED
  "src/gps_receiver_node.cpp"
  "src/ultrasonic_sensor_node.cpp"
)
target_link_libraries(wrp_sdk_periph_base
  westonrobot::wrp_sdk_peripheral
)
target_include_directories(wrp_sdk_periph_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(wrp_sdk_periph_base
  rclcpp
  rclcpp_components
  sensor_msgs
)
rclcpp_components_register_node(wrp_sdk_periph_base
  PLUGIN "westonrobot::GpsReceiverNode"
  EXECUTABLE gps_receiver_node)
rclcpp_components_register_node(wrp_sdk_periph_base
  PLUGIN "westonrobot::UltrasonicSensorNode"
  EXECUTABLE ultrasonic_sensor_node)

install(TARGETS
  wrp_sdk_periph_base
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_CANOPEN_MODULE)
  add_library(wrp_sdk_periph_canopen SHARED
    src/power_regulator_node.cpp
    src/imu_sensor_node.cpp
  )
  target_link_libraries(wrp_sdk_periph_canopen
    # westonrobot::wrp_sdk_canopen
    westonrobot::wrp_sdk_peripheral
  )
  target_include_directories(wrp_sdk_periph_canopen PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(wrp_sdk_periph_canopen
    rclcpp
    rclcpp_components
    std_msgs
    sensor_msgs
    wrp_sdk_msgs
  )
  rclcpp_components_register_node(wrp_sdk_periph_canopen
    PLUGIN "westonrobot::PowerRegulatorNode"
    EXECUTABLE power_regulator_node)
  rclcpp_components_register_node(wrp_sdk_periph_canopen
    PLUGIN "westonrobot::ImuSensorNode"
    EXECUTABLE imu_sensor_node)

  install(TARGETS
    wrp_sdk_periph_canopen
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
