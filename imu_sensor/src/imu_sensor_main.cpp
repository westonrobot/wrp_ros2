/**
 * imu_sensor_main.cpp
 *
 * Created on Tue Nov 23 2021 17:28:45
 *
 * Description:
 *   
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#include "imu_sensor/imu_sensor_node.hpp"

int main(int argc, char const *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<imu_sensor::ImuSensorNode>());
  rclcpp::shutdown();
  return 0;
}
