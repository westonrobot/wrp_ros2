/**
 * gps_receiver_main.cpp
 *
 * Created on Tue Nov 23 2021 15:17:37
 *
 * Description:
 *   
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */
#include "gps_receiver/gps_receiver_node.hpp"

int main(int argc, char const *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gps_receiver::GpsReceiverNode>());
  rclcpp::shutdown();
  return 0;
}
