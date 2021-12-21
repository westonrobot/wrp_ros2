/**
 * gps_receiver_node.hpp
 *
 * Created on Tue Nov 23 2021 13:43:06
 *
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */
#ifndef GPS_RECEIVER_NODE_HPP
#define GPS_RECEIVER_NODE_HPP

// C++
#include <string>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/time.hpp"

// wrp_sdk
#include "wrp_sdk/peripheral/gps_receiver.hpp"

// ROS Messages
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"

namespace westonrobot {

class GpsReceiverNode : public rclcpp::Node {
 public:
  GpsReceiverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~GpsReceiverNode();

 private:
  // ----- ROS Node Parameters -----
  std::string device_path_;
  int publish_interval_;
  int baud_rate_;
  std::string frame_id_;
  // ----- Internal Variables -----
  std::unique_ptr<GpsReceiver> receiver_;
  // ----- Published Messages-----
  sensor_msgs::msg::NavSatFix sat_fix_;
  // ----- Subscribers & Publishers -----
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
  // ----- Timers -----
  // ----- Callbacks -----
  void PublishCallback(NavSatFix gps_fix);

  bool SetupReceiver();
  bool ReadParameters();
};

}  // namespace westonrobot

#endif /* GPS_RECEIVER_NODE_HPP */
