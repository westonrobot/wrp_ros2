/**
 * @file gps_receiver_node.hpp
 * @brief 
 * @date 03-05-2024
 * 
 * @copyright Copyright (c) 2024 Weston Robot Pte. Ltd.
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
#include "wrp_sdk/interface/gps_interface.hpp"

// ROS Messages
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"

namespace westonrobot {
class GpsReceiverNode : public rclcpp::Node {
 public:
  GpsReceiverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~GpsReceiverNode(){};

 private:
  // ----- ROS Node Parameters -----
  std::string device_path_;
  int publish_interval_;
  int baud_rate_;
  std::string frame_id_;
  // ----- Internal Variables -----
  std::unique_ptr<GpsInterface> receiver_;
  // ----- Published Messages-----
  sensor_msgs::msg::NavSatFix sat_fix_;
  // ----- Subscribers & Publishers -----
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
  // ----- Timers -----
  // ----- Callbacks -----
  void PublishCallback(const NavSatFixMsg& gps_fix);

  bool ReadParameters();
  bool SetupHardware();
  bool SetupInterfaces();
};  // GpsReceiverNode
}  // namespace westonrobot
#endif /* GPS_RECEIVER_NODE_HPP */
