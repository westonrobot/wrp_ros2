/**
 * ultrasonic_sensor_node.hpp
 *
 * Created on Tue Apr 12 2022 16:26:07
 *
 * Description:
 *
 * Copyright (c) 2022 Weston Robot Pte. Ltd.
 */

#ifndef ULTRASONIC_SENSOR_NODE_HPP
#define ULTRASONIC_SENSOR_NODE_HPP

// C++
#include <string>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/time.hpp"

// wrp_sdk
#include "wrp_sdk/interface/ultrasonic_interface.hpp"

// ROS Messages
#include "sensor_msgs/msg/range.hpp"

namespace westonrobot {
class UltrasonicSensorNode : public rclcpp::Node {
 public:
  UltrasonicSensorNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~UltrasonicSensorNode(){};

 private:
  // ----- ROS Node Parameters -----
  std::string sensor_model_ = "dyp_a05";
  std::string device_path_ = "/dev/ttyUSB0";
  int baud_rate_ = 115200;
  std::string frame_id_ = "ultrasonic_link";
  std::string topic_name_ = "ultrasonic";
  // ----- Internal Variables -----
  std::shared_ptr<UltrasonicInterface> sensor_;
  // ----- Published Messages-----
  // ----- Subscribers & Publishers & Services -----
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr>
      publishers_;
  // ----- Callbacks -----
  void PublishCallback(const UltrasonicMsg& imu_msg);

  bool ReadParameters();
  bool SetupHardware();
  bool SetupInterfaces();
};  // UltrasonicSensorNode
}  // namespace westonrobot

#endif /* ULTRASONIC_SENSOR_NODE_HPP */
