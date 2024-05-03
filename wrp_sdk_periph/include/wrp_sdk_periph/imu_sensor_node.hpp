/**
 * imu_sensor_node.hpp
 *
 * Created on Tue Nov 23 2021 16:19:12
 *
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */
#ifndef IMU_SENSOR_NODE_HPP
#define IMU_SENSOR_NODE_HPP

// C++
#include <string>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/time.hpp"

// wrp_sdk
#include "wrp_sdk/interface/imu_interface.hpp"

// ROS Messages
#include "sensor_msgs/msg/imu.hpp"

namespace westonrobot {
class ImuSensorNode : public rclcpp::Node {
 public:
  ImuSensorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~ImuSensorNode(){};

 private:
  // ----- ROS Node Parameters -----
  std::string sensor_model_;
  std::string device_path_;
  int baud_rate_;
  std::string frame_id_;
  // ----- Internal Variables -----
  std::unique_ptr<ImuInterface> imu_;
  // ----- Published Messages-----
  sensor_msgs::msg::Imu imu_data_;
  // ----- Subscribers & Publishers -----
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  // ----- Timers -----
  // ----- Callbacks -----
  void PublishCallback(const ImuMsg& imu_msg);

  bool ReadParameters();
  bool SetupHardware();
  bool SetupInterfaces();
};  // ImuSensorNode
}  // namespace westonrobot
#endif /* IMU_SENSOR_NODE_HPP */
