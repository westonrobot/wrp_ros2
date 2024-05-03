/**
 * power_regulator_node.hpp
 *
 * Created on Wed Apr 13 2022 14:24:40
 *
 * Description:
 *
 * Copyright (c) 2022 Weston Robot Pte. Ltd.
 */

#ifndef POWER_REGULATOR_NODE_HPP
#define POWER_REGULATOR_NODE_HPP

// C++
#include <string>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/time.hpp"

// wrp_sdk
#include "wrp_sdk/interface/power_regulator_interface.hpp"

// ROS Messages
#include "wrp_sdk_msgs/msg/power_regulator_channel_state.hpp"
#include "wrp_sdk_msgs/msg/power_regulator_device_state.hpp"
#include "wrp_sdk_msgs/srv/power_regulator_control.hpp"

namespace westonrobot {
class PowerRegulatorNode : public rclcpp::Node {
 public:
  PowerRegulatorNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~PowerRegulatorNode(){};

 private:
  // ----- ROS Node Parameters -----
  std::string device_path_ = "can0";
  // ----- Internal Variables -----
  std::shared_ptr<PowerRegulatorInterface> regulator_;
  // ----- Published Messages-----
  wrp_sdk_msgs::msg::PowerRegulatorDeviceState device_state_;
  // ----- Subscribers & Publishers & Services -----
  rclcpp::Publisher<wrp_sdk_msgs::msg::PowerRegulatorDeviceState>::SharedPtr
      device_state_publisher_;

  rclcpp::Service<wrp_sdk_msgs::srv::PowerRegulatorControl>::SharedPtr
      output_control_service_;

  // ----- Callbacks -----
  void PublishCallback(const PowerRegulatorInterface::DeviceState& device_state);
  void HandleCommand(
      const wrp_sdk_msgs::srv::PowerRegulatorControl::Request::SharedPtr request,
      wrp_sdk_msgs::srv::PowerRegulatorControl::Response::SharedPtr response);

  bool ReadParameters();
  bool SetupHardware();
  bool SetupInterfaces();
};  // PowerRegulatorNode
}  // namespace westonrobot

#endif /* POWER_REGULATOR_NODE_HPP */
