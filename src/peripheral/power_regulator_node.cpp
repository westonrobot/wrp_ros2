/**
 * power_regulator_node.cpp
 *
 * Created on Wed Apr 13 2022 14:24:34
 *
 * Description:
 *
 * Copyright (c) 2022 Weston Robot Pte. Ltd.
 */
#include "wrp_ros2/peripheral/power_regulator_node.hpp"

#include "wrp_sdk/peripheral/power_regulator_v2.hpp"
using namespace std::placeholders;

namespace westonrobot {
PowerRegulatorNode::PowerRegulatorNode(const rclcpp::NodeOptions& options)
    : Node("power_regulator_node", options) {
  if (!PowerRegulatorNode::ReadParameters()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Could not load parameters");
    rclcpp::shutdown();
  }

  if (!PowerRegulatorNode::SetupInterfaces()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Could not setup ros interfaces");
    rclcpp::shutdown();
  }

  if (!PowerRegulatorNode::SetupHardware()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Could not setup power regulator");
    rclcpp::shutdown();
  }
}

bool PowerRegulatorNode::ReadParameters() {
  // Declare default parameters
  this->declare_parameter<std::string>("device_path", "can0");

  // Get parameters
  RCLCPP_INFO_STREAM(this->get_logger(), "--- Parameters loaded are ---");

  this->get_parameter("device_path", device_path_);
  RCLCPP_INFO_STREAM(this->get_logger(), "device_path: " << device_path_);

  RCLCPP_INFO_STREAM(this->get_logger(), "-----------------------------");

  return true;
}

bool PowerRegulatorNode::SetupInterfaces() {
  device_state_publisher_ =
      this->create_publisher<wrp_ros2::msg::PowerRegulatorDeviceState>(
          "~/state", 5);

  output_control_service_ =
      this->create_service<wrp_ros2::srv::PowerRegulatorControl>(
          "~/cmd", std::bind(&PowerRegulatorNode::HandleCommand, this,
                             std::placeholders::_1, std::placeholders::_2));

  return true;
}

bool PowerRegulatorNode::SetupHardware() {
  regulator_ = std::unique_ptr<PowerRegulatorV2>(new PowerRegulatorV2());

  if (!regulator_->Connect(device_path_)) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "Failed to connect to " << device_path_);
    return false;
  }

  regulator_->SetDataReceivedCallback(std::bind(
      &PowerRegulatorNode::PublishCallback, this, std::placeholders::_1));

  return true;
}

void PowerRegulatorNode::PublishCallback(
    const PowerRegulatorInterface::DeviceState& device_state) {
  wrp_ros2::msg::PowerRegulatorDeviceState state_msg;

  state_msg.input_voltage = device_state.input_voltage;
  state_msg.fan_speed = device_state.fan_speed;
  state_msg.temperature = device_state.temperature;

  state_msg.channels[0].name = "19V";
  state_msg.channels[0].enabled =
      device_state.channels.at(PowerRegulatorV2::kChannel19V).enabled;
  state_msg.channels[0].voltage =
      device_state.channels.at(PowerRegulatorV2::kChannel19V).voltage;
  state_msg.channels[0].current =
      device_state.channels.at(PowerRegulatorV2::kChannel19V).current;

  state_msg.channels[1].name = "12V";
  state_msg.channels[1].enabled =
      device_state.channels.at(PowerRegulatorV2::kChannel12V).enabled;
  state_msg.channels[1].voltage =
      device_state.channels.at(PowerRegulatorV2::kChannel12V).voltage;
  state_msg.channels[1].current =
      device_state.channels.at(PowerRegulatorV2::kChannel12V).current;

  state_msg.channels[2].name = "5V isolated";
  state_msg.channels[2].enabled =
      device_state.channels.at(PowerRegulatorV2::kChannel5Vi).enabled;
  state_msg.channels[2].voltage =
      device_state.channels.at(PowerRegulatorV2::kChannel5Vi).voltage;
  state_msg.channels[2].current =
      device_state.channels.at(PowerRegulatorV2::kChannel5Vi).current;

  state_msg.channels[3].name = "12V isolated";
  state_msg.channels[3].enabled =
      device_state.channels.at(PowerRegulatorV2::kChannel12Vi).enabled;
  state_msg.channels[3].voltage =
      device_state.channels.at(PowerRegulatorV2::kChannel12Vi).voltage;
  state_msg.channels[3].current =
      device_state.channels.at(PowerRegulatorV2::kChannel12Vi).current;

  device_state_publisher_->publish(state_msg);
}

void PowerRegulatorNode::HandleCommand(
    const wrp_ros2::srv::PowerRegulatorControl::Request::SharedPtr request,
    wrp_ros2::srv::PowerRegulatorControl::Response::SharedPtr response) {
  PowerRegulatorInterface::OutputChannel chn;
  if (request->channel ==
      wrp_ros2::srv::PowerRegulatorControl::Request::CHANNEL_19V) {
    chn = PowerRegulatorV2::kChannel19V;
  } else if (request->channel ==
             wrp_ros2::srv::PowerRegulatorControl::Request::CHANNEL_12V) {
    chn = PowerRegulatorV2::kChannel12V;
  } else if (request->channel ==
             wrp_ros2::srv::PowerRegulatorControl::Request::CHANNEL_5VI) {
    chn = PowerRegulatorV2::kChannel5Vi;
  } else if (request->channel ==
             wrp_ros2::srv::PowerRegulatorControl::Request::CHANNEL_12VI) {
    chn = PowerRegulatorV2::kChannel12Vi;
  } else {
    response->success = false;
    return;
  }

  regulator_->SetChannelState(chn, request->enable);
  response->success = true;

  return;
}
}  // namespace westonrobot

int main(int argc, char const* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<westonrobot::PowerRegulatorNode>());
  rclcpp::shutdown();
  return 0;
}