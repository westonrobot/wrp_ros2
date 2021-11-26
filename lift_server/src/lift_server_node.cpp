/**
 * lift_server_node.cpp
 *
 * Created on Thu Nov 25 2021 17:26:22
 *
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */
#include "lift_server/lift_server_node.hpp"
using namespace std::placeholders;

namespace lift_server {
using namespace westonrobot;

LiftServerNode::LiftServerNode() : Node("lift_server_node") {
  if (!LiftServerNode::ReadParameters()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Could not load parameters");
    rclcpp::shutdown();
  }

  lift_ = std::make_unique<CameraLift>();

  if (!lift_->Connect(port_name_, baud_rate_)) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Could not setup lift!!!");
    rclcpp::shutdown();
  }

  goal_server_ = rclcpp_action::create_server<LiftGoal>(
      this, "lift_position_control",
      std::bind(&LiftServerNode::HandleGoal, this, _1, _2),
      std::bind(&LiftServerNode::HandleCancel, this, _1),
      std::bind(&LiftServerNode::HandleAccepted, this, _1));
}

bool LiftServerNode::ReadParameters() {
  // Declare default parameters
  this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
  this->declare_parameter<int>("baud_rate", 115200);

  // Get parameters
  RCLCPP_INFO_STREAM(this->get_logger(), "--- Parameters loaded are ---");

  this->get_parameter("port_name", port_name_);
  RCLCPP_INFO_STREAM(this->get_logger(), "port_name: " << port_name_);
  RCLCPP_INFO_STREAM(this->get_logger(), "-----------------------------");

  this->get_parameter("baud_rate", baud_rate_);
  RCLCPP_INFO_STREAM(this->get_logger(), "baud_rate: " << baud_rate_);
  RCLCPP_INFO_STREAM(this->get_logger(), "-----------------------------");

  return true;
}

rclcpp_action::GoalResponse LiftServerNode::HandleGoal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const LiftGoal::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "Received goal request with position %d",
              goal->position);
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LiftServerNode::HandleCancel(
    const std::shared_ptr<GoalHandleLiftGoal> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void LiftServerNode::HandleAccepted(
    const std::shared_ptr<GoalHandleLiftGoal> goal_handle) {
  // this needs to return quickly to avoid blocking the executor, so spin up a
  // new thread
  std::thread{std::bind(&LiftServerNode::GoalCallback, this, _1), goal_handle}
      .detach();
}

void LiftServerNode::GoalCallback(
    const std::shared_ptr<GoalHandleLiftGoal> goal_handle) {
  (void)goal_handle;
}

}  // namespace lift_server
