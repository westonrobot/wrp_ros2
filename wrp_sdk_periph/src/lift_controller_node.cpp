/**
 * @file lift_controller_node.cpp
 * @brief
 * @date 17-05-2024
 *
 * @copyright Copyright (c) 2024 Weston Robot Pte. Ltd.
 */
#include "wrp_sdk_periph/lift_controller_node.hpp"

#include <thread>

namespace westonrobot {
LiftControllerNode::LiftControllerNode(const rclcpp::NodeOptions& options)
    : Node("lfit_controller_node", options) {
  if (!LiftControllerNode::ReadParameters()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Could not load parameters");
    rclcpp::shutdown();
  }

  if (!LiftControllerNode::SetupHardware()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Could not setup hardware");
    rclcpp::shutdown();
  }

  if (!LiftControllerNode::SetupInterfaces()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Could not setup ros interfaces");
    rclcpp::shutdown();
  }
}

bool LiftControllerNode::ReadParameters() {
  // Declare default parameters
  this->declare_parameter<std::string>("device_path", "/dev/ttyUSB0");
  this->declare_parameter<int>("baud_rate", 115200);
  this->declare_parameter<double>("sampling_freq", 10.0);

  // Get parameters
  RCLCPP_INFO_STREAM(this->get_logger(), "--- Parameters loaded are ---");

  this->get_parameter("device_path", device_path_);
  RCLCPP_INFO_STREAM(this->get_logger(), "device_path: " << device_path_);

  this->get_parameter("baud_rate", baud_rate_);
  RCLCPP_INFO_STREAM(this->get_logger(), "baud_rate: " << baud_rate_);

  this->get_parameter("sampling_freq", sampling_freq_);
  RCLCPP_INFO_STREAM(this->get_logger(), "sampling_freq: " << sampling_freq_);

  RCLCPP_INFO_STREAM(this->get_logger(), "-----------------------------");

  return true;
}

bool LiftControllerNode::SetupHardware() {
  lift_controller_ = std::make_unique<LiftController>();

  if (!lift_controller_->Connect(device_path_, baud_rate_)) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "Could not connect to lift controller");
    return false;
  }

  return true;
}

bool LiftControllerNode::SetupInterfaces() {
  using namespace std::placeholders;

  lift_state_pub_ = this->create_publisher<wrp_sdk_msgs::msg::LiftState>(
      "~/lift_status", rclcpp::SensorDataQoS());

  lift_query_srv_ = this->create_service<wrp_sdk_msgs::srv::LiftQuery>(
      "~/lift_query",
      std::bind(&LiftControllerNode::LiftQueryCallback, this, _1, _2));

  lift_control_server_ =
      rclcpp_action::create_server<wrp_sdk_msgs::action::LiftControl>(
          this, "~/lift_control",
          std::bind(&LiftControllerNode::LiftControlGoalCallback, this, _1, _2),
          std::bind(&LiftControllerNode::LiftControlCancelCallback, this, _1),
          std::bind(&LiftControllerNode::LiftControlAcceptedCallback, this,
                    _1));

  auto period_ns = rclcpp::Rate(sampling_freq_).period();
  timer_ = rclcpp::create_timer(
      this, this->get_clock(), period_ns,
      std::bind(&LiftControllerNode::PublishLiftState, this));
  return true;
}

void LiftControllerNode::PublishLiftState() {
  for (uint8_t id = 0; id < 2; id++) {
    auto state = lift_controller_->GetLiftState(id);
    auto state_msg = wrp_sdk_msgs::msg::LiftState();

    state_msg.id = id;
    state_msg.position = state.position;
    lift_state_pub_->publish(state_msg);
  }
}

void LiftControllerNode::LiftQueryCallback(
    const std::shared_ptr<wrp_sdk_msgs::srv::LiftQuery::Request> request,
    std::shared_ptr<wrp_sdk_msgs::srv::LiftQuery::Response> response) {
  auto state = lift_controller_->GetLiftState(request->id);
  response->position = state.position;
  response->speed = state.speed;
  return;
}

rclcpp_action::GoalResponse LiftControllerNode::LiftControlGoalCallback(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const wrp_sdk_msgs::action::LiftControl::Goal> goal) {
  (void)uuid;
  using wrp_sdk_msgs::action::LiftControl;

  // Check if the goal is valid
  if (goal->id != LiftControl::Goal::LIFT_HORIZONTAL ||
      goal->id != LiftControl::Goal::LIFT_VERTICAL) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Invalid lift id");
    return rclcpp_action::GoalResponse::REJECT;
  }

  auto state = lift_controller_->GetLiftState(goal->id);
  if (goal->position == state.position) {
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "Current position already in goal position");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "Lift control goal accepted");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LiftControllerNode::LiftControlCancelCallback(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<wrp_sdk_msgs::action::LiftControl>>
        goal_handle) {
  (void)goal_handle;
  RCLCPP_ERROR_STREAM(this->get_logger(),
                     "Lift control goal cannot be canceled once accepted");

  return rclcpp_action::CancelResponse::REJECT;
}

void LiftControllerNode::LiftControlAcceptedCallback(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<wrp_sdk_msgs::action::LiftControl>>
        goal_handle) {
  using namespace std::placeholders;

  std::thread{std::bind(&LiftControllerNode::ExecuteLiftControl, this, _1),
              goal_handle}
      .detach();
}

void LiftControllerNode::ExecuteLiftControl(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<wrp_sdk_msgs::action::LiftControl>>
        goal_handle) {
  RCLCPP_INFO_STREAM(this->get_logger(), "Executing lift control goal");

  using wrp_sdk_msgs::action::LiftControl;
  rclcpp::Rate loop_rate(2);

  auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<LiftControl::Feedback>();
  auto result = std::make_shared<LiftControl::Result>();

  auto state = lift_controller_->GetLiftState(goal->id);
  auto target_position = goal->position;

  // Move to target position
  lift_controller_->SendCommandToLift(goal->position, goal->speed, goal->id);

  // Wait for the lift to reach the target position
  do {
    state = lift_controller_->GetLiftState(goal->id);

    feedback->id = goal->id;
    feedback->position = state.position;
    feedback->speed = state.speed;
    goal_handle->publish_feedback(feedback);

    loop_rate.sleep();
  } while (state.position != target_position && rclcpp::ok());

  result->id = goal->id;
  result->position = state.position;
  result->speed = state.speed;
  goal_handle->succeed(result);
  RCLCPP_INFO_STREAM(this->get_logger(), "Lift control goal succeeded");
}

}  // namespace westonrobot

int main(int argc, char const* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<westonrobot::LiftControllerNode>());
  rclcpp::shutdown();
  return 0;
}