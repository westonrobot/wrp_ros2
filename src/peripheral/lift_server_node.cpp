/**
 * lift_server_node.cpp
 *
 * Created on Thu Nov 25 2021 17:26:22
 *
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */
#include "wrp_ros2/peripheral/lift_server_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"
using namespace std::placeholders;

namespace westonrobot {

LiftServerNode::LiftServerNode(const rclcpp::NodeOptions& options)
    : Node("lift_server_node", options) {
  if (!LiftServerNode::ReadParameters()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Could not load parameters");
    rclcpp::shutdown();
  }

  lift_ = std::make_unique<CameraLift>();

  if (!lift_->Connect(device_path_, baud_rate_)) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Could not setup lift!!!");
    rclcpp::shutdown();
  }

  speed_control_sub_ = this->create_subscription<std_msgs::msg::Int8>(
      "/lift_server/speed_cmd", 1,
      std::bind(&LiftServerNode::SpeedCmdCallback, this, _1));

  state_pub_ = this->create_publisher<wrp_ros2::msg::LiftState>(
      "/lift_server/state", 1);

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(publish_interval_),
      std::bind(&LiftServerNode::PublishStateCallback, this));

  action_server_ = rclcpp_action::create_server<LiftGoal>(
      this, "lift_position_control",
      std::bind(&LiftServerNode::HandleGoal, this, _1, _2),
      std::bind(&LiftServerNode::HandleCancel, this, _1),
      std::bind(&LiftServerNode::HandleAccepted, this, _1));
}

LiftServerNode::~LiftServerNode() {}

bool LiftServerNode::ReadParameters() {
  // Declare default parameters
  this->declare_parameter<std::string>("device_path", "/dev/ttyUSB0");
  this->declare_parameter<int>("baud_rate", 115200);
  this->declare_parameter<int>("publish_interval", 500);
  this->declare_parameter<bool>("command_preemption", true);

  // Get parameters
  RCLCPP_INFO_STREAM(this->get_logger(), "--- Parameters loaded are ---");

  this->get_parameter("device_path", device_path_);
  RCLCPP_INFO_STREAM(this->get_logger(), "device_path: " << device_path_);

  this->get_parameter("baud_rate", baud_rate_);
  RCLCPP_INFO_STREAM(this->get_logger(), "baud_rate: " << baud_rate_);

  this->get_parameter("publish_interval", publish_interval_);
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "publish_interval: " << publish_interval_);

  this->get_parameter("command_preemption", command_preemption_);
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "command_preemption: " << command_preemption_);

  RCLCPP_INFO_STREAM(this->get_logger(), "-----------------------------");

  return true;
}

rclcpp_action::GoalResponse LiftServerNode::HandleGoal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const LiftGoal::Goal> goal) {
  RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Received goal request with position %d" << goal->position);
  (void)uuid;
  if (position_control_active_ && !command_preemption_) {
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Goal command cannot pre-empt previous speed command");
    return rclcpp_action::GoalResponse::REJECT;
  } else if (position_control_active_) {
    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Goal command attempting pre-empt of previous speed command");
  }
  position_control_active_ = true;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LiftServerNode::HandleCancel(
    const std::shared_ptr<GoalHandleLiftGoal> goal_handle) {
  RCLCPP_INFO_STREAM(this->get_logger(), "Received request to cancel goal");
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
  RCLCPP_INFO_STREAM(this->get_logger(), "Executing lift command");
  rclcpp::Rate loop_rate(10);
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<LiftGoal::Feedback>();
  auto result = std::make_shared<LiftGoal::Result>();
  uint8_t goal_position = 0;

  if (lift_->IsOkay()) {   // Check if lift connected
    switch (goal->type) {  // check goal type and send accordingly
      case LiftGoal::Goal::RESET_CMD: {
        lift_->ResetState();
        goal_position = 0;
        break;
      }
      case LiftGoal::Goal::POSITION_CMD: {
        lift_->SendCommand(goal->position, goal->speed);
        goal_position = goal->position;
        break;
      }
      default:  // invalid type
        return;
        break;
    }

    while (rclcpp::ok() && lift_->IsOkay()) {
      // Check if cancel request received
      if (goal_handle->is_canceling()) {
        result->reached = (lift_->GetLiftState().position == goal_position);
        goal_handle->canceled(result);
        RCLCPP_INFO_STREAM(this->get_logger(), "Goal canceled");
        position_control_active_ = false;
        return;
      }
      position_control_active_ = true;
      auto lift_state = lift_->GetLiftState();
      // update result and feedback
      result->reached = (lift_state.position == goal_position);
      if (!result->reached) {
        feedback->position = lift_state.position;
        feedback->speed = lift_state.speed;
        feedback->lift_connected = true;
        goal_handle->publish_feedback(feedback);
        position_control_active_ = true;
      } else {
        result->reached = true;
        goal_handle->succeed(result);
        RCLCPP_INFO_STREAM(this->get_logger(), "Goal succeeded");
        position_control_active_ = false;
        return;
      }
      loop_rate.sleep();
    }

    // Should not reach here when normal
    feedback->lift_connected = false;
    goal_handle->publish_feedback(feedback);  // publish last know feedback
    result->reached = false;
    goal_handle->succeed(result);
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "Lift disconnected before goal reached");

  } else {  // lift not okay
    result->reached = false;
    goal_handle->canceled(result);  // return last know result
    RCLCPP_WARN_STREAM(this->get_logger(), "Lift not okay");
    return;
  }
}

void LiftServerNode::SpeedCmdCallback(
    const std_msgs::msg::Int8::SharedPtr speed) {
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Received speed command of " << speed->data);
  if (lift_->IsOkay()) {
    if (position_control_active_ && !command_preemption_) {
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Speed command cannot pre-empt previous goal command");
      return;
    }
    lift_->SendCommand(speed->data);
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "Lift not okay");
  }
}

void LiftServerNode::PublishStateCallback() {
  if (lift_->IsOkay()) {
    auto lift_state = lift_->GetLiftState();
    lift_state_.position = lift_state.position;
    lift_state_.speed = lift_state.speed;
    state_pub_->publish(lift_state_);
  }
}

}  // namespace westonrobot

RCLCPP_COMPONENTS_REGISTER_NODE(westonrobot::LiftServerNode)