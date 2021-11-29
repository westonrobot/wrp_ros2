#ifndef LIFT_SERVER_NODE_HPP
#define LIFT_SERVER_NODE_HPP

// C++
#include <functional>
#include <memory>
#include <thread>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

// wrp_sdk
#include "wrp_sdk/peripheral/camera_lift.hpp"

// interfaces
#include "sdk_interfaces/msg/lift_state.hpp"
#include "sdk_interfaces/action/lift_goal.hpp"
#include "std_msgs/msg/int8.hpp"

namespace lift_server {
using namespace westonrobot;
class LiftServerNode : public rclcpp::Node {
 public:
  explicit LiftServerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~LiftServerNode();

  using LiftGoal = sdk_interfaces::action::LiftGoal;
  using GoalHandleLiftGoal = rclcpp_action::ServerGoalHandle<LiftGoal>;

 private:
  // ----- ROS Node Parameters -----
  std::string device_path_;
  int baud_rate_;
  int publish_interval_;
  bool command_preemption_;
  // ----- Internal Variables -----
  std::unique_ptr<CameraLift> lift_;
  rclcpp_action::Server<LiftGoal>::SharedPtr action_server_;
  bool position_control_active_ = true;
  // ----- Published Messages-----
  sdk_interfaces::msg::LiftState lift_state_;
  // ----- Subscribers & Publishers -----
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr speed_control_sub_;
  rclcpp::Publisher<sdk_interfaces::msg::LiftState>::SharedPtr state_pub_;
  // ----- Timers -----
  rclcpp::TimerBase::SharedPtr timer_;
  // ----- Callbacks -----
  rclcpp_action::GoalResponse HandleGoal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const LiftGoal::Goal> goal);
  rclcpp_action::CancelResponse HandleCancel(
      const std::shared_ptr<GoalHandleLiftGoal> goal_handle);
  void HandleAccepted(const std::shared_ptr<GoalHandleLiftGoal> goal_handle);
  void GoalCallback(const std::shared_ptr<GoalHandleLiftGoal> goal_handle);
  void SpeedCmdCallback(const std_msgs::msg::Int8::SharedPtr speed);
  void PublishStateCallback();
  bool ReadParameters();
};
}  // namespace lift_server

#endif /* LIFT_SERVER_NODE_HPP */
