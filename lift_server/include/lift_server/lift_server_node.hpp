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
#include "std_msgs/msg/u_int16.hpp"

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
  std::string port_name_;
  int baud_rate_;
  // ----- Internal Variables -----
  std::unique_ptr<CameraLift> lift_;
  rclcpp_action::Server<LiftGoal>::SharedPtr action_server_;
  // ----- Published Messages-----
  // ----- Subscribers & Publishers -----
  // rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_;
  // rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_;
  // ----- Timers -----
  // rclcpp::TimerBase::SharedPtr timer_;
  // ----- Callbacks -----
  // void Callback(const std_msgs::msg::Empty::SharedPtr msg);
  rclcpp_action::GoalResponse HandleGoal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const LiftGoal::Goal> goal);

  rclcpp_action::CancelResponse HandleCancel(
      const std::shared_ptr<GoalHandleLiftGoal> goal_handle);

  void HandleAccepted(const std::shared_ptr<GoalHandleLiftGoal> goal_handle);

  void GoalCallback(const std::shared_ptr<GoalHandleLiftGoal> goal_handle);
  bool ReadParameters();
};
}  // namespace lift_server

#endif /* LIFT_SERVER_NODE_HPP */
