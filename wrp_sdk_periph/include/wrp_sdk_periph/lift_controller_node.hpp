/**
 * @file lift_controller_node.hpp
 * @brief Periph node for lift controller
 * @date 17-05-2024
 *
 * Weston Robot's lift controller node.
 *
 * @copyright Copyright (c) 2024 Weston Robot Pte. Ltd.
 */

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "wrp_sdk/peripheral/lift_controller.hpp"

#include "wrp_sdk_msgs/msg/lift_state.hpp"
#include "wrp_sdk_msgs/srv/lift_query.hpp"
#include "wrp_sdk_msgs/action/lift_control.hpp"

namespace westonrobot {
class LiftControllerNode : public rclcpp::Node {
 public:
  LiftControllerNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~LiftControllerNode() = default;

 private:
  // ----- ROS Node Parameters -----
  std::string device_path_;
  int baud_rate_;
  double sampling_freq_;
  // ----- Internal Variables -----
  std::unique_ptr<LiftController> lift_controller_;
  // ----- Published Messages-----
  // ----- Subscribers & Publishers & Services -----
  rclcpp::Publisher<wrp_sdk_msgs::msg::LiftState>::SharedPtr lift_state_pub_;

  rclcpp::Service<wrp_sdk_msgs::srv::LiftQuery>::SharedPtr lift_query_srv_;

  rclcpp_action::Server<wrp_sdk_msgs::action::LiftControl>::SharedPtr
      lift_control_server_;
  // ----- Timers -----
  rclcpp::TimerBase::SharedPtr timer_;
  // ----- Callbacks -----
  void PublishLiftState();
  void LiftQueryCallback(
      const std::shared_ptr<wrp_sdk_msgs::srv::LiftQuery::Request> request,
      std::shared_ptr<wrp_sdk_msgs::srv::LiftQuery::Response> response);

  rclcpp_action::GoalResponse LiftControlGoalCallback(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const wrp_sdk_msgs::action::LiftControl::Goal> goal);
  rclcpp_action::CancelResponse LiftControlCancelCallback(
      const std::shared_ptr<
          rclcpp_action::ServerGoalHandle<wrp_sdk_msgs::action::LiftControl>>
          goal_handle);
  void LiftControlAcceptedCallback(
      const std::shared_ptr<
          rclcpp_action::ServerGoalHandle<wrp_sdk_msgs::action::LiftControl>>
          goal_handle);
  void ExecuteLiftControl(
      const std::shared_ptr<
          rclcpp_action::ServerGoalHandle<wrp_sdk_msgs::action::LiftControl>>
          goal_handle);

  bool ReadParameters();
  bool SetupHardware();
  bool SetupInterfaces();
};  // LiftControllerNode

}  // namespace westonrobot
