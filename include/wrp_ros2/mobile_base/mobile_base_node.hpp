/**
 * mobile_base_node.hpp
 *
 * Created on Tue Dec 21 2021 11:22:47
 *
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */
#ifndef MOBILE_BASE_NODE_HPP
#define MOBILE_BASE_NODE_HPP

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/transform_broadcaster.h>

#include "wrp_sdk/interface/mobile_robot_interface.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "wrp_ros2/msg/system_state.hpp"
#include "wrp_ros2/msg/motion_state.hpp"
#include "wrp_ros2/msg/actuator_state_array.hpp"
#include "wrp_ros2/msg/actuator_state.hpp"
#include "wrp_ros2/msg/range_data.hpp"
#include "wrp_ros2/msg/range_data_array.hpp"
#include "wrp_ros2/msg/motion_command.hpp"

#include "wrp_ros2/srv/access_control.hpp"
#include "wrp_ros2/srv/assisted_mode_control.hpp"
#include "wrp_ros2/srv/light_control.hpp"
#include "wrp_ros2/srv/motion_reset.hpp"

namespace westonrobot {
class MobileBaseNode : public rclcpp::Node {
 public:
  MobileBaseNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~MobileBaseNode() {};

 private:
  // ----- ROS Node Parameters -----
  std::string robot_type_;
  std::string can_device_;
  std::string base_frame_;
  std::string odom_frame_;
  bool auto_reconnect_;
  std::string motion_type_;
  // ----- Internal Variables -----
  std::shared_ptr<MobileRobotInterface> robot_ = nullptr;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Time last_time_;
  float position_x_ = 0.0;
  float position_y_ = 0.0;
  float theta_ = 0.0;
  float loop_period_ = 0.02; // in secs
  // ----- Published Messages-----
  // ----- Subscribers & Publishers & Services -----
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      motion_cmd_subscriber_;

  rclcpp::Publisher<wrp_ros2::msg::SystemState>::SharedPtr
      system_state_publisher_;
  rclcpp::Publisher<wrp_ros2::msg::MotionState>::SharedPtr
      motion_state_publisher_;
  rclcpp::Publisher<wrp_ros2::msg::ActuatorStateArray>::SharedPtr
      actuator_state_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<wrp_ros2::msg::RangeDataArray>::SharedPtr tof_publisher_;
  rclcpp::Publisher<wrp_ros2::msg::RangeDataArray>::SharedPtr ultrasonic_publisher_;

  rclcpp::Service<wrp_ros2::srv::AccessControl>::SharedPtr
      access_control_service_;
  rclcpp::Service<wrp_ros2::srv::AssistedModeControl>::SharedPtr
      assisted_mode_control_service_;
  rclcpp::Service<wrp_ros2::srv::LightControl>::SharedPtr
      light_control_service_;
  rclcpp::Service<wrp_ros2::srv::MotionReset>::SharedPtr motion_reset_service_;
  //  ----- Timers -----
  rclcpp::TimerBase::SharedPtr publish_timer_;
  //  ----- Callbacks -----
  void MotionCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void AccessControlCallback(
      const wrp_ros2::srv::AccessControl::Request::SharedPtr request,
      wrp_ros2::srv::AccessControl::Response::SharedPtr response);
  void AssistedModeControlCallback(
      const wrp_ros2::srv::AssistedModeControl::Request::SharedPtr request,
      wrp_ros2::srv::AssistedModeControl::Response::SharedPtr response);
  void LightControlCallback(
      const wrp_ros2::srv::LightControl::Request::SharedPtr request,
      wrp_ros2::srv::LightControl::Response::SharedPtr response);
  void MotionResetCallback(
      const wrp_ros2::srv::MotionReset::Request::SharedPtr request,
      wrp_ros2::srv::MotionReset::Response::SharedPtr response);
  void PublishLoopCallback();

  bool ReadParameters();
  bool SetupRobot();
  bool SetupInterfaces();
  void PublishRobotState();
  void PublishSensorData();
  void PublishWheelOdometry();
  geometry_msgs::msg::Quaternion CreateQuaternionMsgFromYaw(double yaw);
  nav_msgs::msg::Odometry CalculateOdometry(geometry_msgs::msg::Twist robot_twist);
};  // MobileBaseNode
}  // namespace westonrobot
#endif /* MOBILE_BASE_NODE_HPP */
