/**
 * @file mobile_base_node.hpp
 * @brief 
 * @date 03-05-2024
 * 
 * @copyright Copyright (c) 2024 Weston Robot Pte. Ltd.
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
#include <sensor_msgs/msg/battery_state.hpp>
#include "wrp_sdk_msgs/msg/system_state.hpp"
#include "wrp_sdk_msgs/msg/motion_state.hpp"
#include "wrp_sdk_msgs/msg/actuator_state_array.hpp"
#include "wrp_sdk_msgs/msg/actuator_state.hpp"
#include "wrp_sdk_msgs/msg/range_data.hpp"
#include "wrp_sdk_msgs/msg/range_data_array.hpp"
#include "wrp_sdk_msgs/msg/motion_command.hpp"

#include "wrp_sdk_msgs/srv/access_control.hpp"
#include "wrp_sdk_msgs/srv/assisted_mode_control.hpp"
#include "wrp_sdk_msgs/srv/light_control.hpp"
#include "wrp_sdk_msgs/srv/motion_reset.hpp"

namespace westonrobot {
class MobileBaseNode : public rclcpp::Node {
 public:
  MobileBaseNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~MobileBaseNode(){};

 private:
  enum class RobotVariant {
    // AglieX
    kAgilexScoutV2 = 0,
    kAgilexScoutMini,
    kAgilexScoutMiniOmni,
    kAgilexRangerMiniV1,
    kAgilexRangerMiniV2,
    kAgilexTracer,
    kAgilexTracerMini,
    kAgilexHunter,
    kAgilexHunterSE,
    kAgilexBunker,

    // WestonRobot
    kWRScout,
    kWRVBot,

    // BangBang
    kBangBangRobooterX,

    kNumOfVariants
  };

  // ----- ROS Node Parameters -----
  int robot_type_;            /**< Robot Type/Variant*/
  std::string can_device_;    /**< CAN device to use*/
  std::string base_frame_;    /**< Robot base frame name*/
  std::string odom_frame_;    /**< Odometry frame name*/
  bool publish_odom_tf_;      /**< If publish odom_frame->base_frame tf*/
  bool auto_request_control_; /**< If auto request for control*/
  // ----- Internal Variables -----
  RobotVariant robot_variant_;
  std::shared_ptr<MobileRobotInterface> robot_ = nullptr;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Time last_time_;
  float position_x_ = 0.0;
  float position_y_ = 0.0;
  float theta_ = 0.0;
  float loop_period_ = 0.02;  // in secs
  // ----- Published Messages-----
  // ----- Subscribers & Publishers & Services -----
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      motion_cmd_subscriber_;

  rclcpp::Publisher<wrp_sdk_msgs::msg::SystemState>::SharedPtr
      system_state_publisher_;
  rclcpp::Publisher<wrp_sdk_msgs::msg::MotionState>::SharedPtr
      motion_state_publisher_;
  rclcpp::Publisher<wrp_sdk_msgs::msg::ActuatorStateArray>::SharedPtr
      actuator_state_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr
      battery_state_publisher_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<wrp_sdk_msgs::msg::RangeDataArray>::SharedPtr
      tof_data_publisher_;
  rclcpp::Publisher<wrp_sdk_msgs::msg::RangeDataArray>::SharedPtr
      ultrasonic_data_publisher_;

  rclcpp::Service<wrp_sdk_msgs::srv::AccessControl>::SharedPtr
      access_control_service_;
  rclcpp::Service<wrp_sdk_msgs::srv::AssistedModeControl>::SharedPtr
      assisted_mode_control_service_;
  rclcpp::Service<wrp_sdk_msgs::srv::LightControl>::SharedPtr
      light_control_service_;
  rclcpp::Service<wrp_sdk_msgs::srv::MotionReset>::SharedPtr motion_reset_service_;
  //  ----- Timers -----
  rclcpp::TimerBase::SharedPtr publish_timer_;
  //  ----- Callbacks -----
  void MotionCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void AccessControlCallback(
      const wrp_sdk_msgs::srv::AccessControl::Request::SharedPtr request,
      wrp_sdk_msgs::srv::AccessControl::Response::SharedPtr response);
  void AssistedModeControlCallback(
      const wrp_sdk_msgs::srv::AssistedModeControl::Request::SharedPtr request,
      wrp_sdk_msgs::srv::AssistedModeControl::Response::SharedPtr response);
  void LightControlCallback(
      const wrp_sdk_msgs::srv::LightControl::Request::SharedPtr request,
      wrp_sdk_msgs::srv::LightControl::Response::SharedPtr response);
  void MotionResetCallback(
      const wrp_sdk_msgs::srv::MotionReset::Request::SharedPtr request,
      wrp_sdk_msgs::srv::MotionReset::Response::SharedPtr response);
  void PublishLoopCallback();

  bool ReadParameters();
  bool SetupHardware();
  bool SetupInterfaces();

  void PublishSystemState();
  void PublishBatteryState();
  void PublishActuatorState();
  void PublishOdometry();
  nav_msgs::msg::Odometry CalculateOdometry(
      geometry_msgs::msg::Twist robot_twist);
  void PublishRcState() {};
  void PublishLightState() {};

  void PublishSensorData();

  geometry_msgs::msg::Quaternion CreateQuaternionMsgFromYaw(double yaw);
};  // MobileBaseNode
}  // namespace westonrobot
#endif /* MOBILE_BASE_NODE_HPP */