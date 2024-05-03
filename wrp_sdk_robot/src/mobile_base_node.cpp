/**
 * @file mobile_base_node.cpp
 * @brief
 * @date 03-05-2024
 *
 * @copyright Copyright (c) 2024 Weston Robot Pte. Ltd.
 */
#include "wrp_sdk_robot/mobile_base_node.hpp"

#include "wrp_sdk/mobile_base/westonrobot/mobile_base.hpp"
#include "wrp_sdk/mobile_base/agilex/agilex_base_v2_adapter.hpp"
#include "wrp_sdk/mobile_base/bangbang/robooterx_base_adapter.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/transform_datatypes.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
using namespace std::placeholders;

namespace westonrobot {

MobileBaseNode::MobileBaseNode(const rclcpp::NodeOptions& options)
    : Node("mobile_base_node", options) {
  if (!MobileBaseNode::ReadParameters()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Could not load parameters");
    rclcpp::shutdown();
  }

  if (!MobileBaseNode::SetupHardware()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to setup robot");
    rclcpp::shutdown();
  }

  if (!MobileBaseNode::SetupInterfaces()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to setup interfaces");
    rclcpp::shutdown();
  }
}

bool MobileBaseNode::ReadParameters() {
  // Declare default parameters
  this->declare_parameter<int>("robot_type", 0);
  this->declare_parameter<std::string>("can_device", "can0");
  this->declare_parameter<std::string>("base_frame", "base_link");
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<bool>("auto_reconnect", true);
  this->declare_parameter<bool>("publish_odom", true);

  // Get parameters
  RCLCPP_INFO_STREAM(this->get_logger(), "--- Parameters loaded are ---");

  this->get_parameter("robot_type", robot_type_);
  if (robot_type_ < 0 ||
      robot_type_ >= static_cast<int>(RobotVariant::kNumOfVariants)) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid robot type");
    return false;
  }
  robot_variant_ = static_cast<RobotVariant>(robot_type_);
  RCLCPP_INFO_STREAM(this->get_logger(), "robot_type: " << robot_type_);

  this->get_parameter("can_device", can_device_);
  RCLCPP_INFO_STREAM(this->get_logger(), "can_device: " << can_device_);

  this->get_parameter("base_frame", base_frame_);
  RCLCPP_INFO_STREAM(this->get_logger(), "base_frame: " << base_frame_);

  this->get_parameter("odom_frame", odom_frame_);
  RCLCPP_INFO_STREAM(this->get_logger(), "odom_frame: " << odom_frame_);

  this->get_parameter("auto_reconnect", auto_request_control_);
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "auto_reconnect: " << auto_request_control_);

  this->get_parameter("publish_odom", publish_odom_tf_);
  RCLCPP_INFO_STREAM(this->get_logger(), "publish_odom: " << publish_odom_tf_);

  RCLCPP_INFO_STREAM(this->get_logger(), "-----------------------------");

  return true;
}

bool MobileBaseNode::SetupHardware() {
  // Create appropriate adaptor
  switch (robot_variant_) {
    case RobotVariant::kWRScout: {
      robot_ = std::make_shared<MobileBase>();
      break;
    }
    case RobotVariant::kWRVBot: {
      robot_ = std::make_shared<MobileBase>(true);
      break;
    }
    case RobotVariant::kAgilexScoutV2:
    case RobotVariant::kAgilexScoutMini:
    case RobotVariant::kAgilexScoutMiniOmni:
    case RobotVariant::kAgilexRangerMiniV1:
    case RobotVariant::kAgilexRangerMiniV2:
    case RobotVariant::kAgilexTracer:
    case RobotVariant::kAgilexTracerMini:
    case RobotVariant::kAgilexHunter:
    case RobotVariant::kAgilexHunterSE:
    case RobotVariant::kAgilexBunker: {
      robot_ = std::make_shared<AgilexBaseV2Adapter>();
      break;
    }
    case RobotVariant::kBangBangRobooterX: {
      robot_ = std::make_shared<RobooterXBaseAdapter>();
      break;
    }
    default: {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Unknown robot base type");
      return false;
    }
  }

  // Connect to robot through can device
  if (!robot_->Connect(can_device_)) {
    RCLCPP_ERROR_STREAM(
        this->get_logger(),
        "Failed to connect to robot through port: " << can_device_);
    return false;
  }

  last_time_ = this->now();
  return true;
}

bool MobileBaseNode::SetupInterfaces() {
  // setup subscribers
  motion_cmd_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 5, std::bind(&MobileBaseNode::MotionCmdCallback, this, _1));

  // setup publishers
  system_state_publisher_ =
      this->create_publisher<wrp_sdk_msgs::msg::SystemState>("~/system_state",
                                                             10);
  motion_state_publisher_ =
      this->create_publisher<wrp_sdk_msgs::msg::MotionState>("~/motion_state",
                                                             10);
  actuator_state_publisher_ =
      this->create_publisher<wrp_sdk_msgs::msg::ActuatorStateArray>(
          "~/actuator_state", 10);
  battery_state_publisher_ =
      this->create_publisher<sensor_msgs::msg::BatteryState>("~/battery_state",
                                                             10);
  rc_state_publisher_ =
      this->create_publisher<wrp_sdk_msgs::msg::RcState>("~/rc_state", 10);

  if (publish_odom_tf_) {
    // setup tf broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    odom_publisher_ =
        this->create_publisher<nav_msgs::msg::Odometry>("~/odom", 50);
  }
  ultrasonic_data_publisher_ =
      this->create_publisher<wrp_sdk_msgs::msg::RangeDataArray>(
          "~/ultrasonic_data", 10);
  tof_data_publisher_ =
      this->create_publisher<wrp_sdk_msgs::msg::RangeDataArray>("~/tof_data",
                                                                10);

  // setup services
  access_control_service_ =
      this->create_service<wrp_sdk_msgs::srv::AccessControl>(
          "~/access_control",
          std::bind(&MobileBaseNode::AccessControlCallback, this, _1, _2));
  assisted_mode_control_service_ =
      this->create_service<wrp_sdk_msgs::srv::AssistedModeControl>(
          "~/assisted_mode_control",
          std::bind(&MobileBaseNode::AssistedModeControlCallback, this, _1,
                    _2));
  light_control_service_ =
      this->create_service<wrp_sdk_msgs::srv::LightControl>(
          "~/light_control",
          std::bind(&MobileBaseNode::LightControlCallback, this, _1, _2));
  motion_reset_service_ = this->create_service<wrp_sdk_msgs::srv::MotionReset>(
      "~/motion_reset",
      std::bind(&MobileBaseNode::MotionResetCallback, this, _1, _2));

  // setup timers
  publish_timer_ = this->create_wall_timer(  // 50hz loop rate
      std::chrono::milliseconds(20),
      std::bind(&MobileBaseNode::PublishLoopCallback, this));

  return true;
}

void MobileBaseNode::MotionCmdCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  MotionCommand cmd;

  cmd.linear.x = msg->linear.x;
  cmd.linear.y = msg->linear.y;
  cmd.linear.z = msg->linear.z;
  cmd.angular.x = msg->angular.x;
  cmd.angular.y = msg->angular.y;
  cmd.angular.z = msg->angular.z;

  if (robot_->SdkHasControlToken()) {
    robot_->SetMotionCommand(cmd);
  }
}

void MobileBaseNode::AccessControlCallback(
    const wrp_sdk_msgs::srv::AccessControl::Request::SharedPtr request,
    wrp_sdk_msgs::srv::AccessControl::Response::SharedPtr response) {
  HandshakeReturnCode result;
  switch (request->action_type) {
    case wrp_sdk_msgs::srv::AccessControl::Request::
        ACTION_TYPE_REQUEST_CONTROL: {
      result = robot_->RequestControl();
      response->result_code = static_cast<uint32_t>(result);
      break;
    }
    case wrp_sdk_msgs::srv::AccessControl::Request::
        ACTION_TYPE_RENOUNCE_CONTROL: {
      result = robot_->RenounceControl();
      response->result_code = static_cast<uint32_t>(result);
      break;
    }

    default: {
      response->result_code = 11;
      break;
    }
  }
  return;
}

void MobileBaseNode::AssistedModeControlCallback(
    const wrp_sdk_msgs::srv::AssistedModeControl::Request::SharedPtr request,
    wrp_sdk_msgs::srv::AssistedModeControl::Response::SharedPtr response) {
  AssistedModeSetCommand cmd;
  cmd.enable = request->enable;
  robot_->SetAssistedMode(cmd);
  response->state = request->enable;
  return;
}

void MobileBaseNode::LightControlCallback(
    const wrp_sdk_msgs::srv::LightControl::Request::SharedPtr request,
    wrp_sdk_msgs::srv::LightControl::Response::SharedPtr response) {
  LightCommand cmd;

  cmd.id = request->id;
  cmd.command.mode = static_cast<LightMode>(request->command.mode);
  cmd.command.intensity = request->command.intensity;

  if (robot_->SdkHasControlToken()) {
    robot_->SetLightCommand(cmd);
  }
  auto light_state = robot_->GetLightState();
  response->state.mode = static_cast<uint32_t>(light_state.state.mode);
  response->state.intensity = light_state.state.intensity;
  return;
}

void MobileBaseNode::MotionResetCallback(
    const wrp_sdk_msgs::srv::MotionReset::Request::SharedPtr request,
    wrp_sdk_msgs::srv::MotionReset::Response::SharedPtr response) {
  MotionResetCommand cmd;

  cmd.type = static_cast<MotionResetCommandType>(request->type);

  if (robot_->SdkHasControlToken()) {
    robot_->SetMotionResetCommand(cmd);
    response->result_code =
        wrp_sdk_msgs::srv::MotionReset::Response::MOTION_RESET_SUCCCESS;
  } else {
    response->result_code =
        wrp_sdk_msgs::srv::MotionReset::Response::MOTION_RESET_FAILURE;
  }

  return;
}

void MobileBaseNode::PublishLoopCallback() {
  if (auto_request_control_ && !robot_->SdkHasControlToken()) {
    auto return_code = robot_->RequestControl();
    if (return_code != HandshakeReturnCode::kControlAcquired) {
      RCLCPP_WARN_STREAM(this->get_logger(),
                         "Failed to gain control token, error code: "
                             << static_cast<int>(return_code));
    }
  }

  PublishSystemState();
  PublishBatteryState();
  PublishActuatorState();
  PublishOdometry();
  PublishRcState();

  if (publish_odom_tf_) {
    PublishOdometry();
  }
}

void MobileBaseNode::PublishSystemState() {
  auto system_state = robot_->GetSystemState();
  auto motion_state = robot_->GetMotionState();

  // system state
  wrp_sdk_msgs::msg::SystemState system_state_msg;
  system_state_msg.rc_connected = system_state.rc_connected;
  system_state_msg.error_code = static_cast<uint32_t>(system_state.error_code);
  system_state_msg.operational_state =
      static_cast<uint32_t>(system_state.operational_state);
  system_state_msg.control_state =
      static_cast<uint32_t>(system_state.control_state);

  system_state_publisher_->publish(system_state_msg);

  // motion state
  wrp_sdk_msgs::msg::MotionState motion_state_msg;
  motion_state_msg.desired_linear.x = motion_state.desired_linear.x;
  motion_state_msg.desired_linear.y = motion_state.desired_linear.y;
  motion_state_msg.desired_linear.z = motion_state.desired_linear.z;
  motion_state_msg.desired_angular.x = motion_state.desired_angular.x;
  motion_state_msg.desired_angular.y = motion_state.desired_angular.y;
  motion_state_msg.desired_angular.z = motion_state.desired_angular.z;

  motion_state_msg.source = static_cast<uint32_t>(motion_state.source);
  motion_state_msg.collision_detected = motion_state.collision_detected;
  motion_state_msg.assisted_mode_enabled = motion_state.assisted_mode_enabled;

  motion_state_publisher_->publish(motion_state_msg);
}

void MobileBaseNode::PublishBatteryState() {
  auto battery_state = robot_->GetBatteryState();

  sensor_msgs::msg::BatteryState battery_state_msg;
  battery_state_msg.header.stamp = this->now();
  battery_state_msg.voltage = battery_state.voltage;
  battery_state_msg.current = battery_state.current;
  battery_state_msg.charge = battery_state.charge;
  battery_state_msg.capacity = battery_state.capacity;
  battery_state_msg.design_capacity = battery_state.design_capacity;
  battery_state_msg.percentage = battery_state.percentage / 100.0f;
  battery_state_msg.power_supply_status =
      static_cast<uint8_t>(battery_state.power_supply_status);
  battery_state_msg.power_supply_health =
      static_cast<uint8_t>(battery_state.power_supply_health);
  battery_state_msg.power_supply_technology =
      static_cast<uint8_t>(battery_state.power_supply_technology);
  battery_state_msg.present = battery_state.present;

  battery_state_publisher_->publish(battery_state_msg);
}

void MobileBaseNode::PublishActuatorState() {
  auto actuator_state = robot_->GetActuatorState();

  wrp_sdk_msgs::msg::ActuatorStateArray actuator_state_msg;
  for (size_t i = 0; i < actuator_state.size(); ++i) {
    wrp_sdk_msgs::msg::ActuatorState actuator_msg;
    actuator_msg.id = actuator_state[i].id;
    actuator_msg.motor.rpm = actuator_state[i].motor.rpm;
    actuator_msg.motor.current = actuator_state[i].motor.current;
    actuator_msg.motor.pulse_count = actuator_state[i].motor.pulse_count;
    actuator_msg.driver.driver_voltage =
        actuator_state[i].driver.driver_voltage;
    actuator_msg.driver.driver_temperature =
        actuator_state[i].driver.driver_temperature;
    actuator_msg.driver.motor_temperature =
        actuator_state[i].driver.motor_temperature;
    actuator_msg.driver.driver_state = actuator_state[i].driver.driver_state;
    actuator_state_msg.states.push_back(actuator_msg);
  }
  actuator_state_publisher_->publish(actuator_state_msg);
}

void MobileBaseNode::PublishRcState() {
  auto rc_state = robot_->GetRcState();

  wrp_sdk_msgs::msg::RcState rc_state_msg;
  rc_state_msg.header.stamp = this->now();
  rc_state_msg.header.frame_id = base_frame_;

  for (size_t i = 0; i < 8; i++) {
    rc_state_msg.axes[i] = rc_state.axes[i];
  }

  for (size_t i = 0; i < 8; i++) {
    rc_state_msg.buttons[i] = rc_state.buttons[i];
  }

  rc_state_publisher_->publish(rc_state_msg);
}

void MobileBaseNode::PublishOdometry() {
  // TODO calculate odometry according to robot type
  auto robot_odom = robot_->GetOdometry();

  // ATTN: odometry directly from wrp_sdk still in progress
  geometry_msgs::msg::Twist robot_twist;
  robot_twist.linear.x = robot_odom.linear.x;
  robot_twist.linear.y = robot_odom.linear.y;
  robot_twist.linear.z = robot_odom.linear.z;
  robot_twist.angular.x = robot_odom.angular.x;
  robot_twist.angular.y = robot_odom.angular.y;
  robot_twist.angular.z = robot_odom.angular.z;

  nav_msgs::msg::Odometry odom_msg =
      MobileBaseNode::CalculateOdometry(robot_twist);

  // publish tf transformation
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = odom_msg.header.stamp;
  tf_msg.header.frame_id = odom_msg.header.frame_id;
  tf_msg.child_frame_id = odom_msg.child_frame_id;

  tf_msg.transform.translation.x = odom_msg.pose.pose.position.x;
  tf_msg.transform.translation.y = odom_msg.pose.pose.position.y;
  tf_msg.transform.translation.z = 0.0;
  tf_msg.transform.rotation = odom_msg.pose.pose.orientation;

  tf_broadcaster_->sendTransform(tf_msg);
  odom_publisher_->publish(odom_msg);
}

nav_msgs::msg::Odometry MobileBaseNode::CalculateOdometry(
    geometry_msgs::msg::Twist robot_twist) {
  auto current_time = this->now();
  double dt = (current_time - last_time_).seconds();
  last_time_ = current_time;

  // TODO: perform calculation based on robot type & wheel base other than scout
  // & scout mini
  double linear_speed = robot_twist.linear.x;
  double angular_speed = robot_twist.angular.z;
  double lateral_speed = robot_twist.linear.y;

  double d_x =
      (linear_speed * std::cos(theta_) - lateral_speed * std::sin(theta_)) * dt;
  double d_y =
      (linear_speed * std::sin(theta_) + lateral_speed * std::cos(theta_)) * dt;
  double d_theta = angular_speed * dt;

  position_x_ += d_x;
  position_y_ += d_y;
  theta_ += d_theta;

  geometry_msgs::msg::Quaternion odom_quat =
      MobileBaseNode::CreateQuaternionMsgFromYaw(theta_);

  // publish odometry and tf messages
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = current_time;
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_frame_;

  odom_msg.pose.pose.position.x = position_x_;
  odom_msg.pose.pose.position.y = position_y_;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = odom_quat;

  odom_msg.twist.twist.linear.x = linear_speed;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.angular.z = angular_speed;

  return odom_msg;
}

geometry_msgs::msg::Quaternion MobileBaseNode::CreateQuaternionMsgFromYaw(
    double yaw) {
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

}  // namespace westonrobot

int main(int argc, char const* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<westonrobot::MobileBaseNode>());
  rclcpp::shutdown();
  return 0;
}