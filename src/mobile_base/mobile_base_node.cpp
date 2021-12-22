/**
 * mobile_base_node.cpp
 *
 * Created on Tue Dec 21 2021 11:21:19
 *
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */
#include "wrp_ros2/mobile_base/mobile_base_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "wrp_sdk/mobile_base/westonrobot/mobile_base.hpp"
#include "wrp_sdk/mobile_base/agilex/agilex_base_v2_adapter.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
using namespace std::placeholders;

namespace westonrobot {

MobileBaseNode::MobileBaseNode(const rclcpp::NodeOptions& options)
    : Node("mobile_base_node", options) {
  if (!MobileBaseNode::ReadParameters()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Could not load parameters");
    rclcpp::shutdown();
  }

  if (!MobileBaseNode::SetupRobot()) {
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
  this->declare_parameter<std::string>("robot_base_type", "weston");
  this->declare_parameter<std::string>("can_device", "can0");
  this->declare_parameter<std::string>("base_frame", "base_link");
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<bool>("auto_reconnect", true);

  // Get parameters
  RCLCPP_INFO_STREAM(this->get_logger(), "--- Parameters loaded are ---");

  this->get_parameter("robot_base_type", robot_base_type_);
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "robot_base_type: " << robot_base_type_);

  this->get_parameter("can_device", can_device_);
  RCLCPP_INFO_STREAM(this->get_logger(), "can_device: " << can_device_);

  this->get_parameter("base_frame", base_frame_);
  RCLCPP_INFO_STREAM(this->get_logger(), "base_frame: " << base_frame_);

  this->get_parameter("odom_frame", odom_frame_);
  RCLCPP_INFO_STREAM(this->get_logger(), "odom_frame: " << odom_frame_);

  this->get_parameter("auto_reconnect", auto_reconnect_);
  RCLCPP_INFO_STREAM(this->get_logger(), "auto_reconnect: " << auto_reconnect_);

  RCLCPP_INFO_STREAM(this->get_logger(), "-----------------------------");

  return true;
}

bool MobileBaseNode::SetupRobot() {
  // Create appropriate adaptor
  if (robot_base_type_ == "weston") {
    robot_ = std::make_shared<MobileBase>();
  } else if (robot_base_type_ == "agilex") {
    robot_ = std::make_shared<AgilexBaseV2Adapter>();
  } else if (robot_base_type_ == "vbot") {
    robot_ = std::make_shared<MobileBase>(true);
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "Unknown robot base type\n Supported are \"weston\", "
                        "\"agilex\" & \"vbot\"");
    return false;
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
      this->create_publisher<wrp_ros2::msg::SystemState>("/system_state", 10);
  motion_state_publisher_ =
      this->create_publisher<wrp_ros2::msg::MotionState>("/motion_state", 10);
  actuator_state_publisher_ =
      this->create_publisher<wrp_ros2::msg::ActuatorStateArray>(
          "/actuator_state", 10);
  odom_publisher_ =
      this->create_publisher<nav_msgs::msg::Odometry>("/odom", 50);
  ultrasonic_publisher_ =
      this->create_publisher<wrp_ros2::msg::RangeData>("/ultrasonic_data", 10);
  tof_publisher_ =
      this->create_publisher<wrp_ros2::msg::RangeData>("/tof_data", 10);

  // setup services
  access_control_service_ = this->create_service<wrp_ros2::srv::AccessControl>(
      "/access_control",
      std::bind(&MobileBaseNode::AccessControlCallback, this, _1, _2));
  assisted_mode_control_service_ =
      this->create_service<wrp_ros2::srv::AssistedModeControl>(
          "/assisted_mode_control",
          std::bind(&MobileBaseNode::AssistedModeControlCallback, this, _1,
                    _2));
  light_control_service_ = this->create_service<wrp_ros2::srv::LightControl>(
      "/light_control",
      std::bind(&MobileBaseNode::LightControlCallback, this, _1, _2));
  motion_reset_service_ = this->create_service<wrp_ros2::srv::MotionReset>(
      "/motion_reset",
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
    const wrp_ros2::srv::AccessControl::Request::SharedPtr request,
    wrp_ros2::srv::AccessControl::Response::SharedPtr response) {
  HandshakeReturnCode result;
  switch (request->action_type) {
    case wrp_ros2::srv::AccessControl::Request::ACTION_TYPE_REQUEST_CONTROL: {
      result = robot_->RequestControl();
      response->result_code = static_cast<uint32_t>(result);
      break;
    }
    case wrp_ros2::srv::AccessControl::Request::ACTION_TYPE_RENOUNCE_CONTROL: {
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
    const wrp_ros2::srv::AssistedModeControl::Request::SharedPtr request,
    wrp_ros2::srv::AssistedModeControl::Response::SharedPtr response) {
  AssistedModeSetCommand cmd;
  cmd.enable = request->enable;
  /**ATTN: Should we check for token or change the interface here?
   * If multiple user attempts to change, each one might have a different
   * thinking if assisted mode is on or off.
   */
  robot_->SetAssistedMode(cmd);
  response->state = request->enable;
  return;
}

void MobileBaseNode::LightControlCallback(
    const wrp_ros2::srv::LightControl::Request::SharedPtr request,
    wrp_ros2::srv::LightControl::Response::SharedPtr response) {
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
    const wrp_ros2::srv::MotionReset::Request::SharedPtr request,
    wrp_ros2::srv::MotionReset::Response::SharedPtr response) {
  MotionResetCommand cmd;

  cmd.type = static_cast<MotionResetCommandType>(request->type);

  if (robot_->SdkHasControlToken()) {
    robot_->SetMotionResetCommand(cmd);
    response->result_code =
        wrp_ros2::srv::MotionReset::Response::MOTION_RESET_SUCCCESS;
  } else {
    response->result_code =
        wrp_ros2::srv::MotionReset::Response::MOTION_RESET_FAILURE;
  }

  return;
}

void MobileBaseNode::PublishLoopCallback() {
  if (auto_reconnect_ && !robot_->SdkHasControlToken()) {
    robot_->RequestControl(50);
  }

  PublishRobotState();
  PublishSensorData();
  PublishWheelOdometry();
}

void MobileBaseNode::PublishRobotState() {
  auto system_state = robot_->GetSystemState();
  auto motion_state = robot_->GetMotionState();
  auto actuator_state = robot_->GetActuatorState();

  // system state
  wrp_ros2::msg::SystemState system_state_msg;
  system_state_msg.rc_connected = system_state.rc_connected;
  system_state_msg.error_code = static_cast<uint32_t>(system_state.error_code);
  system_state_msg.operational_state =
      static_cast<uint32_t>(system_state.operational_state);
  system_state_msg.control_state =
      static_cast<uint32_t>(system_state.control_state);

  system_state_publisher_->publish(system_state_msg);

  // motion state
  wrp_ros2::msg::MotionState motion_state_msg;
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

  // actuator state
  wrp_ros2::msg::ActuatorStateArray actuator_state_msg;
  for (size_t i = 0; i < actuator_state.size(); ++i) {
    wrp_ros2::msg::ActuatorState actuator_msg;
    actuator_msg.id = actuator_state[i].id;
    actuator_state_msg.states.push_back(actuator_msg);
  }
  actuator_state_publisher_->publish(actuator_state_msg);
}

void MobileBaseNode::PublishSensorData() {
  // TODO: bumper publishing?
  auto ultrasonic_data = robot_->GetUltrasonicData();
  auto tof_data = robot_->GetTofData();

  // ultrasonic data
  wrp_ros2::msg::RangeData ultrasonic_data_msg;
  ultrasonic_data_msg.type =
      wrp_ros2::msg::RangeData::RANGE_SENSOR_TYPE_ULTRASONIC;
  for (size_t i = 0; i < sizeof(ultrasonic_data.data); i++) {
    wrp_ros2::msg::RangeDataType range_data;
    range_data.id = ultrasonic_data.data[i].id;
    range_data.threshold = ultrasonic_data.data[i].threshold;
    range_data.range = ultrasonic_data.data[i].range;
    ultrasonic_data_msg.data[i] = range_data;
  }
  ultrasonic_publisher_->publish(ultrasonic_data_msg);

  // tof data
  wrp_ros2::msg::RangeData tof_data_msg;
  tof_data_msg.type = wrp_ros2::msg::RangeData::RANGE_SENSOR_TYPE_TOF;
  for (size_t i = 0; i < sizeof(tof_data.data); i++) {
    wrp_ros2::msg::RangeDataType range_data;
    range_data.id = tof_data.data[i].id;
    range_data.threshold = tof_data.data[i].threshold;
    range_data.range = tof_data.data[i].range;
    tof_data_msg.data[i] = range_data;
  }
  tof_publisher_->publish(tof_data_msg);
}

void MobileBaseNode::PublishWheelOdometry() {
  // TODO calculate odometry according to robot type
  auto robot_odom = robot_->GetOdometry();

  auto current_time = this->now();
  float dt = (current_time - last_time_).seconds();
  last_time_ = current_time;

  // if last_time is too old, reset calculation
  if (dt > 10 * loop_period_) return;

  auto linear_speed = robot_odom.linear.x;
  auto angular_speed = robot_odom.angular.z;

  double d_x = linear_speed * std::cos(theta_) * dt;
  double d_y = linear_speed * std::sin(theta_) * dt;
  double d_theta = angular_speed * dt;

  position_x_ += d_x;
  position_y_ += d_y;
  theta_ += d_theta;

  geometry_msgs::msg::Quaternion odom_quat = MobileBaseNode::CreateQuaternionMsgFromYaw(theta_);

  // publish tf transformation
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = current_time;
  tf_msg.header.frame_id = odom_frame_;
  tf_msg.child_frame_id = base_frame_;

  tf_msg.transform.translation.x = position_x_;
  tf_msg.transform.translation.y = position_y_;
  tf_msg.transform.translation.z = 0.0;
  tf_msg.transform.rotation = odom_quat;

  tf_broadcaster_->sendTransform(tf_msg);

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

  odom_publisher_->publish(odom_msg);
}

geometry_msgs::msg::Quaternion MobileBaseNode::CreateQuaternionMsgFromYaw(double yaw) {
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

}  // namespace westonrobot

RCLCPP_COMPONENTS_REGISTER_NODE(westonrobot::MobileBaseNode)