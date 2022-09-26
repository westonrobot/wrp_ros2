/**
 * imu_sensor_node.cpp
 *
 * Created on Tue Nov 23 2021 16:20:11
 *
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */
#include "wrp_ros2/peripheral/imu_sensor_node.hpp"

#include "wrp_sdk/peripheral/imu_sensor_wit.hpp"
#include "wrp_sdk/peripheral/imu_sensor_hipnuc.hpp"

namespace westonrobot {
ImuSensorNode::ImuSensorNode(const rclcpp::NodeOptions& options)
    : Node("imu_sensor_node", options) {
  if (!ImuSensorNode::ReadParameters()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Could not load parameters");
    rclcpp::shutdown();
  }

  if (!ImuSensorNode::SetupInterfaces()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Could not setup ros interfaces");
    rclcpp::shutdown();
  }

  if (!ImuSensorNode::SetupHardware()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Could not setup hardware");
    rclcpp::shutdown();
  }
}

bool ImuSensorNode::ReadParameters() {
  // Declare default parameters
  this->declare_parameter<std::string>(
      "device_path", "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0");
  this->declare_parameter<std::string>("sensor_model", "wit");
  this->declare_parameter<int>("baud_rate", 115200);
  this->declare_parameter<std::string>("frame_id", "imu");

  // Get parameters
  RCLCPP_INFO_STREAM(this->get_logger(), "--- Parameters loaded are ---");

  this->get_parameter("sensor_model", sensor_model_);
  if (sensor_model_ != "wit" && sensor_model_ != "hipnuc") {
    RCLCPP_WARN_STREAM(this->get_logger(), "Invalid IMU sensor model detected, defaulting to \"wit\"!!!!!!!!!");
    sensor_model_ = "wit";
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "sensor_model: " << sensor_model_);

  this->get_parameter("device_path", device_path_);
  RCLCPP_INFO_STREAM(this->get_logger(), "device_path: " << device_path_);

  this->get_parameter("baud_rate", baud_rate_);
  RCLCPP_INFO_STREAM(this->get_logger(), "baud_rate: " << baud_rate_);

  this->get_parameter("frame_id", frame_id_);
  RCLCPP_INFO_STREAM(this->get_logger(), "frame_id: " << frame_id_);

  RCLCPP_INFO_STREAM(this->get_logger(), "-----------------------------");

  return true;
}

bool ImuSensorNode::SetupInterfaces() {
  publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("~/imu", 5);

  return true;
}

bool ImuSensorNode::SetupHardware() {

  if (sensor_model_ == "hipnuc") {
    imu_ = std::make_unique<ImuSensorHipnuc>();
  } else {
    imu_ = std::make_unique<ImuSensorWit>();
  }


  imu_->SetDataReceivedCallback(
      std::bind(&ImuSensorNode::PublishCallback, this, std::placeholders::_1));
  if (!imu_->Connect(device_path_, baud_rate_)) {
    return false;
  }

  return true;
}

void ImuSensorNode::PublishCallback(const ImuMsg& imu_msg) {
  imu_data_.header.stamp = this->get_clock()->now();
  imu_data_.header.frame_id = frame_id_;
  imu_data_.orientation.x = imu_msg.orientation.x;
  imu_data_.orientation.y = imu_msg.orientation.y;
  imu_data_.orientation.z = imu_msg.orientation.z;
  imu_data_.orientation.w = imu_msg.orientation.w;
  imu_data_.orientation_covariance = imu_msg.orientation_covariance;
  imu_data_.angular_velocity.x = imu_msg.angular_velocity.x;
  imu_data_.angular_velocity.y = imu_msg.angular_velocity.y;
  imu_data_.angular_velocity.z = imu_msg.angular_velocity.z;
  imu_data_.angular_velocity_covariance = imu_msg.angular_velocity_covariance;
  imu_data_.linear_acceleration.x = imu_msg.linear_acceleration.x;
  imu_data_.linear_acceleration.y = imu_msg.linear_acceleration.y;
  imu_data_.linear_acceleration.z = imu_msg.linear_acceleration.z;
  imu_data_.linear_acceleration_covariance =
      imu_msg.linear_acceleration_covariance;

  publisher_->publish(imu_data_);
}
}  // namespace westonrobot
int main(int argc, char const* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<westonrobot::ImuSensorNode>());
  rclcpp::shutdown();
  return 0;
}