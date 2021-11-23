/**
 * imu_sensor_node.cpp
 *
 * Created on Tue Nov 23 2021 16:20:11
 *
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */
#include "imu_sensor/imu_sensor_node.hpp"
using std::placeholders::_1;

namespace imu_sensor {
using namespace westonrobot;

ImuSensorNode::ImuSensorNode() : Node("imu_sensor_node") {
  if (!ImuSensorNode::ReadParameters()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Could not load parameters");
    rclcpp::shutdown();
  }

  imu_ = std::make_unique<ImuSensor>();

  if (!imu_->Connect(device_path_, baud_rate_)) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to setup Imu Sensor!!");
    rclcpp::shutdown();
  }

  pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu_sensor/imu", 1);

  loop_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(publish_interval_),
                              std::bind(&ImuSensorNode::PublishCallback, this));
}

ImuSensorNode::~ImuSensorNode() {}

bool ImuSensorNode::ReadParameters() {
  // Declare default parameters
  this->declare_parameter<std::string>("device_path", "/dev/ttyUSB0");
  this->declare_parameter<int>("publish_interval", 500);
  this->declare_parameter<int>("baud_rate", 115200);

  // Get parameters
  RCLCPP_INFO_STREAM(this->get_logger(), "--- Parameters loaded are ---");

  this->get_parameter("device_path", device_path_);
  RCLCPP_INFO_STREAM(this->get_logger(), "device_path: " << device_path_);

  this->get_parameter("publish_interval", publish_interval_);
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "publish_interval: " << publish_interval_);

  this->get_parameter("baud_rate", baud_rate_);
  RCLCPP_INFO_STREAM(this->get_logger(), "baud_rate: " << baud_rate_);

  RCLCPP_INFO_STREAM(this->get_logger(), "-----------------------------");

  return true;
}

void ImuSensorNode::PublishCallback() {
  if (imu_->IsOkay()) {
    auto data = imu_->GetImuMessage();
    imu_data_.header.stamp = this->get_clock()->now();
    imu_data_.orientation.x = data.orientation.q0;
    imu_data_.orientation.y = data.orientation.q1;
    imu_data_.orientation.z = data.orientation.q2;
    imu_data_.orientation.w = data.orientation.q3;
    imu_data_.orientation_covariance = data.orientation_covariance;
    imu_data_.angular_velocity.x = data.angular_velocity.x;
    imu_data_.angular_velocity.y = data.angular_velocity.y;
    imu_data_.angular_velocity.z = data.angular_velocity.z;
    imu_data_.angular_velocity_covariance = data.angular_velocity_covariance;
    imu_data_.linear_acceleration.x = data.linear_acceleration.x;
    imu_data_.linear_acceleration.y = data.linear_acceleration.y;
    imu_data_.linear_acceleration.z = data.linear_acceleration.z;
    imu_data_.linear_acceleration_covariance =
        data.linear_acceleration_covariance;

    pub_->publish(imu_data_);
  }
}
}  // namespace imu_sensor