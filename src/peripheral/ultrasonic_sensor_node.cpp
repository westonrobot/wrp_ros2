/**
 * ultrasonic_sensor_node.cpp
 *
 * Created on Tue Apr 12 2022 16:26:01
 *
 * Description:
 *
 * Copyright (c) 2022 Weston Robot Pte. Ltd.
 */
#include "wrp_ros2/peripheral/ultrasonic_sensor_node.hpp"

#include "wrp_sdk/peripheral/ultrasonic_sensor_dyp.hpp"
#include "wrp_sdk/peripheral/ultrasonic_sensor_w200d.hpp"

namespace westonrobot {
UltrasonicSensorNode::UltrasonicSensorNode(const rclcpp::NodeOptions& options)
    : Node("ultrasonic_sensor_node", options) {
  if (!UltrasonicSensorNode::ReadParameters()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Could not load parameters");
    rclcpp::shutdown();
  }

  if (!UltrasonicSensorNode::SetupInterfaces()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to setup interfaces!!");
    rclcpp::shutdown();
  }

  if (!UltrasonicSensorNode::SetupHardware()) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "Failed to setup Ultrasonic Sensor!!");
    rclcpp::shutdown();
  }
}

bool UltrasonicSensorNode::ReadParameters() {
  // Declare default parameters
  this->declare_parameter<std::string>("sensor_model", "dyp_a05");
  this->declare_parameter<std::string>("device_path", "/dev/ttyUSB0");
  this->declare_parameter<int>("baud_rate", 115200);
  this->declare_parameter<std::string>("frame_id", "ultrasonic_link");
  this->declare_parameter<std::string>("topic_name", "ultrasonic");

  // Get parameters
  RCLCPP_INFO_STREAM(this->get_logger(), "--- Parameters loaded are ---");

  this->get_parameter("sensor_model", sensor_model_);
  RCLCPP_INFO_STREAM(this->get_logger(), "sensor_model: " << sensor_model_);

  this->get_parameter("device_path", device_path_);
  RCLCPP_INFO_STREAM(this->get_logger(), "device_path: " << device_path_);

  this->get_parameter("baud_rate", baud_rate_);
  RCLCPP_INFO_STREAM(this->get_logger(), "baud_rate: " << baud_rate_);

  this->get_parameter("frame_id", frame_id_);
  RCLCPP_INFO_STREAM(this->get_logger(), "frame_id: " << frame_id_);

  this->get_parameter("topic_name", topic_name_);
  RCLCPP_INFO_STREAM(this->get_logger(), "topic_name: " << topic_name_);

  RCLCPP_INFO_STREAM(this->get_logger(), "-----------------------------");

  return true;
}

bool UltrasonicSensorNode::SetupHardware() {
  if (sensor_model_ == "dyp_a05") {
    sensor_ = std::make_shared<UltrasonicSensorDyp>();
  } else if (sensor_model_ == "w200d") {
    sensor_ = std::make_shared<UltrasonicSensorW200d>();
  } else {
  }

  sensor_->SetDataReceivedCallback(std::bind(
      &UltrasonicSensorNode::PublishCallback, this, std::placeholders::_1));

  if (!sensor_->Connect(device_path_, baud_rate_)) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "Failed to connect to ultrasonic sensor: "
                            << device_path_ << "@" << baud_rate_);
    return false;
  }

  return true;
}

bool UltrasonicSensorNode::SetupInterfaces() {
  publishers_.resize(8);
  for (int i = 0; i < 8; ++i) {
    std::string topic_name = "~/" + topic_name_ + std::to_string(i);
    publishers_[i] =
        this->create_publisher<sensor_msgs::msg::Range>(topic_name, 10);
  }

  return true;
}

void UltrasonicSensorNode::PublishCallback(UltrasonicMsg imu_msg) {
  sensor_msgs::msg::Range ultrasonic_msg;
  ultrasonic_msg.header.stamp = this->now();

  for (size_t i = 0; i < imu_msg.size(); ++i) {
    ultrasonic_msg.header.frame_id = frame_id_ + std::to_string(i);
    ultrasonic_msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    ultrasonic_msg.field_of_view = imu_msg[i].field_of_view;
    ultrasonic_msg.min_range = imu_msg[i].min_range;
    ultrasonic_msg.max_range = imu_msg[i].max_range;
    ultrasonic_msg.range = imu_msg[i].range;

    publishers_[i]->publish(ultrasonic_msg);
  }
}

}  // namespace westonrobot

int main(int argc, char const *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<westonrobot::UltrasonicSensorNode>());
  rclcpp::shutdown();
  return 0;
}