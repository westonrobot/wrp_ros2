/**
 * gps_receiver_node.cpp
 *
 * Created on Tue Nov 23 2021 13:43:01
 *
 * Description:
 * ROS2 wrapper for wrp_sdk gps receiver
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */
#include "wrp_ros2/peripheral/gps_receiver_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"
using std::placeholders::_1;

namespace westonrobot {

GpsReceiverNode::GpsReceiverNode(const rclcpp::NodeOptions& options)
    : Node("gps_receiver_node", options) {
  if (!GpsReceiverNode::ReadParameters()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Could not load parameters");
    rclcpp::shutdown();
  }

  receiver_ = std::make_unique<GpsReceiver>();

  if (!receiver_->Connect(device_path_, baud_rate_)) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to setup gps receiver");
    rclcpp::shutdown();
  }

  pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
      "/gps_receiver/navsat_fix", 1);

  loop_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(publish_interval_),
      std::bind(&GpsReceiverNode::PublishCallback, this));
}

GpsReceiverNode::~GpsReceiverNode() {}

bool GpsReceiverNode::ReadParameters() {
  // Declare default parameters
  this->declare_parameter<std::string>("device_path", "/dev/ttyUSB0");
  this->declare_parameter<int>("publish_interval", 500);
  this->declare_parameter<int>("baud_rate", 115200);
  this->declare_parameter<std::string>("frame_id", "gps");

  // Get parameters
  RCLCPP_INFO_STREAM(this->get_logger(), "--- Parameters loaded are ---");

  this->get_parameter("device_path", device_path_);
  RCLCPP_INFO_STREAM(this->get_logger(), "device_path: " << device_path_);

  this->get_parameter("publish_interval", publish_interval_);
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "publish_interval: " << publish_interval_);

  this->get_parameter("baud_rate", baud_rate_);
  RCLCPP_INFO_STREAM(this->get_logger(), "baud_rate: " << baud_rate_);

  this->get_parameter("frame_id", frame_id_);
  RCLCPP_INFO_STREAM(this->get_logger(), "frame_id: " << frame_id_);

  RCLCPP_INFO_STREAM(this->get_logger(), "-----------------------------");

  return true;
}

void GpsReceiverNode::PublishCallback() {
  if (receiver_->IsOkay()) {
    auto gps_fix = receiver_->GetFixData();

    sat_fix_.header.stamp = this->get_clock()->now();
    sat_fix_.header.frame_id = frame_id_;
    sat_fix_.status.status = gps_fix.status.status;
    sat_fix_.status.service = gps_fix.status.service;
    sat_fix_.latitude = gps_fix.latitude;
    sat_fix_.longitude = gps_fix.longitude;
    sat_fix_.altitude = gps_fix.altitude;
    sat_fix_.position_covariance = gps_fix.position_covariance;
    pub_->publish(sat_fix_);
  }
}

}  // namespace westonrobot

RCLCPP_COMPONENTS_REGISTER_NODE(westonrobot::GpsReceiverNode)