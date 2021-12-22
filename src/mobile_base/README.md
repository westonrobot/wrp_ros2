# wrp_ros2 mobile base

## About

ROS2 wrappers around wrp_sdk peripheral drivers.

## Nodes

### mobile_base_node
| Published Topic    | Type                              | Description                         |
| ------------------ | --------------------------------- | ----------------------------------- |
| `/system_state`    | wrp_ros2::msg::SystemState        | Outputs robot's system state        |
| `/motion_state`    | wrp_ros2::msg::MotionState        | Outputs robot's motion state        |
| `/actuator_state`  | wrp_ros2::msg::ActuatorStateArray | Outputs robot's actuator states     |
| `/odom`            | nav_msgs::msg::Odometry           | Outputs robot's wheel odometry      |
| `/ultrasonic_data` | wrp_ros2::msg::RangeData          | Outputs robot's ultrasonic readings |
| `/tof_data`        | wrp_ros2::msg::RangeData          | Outputs robot's tof readings        |

| Subscribed Topic | Type                      | Description              |
| ---------------- | ------------------------- | ------------------------ |
| `/cmd_vel`       | geometry_msgs::msg::Twist | Control robot's movement |

| Service                  | Type                               | Description                             |
| ------------------------ | ---------------------------------- | --------------------------------------- |
| `/access_control`        | wrp_ros2::srv::AccessControl       | (Re)Gain or Renounce control token      |
| `/assisted_mode_control` | wrp_ros2::srv::AssistedModeControl | (En/Dis)able Assisted mode              |
| `/light_control`         | wrp_ros2::srv::LightControl        | Control robot's lights                  |
| `/motion_reset`          | wrp_ros2::srv::MotionReset         | Reset wheel position or odometry values |

| Parameter         | Type | Description                                                                            |
| ----------------- | ---- | -------------------------------------------------------------------------------------- |
| `robot_base_type` | str  | Robot base type.<br/>Default: "/dev/ttyUSB0"<br/>Supported: "weston", "agilex" & "vbot |
| `can_device`      | str  | Robot's CAN port.<br/>Default: "can0"                                                  |
| `base_frame`      | str  | Base frame id.<br/>Default: "base_link"<br/>                                           |
| `odom_frame`      | str  | Odometry frame id.<br />Default: "odom"                                                |
| `auto_reconnect`  | bool | Automatically attempt to gain control token.<br />Default: true                        |