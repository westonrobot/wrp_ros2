# wrp_ros2 mobile base

## About

ROS2 wrappers around wrp_sdk peripheral drivers.

## Support

### Robot platforms
| Robot Platform | Robot Base Type | CAN Bitrate | Support             |
| -------------- | --------------- | ----------- | ------------------- |
| Scout          | "weston"        | 1000000     | Supported           |
| Scout Mini     | "agilex"        | 500000      | Supported           |
| Hunter         | "agilex"        | 500000      | Not fully supported |
| Tracer         | "agilex"        | 500000      | Supported           |
| Bunker         | "agilex"        | 500000      | Not fully supported |
| Ranger Mini    | "agilex"        | 500000      | Not fully supported |

## Nodes

### mobile_base_node
| Published Topic     | Type                              | Description                         |
| ------------------- | --------------------------------- | ----------------------------------- |
| `~/system_state`    | wrp_ros2::msg::SystemState        | Outputs robot's system state        |
| `~/motion_state`    | wrp_ros2::msg::MotionState        | Outputs robot's motion state        |
| `~/actuator_state`  | wrp_ros2::msg::ActuatorStateArray | Outputs robot's actuator states     |
| `~/odom`            | nav_msgs::msg::Odometry           | Outputs robot's wheel odometry      |
| `~/battery_state`   | sensor_msgs::msg::BatteryState    | Outputs robot's battery state       |
| `~/ultrasonic_data` | wrp_ros2::msg::RangeData          | Outputs robot's ultrasonic readings |
| `~/tof_data`        | wrp_ros2::msg::RangeData          | Outputs robot's tof readings        |

| Subscribed Topic | Type                      | Description              |
| ---------------- | ------------------------- | ------------------------ |
| `/cmd_vel`       | geometry_msgs::msg::Twist | Control robot's movement |

| Service                   | Type                               | Description                             |
| ------------------------- | ---------------------------------- | --------------------------------------- |
| `~/access_control`        | wrp_ros2::srv::AccessControl       | (Re)Gain or Renounce control token      |
| `~/assisted_mode_control` | wrp_ros2::srv::AssistedModeControl | (En/Dis)able Assisted mode              |
| `~/light_control`         | wrp_ros2::srv::LightControl        | Control robot's lights                  |
| `~/motion_reset`          | wrp_ros2::srv::MotionReset         | Reset wheel position or odometry values |

| Parameter        | Type | Description                                                                                         |
| ---------------- | ---- | --------------------------------------------------------------------------------------------------- |
| `robot_type`     | str  | Robot base type.<br/>Default: "weston"<br/>Supported: "weston", "agilex"                            |
| `can_device`     | str  | Robot's CAN port.<br/>Default: "can0"                                                               |
| `base_frame`     | str  | Base frame id.<br/>Default: "base_link"<br/>                                                        |
| `odom_frame`     | str  | Wheel odometry frame id (when publishing odometry).<br />Default: "odom"                            |
| `auto_reconnect` | bool | Automatically attempt to gain control token.<br />Default: true                                     |
| `publish_odom`   | bool | If node should publish wheel odometry.<br />Default: true                                           |
| `motion_type`    | str  | Robot's motion type. <br/>Default: "skid_steer"<br/>Supported: "skid_steer", "omni", "differential" |