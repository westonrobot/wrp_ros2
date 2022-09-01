# wrp_ros2 peripherals

## About

ROS2 wrappers around wrp_sdk peripheral drivers.

## Nodes

### imu_sensor_node
| Published Topic | Type                  | Description                  |
| --------------- | --------------------- | ---------------------------- |
| `/imu`          | sensor_msgs::msg::Imu | Outputs the IMU Sensor data. |

| Parameter     | Type | Description                                                                                 |
| ------------- | ---- | ------------------------------------------------------------------------------------------- |
| `sensor_model` | str  | IMU sensor model.<br />Default: "wit"<br />Supported: "wit", "hipnuc" |
| `device_path` | str  | Path to sensor port.<br />Default: "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"<br /> |
| `baud_rate`   | int  | Sensor's communication baud rate.<br />Default: "115200"                                    |
| `frame_id`    | str  | Frame id used in /imu_sensor/imu's header.<br />Default: "imu_link"                         |


### gps_receiver_node
| Published Topic | Type                        | Description                                |
| --------------- | --------------------------- | ------------------------------------------ |
| `/fix`          | sensor_msgs::msg::NavSatFix | Outputs the navigation satellite fix data. |

| Parameter     | Type | Description                                                                                                             |
| ------------- | ---- | ----------------------------------------------------------------------------------------------------------------------- |
| `device_path` | str  | Path to receiver port.<br />Default: "/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00"<br /> |
| `baud_rate`   | int  | Sensor's communication baud rate.<br />Default: "115200"                                                                |
| `frame_id`    | str  | Frame id used in /gps_receiver/navsat_fix's header.<br />Default: "gps_link"                                            |

### ultrasonic_sensor_node
| Published Topic      | Type                    | Description                         |
| -------------------- | ----------------------- | ----------------------------------- |
| `/<topic_name><num>` | sensor_msgs::msg::Range | Outputs the ultrasonic sensor data. |

| Parameter      | Type | Description                                                                                   |
| -------------- | ---- | --------------------------------------------------------------------------------------------- |
| `sensor_model` | str  | Sensor's model <br />Default: "dyp_a05"<br />Supported: "dyp_a05", "w200d"                    |
| `device_path`  | str  | Path to receiver port.<br />Default: "/dev/ttyUSB0"<br />                                     |
| `baud_rate`    | int  | Sensor's communication baud rate.<br />Default: "115200"                                      |
| `frame_id`     | str  | Frame id used in /ultrasonic_sensor_node/ultrasonic's header.<br />Default: "ultrasonic_link" |
| `topic_name`   | str  | Topic name prefix used to publish data <br />Default: "ultrasonic"                            |

### power_regulator_node
| Published Topic | Type                                     | Description                               |
| --------------- | ---------------------------------------- | ----------------------------------------- |
| `/state`        | wrp_ros2::msg::PowerRegulatorDeviceState | Outputs the power regulator device state. |

| Service | Type                                 | Description                               |
| ------- | ------------------------------------ | ----------------------------------------- |
| `/cmd`  | wrp_ros2::srv::PowerRegulatorControl | (Dis)able power regulator output channels |

| Parameter     | Type | Description                                       |
| ------------- | ---- | ------------------------------------------------- |
| `device_path` | str  | Path to receiver port.<br />Default: "can0"<br /> |