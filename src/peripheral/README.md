# wrp_ros2 peripherals

## About

ROS2 wrappers around wrp_sdk peripheral drivers.

## Nodes


### imu_sensor_node
| Published Topic | Type | Description |
| --- | --- | --- |
| `/imu_sensor/imu` | sensor_msgs::msg::Imu | Outputs the IMU Sensor data. |

| Parameter | Type | Description |
| --- | --- | --- |
| `device_path` |str | Path to sensor port.<br />Default: "/dev/ttyUSB0"<br /> |
| `baud_rate` | int | Sensor's communication baud rate.<br />Default: "115200"|
| `frame_id` | str | Frame id used in /imu_sensor/imu's header.<br />Default: "imu"|


### gps_receiver_node
| Published Topic | Type | Description |
| --- | --- | --- |
| `/gps_receiver/navsat_fix` | sensor_msgs::msg::NavSatFix | Outputs the navigation satellite fix data. |

| Parameter | Type | Description |
| --- | --- | --- |
| `device_path` |str | Path to receiver port.<br />Default: "/dev/ttyUSB0"<br /> |
| `baud_rate` | int | Sensor's communication baud rate.<br />Default: "115200"|
| `frame_id` | str | Frame id used in /gps_receiver/navsat_fix's header.<br />Default: "gps"|


### lift_server_node
| Published Topic | Type | Description |
| --- | --- | --- |
| `/lift_server/state` | wrp_ros2::msg::LiftState | Outputs the lift's current state data. |

| Subscribed Topic | Type | Description |
| --- | --- | --- |
|`/lift_server/speed_cmd` | std_msgs::msg::Int8 | Control lift using speed command <br/> (-100(down) to 100(up))|

| Parameter | Type | Description |
| --- | --- | --- |
| `device_path` |str | path to lift's serial port.<br />Default: "/dev/ttyUSB0"<br /> |
| `publish_interval` | int | Data publish rate in milliseconds.<br />Default: 500<br />Lower==faster, Higher==slower|
| `baud_rate` | int | Sensor's communication baud rate.<br />Default: "115200"|
| `command_preemption` | bool | If a command can pre-empt an active goal command<br />Default: true|
