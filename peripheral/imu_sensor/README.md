# imu_sensor

## About

Wrapper around wrp_sdk's imu_sensor

## Nodes

1. phoenix_base_node
    * Published topics
        * /imu_sensor/imu (sensor_msgs::msg::Imu)  
            * Outputs the IMU Sensor data.
    * Parameters
        * device_path (str, default: "/dev/ttyUSB0")
            * path to receiver port.
        * publish_interval (int, default: 500)
            * data publish rate in milliseconds.
        * baud_rate (int, default: 115200)
            * receiver's communication baud rate
        *  frame_id (string, default: "imu")
            * Frame id to be used in imu's header

## Configuration

Change the default launch time parameters in the [config file](./config/imu_sensor_node_config.yaml).
