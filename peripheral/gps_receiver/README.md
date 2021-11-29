# gps_receiver

## About

Wrapper around wrp_sdk's gps_receiver

## Nodes

1. phoenix_base_node
    * Published topics
        * /gps_receiver/navsat_fix (sensor_msgs::msg::NavSatFix)  
            * Outputs the navigation satellite fix data.
    * Parameters
        * device_path (str, default: "/dev/ttyUSB0")
            * path to receiver port.
        * publish_interval (int, default: 500)
            * data publish rate in milliseconds.
        * baud_rate (int, default: 115200)
            * receiver's communication baud rate
        * frame_id (string, default: "gps")
            * Frame id to be used in navasat_fix's header

## Configuration

Change the default launch time parameters in the [config file](./config/gps_receiver_node_config.yaml).
