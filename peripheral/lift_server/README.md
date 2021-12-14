# lift_server

## About

Wrapper around wrp_sdk's camera_lift

## Nodes

1. phoenix_base_node
    * Published topics
        * /lift_server/state (wrp_ros2::msg::LiftState)  
            * Outputs the lift's current state data.
    * Subscribed topics
        * /lift_server/speed_cmd (std_msgs::msg::Int8)
            * Control lift using speed (-100(down) to 100(up))
    * Parameters
        * device_path (str, default: "/dev/ttyUSB0")
            * path to lift's serial port.
        * baud_rate (int, default 115200)
            * serial baud rate
        * publish_interval (int, default: 500)
            * data publish rate in milliseconds.
        * command_preemption (bool, default: true)
            * if a command can pre-empt an active goal command

## Configuration

Change the default launch time parameters in the [config file](./config/lift_server_node_config.yaml).
