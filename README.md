# wrp_ros2

## Packages

This contains minimal packages to drive peripherals in wrp_sdk

* gps_receiver: a ROS2 wrapper around wrp_sdk's gps_receiver
* imu_sensor: a ROS2 wrapper around wrp_sdk's imu_sensor
* lift_server: a ROS2 wrapper around wrp_sdk's camera_lift

More details in the individual packages

## Basic Usage

1. Clone the packages into a colcon workspace and compile/source.  
(the following instructions assume your catkin workspace is at: ~/ros2_ws/src)

    ````bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    git clone https://gitlab.com/westonrobot/ros2/wrp_ros2.git
    cd ..
    colcon build
    . install/setup.bash
    ````

2. Launch ROS nodes  

    1. GPS Receiver Node

        ````bash
        ros2 launch gps_receiver gps_receiver_launch.py 
        ````

    2. IMU Sensor Node

        ````bash
        ros2 launch imu_sensor imu_sensor_launch.py 
        ````

    3. Lift Action Server Node

        ````bash
        ros2 launch lift_server lift_server_launch.py 
        ````
