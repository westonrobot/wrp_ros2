# wrp_ros2

## About

This package contains a minimal wrapper around wrp_sdk
More details in the individual src sub-folders

## Basic Usage

1. Clone the packages into a colcon workspace and compile/source.  
(the following instructions assume your catkin workspace is at: ~/ros2_ws/src)

    ````bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    git clone https://gitlab.com/westonrobot/ros2/wrp_ros2.git
    cd ..
    colcon build --symlink-install
    . install/setup.bash
    ````

2. Launch ROS nodes  
    **_Change run time parameters by editing the corresponding config file_**

    1. GPS Receiver Node

        ````bash
        ros2 launch wrp_ros2 gps_receiver_launch.py 
        ````

    2. IMU Sensor Node

        ````bash
        ros2 launch wrp_ros2 imu_sensor_launch.py 
        ````

    3. Lift Action Server Node

        ````bash
        ros2 launch wrp_ros2 lift_server_launch.py 
        ````
