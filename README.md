# wrp_ros2

## About

This package contains a minimal wrapper around wrp_sdk and provides a ROS2 interface to hardware platforms from Weston Robot. Please first check if your device and environment are supported by the SDK and this ROS package before proceeding:

**Supported Environments**:

* Architecture: x86_64/arm64
* OS: Ubuntu 20.04
* ROS: Foxy
  
**Supported Robots**:

* Scout
* Scout Mini
* Tracer

**Supported Peripherals**:

* Power Regulator V2.1
* Ultrasonic Sensor
* Hipnuc and WitMotion IMU
* NMEA-compatible GPS Receiver

More details can be found in the README inside individual src sub-folders

  * [Mobile Base Node](./src/mobile_base)
  * [Peripheral Nodes](./src/peripheral)

## Getting the package

### Install dependencies

* wrp_sdk >= v1.2.0: please follow setup instructions from [here](https://github.com/westonrobot/wrp_sdk/)

### Build the package

* Clone the packages into a colcon workspace and compile/source.  
(the following instructions assume your catkin workspace is at: ~/ros2_ws/src)

    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    git clone https://gitlab.com/westonrobot/ros2/wrp_ros2.git
    cd ..
    colcon build --symlink-install
    . install/setup.bash
    ```

## Setup CAN-To-USB adapter
 
1. Enable gs_usb kernel module
    ```
    $ sudo modprobe gs_usb
    ```
2. Bringup can device
   ```
   # (NOTE: refer to the following table for the correct bitrate)
   $ sudo ip link set can0 up type can bitrate 500000
   $ sudo ip link set can0 txqueuelen 1000
   ```

    | Hardware                 | CAN Bitrate |
    | ------------------------ | ----------- |
    | Scout V2.0               | 500000      |
    | Scout V2.5               | 1000000     |
    | Scout Mini               | 500000      |
    | Power Regulator          | 500000      |
    | Otherwise if unspecified | 500000      |

3. If no error occured during the previous steps, you should be able to see the can device now by using command
   ```
   $ ifconfig -a
   ```
4. Install and use can-utils to test the hardware
    ```
    $ sudo apt install can-utils
    ```
5. Testing command
    ```
    # receiving data from can0
    $ candump can0
    # send data to can0
    $ cansend can0 001#1122334455667788
    ```

Scripts are provided [here](./scripts) for convenience. You can run "./setup_can2usb.bash" for the first-time setup and run "./bringup_can2usb_1m.bash" to bring up the device each time you unplug and re-plug the adapter.

## Example Usage

You can find more information about robot base control from [this page](https://docs.westonrobot.net/getting_started/basics/robot_base_control.html).

**_You may need to change run time parameters by editing the corresponding launch file_**

1. Mobile Base Node (and [variants](./launch/mobile_base))

    ```bash
    ros2 launch wrp_ros2 mobile_base.launch.py
    ```

2. Power Regulator Node

    ```bash
    roslaunch wrp_ros power_regulator.launch.py
    ```

3. GPS Receiver Node

    ```bash
    roslaunch wrp_ros gps_receiver_launch.py 
    ```

4. IMU Sensor Node

    ```bash
    roslaunch wrp_ros imu_sensor_launch.py 
    ```

5. Ultrasonic Sensor Node

    ```bash
    roslaunch wrp_ros ultrasonic_sensor.launch.py
    ```
