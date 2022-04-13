import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction  # to scope parameters
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    mobile_base_weston_node = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory("wrp_ros2"),
                                                        "launch",
                                                        "mobile_base",
                                                        "mobile_base_weston.launch.py")]),
        )
    ])

    imu_sensor_node = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory("wrp_ros2"),
                                                        "launch",
                                                        "peripheral",
                                                        "imu_sensor.launch.py")]),
        )
    ])

    gps_receiver_node = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory("wrp_ros2"),
                                                        "launch",
                                                        "peripheral",
                                                        "gps_receiver.launch.py")]),
        )
    ])

    ultrasonic_sensor_node = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory("wrp_ros2"),
                                                        "launch",
                                                        "peripheral",
                                                        "ultrasonic_sensor.launch.py")
            ]),
            launch_arguments={
                'device_path': '/dev/ttyUSB0'
            }.items()
        )
    ])

    ld.add_action(mobile_base_weston_node)
    ld.add_action(imu_sensor_node)
    ld.add_action(gps_receiver_node)
    ld.add_action(ultrasonic_sensor_node)
    return ld
