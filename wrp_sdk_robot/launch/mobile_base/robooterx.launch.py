import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    mobile_base_robooterx_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory("wrp_sdk_robot"),
                                                    "launch",
                                                    "mobile_base",
                                                    "mobile_base.launch.py")]),
        launch_arguments={
            "robot_type": "13",
            "can_device": "can0",
            "base_frame": "base_link",
            "odom_frame": "odom",
            "auto_reconnect": "true",
            "publish_odom_tf": "true",
        }.items(),
    )

    ld.add_action(mobile_base_robooterx_node)
    return ld
