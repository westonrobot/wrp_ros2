import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    mobile_base_tracer_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory("wrp_ros2"),
                                                    "launch",
                                                    "mobile_base",
                                                    "mobile_base.launch.py")]),
        launch_arguments={
            "robot_type": "agilex",
            "can_device": "can0",
            "base_frame": "base_link",
            "odom_frame": "odom",
            "auto_reconnect": "true",
            "motion_type": "differential",
        }.items(),
    )

    ld.add_action(mobile_base_tracer_node)
    return ld
