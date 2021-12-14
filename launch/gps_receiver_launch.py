import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory("wrp_ros2"),
       "config",
       "peripheral",
       "gps_receiver_node_config.yaml",
    )

    node = Node(
        package= "wrp_ros2",
        name= "gps_receiver_node",
        executable= "gps_receiver_node",
        output= "screen",
        parameters= [config],
    )

    ld.add_action(node)
    return ld