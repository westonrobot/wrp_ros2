import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory("lift_server"),
       "config",
       "lift_server_node_config.yaml",
    )

    node = Node(
        package= "lift_server",
        name= "lift_server_node",
        executable= "lift_server_node",
        output= "screen",
        parameters= [config],
    )

    ld.add_action(node)
    return ld