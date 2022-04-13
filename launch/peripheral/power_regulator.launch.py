from launch import LaunchDescription
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    device_path_launch_arg = DeclareLaunchArgument(
        "device_path", default_value="can0",
        description="Path to power regulator can port"
    )

    node = Node(
        package="wrp_ros2",
        name="power_regulator_node",
        executable="power_regulator_node",
        output="screen",
        parameters=[{
            "device_path": LaunchConfiguration("device_path"),
        }],
    )

    ld.add_action(device_path_launch_arg)
    ld.add_action(node)
    return ld
