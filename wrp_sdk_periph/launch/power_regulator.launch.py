from launch import LaunchDescription
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    sim_time_launch_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description='Use simulation clock if true')

    device_path_launch_arg = DeclareLaunchArgument(
        "device_path", default_value="can0",
        description="Path to power regulator can port"
    )

    node = Node(
        package="wrp_sdk_periph",
        name="power_regulator_node",
        executable="power_regulator_node",
        output="screen",
        parameters=[{
            "device_path": LaunchConfiguration("device_path"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    )

    return LaunchDescription([
        sim_time_launch_arg,
        device_path_launch_arg,
        node,
    ])
