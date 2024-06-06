from launch import LaunchDescription
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    sim_time_launch_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description='Use simulation clock if true')

    device_path_launch_arg = DeclareLaunchArgument(
        "device_path", default_value="/dev/ttyUSB0",
        description="Path to receiver port"
    )

    baud_rate_launch_arg = DeclareLaunchArgument(
        "baud_rate", default_value="115200",
        description="Sensor baudrate"
    )

    sampling_freq_launch_arg = DeclareLaunchArgument(
        "sampling_freq", default_value="10.0",
        description="Sampling Frequency"
    )


    node = Node(
        package="wrp_sdk_periph",
        name="lift_controller_node",
        executable="lift_controller_node",
        output="screen",
        parameters=[{
            "device_path": LaunchConfiguration("device_path"),
            "baud_rate": LaunchConfiguration("baud_rate"),
            "sampling_freq": LaunchConfiguration("sampling_freq"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    )

    return LaunchDescription([
        sim_time_launch_arg,
        device_path_launch_arg,
        baud_rate_launch_arg,
        sampling_freq_launch_arg,
        node,
    ])
