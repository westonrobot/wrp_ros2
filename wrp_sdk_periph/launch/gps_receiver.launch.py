from launch import LaunchDescription
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    sim_time_launch_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description='Use simulation clock if true')

    device_path_launch_arg = DeclareLaunchArgument(
        "device_path", default_value="/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00",
        description="Path to receiver port"
    )

    baud_rate_launch_arg = DeclareLaunchArgument(
        "baud_rate", default_value="115200",
        description="Receiver baudrate"
    )

    frame_id_launch_arg = DeclareLaunchArgument(
        "frame_id", default_value="gps_link",
        description="GPS frame id"
    )

    node = Node(
        package="wrp_ros2",
        name="gps_receiver_node",
        executable="gps_receiver_node",
        output="screen",
        parameters=[{
            "device_path": LaunchConfiguration("device_path"),
            "baud_rate": LaunchConfiguration("baud_rate"),
            "frame_id": LaunchConfiguration("frame_id"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    )

    return LaunchDescription([
        sim_time_launch_arg,
        device_path_launch_arg,
        baud_rate_launch_arg,
        frame_id_launch_arg,
        node,
    ])
