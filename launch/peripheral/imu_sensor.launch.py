from launch import LaunchDescription
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    device_path_launch_arg = DeclareLaunchArgument(
        "device_path", default_value="/dev/ttyUSB0",
        description="Path to receiver port"
    )

    baud_rate_launch_arg = DeclareLaunchArgument(
        "baud_rate", default_value="115200",
        description="Sensor baudrate"
    )

    frame_id_launch_arg = DeclareLaunchArgument(
        "frame_id", default_value="imu",
        description="IMU frame id"
    )

    node = Node(
        package="wrp_ros2",
        name="imu_sensor_node",
        executable="imu_sensor_node",
        output="screen",
        parameters=[{
            "device_path": LaunchConfiguration("device_path"),
            "baud_rate": LaunchConfiguration("baud_rate"),
            "frame_id": LaunchConfiguration("frame_id"),
        }],
    )

    ld.add_action(device_path_launch_arg)
    ld.add_action(baud_rate_launch_arg)
    ld.add_action(frame_id_launch_arg)
    ld.add_action(node)
    return ld
