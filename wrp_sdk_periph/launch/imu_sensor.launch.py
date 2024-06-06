from launch import LaunchDescription
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    sim_time_launch_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description='Use simulation clock if true')
    
    sensor_model_launch_arg = DeclareLaunchArgument(
        "sensor_model", default_value="hipnuc",
        description="Sensor model: hipnuc/wit"
    )

    device_path_launch_arg = DeclareLaunchArgument(
        "device_path", default_value="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
        description="Path to receiver port"
    )

    baud_rate_launch_arg = DeclareLaunchArgument(
        "baud_rate", default_value="115200",
        description="Sensor baudrate"
    )

    frame_id_launch_arg = DeclareLaunchArgument(
        "frame_id", default_value="imu_link",
        description="IMU frame id"
    )

    node = Node(
        package="wrp_sdk_periph",
        name="imu_sensor_node",
        executable="imu_sensor_node",
        output="screen",
        parameters=[{
            "sensor_model": LaunchConfiguration("sensor_model"),
            "device_path": LaunchConfiguration("device_path"),
            "baud_rate": LaunchConfiguration("baud_rate"),
            "frame_id": LaunchConfiguration("frame_id"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    )

    return LaunchDescription([
        sim_time_launch_arg,
        sensor_model_launch_arg,
        device_path_launch_arg,
        baud_rate_launch_arg,
        frame_id_launch_arg,
        node,
    ])
