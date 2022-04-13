from launch import LaunchDescription
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    sensor_model_launch_arg = DeclareLaunchArgument(
        "sensor_model", default_value="dyp_a05",
        description="ultrasonic sensor model"
    )

    device_path_launch_arg = DeclareLaunchArgument(
        "device_path", default_value="/dev/ttyUSB0",
        description="Path to receiver port"
    )

    baud_rate_launch_arg = DeclareLaunchArgument(
        "baud_rate", default_value="115200",
        description="Sensor baudrate"
    )

    frame_id_launch_arg = DeclareLaunchArgument(
        "frame_id", default_value="ultrasonic_link",
        description="Ultrasonic frame id"
    )

    topic_name_launch_arg = DeclareLaunchArgument(
        "topic_name", default_value="ultrasonic",
        description="Ultrasonic topic name"
    )

    node = Node(
        package="wrp_ros2",
        name="ultrasonic_sensor_node",
        executable="ultrasonic_sensor_node",
        output="screen",
        parameters=[{
            "sensor_model": LaunchConfiguration("sensor_model"),
            "device_path": LaunchConfiguration("device_path"),
            "baud_rate": LaunchConfiguration("baud_rate"),
            "frame_id": LaunchConfiguration("frame_id"),
            "topic_name": LaunchConfiguration("topic_name"),
        }],
    )

    ld.add_action(sensor_model_launch_arg)
    ld.add_action(device_path_launch_arg)
    ld.add_action(baud_rate_launch_arg)
    ld.add_action(frame_id_launch_arg)
    ld.add_action(topic_name_launch_arg)
    ld.add_action(node)
    return ld
