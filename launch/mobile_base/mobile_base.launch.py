from launch import LaunchDescription
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # "weston" == weston robot base
    # "agilex" == agilexV2 robot base
    # "vbot"   == virtual weston robot base
    robot_base_type_launch_arg = DeclareLaunchArgument(
        "robot_base_type", default_value="weston",
        description="Mobile base robot type"
    )

    can_device_launch_arg = DeclareLaunchArgument(
        "can_device", default_value="can0",
        description="CAN device port"
    )

    base_frame_launch_arg = DeclareLaunchArgument(
        "base_frame", default_value="base_link",
        description="Base frame id"
    )

    odom_frame_launch_arg = DeclareLaunchArgument(
        "odom_frame", default_value="odom",
        description="Odometry frame id"
    )

    # true == always try to take control token
    # false == only take using the access control service
    auto_reconnect_launch_arg = DeclareLaunchArgument(
        "auto_reconnect", default_value="true",
        description="Control token access control"
    )

    node = Node(
        package="wrp_ros2",
        name="mobile_base_node",
        executable="mobile_base_node",
        output="screen",
        parameters=[{
            "robot_base_type": LaunchConfiguration("robot_base_type"),
            "can_device": LaunchConfiguration("can_device"),
            "base_frame": LaunchConfiguration("base_frame"),
            "odom_frame": LaunchConfiguration("odom_frame"),
            "auto_reconnect": LaunchConfiguration("auto_reconnect"),
        }],
    )

    ld.add_action(robot_base_type_launch_arg)
    ld.add_action(can_device_launch_arg)
    ld.add_action(base_frame_launch_arg)
    ld.add_action(odom_frame_launch_arg)
    ld.add_action(auto_reconnect_launch_arg)
    ld.add_action(node)
    return ld
