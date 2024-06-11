from launch import LaunchDescription
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    sim_time_launch_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description='Use simulation clock if true')

    robot_type_launch_arg = DeclareLaunchArgument(
        "robot_type", default_value="0",
        description="Mobile base robot type, Refer to README.md for more details."
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

    publish_odom_launch_arg = DeclareLaunchArgument(
        "publish_odom_tf", default_value="true",
        description="Publish wheel odometry tf"
    )

    node = Node(
        package="wrp_sdk_robot",
        name="mobile_base_node",
        executable="mobile_base_node",
        output="screen",
        parameters=[{
            "robot_type": LaunchConfiguration("robot_type"),
            "can_device": LaunchConfiguration("can_device"),
            "base_frame": LaunchConfiguration("base_frame"),
            "odom_frame": LaunchConfiguration("odom_frame"),
            "auto_reconnect": LaunchConfiguration("auto_reconnect"),
            "publish_odom_tf": LaunchConfiguration("publish_odom_tf"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }]
    )

    return LaunchDescription([
        sim_time_launch_arg,
        robot_type_launch_arg,
        can_device_launch_arg,
        base_frame_launch_arg,
        odom_frame_launch_arg,
        auto_reconnect_launch_arg,
        publish_odom_launch_arg,
        node,
    ])
