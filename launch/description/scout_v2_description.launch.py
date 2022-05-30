import os
import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    model_name = 'scout_v2.xacro'
    model_path = os.path.join(get_package_share_directory('wrp_ros2'), "urdf/scout_v2", model_name)
    print(model_path)
    return launch.LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
            description='Use simulation clock if true'),
        launch.actions.LogInfo(msg='use_sim_time: '),
        launch.actions.LogInfo(msg=launch.substitutions.LaunchConfiguration('use_sim_time')),
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time'),
                'robot_description':Command(['xacro',' ', model_path])
            }]),
    ])
