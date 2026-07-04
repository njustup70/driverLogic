# USAGE: ros2 launch odin_ros_driver odin1_ros2.launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('odin_ros_driver')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(package_dir, 'config', 'control_command.yaml'),
        description='Path to the control config YAML file',
    )

    host_sdk_node = Node(
        package='odin_ros_driver',
        executable='host_sdk_sample',
        name='host_sdk_sample',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
        }],
    )

    ld = LaunchDescription()
    ld.add_action(config_file_arg)
    ld.add_action(host_sdk_node)
    return ld
