#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    declare_odom_frame = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom_wheel',
        description='Wheel odometry frame (corrected by SLAM)'
    )

    declare_base_frame = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Base link frame in map'
    )

    declare_slam_odom = DeclareLaunchArgument(
        'slam_odom',
        default_value="['camera_init']",
        description='SLAM map frames list'
    )

    declare_slam_base_link = DeclareLaunchArgument(
        'slam_base_link',
        default_value="['body','aft_mapped']",
        description='SLAM base frames list'
    )

    declare_odom_topic = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Wheel odometry topic'
    )

    declare_laser_to_base = DeclareLaunchArgument(
        'laser_to_base',
        default_value='[0.1, -0.1, 0.0]',
        description='Laser to base transform [x, y, yaw(rad)]'
    )

    declare_riqiang_y = DeclareLaunchArgument(
        'riqiang_y',
        default_value='-0.10975',
        description='Riqiang lidar Y offset'
    )

    declare_slam_to_map = DeclareLaunchArgument(
        'slam_to_map',
        default_value='[0.73651, -0.16625, 0.0]',
        description='SLAM to map offset'
    )

    declare_debug = DeclareLaunchArgument(
        'debug',
        default_value='true',
        description='Enable debug print'
    )

    fusion_node = Node(
        package='perception',              # ⚠️ 改成你真实的包名
        executable='slam_riqiang.py',        # ⚠️ 改成你安装后的可执行名
        name='fusion_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'odom_frame': LaunchConfiguration('odom_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'slam_odom': LaunchConfiguration('slam_odom'),
            'slam_base_link': LaunchConfiguration('slam_base_link'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'laser_to_base': LaunchConfiguration('laser_to_base'),
            'riqiang_y': LaunchConfiguration('riqiang_y'),
            'slam_to_map': LaunchConfiguration('slam_to_map'),
            'debug': LaunchConfiguration('debug'),
        }]
    )

    return LaunchDescription([
        declare_odom_frame,
        declare_base_frame,
        declare_slam_odom,
        declare_slam_base_link,
        declare_odom_topic,
        declare_laser_to_base,
        declare_riqiang_y,
        declare_slam_to_map,
        declare_debug,
        fusion_node,
    ])