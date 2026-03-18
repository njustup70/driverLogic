#!/usr/bin/env python3
"""启动底盘测试：airy + MainLogic(slamMain) + foxglove。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    airy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('my_driver'), 'launch', 'rs_airy.launch.py')
        ),
        launch_arguments={
            'use_rviz': 'false',
        }.items(),
    )

    slam_main = ExecuteProcess(
        cmd=[
            'bash',
            '-c',
            'python3 ~/ros2_ws/src/MainLogic/Main.py --main-module slamMain --main-func async_main',
        ],
        output='screen',
        emulate_tty=True,
    )

    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        emulate_tty=False,
    )

    ld.add_action(airy_launch)
    # ld.add_action(slam_main)
    ld.add_action(foxglove)
    return ld
