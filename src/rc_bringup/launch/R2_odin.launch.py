#!/usr/bin/env python3
"""启动底盘测试：airy + MainLogic(slamMain) + foxglove。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    ld = LaunchDescription()

    odin_driver=IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('odin_ros_driver'), 'launch', 'odin1_ros2.launch.py')
        ),
        launch_arguments={
            'config_file': '/home/Elaina/ros2_ws/src/odin_ros_driver/config/control_command.yaml',
        }.items(),
    )
    r2_main = ExecuteProcess(
        cmd=[
            'bash',
            '-c',
            'python3 ~/ros2_ws/src/MainLogic/Main.py --main-module odintestMain --main-func async_main',
        ],
        output='screen',
        emulate_tty=True,
    )

    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        emulate_tty=False,
        arguments=['--ros-args', '--log-level', 'FATAL'],
    )
    ros_bag_node=  Node(
                    # condition=IfCondition(LaunchConfiguration('use_rosbag_record')),
                    package='python_pkg',
                    executable='rosbag_record',
                    name='rosbag_record',
                    output='screen',
                    emulate_tty=True,
                    parameters=[
                        # {'topic_blacklist':LaunchConfiguration("topic_blacklist")}
                        {'topic_blacklist':['*/compressed*','/livox/lidar','livox/imu']}
                    ]
                )
    ros_bag_action=TimerAction(
        period=5.0,  # Delay in seconds
        actions=[ros_bag_node]
    )
    # ld.add_action(airy_launch)
    ld.add_action(r2_main)
    ld.add_action(odin_driver)
    ld.add_action(foxglove)
    ld.add_action(ros_bag_action)
    return ld
