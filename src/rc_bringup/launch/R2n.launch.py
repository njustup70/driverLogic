'''
Author: Nagisa 2964793117@qq.com
Date: 2026-06-26 11:01:15
LastEditors: Nagisa 2964793117@qq.com
LastEditTime: 2026-07-09 19:15:05
FilePath: \driverLogic\src\rc_bringup\launch\R2n.launch.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
#!/usr/bin/env python3
"""启动底盘测试：airy + MainLogic(slamMain) + foxglove。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import TimerAction,ExecuteProcess

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
    hik_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('my_driver'), 'launch', 'hik_camera.launch.py')
        )
    )

    slam_main = ExecuteProcess(
        cmd=[
            'bash',
            '-c',
            'python3 ~/ros2_ws/src/MainLogic/Main.py --main-module R2n_Main --main-func async_main',
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
    mcu_log=ExecuteProcess(
        cmd=['python3', os.path.join( '/home/Elaina/ros2_ws/src/my_driver/scripts', 'mcu_log.py')],
        output='screen',
        emulate_tty=True,
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
                        {'topic_blacklist':['*/compressed*','/livox/lidar','/map*','/hik_camera*','serial_tx','serial_rx','/*imu*']}
                    ]
                )
    ros_bag_action=TimerAction(
        period=5.0,  # Delay in seconds
        actions=[ros_bag_node]
    )
    ld.add_action(airy_launch)
    ld.add_action(hik_launch)
    ld.add_action(slam_main)
    ld.add_action(foxglove)
    ld.add_action(ros_bag_action)
    ld.add_action(mcu_log)
    return ld
