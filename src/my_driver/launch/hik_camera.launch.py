'''
Author: Nagisa 2964793117@qq.com
Date: 2026-01-25 15:07:59
LastEditors: Nagisa 2964793117@qq.com
LastEditTime: 2026-06-27 16:27:17
FilePath: \driverLogic\src\my_driver\launch\hik_camera.launch.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 使用 hik_camera_ros2_driver，並在單一 launch 中直接傳參
    camera_node = Node(
        package='hik_camera_ros2_driver',
        executable='hik_camera_ros2_driver_node',
        name='hik_camera_ros2_driver',
        output='screen',
        parameters=[
            {'serial_number': 'DA4976553'},
            {'camera_topic': '/hik_camera/image_raw'},
            {'exposure_time': 10000},
            {'gain': 0.0},
            {'acquisition_frame_rate': 50.0},
            {'pixel_format': 'BayerGB8'},
        ],
    )

    return LaunchDescription([
        camera_node,
    ])
