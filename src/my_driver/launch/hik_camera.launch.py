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
            {'camera_name': '/hik_camera'},
            {'exposure_time': 7500},
            {'gain': 100.0},
            {'acquisition_frame_rate': 50.0},
            {'pixel_format': 'BayerGB8'},
        ],
    )

    return LaunchDescription([
        camera_node,
    ])
