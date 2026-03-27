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
            {'topic_name': '/hik_camera/image_raw'},
            {'exposure_time': 15000},
            {'gain': 10.0},
            {'acquisition_frame_rate': 50.0},
            {'pixel_format': 'BayerGB8'},
        ],
    )

    return LaunchDescription([
        camera_node,
    ])
