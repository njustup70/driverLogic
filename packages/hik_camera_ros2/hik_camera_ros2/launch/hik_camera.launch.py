import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 定義 rviz 設定檔的路徑
    rviz_config_file = os.path.join(
        get_package_share_directory('hik_camera_ros2'),
        'config',
        'default.rviz'
    )

    # 定義相機節點
    camera_node = Node(
        package='hik_camera_ros2',
        executable='hik_camera_node',
        name='hik_camera_node',
        output='screen',
        parameters=[
            {'serial_number': 'DA4976553'}, 
            {'topic_name': '/hik_camera/image_raw'}, 
            {'exposure_time': 15000.0},
            {'gain': 10.0},
            {'frame_rate': 200.0},
            {'pixel_format': 'BayerGB8'},
        ],
    )

    # 【新增】定義 rviz2 節點
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # 使用 -d 參數來加載我們的設定檔
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # 將兩個節點都加入到 LaunchDescription 中
    return LaunchDescription([
        camera_node,
        rviz_node
    ])
