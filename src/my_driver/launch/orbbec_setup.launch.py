import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, LogInfo

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    # 获取 OrbbecSDK_ROS2 包的路径
    orbbec_ws_package_prefix = os.path.join(os.environ['HOME'], 'packages/orbbec_ws/src/OrbbecSDK_ROS2')
    orbbec_pkg_package_prefix = get_package_share_directory('orbbec_camera')
    
    # 创建 LaunchDescription 对象
    ld = LaunchDescription()
    # 使用 LaunchConfiguration 获取 YAML 文件的路径
 
    # 在终端打印 YAML 文件路径
    launch_arguments = {
        'color_format':'YUYV',
        'depth_format': 'Y16',
        'depth_width':'1280',
        'depth_height':'800',
        'depth_fps':'30',
        'color_width': '1280',
        'color_height': '800',
        'color_fps': '30',
        'color.image_raw.enable_pub_plugins': ["image_transport/raw"],
        'depth.image_raw.enable_pub_plugins': ["image_transport/raw"],
        'color_qos':'SENSOR_DATA',
        'depth_qos':'SENSOR_DATA',
        #此处可添加更多参数
    }

    log_launch_arguments = LogInfo(msg=['Launch arguments: ', str(launch_arguments)])

    include_sdk_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(orbbec_pkg_package_prefix, 'launch', 'gemini_330_series_low_cpu.launch.py')),
        launch_arguments=launch_arguments.items()
    )
    ld.add_action(log_launch_arguments)
    ld.add_action(include_sdk_launch)

    return ld
