'''
启动驱动节点和工具节点
主参考,后续化简用
'''
import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess,IncludeLaunchDescription,DeclareLaunchArgument,TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node 
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable

def generate_launch_description():
    ld=LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_extern_imu',default_value='false',description='Start extern imu node if use is True'))
    ld.add_action(DeclareLaunchArgument('use_imu_transform',default_value='true',description='Start imu transform node if use is True'))
    ld.add_action(DeclareLaunchArgument('record_lidar',default_value='true',description='Record lidar data if use is True'))
    ld.add_action(DeclareLaunchArgument('record_imu',default_value='true',description='Record imu data if use is True'))
    ld.add_action(DeclareLaunchArgument('record_ms200',default_value='true',description='Record ms200 data if use is True'))

    # ld.add_action(DeclareLaunchArgument('topic_blacklist',default_value=['*/compressed*']))
    get_package_share_directory('my_driver')
    get_package_share_directory('rc_bringup')
    #启动mid360
    mid360_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('my_driver'),'launch','rs_airy.launch.py')
        ),
        launch_arguments={
            'use_rviz': 'false',  #不启动rviz
        }.items(),
    )
    #启动下位机通信
    serial_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('serial_dispose'), 'launch', 'serial_dispose.launch.py')
        )
    )
    #启动imu转换
    imu_transform_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('perception'),'launch','imu_transform.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_imu_transform'))
    )
    #启动utils
    utils_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rc_bringup'),'launch','utils_bringup.launch.py')
        ),
    )
    #启动手柄
    joy_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('my_driver'),'launch','joy.launch.py')
        ),
    )
    #启动fuck_slam
    fuck_slam_node=Node(
        package='my_driver',
        executable='fuck_slam.py',
        name='fuck_slam',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'docker_name':'voxel_slam_container'
            }
        ]
    )
    #启动ms200
    ms200_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('my_driver'),'launch','ms200_scan.launch.py')
        ),
    )
    #启动riqiang
    riqiang_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('perception'),'launch','riqiang.launch.py')
        )
    )
    #启动海康相机
    hik_camera_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('my_driver'),'launch','hik_camera.launch.py')
        ),
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
    # ld.add_action(fusion_node)
    ld.add_action(mid360_launch)
    #ld.add_action(communicate_node)
    # ld.add_action(imu_transform_launch)
    ld.add_action(serial_launch)
    ld.add_action(ms200_launch)
    ld.add_action(utils_launch)
    ld.add_action(joy_launch)
    ld.add_action(riqiang_launch)
    ld.add_action(hik_camera_launch)
    #ld.add_action(slam_fusion)
    ld.add_action(ros_bag_action)
    ld.add_action(fuck_slam_node)
    return ld
     