'''
启动调试与仿真环境
包含：
1. Robot Simulation (可选启动, 可配内部模块)
2. Sick Driver (可配端口)
3. SLAM Fusion
'''
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_simulation', default_value='true', description='是否启动机器人仿真节点'))
    ld.add_action(DeclareLaunchArgument('sick_port', default_value='/dev/ttyACM0', description='Sick雷达的串口路径'))
    # use_simulation=true 时参数才影响
    ld.add_action(DeclareLaunchArgument('enable_odom', default_value='true', description='仿真: 开启里程计'))
    ld.add_action(DeclareLaunchArgument('enable_sick', default_value='true', description='仿真: 开启虚拟雷达'))
    ld.add_action(DeclareLaunchArgument('enable_slam_tf', default_value='false', description='仿真: 开启虚拟SLAM TF'))

    sim_node = Node(
        package='perception',
        executable='robot_simulation.py', 
        name='sim_all_in_one',
        output='screen',
        emulate_tty=True,
        # 当 use_simulation 为 true 时才启动
        condition=IfCondition(LaunchConfiguration('use_simulation')),
        parameters=[{
            'enable_odom': LaunchConfiguration('enable_odom'),
            'enable_sick': LaunchConfiguration('enable_sick'),
            'enable_slam_tf': LaunchConfiguration('enable_slam_tf')
        }]
    )

    sick_node = Node(
        package='perception',
        executable='sick_get.py',
        name='sick_get_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'port': LaunchConfiguration('sick_port')
        }]
    )


    slam_fusion_node = Node(
        package='perception',
        executable='slam.py', # 或者是 slam_riqiang.py，看你具体用哪个
        name='fusion_node',
        output='screen',
        emulate_tty=True,
        # parameters=[{'odom_topic': '/odom'}]
    )

    ld.add_action(sim_node)
    ld.add_action(sick_node)
    ld.add_action(slam_fusion_node)

    return ld