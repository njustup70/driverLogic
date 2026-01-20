'''
启动rosbag 记录数据 
rosbridge 桥接ros1 ros2话题
启动foxglove用来可视化
'''
import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess,IncludeLaunchDescription,DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch.actions import RegisterEventHandler, Shutdown
from launch.event_handlers import OnShutdown
def generate_launch_description():
    local_path=os.path.join(get_package_share_directory('rc_bringup'))
    ld=LaunchDescription()
    
    # ld.add_action(DeclareLaunchArgument('use_tf_publish',default_value='false',description='Publish tf tree if use is True'))
    # ld.add_action(DeclareLaunchArgument('use_ros1_bridge',default_value='true',description='Use ros1_bridge if use is True'))
    # ld.add_action(DeclareLaunchArgument('xacro_file',default_value=os.path.join(get_package_share_directory('my_tf_tree'),'urdf','r2.urdf.xacro'),description='xacro file path'))
    foxglove_node=ComposableNode(
        package='foxglove_bridge',
        plugin='foxglove_bridge::FoxgloveBridge',
        name='foxglove_bridge_node',
        parameters=[ {'send_buffer_limit': 1000000000}],
        extra_arguments=[{'use_intra_process_comms': True},
                    {'use_multi_threaded_executor': True},
                    {'ros__arguments': ['--log-level', 'fatal']}
        ]
        )
    ros_master_rm=ExecuteProcess(
        # condition=IfCondition(LaunchConfiguration('use_ros1_bridge')),
        cmd=["bash","-c","sudo docker stop ros-noetic-full && sudo docker rm ros-noetic-full"]
    )
    ros_master_exe=ExecuteProcess(
        # condition=IfCondition(LaunchConfiguration('use_ros1_bridge')),
        cmd=["bash","-c","cd ~/ros2_ws/packages/roscore && sudo docker-compose up -d"]
    )
    shutdown_docker= RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                ExecuteProcess(
                    cmd=["bash", "-c", "cd ~/ros2_ws/packages/roscore && sudo docker-compose down"],
                    output='screen'
                )
            ]
        )
    )
    ros_bridge_exe=ExecuteProcess(
        # condition=IfCondition(LaunchConfiguration('use_ros1_bridge')),
        # cmd=["bash","-c","~/docker/ros2-modules/packages/ros-bridge/ros_bridge_run.sh"],
        cmd=["bash","-c","python3 ~/ros2_ws/packages/ros-bridge/ros_bridge_run.py"],
        output='screen',
    )
    #TF树相关
    xacro_file_path=LaunchConfiguration('xacro_file') # 获取 xacro 文件路径
    # 解析 Xacro 文件并生成 URDF
    '''
    在其他地方自己定义机器人的tf
    '''
    # robot_description = Command([
    #     FindExecutable(name='xacro'),  # 查找 xacro 可执行文件
    #     ' ',  # 确保命令和路径之间有空格
    #     xacro_file_path  # 传入 xacro 文件路径
    # ])

    # # 机器人状态发布节点
    # robot_state_publisher_node = ComposableNode(
    #     condition=IfCondition(LaunchConfiguration('use_tf_publish')),
    #     package='robot_state_publisher',
    #     plugin='robot_state_publisher::RobotStatePublisher',
    #     name='robot_state_publisher',
    #     parameters=[{'robot_description': robot_description}],
    # )
   


    compose_container=ComposableNodeContainer(
        namespace='',
        name='start_container',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[foxglove_node],
        arguments=['--ros-args', '--log-level', 'fatal'],
        output='screen',
        emulate_tty=True,
    )
    ld.add_action(ros_master_rm)
    ld.add_action(ros_master_exe)
    ld.add_action(ros_bridge_exe)
    # ld.add_action(ros_bag_exe)
    ld.add_action(shutdown_docker)
    # ld.add_action(ros_bag_action)
    ld.add_action(compose_container)
    return ld
