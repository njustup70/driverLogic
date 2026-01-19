'''
启动驱动节点和工具节点
播放数据包
'''
import os
import sys
import glob
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess,IncludeLaunchDescription,DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch.conditions import LaunchConfigurationEquals
def generate_launch_description():
    ld=LaunchDescription()
    ld.add_action(DeclareLaunchArgument('rate',default_value='1',description='rate of rosbag play'))
    ld.add_action(DeclareLaunchArgument('loop',default_value='false',description='loop of rosbag play'))
    package_path=get_package_share_directory('rc_bringup')
    #启动rosbag
    rosbag_root_path='/home/Elaina/ros2_ws/bag_play'
    qos_file=os.path.join(package_path,'config','ros_bag_play_qos.yaml')
    #查找root_path中的第一个文件夹中的db播放
    folders = [d for d in os.listdir(rosbag_root_path) if os.path.isdir(os.path.join(rosbag_root_path, d))]
    if folders:
        first_folder = folders[0]  # 取第一个文件夹
        rosbag_path = os.path.join(rosbag_root_path, first_folder)

        # 查找 .db3 文件
        db3_files = glob.glob(os.path.join(rosbag_path, "*.db3"))
        if db3_files:
            rosbag_path=db3_files[0]
    else:
        print("没有找到文件夹")
    ros_bag_exe=ExecuteProcess(
        # cmd=["bash","-c","ros2 bag play --loop {}".format(rosbag_path)],
        cmd=["ros2","bag","play","--rate",LaunchConfiguration('rate'),rosbag_path,"--qos-profile-overrides-path",qos_file,'--remap','/tf_static:=/trash'],
        output='screen',
        condition=LaunchConfigurationEquals('loop', 'false')
    )
    ros_bag_loop_exe=ExecuteProcess(
        cmd=["ros2","bag","play","--rate",LaunchConfiguration('rate'),"--loop",rosbag_path,"--qos-profile-overrides-path",qos_file,'--remap','/tf_static:=/trash'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('loop'))
    )
    # ros_bag_node=Node(
    #     package='python_pkg',
    #     executable='bag_play',
    #     name='bag_play_node',
    #     parameters=[{'loop': LaunchConfiguration('loop'),
    #                  'rate': LaunchConfiguration('rate')}],
    #     output='screen',
    #     emulate_tty=True,
    # )
    #启动utils
    utils_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rc_bringup'),'launch','utils_bringup.launch.py')
        ),
        
    )
    ld.add_action(utils_launch)
    # ld.add_action(ros_bag_node)
    # ld.add_action(odomtransform_tf_node)
    ld.add_action(ros_bag_exe)
    ld.add_action(ros_bag_loop_exe)
    # ld.add_action(image_bridge_node)
    return ld
     