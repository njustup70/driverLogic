import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # 获取 package 路径
    my_driver_dir = get_package_share_directory('my_driver')
    rc_bringup_dir = get_package_share_directory('rc_bringup')

    # workspace/src 路径（用于找到 MainLogic）
    workspace_src = os.path.dirname(os.path.dirname(os.path.dirname(my_driver_dir)))
    main_logic_path = os.path.join(workspace_src, 'MainLogic', 'Main.py')

    # mid360
    mid360_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_driver_dir, 'launch', 'mid360_bringup.launch.py')
        )
    )

    # sick with slam
    sick_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rc_bringup_dir, 'launch', 'sick_with_slam.launch.py')
        )
    )

    # MainLogic
    main_logic_process = ExecuteProcess(
        cmd=['python3', main_logic_path],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(mid360_launch)
    ld.add_action(sick_slam_launch)
    ld.add_action(main_logic_process)

    return ld