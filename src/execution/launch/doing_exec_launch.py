from launch import LaunchDescription
from launch_ros.actions import Node
import json

def generate_launch_description():

    exec_list = [
        ["move", 11, 11],
        ["turn", 90]
    ]

    doing_exec_node = Node(
        package='execution',
        executable='doing_exec.py',
        name='doing_exec',
        output='screen',
        parameters=[{
            'error_value_xy': 0.1,
            'error_value_yaw': 0.1,
            'R2place1_exec_list': json.dumps(exec_list)
        }]
    )

    return LaunchDescription([
        doing_exec_node
    ])