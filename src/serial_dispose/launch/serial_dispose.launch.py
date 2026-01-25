from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import LogInfo

def generate_launch_description():
    # -------- Launch 参数声明 --------
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/serial_sick',
        description='Serial port device'
    )

    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Serial baudrate'
    )

    # -------- Node 定义 --------
    serial_node = Node(
        package='serial_dispose',          # ⚠️ 改成你的包名
        executable='serial_dispose',     # ⚠️ setup.py / entry point 中的可执行名
        name='serial_dispose_node',
        output='screen',
        parameters=[
            {
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate'),
            }
        ]
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        serial_node,
    ])