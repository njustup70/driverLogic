import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    hik_share_dir = get_package_share_directory("hik_camera_ros2")
    default_rviz_config = os.path.join(hik_share_dir, "config", "default.rviz")

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument("serial_number", default_value=""))
    ld.add_action(DeclareLaunchArgument("topic_name", default_value="/hik_camera/image_raw"))
    ld.add_action(DeclareLaunchArgument("exposure_time", default_value="28000.0"))
    ld.add_action(DeclareLaunchArgument("gain", default_value="10.0"))
    ld.add_action(DeclareLaunchArgument("frame_rate", default_value="200.0"))
    ld.add_action(DeclareLaunchArgument("pixel_format", default_value="BayerGB8"))

    ld.add_action(DeclareLaunchArgument("use_rviz", default_value="false"))
    ld.add_action(DeclareLaunchArgument("rviz_config", default_value=default_rviz_config))

    camera_node = Node(
        package="hik_camera_ros2",
        executable="hik_camera_node",
        name="hik_camera_node",
        output="screen",
        parameters=[
            {
                "serial_number": LaunchConfiguration("serial_number"),
                "topic_name": LaunchConfiguration("topic_name"),
                "exposure_time": ParameterValue(LaunchConfiguration("exposure_time"), value_type=float),
                "gain": ParameterValue(LaunchConfiguration("gain"), value_type=float),
                "frame_rate": ParameterValue(LaunchConfiguration("frame_rate"), value_type=float),
                "pixel_format": LaunchConfiguration("pixel_format"),
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="hik_camera_rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    ld.add_action(camera_node)
    ld.add_action(rviz_node)

    return ld
