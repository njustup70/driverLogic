"""
仅执行 build_spear 的精简入口。
"""

import asyncio

from geometry_msgs.msg import PointStamped
from std_msgs.msg import String, UInt8MultiArray

from MainLogic import globalCallback as gcb
from MainLogic.app import build_spear
from MainLogic.core import ros_bridge_node as ros_bridge_module
from MainLogic.core.serial_node import start_serial_process


async def async_main():
    serial_port = "/dev/serial_qh"
    baudrate = 921600
    start_serial_process(serial_port=serial_port, baudrate=baudrate)

    assert ros_bridge_module.RosBridgeNodeInstance is not None, (
        "RosBridgeNodeInstance is not initialized yet!"
    )

    bridge = ros_bridge_module.RosBridgeNodeInstance
    bridge.register_serial_sub(gcb.mcu_transmit_callback)
    bridge.register_serial_sub(gcb.serial_action_return_callback)
    bridge.register_ros2_sub("qr_detection_result", gcb.ros_qr_callback, type=String)
    bridge.register_ros2_sub("spear_status", gcb.spear_callback, type=UInt8MultiArray)
    bridge.register_ros2_sub("/arucopnp/offset_mm", gcb.spear_offset_callback, type=PointStamped)
    bridge.register_ros2_pub("/update_exec_req", String)

    await build_spear()
