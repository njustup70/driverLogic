"""
仅执行 build_spear 的精简入口。
"""

import asyncio
import math

from geometry_msgs.msg import PointStamped
from std_msgs.msg import String, UInt8MultiArray

from MainLogic import globalCallback as gcb
from MainLogic.app import build_spear
from MainLogic.app.actions import build_spear_active
from MainLogic.Lib.bytes import turn_to_bytes
from MainLogic.core import ros_bridge_node as ros_bridge_module
from MainLogic.core.serial_node import start_serial_process


def debug_spear_offset_callback(msg):
    if not build_spear_active.value:
        return

    left_mm = float(msg.point.x)
    up_mm = float(msg.point.y)
    if not math.isfinite(left_mm) or not math.isfinite(up_mm):
        return

    bridge = ros_bridge_module.RosBridgeNodeInstance
    if bridge is None:
        return

    payload = gcb.SPEAR_OFFSET_COMMAND + turn_to_bytes([left_mm, up_mm])
    frame = b'\xFA' + payload
    bridge.writeBytes(payload)
    print(
        "[R2_build_spear] publish serial_tx "
        f"frame={frame.hex(' ')} "
        f"left_mm={left_mm:.2f} up_mm={up_mm:.2f}",
        flush=True,
    )


async def async_main():
    serial_port = "/dev/serial_ch340"
    baudrate = 921600
    start_serial_process(serial_port=serial_port, baudrate=baudrate)

    assert ros_bridge_module.RosBridgeNodeInstance is not None, (
        "RosBridgeNodeInstance is not initialized yet!"
    )

    bridge = ros_bridge_module.RosBridgeNodeInstance
    bridge.register_serial_sub(gcb.mcu_transmit_callback, 0xAA)
    bridge.register_serial_sub(gcb.serial_action_return_callback, 0xA3)
    bridge.register_ros2_sub("qr_detection_result", gcb.ros_qr_callback, type=String)
    bridge.register_ros2_sub("spear_status", gcb.spear_callback, type=UInt8MultiArray)
    bridge.register_ros2_sub("/arucopnp/offset_mm", debug_spear_offset_callback, type=PointStamped)
    bridge.register_ros2_pub("/update_exec_req", String)

    await build_spear()
