"""
简单动作
"""

import asyncio
import math

from MainLogic import globalCallback as gcb
from MainLogic.core import ros_bridge_node as ros_bridge_module
from MainLogic.Lib.AsyncTools import AsyncVariable
from MainLogic.Lib.AsyncTools import async_property
from MainLogic.Lib.bytes import turn_to_bytes
from MainLogic.Lib.CheckActions import check_finish
from std_msgs.msg import String

# 获取矛头
async def take_spear_head():
    assert ros_bridge_module.RosBridgeNodeInstance is not None, "RosBridgeNodeInstance is not initialized yet!"
    action_type = b'\xA2'
    ros_bridge_module.RosBridgeNodeInstance.writeBytes(action_type)  # 发送取矛头指令
    await check_finish()


# 矛头对接
order_spear = async_property(float)
build_spear_active = AsyncVariable(False)
build_spear_finish = AsyncVariable(b'')
BUILD_SPEAR_ACTION_TYPE = b'\xA3'

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
        "[build_spear] publish serial_tx "
        f"frame={frame.hex(' ')} "
        f"left_mm={left_mm:.2f} up_mm={up_mm:.2f}",
        flush=True,
    )


async def build_spear_until_finish():
    assert ros_bridge_module.RosBridgeNodeInstance is not None, (
        "RosBridgeNodeInstance is not initialized yet!"
    )

    build_spear_finish.value = b""
    build_spear_active.value = True

    try:
        while build_spear_finish.value != BUILD_SPEAR_ACTION_TYPE:
            msg = String()
            msg.data = "spear_build"
            ros_bridge_module.RosBridgeNodeInstance.publish_ros2("/update_exec_req", msg)
            print("[build_spear] publish /update_exec_req spear_build", flush=True)

            try:
                result = await asyncio.wait_for(build_spear_finish, timeout=1.0)
            except asyncio.TimeoutError:
                continue
            if result == BUILD_SPEAR_ACTION_TYPE:
                print("[build_spear] build_spear finished by lower controller A3", flush=True)
                return
    finally:
        build_spear_active.value = False


async def QR():
    msg = "qr_recog"
    ros_bridge_module.RosBridgeNodeInstance.publish_ros2('/update_exec_req', msg)
    
    
# QR识别
class QRRecog:
    recog_qr_result = async_property(str)
QRRecogInstance = QRRecog()
def QR_recog():
    msg = "qr_recog"
    ros_bridge_module.RosBridgeNodeInstance.publish_ros2('/update_exec_req', msg)

def QR_recog_off():
    msg = "qr_recog_off"
    ros_bridge_module.RosBridgeNodeInstance.publish_ros2('/update_exec_req', msg)
