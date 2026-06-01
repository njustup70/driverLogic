"""
简单动作
"""

import asyncio

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

async def build_spear():
    assert ros_bridge_module.RosBridgeNodeInstance is not None, "RosBridgeNodeInstance is not initialized yet!"
    build_spear_finish.value = b''
    build_spear_active.value = True
    msg = String()
    msg.data = "spear_build"
    ros_bridge_module.RosBridgeNodeInstance.publish_ros2('/update_exec_req', msg)
    loop = asyncio.get_running_loop()
    deadline = loop.time() + 15.0

    while True:
        timeout = deadline - loop.time()
        if timeout <= 0.0:
            build_spear_active.value = False
            raise TimeoutError("build_spear 等待下位机成功回传超时")
        try:
            result = await asyncio.wait_for(build_spear_finish, timeout=timeout)
        except asyncio.TimeoutError as exc:
            build_spear_active.value = False
            raise TimeoutError("build_spear 等待下位机成功回传超时") from exc
        if result == BUILD_SPEAR_ACTION_TYPE:
            build_spear_active.value = False
            return


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
