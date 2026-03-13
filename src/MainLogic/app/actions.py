"""
简单动作
"""

import asyncio
import Lib.rosBridgeNode as ros_bridge_module
from Lib.AsyncTools import async_property
from std_msgs.msg import String

serial_action_finish = async_property(bytes)


async def check_finish(action_type , timeout = 5):
    start_time = asyncio.get_event_loop().time()
    while serial_action_finish != action_type:
        if asyncio.get_event_loop().time() - start_time > timeout:
            raise TimeoutError(f"等待动作完成超时: {action_type}")
        await asyncio.sleep(0.01)  # 每100ms检查一次状态

# 获取矛头
def take_spear_head():
    assert ros_bridge_module.RosBridgeNodeInstance is not None, "RosBridgeNodeInstance is not initialized yet!"
    action_type = b'\xA2'
    ros_bridge_module.RosBridgeNodeInstance.writeBytes(action_type)  # 发送取矛头指令
    check_finish()


def take_spear_head_off(): 
    msg = "spear_build_off"
    ros_bridge_module.RosBridgeNodeInstance.publish_ros2('/update_exec_req', msg)

# 矛头对接
def build_spear():
    msg = "spear_build"
    ros_bridge_module.RosBridgeNodeInstance.publish_ros2('/update_exec_req', msg)

def build_spear_off():
    msg = "spear_build_off"
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