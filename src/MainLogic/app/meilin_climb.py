"""
简单动作
"""

from MainLogic.Lib import rosBridgeNode as ros_bridge_module
from MainLogic.Lib.AsyncTools import async_property
from MainLogic.Lib.bytes import turn_to_bytes
from MainLogic.Lib.CheckActions import check_finish


serial_action_finish = async_property(bytes)



    

# 获取矛头
async def take_spear_head():
    assert ros_bridge_module.RosBridgeNodeInstance is not None, "RosBridgeNodeInstance is not initialized yet!"
    action_type = b'\xA2'
    ros_bridge_module.RosBridgeNodeInstance.writeBytes(action_type)  # 发送取矛头指令
    await check_finish()


# 矛头对接
order_spear = async_property(float)
async def build_spear():
    assert ros_bridge_module.RosBridgeNodeInstance is not None, "RosBridgeNodeInstance is not initialized yet!"
    action_type = b'\xA3'
    msg = "spear_build"
    ros_bridge_module.RosBridgeNodeInstance.publish_ros2('/update_exec_req', msg)
    while order_spear != [0, 1, 2]: # 等待order_spear更新到正确的状态
        ros_bridge_module.RosBridgeNodeInstance.writeBytes(action_type + turn_to_bytes(order_spear))  # 发送对接指令，附带矛的状态
    await check_finish()
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