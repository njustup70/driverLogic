"""
简单动作
"""

import asyncio
import Lib.rosBridgeNode as ros_bridge_module
from Lib.AsyncTools import async_property
from std_msgs.msg import String


# 获取矛头
class SpearHeadTake: # 动作信号及实例
    take_spearhead_ok = async_property(bool)
SpearHeadInstance = SpearHeadTake()
def take_spear_head():
    assert ros_bridge_module.RosBridgeNodeInstance is not None, "RosBridgeNodeInstance is not initialized yet!"
    ros_bridge_module.RosBridgeNodeInstance.writeBytes(b'\xA2')  # 发送取矛头指令

# 矛头对接
class SpearBuild:
    build_spear_ok = async_property(bool)
SpearBuildInstance = SpearBuild()
def build_spear():
    msg = "spear_build"
    ros_bridge_module.RosBridgeNodeInstance.publish_ros2('/update_exec_req', msg)

# QR识别
class QRRecog:
    recog_qr_result = async_property(str)
QRRecogInstance = QRRecog()
def QR_recog():
    msg = "qr_recog"
    ros_bridge_module.RosBridgeNodeInstance.publish_ros2('/update_exec_req', msg)