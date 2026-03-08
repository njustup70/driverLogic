"""
简单动作
"""

import asyncio
import Lib.rosBridgeNode as ros_bridge_module
from Lib.AsyncTools import async_property
from std_msgs.msg import String

serial_action_ok = async_property(bool)

# 获取矛头
def take_spear_head():
    assert ros_bridge_module.RosBridgeNodeInstance is not None, "RosBridgeNodeInstance is not initialized yet!"
    ros_bridge_module.RosBridgeNodeInstance.writeBytes(b'\xA2')  # 发送取矛头指令

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


# 小板位姿控制/结果
class SmallBoardPose:
    offset_mm = async_property(lambda: (0.0, 0.0))


SmallBoardPoseInstance = SmallBoardPose()


def start_small_board_pose():
    ros_bridge_module.RosBridgeNodeInstance.publish_ros2('/small_board_pose/command', 'spear')


def stop_small_board_pose():
    ros_bridge_module.RosBridgeNodeInstance.publish_ros2('/small_board_pose/command', 'stop')


def QR_recog_off():
    msg = "qr_recog_off"
    ros_bridge_module.RosBridgeNodeInstance.publish_ros2('/update_exec_req', msg)