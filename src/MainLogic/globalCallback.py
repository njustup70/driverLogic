'''
全局回调函数串口接收回调和ros2话题回调
'''
import struct
from functools import partial
from MainLogic.Lib.AsyncTools import AsyncVariable
from MainLogic.app.actions import order_spear, QRRecogInstance
from MainLogic.app.climb_manager import ClimbManagerInstance
from MainLogic.core.tf_manager import TFManagerInstance
from MainLogic.core import ros_bridge_node as ros_bridge_module

def mcu_transmit_callback(data: bytes):
    """下位机串口回调：单帧输入模式，完成 odom/sick 的检测与解包，sick纠正指令的回调"""
    # odom数据帧：
    _ODOM_FRAME_PREFIX = b'\xFF\xAA'
    _ODOM_FRAME_LEN = 14
    # sick数据帧：
    _SICK_FRAME_LEN = 20
    # slam correct纠正帧：
    _CORRECT_FRAME_PREFIX = b'\xFF\xB2'
    _CORRECT_FRAME_LEN = 4
    
    if not data:
        return

    odom_valid = len(data) == _ODOM_FRAME_LEN and data[:2] == _ODOM_FRAME_PREFIX

    sick_valid = False
    if len(data) == _SICK_FRAME_LEN:
        sick_header = data[0]
        sick_tail = data[19]
        sick_valid = sick_header == sick_tail and ((sum(data[1:19]) & 0xFF) == sick_tail)

    correct_valid = False
    if len(data) == _CORRECT_FRAME_LEN:
        # 帧格式：FF B2 [checksum] FF
        # checksum = 0xB2 (frame type)
        correct_header = data[:2] == _CORRECT_FRAME_PREFIX
        correct_checksum = data[2] == 0xB2
        correct_tail = data[3] == 0xFF
        correct_valid = correct_header and correct_checksum and correct_tail

    # 检查帧类型互斥性
    frame_count = sum([odom_valid, sick_valid, correct_valid])
    if frame_count == 0:
        return
    
    if frame_count > 1:
        print("帧类型歧义，丢弃该帧")
        return

    if odom_valid:
        try:
            odom_payload = data[2:14]
            x, y, yaw = struct.unpack('<fff', odom_payload)
            TFManagerInstance.odom(float(x), float(y), float(yaw))
            # print(f"ODOM数据解析成功: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")
        except Exception as e:
            print(f"ODOM解析错误: {e}")
        return

    if correct_valid:
        serial_correct_callback(data)
        return

    try:
        sick_payload = data[3:19]
        sick_floats = struct.unpack('<4f', sick_payload)
        distance = 1.0667 * sick_floats[0] - 0.0533
        TFManagerInstance.sick(float(distance))
        # print(f"SICK数据解析成功: distance={distance:.3f} m")
    except Exception as e:
        print(f"SICK解析错误: {e}")


def serial_correct_callback(data: bytes):
    """
    correct纠正指令核心处理函数
    帧格式：FF B2 [checksum=0xB2] FF (4 字节)
    """
    try:
        result = TFManagerInstance.apply_sick_initial_yaw_correction()
        if result:
            print("✓ SLAM correct 纠正指令已触发，SICK yaw 纠正成功")
        else:
            print("✗ SLAM correct 纠正指令触发失败：SICK 缓存为空或纠正失败")
        return result
    except Exception as e:
        print(f"✗ SLAM correct 纠正指令处理错误: {e}")
        return False


# def example_serial_callback(data: bytes):
#     #示例函数
#     #检查第一位 非常重要
#     if data[0] != 0xAA:
#         #print(f"Received serial data: {data}")
#         pass
# def serial_action_return_callback(data: bytes):
#     if data[0:2] == b'\xFF\xFF': # 后面根据帧头改
#         return_statu = data[3:4]
#         #print(f"回调函数收到串口数据，状态码:{data.hex()}")
#         serial_action_finish.value = return_statu
def climb_type_callback(data: bytes):
    """
    
    """
    # print(f"回调函数收到串口数据:{data.hex()}")
    if data[0:1] == b'\xFF' and data[1:2] == b'\xB1':
        # print(f"回调函数收到串口数据:{data.hex()}")
        try:
            if len(data) != 4:
                print("数据长度不足，无法解析爬墙类型和臂膀数据")
                return
            # Assign into AsyncVariable from ROS callback thread safely by scheduling
            # the assignment onto the asyncio event loop used by RosBridgeNodeInstance.
            ClimbManagerInstance.climb_type.value = data[2]
            ClimbManagerInstance.climb_arm.value = data[3]
            ClimbManagerInstance.climb_type.value = ClimbManagerInstance.climb_type.value
            ClimbManagerInstance.climb_arm.value = ClimbManagerInstance.climb_arm.value
            # print("爬墙类型和臂膀数据解析成功: 爬墙类型={}, 臂膀={}".format(ClimbManagerInstance.climb_type, ClimbManagerInstance.climb_arm))
        except Exception as e:
            print(f"解析爬墙数据错误: {e}")
        



def spear_callback(msg):
    order_spear.value = msg.data


    
STATUS_MAP = {"空": "00", "R1": "01", "R2": "10", "假": "11"}
REVERSE_MAP = {v: k for k, v in STATUS_MAP.items()}
def ros_qr_callback(msg):
    hex_str = msg.data

    if not hex_str or len(hex_str) != 8:
        return 
    try:
        binary = bin(int(hex_str, 16))[2:].zfill(32)
        state_bits = binary[:24]
        
        states = []
        for i in range(0, 24, 2):
            bits = state_bits[i:i+2]
            states.append(REVERSE_MAP.get(bits, "未知"))
        
        QRRecogInstance.recog_qr_result.value = ", ".join(states)
        return
    except:
        return 