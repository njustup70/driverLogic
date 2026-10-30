'''
全局回调函数串口接收回调和ros2话题回调
'''
import asyncio
import struct
from MainLogic.core.tf_manager import TFManagerInstance
from MainLogic.core.ros_bridge_node import RosBridgeNodeInstance
from typing import List
_MEILIN_MAP_FRAME_LEN = 12

def action_callback(data: bytes):
    """动作执行完成回调函数，收到 FF 6F 帧时触发 ack_event 通知下一帧发送"""
    if not data:
        return
    if data[0:2] == b'\xFF\x6F':
        print(f"动作执行完成回调函数收到数据: {data.hex()}")

def _decode_meilin_map_states(data: bytes) -> List[str]:
    """解析 14 字节梅林地图编码帧，返回 12 个桩位状态。
    
    协议格式：[0xFF] [0xA2] [KFS_1] [KFS_2] ... [KFS_12] (共 14 字节)
    """      
    code_to_name = {
        0: "EMPTY",  # 空
        1: "R1",     # R1
        2: "R2",     # R2
        3: "FAKE",   # 假块 (对应你说的假块)
    }
    states = []
    for stake_idx, byte_value in enumerate(data[0:], start=1):
        if byte_value not in code_to_name:
            raise ValueError(f"KFS_{stake_idx} 包含未知的状态代码: {byte_value}")        
        states.append(code_to_name[byte_value])

        
    return states


def meilin_map_frame_callback(data: bytes):
    """梅林地图编码帧回调：解析 14 字节 FF A2 KFS_1..KFS_12 帧并触发重算。"""

    print(f"{data.hex()}")

    if not data:
        return False

    meilin_map_valid = len(data) == _MEILIN_MAP_FRAME_LEN
    if not meilin_map_valid:
        return False
    
    print(f"{data.hex()}")

    try:
        meilin_states = _decode_meilin_map_states(data)
    except Exception as e:
        print(f"梅林地图编码帧解析错误: {e}")
        return False

'''
全局回调函数串口接收回调和ros2话题回调
'''
from MainLogic.core import ros_bridge_node as ros_bridge_module
from MainLogic.core.tf_manager import TFManagerInstance, TFOdinInstance
from MainLogic.Lib.bytes import turn_to_bytes
from std_msgs.msg import Float32


SPEAR_OFFSET_COMMAND = b'\xB1'
SICK_LEFT_DISTANCE_TOPIC = '/state/sick_left_distance'
SICK_RIGHT_DISTANCE_TOPIC = '/state/sick_right_distance'

def mcu_transmit_callback(data: bytes):
    """下位机串口回调：单帧输入模式，完成 odom/sick 的检测与解包，sick纠正指令的回调"""
    # odom数据帧：
    _ODOM_FRAME_LEN = 12
    
    if not data:
        return

    if len(data) == _ODOM_FRAME_LEN:
        try:
            x, y, yaw = struct.unpack('<fff', data)
            TFManagerInstance.odom(float(x), float(y), float(yaw))
            #if have nan value
            if any(map(lambda v: not isinstance(v, float) or v != v, [x, y, yaw])):
                print(f"ODOM数据包含无效值: x={x}, y={y}, yaw={yaw}")
            print(f"ODOM数据解析成功: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")
        except Exception as e:
            print(f"ODOM解析错误: {e}")
        return


def sick_callback(data: bytes): # 0xAA
    """下位机串口数据帧回调（新协议：无帧头、无功能码）。"""
    # sick数据帧：4个float加头3位，尾1位，共20字节
    _SICK_FRAME_LEN = 20
    if not data:
        return
    
    if len(data) == _SICK_FRAME_LEN:
        sick_header = data[0]
        sick_tail = data[19]
        sick_valid = sick_header == sick_tail and ((sum(data[1:19]) & 0xFF) == sick_tail)
        if not sick_valid:
            print(f"SICK数据校验失败")
            return
      
        sick_data = data[3:19]
        try:
            sick_floats = struct.unpack('<4f', sick_data)
            left_distance = sick_floats[0]
            # print(id(TFManagerInstance), id(TFOdinInstance))
            TFManagerInstance.sick(float(left_distance))
            # TFOdinInstance.sick(float(left_distance))
            ros_bridge_module.RosBridgeNodeInstance.publish_ros2(
                SICK_LEFT_DISTANCE_TOPIC,
                Float32(data=float(left_distance))
            )
            print(f"SICK数据解析成功: distance={left_distance:.3f} m")

            right_distance = sick_floats[1]
            TFManagerInstance.sick_right(float(right_distance))
            # TFOdinInstance.sick_right(float(right_distance))
            ros_bridge_module.RosBridgeNodeInstance.publish_ros2(
                SICK_RIGHT_DISTANCE_TOPIC,
                Float32(data=float(right_distance))
            )
            print(f"SICK右侧数据解析成功: distance={right_distance:.3f} m")
        except Exception as e:
            print(f"SICK解析错误: {e}")

def serial_correct_callback(data: bytes): # 0xB2
    """
    correct纠正指令核心处理函数
    帧格式：FF B2 [checksum=0xB2] FF (4 字节)
    """
    # 检查数据长度和格式
    if len(data) < 2:
        # print(f"✗ SLAM correct 指令格式错误：数据长度不足，期望≥2，实际{len(data)}")
        return False
    # 检查脱头后的前两个字节：[0xB2, 0xFF]
    if data[0] != 0xB2 or data[1] != 0xFF:
        # print(f"✗ SLAM correct 指令格式错误：期望[0xB2, 0xFF]，实际[{data[0]:02x}, {data[1]:02x}]")
        return False
    try:
        result = TFManagerInstance.apply_sick_initial_yaw_correction()
        result = TFOdinInstance.apply_sick_initial_yaw_correction()
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
#     if data[0:2] == b'\xFF\xFF':
#         return_statu = data[3:4]
#         serial_action_finish.value = return_statu

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
        return
    except:
        return 
