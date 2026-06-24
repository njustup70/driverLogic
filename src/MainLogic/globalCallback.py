'''
全局回调函数串口接收回调和ros2话题回调
'''
import struct
from MainLogic.app.actions import order_spear, QRRecogInstance
from MainLogic.app.climb_manager import ClimbManagerInstance
from MainLogic.app.merlin_map_solver_debug import run_solver_on_states
from MainLogic.core.tf_manager import TFManagerInstance
from typing import List
from MainLogic.core.zone2_model.zone2_sender import generate_actions_from_result,determine_start_position
_MEILIN_MAP_FRAME_PREFIX = b'\xff\x0d\xa2'
_MEILIN_MAP_FRAME_LEN = 15


def _decode_meilin_map_states(data: bytes) -> List[str]:
    """解析 14 字节梅林地图编码帧，返回 12 个桩位状态。
    
    协议格式：[0xFF] [0xA2] [KFS_1] [KFS_2] ... [KFS_12] (共 14 字节)
    """
    if len(data) != _MEILIN_MAP_FRAME_LEN:
        raise ValueError(f"梅林地图编码帧长度错误: 期望 {_MEILIN_MAP_FRAME_LEN}，实际 {len(data)}")       
    if data[:3] != _MEILIN_MAP_FRAME_PREFIX:
        raise ValueError(f"梅林地图编码帧头错误: 期望 {_MEILIN_MAP_FRAME_PREFIX.hex()}，实际 {data[:3].hex()}")
    code_to_name = {
        0: "EMPTY",  # 空
        1: "R1",     # R1
        2: "R2",     # R2
        3: "FAKE",   # 假块 (对应你说的假块)
    }
    states = []
    for stake_idx, byte_value in enumerate(data[3:], start=1):
        if byte_value not in code_to_name:
            raise ValueError(f"KFS_{stake_idx} 包含未知的状态代码: {byte_value}")        
        states.append(code_to_name[byte_value])

    if len(states) != 12:
        raise ValueError(f"梅林地图编码帧解析结果长度错误: {len(states)}")
        
    return states


def meilin_map_frame_callback(data: bytes):
    """梅林地图编码帧回调：解析 14 字节 FF A2 KFS_1..KFS_12 帧并触发重算。"""

    print(f"{data.hex()}")

    if not data:
        return False

    meilin_map_valid = len(data) == _MEILIN_MAP_FRAME_LEN and data[:3] == _MEILIN_MAP_FRAME_PREFIX
    if not meilin_map_valid:
        return False
    
    print(f"{data.hex()}")

    try:
        meilin_states = _decode_meilin_map_states(data)
    except Exception as e:
        print(f"梅林地图编码帧解析错误: {e}")
        return False

    try:
        result = run_solver_on_states(meilin_states, render_map=True)
        print("梅林地图编码帧已解析并调用新入口完成求解")
        actions = generate_actions_from_result(result)
        print(f"生成动作序列: {actions}")
        start_pos = determine_start_position(actions, approach_distance=500)
        print(f"起始坐标: x={start_pos[0]:.1f}, y={start_pos[1]:.1f}")
        return True
    except Exception as e:
        print(f"梅林地图编码帧处理错误: {e}")
        return False

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
    # 梅林状态帧与重算触发帧：
    _MEILIN_STATE_FRAME_PREFIX = b'\xFF\xB3'
    _MEILIN_STATE_FRAME_LEN = 7
    
    _MEILIN_CMD_FRAME_PREFIX = b'\xFF\xB4'
    _MEILIN_CMD_FRAME_LEN = 5
    _MEILIN_STATE_CODE_TO_NAME = {
        0: "EMPTY",
        1: "R2",
        2: "R1",
        3: "FAKE",
    }
    
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

    meilin_state_valid = False
    if len(data) == _MEILIN_STATE_FRAME_LEN:
        meilin_state_header = data[:2] == _MEILIN_STATE_FRAME_PREFIX
        meilin_state_checksum = ((0xB3 + data[2] + data[3] + data[4]) & 0xFF) == data[5]
        meilin_state_tail = data[6] == 0xFF
        meilin_state_valid = meilin_state_header and meilin_state_checksum and meilin_state_tail

    meilin_cmd_valid = False
    if len(data) == _MEILIN_CMD_FRAME_LEN:
        meilin_cmd_header = data[:2] == _MEILIN_CMD_FRAME_PREFIX
        meilin_cmd_byte = data[2] == 0x01
        meilin_cmd_checksum = ((0xB4 + data[2]) & 0xFF) == data[3]
        meilin_cmd_tail = data[4] == 0xFF
        meilin_cmd_valid = meilin_cmd_header and meilin_cmd_byte and meilin_cmd_checksum and meilin_cmd_tail

    meilin_map_valid = len(data) == _MEILIN_MAP_FRAME_LEN and data[:2] == _MEILIN_MAP_FRAME_PREFIX

    # 检查帧类型互斥性
    frame_count = sum([odom_valid, sick_valid, correct_valid, meilin_state_valid, meilin_cmd_valid, meilin_map_valid])
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
        print(f"SICK数据解析成功: distance={distance:.3f} m")
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
    解析串口接收的爬墙类型和臂膀数据（基于二进制编码）
    
    数据帧格式：FF B1 [数据1] [数据2] ...
    
    数据1 = 0x0F 时：
    - bit[0-3] = 四个标志（1/0）
    - bit[4-7] = 腿部数据的高位
    
    数据2 = 0x00 时：
    - bit[0-3] = 腿部数据的低位或直接腿部状态
    
    示例：FF B1 0F 00
    - 0x0F = 0b00001111 → 四个标志均为 1，腿部高位为 0
    - 0x00 = 0b00000000 → 腿部低位为 0，所以腿部状态均为 0
    """
    if data[0:2] == b'\xFF\xB1':
        print(f"回调函数收到串口数据:{data.hex()}")
        
        try:
            # ===== 解析 climb_type =====
            # 从 data[2] 提取爬墙类型的 4 个二进制位（bit[0-3]）
            if len(data) > 2:
                climb_type_byte = data[2]
                ClimbManagerInstance.climb_type.value = [
                    bool(climb_type_byte & (1 << 0)),  # 比特 0：标志 1
                    bool(climb_type_byte & (1 << 1)),  # 比特 1：标志 2
                    bool(climb_type_byte & (1 << 2)),  # 比特 2：标志 3
                    bool(climb_type_byte & (1 << 3)),  # 比特 3：标志 4
                ]
                print(f"爬墙类型: [标志1={ClimbManagerInstance.climb_type.value[0]}, "
                        f"标志2={ClimbManagerInstance.climb_type.value[1]}, "
                        f"标志3={ClimbManagerInstance.climb_type.value[2]}, "
                        f"标志4={ClimbManagerInstance.climb_type.value[3]}]")
            
            # ===== 解析 climb_arm =====
            # 从 data[2] 和 data[3] 提取臂膀数据
            # data[2] 的高 4 位（bit[4-7]）+ data[3] 的低 4 位（bit[0-3]）组成臂膀数据
            if len(data) > 3:
                # 方法 1：从 data[2] 的高 4 位和 data[3] 的低 4 位提取
                front_leg = (data[2] >> 4) & 0x03        # data[2] 的 bit[4-5]：前腿
                rear_leg = (data[2] >> 6) & 0x03         # data[2] 的 bit[6-7]：后腿
                
                # 或者腿部数据可能在 data[3]
                if front_leg == 0 and rear_leg == 0:
                    # 如果 data[2] 高位全 0，尝试从 data[3] 读取
                    front_leg = data[3] & 0x03           # data[3] 的 bit[0-1]：前腿
                    rear_leg = (data[3] >> 2) & 0x03     # data[3] 的 bit[2-3]：后腿
                
                ClimbManagerInstance.climb_arm.value = [front_leg, rear_leg]
                print(f"臂膀状态: 前腿={front_leg}, 后腿={rear_leg}")
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