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

def mcu_transmit_callback(data: bytes): # 0xAA
    """下位机串口数据帧回调（新协议：无帧头、无功能码）。"""
    # odom数据帧：3个float，共12字节
    _ODOM_FRAME_LEN = 12
    # sick数据帧：4个float加头3位，尾1位，共20字节
    _SICK_FRAME_LEN = 20
    
    if not data:
        return

    if len(data) == _ODOM_FRAME_LEN:
        try:
            x, y, yaw = struct.unpack('<fff', data)
            TFManagerInstance.odom(float(x), float(y), float(yaw))
            # print(f"ODOM数据解析成功: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")
        except Exception as e:
            print(f"ODOM解析错误: {e}")
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
            distance = 1.0667 * sick_floats[0] - 0.0533
            TFManagerInstance.sick(float(distance))
            print(f"SICK数据解析成功: distance={distance:.3f} m")
        except Exception as e:
            print(f"SICK解析错误: {e}")

def serial_correct_callback(data: bytes): # 0xB2
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
def climb_type_callback(data: bytes): # 0xB1
    """
    
    """

    print(f"回调函数收到串口数据:{data.hex()}")
        
    try:
        # ===== 解析 climb_type =====
        if len(data) > 0:
            climb_type_byte = data[0]
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
        if len(data) > 1:
            front_leg = (data[0] >> 4) & 0x03     # data[0] 的 bit[4-5]：前腿
            rear_leg = (data[0] >> 6) & 0x03      # data[0] 的 bit[6-7]：后腿
            
            if front_leg == 0 and rear_leg == 0:
                front_leg = data[1] & 0x03        # data[1] 的 bit[0-1]：前腿
                rear_leg = (data[1] >> 2) & 0x03  # data[1] 的 bit[2-3]：后腿
            
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