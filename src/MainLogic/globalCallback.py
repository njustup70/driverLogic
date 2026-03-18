'''
全局回调函数串口接收回调和ros2话题回调
'''
import struct
from app.actions import order_spear, QRRecogInstance
from Lib.CheckActions import serial_action_finish
from app.climb_manager import ClimbManagerInstance


def sick_serial_callback(data: bytes):
    """解析SICK串口帧并发布 /sick_data"""
    if not data:
        return
    frame_len = 20
    if not hasattr(sick_serial_callback, "_buffer"):
        sick_serial_callback._buffer = bytearray()

    buffer = sick_serial_callback._buffer
    buffer.extend(data)

    if len(buffer) > 1024:
        del buffer[:-frame_len]

    while len(buffer) >= frame_len:
        frame = bytes(buffer[:frame_len])

        header = frame[0]
        tail = frame[19]
        calc_sum = sum(frame[1:19]) & 0xFF
        if header != tail or calc_sum != tail:
            buffer.pop(0)
            continue

        payload = frame[3:19]
        try:
            floats = struct.unpack('<4f', payload)
            distance = 1.0667 * floats[0] - 0.0533
            publish_fn = getattr(sick_serial_callback, '_publish', None)
            if callable(publish_fn):
                publish_fn(float(distance))
        except Exception as e:
            print(f"SICK解析错误: {e}")
        del buffer[:frame_len]


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