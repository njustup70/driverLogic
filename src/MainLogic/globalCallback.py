'''
全局回调函数串口接收回调和ros2话题回调
'''
import asyncio
import struct
from MainLogic.app.actions import (
    BUILD_SPEAR_ACTION_TYPE,
    QRRecogInstance,
    build_spear_active,
    build_spear_finish,
    order_spear,
)
from MainLogic.app.climb_manager import ClimbManagerInstance
from MainLogic.app.merlin_map_solver_debug import run_solver_on_states
from MainLogic.core.tf_manager import TFManagerInstance
from MainLogic.core.ros_bridge_node import RosBridgeNodeInstance
from typing import List
from MainLogic.app.zone2_model_api import generate_actions_from_result, determine_start_position, encode_action_sequence, send_actions, send_r1_nodes, extract_r1_nodes_on_path,send_actions_one_by_one, schedule_repeated_send, stop_repeated_send
from MainLogic.core.zone2_model.zone2_sender import action_ack_event
_MEILIN_MAP_FRAME_LEN = 12
SLAMRESET = b'\x52'  # SLAM correct 指令帧

def action_callback(data: bytes):
    """动作执行完成回调函数，收到 FF 6F 帧时触发 ack_event 通知下一帧发送"""
    if not data:
        return
    if data[0:2] == b'\xFF\x6F':
        print(f"动作执行完成回调函数收到数据: {data.hex()}")
        action_ack_event.set()

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

    try:
        result = run_solver_on_states(meilin_states, render_map=True)
        print("梅林地图编码帧已解析并调用新入口完成求解")
        actions = generate_actions_from_result(result)
        print(f"生成动作序列: {actions}")
        # start_pos = determine_start_position(actions, approach_distance=500)
        # print(f" {start_pos}")
        encode_action_sequence(actions)
        R1 = extract_r1_nodes_on_path(result)
        # 使用定时重复发送：每隔 1 秒发送一次 actions 和 R1 节点帧，直到新的回调触发或被取消
        schedule_repeated_send(actions, R1, interval=5.0)
        return True
    except Exception as e:
        print(f"梅林地图编码帧处理错误: {e}")
        return False

'''
全局回调函数串口接收回调和ros2话题回调
'''
import math
import struct
from MainLogic.app.actions import (
    BUILD_SPEAR_ACTION_TYPE,
    QRRecogInstance,
    build_spear_active,
    build_spear_finish,
    order_spear,
)
from MainLogic.app.climb_manager import ClimbManagerInstance
from MainLogic.core import ros_bridge_node as ros_bridge_module
from MainLogic.core.tf_manager import TFManagerInstance, TFOdinInstance
from MainLogic.Lib.bytes import turn_to_bytes
from std_msgs.msg import Empty, Float32, String


SPEAR_OFFSET_COMMAND = b'\xB1'
YOLO_CLASSNAME_COMMAND = b'\xEA'
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
            # print(f"ODOM数据解析成功: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")
        except Exception as e:
            print(f"ODOM解析错误: {e}")
        return


def sick_callback(data: bytes): # 0xAA
    """下位机串口数据帧回调（新协议：无帧头、无功能码）。"""
    # sick数据帧：4个float加头3位，尾1位，共20字节
    # print(f"回调函数收到串口数据:{data.hex()}")
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
            TFManagerInstance.left_sick(float(left_distance))
            # TFOdinInstance.sick(float(left_distance))
            ros_bridge_module.RosBridgeNodeInstance.publish_ros2(
                SICK_LEFT_DISTANCE_TOPIC,
                Float32(data=float(left_distance))
            )
            # print(f"SICK数据解析成功: distance={left_distance:.3f} m")

            right_distance = sick_floats[1]
            TFManagerInstance.right_sick(float(right_distance))
            # TFOdinInstance.sick_right(float(right_distance))
            ros_bridge_module.RosBridgeNodeInstance.publish_ros2(
                SICK_RIGHT_DISTANCE_TOPIC,
                Float32(data=float(right_distance))
            )
            # print(f"SICK右侧数据解析成功: distance={right_distance:.3f} m")
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

# 场地/区域回调的去重缓存（看门狗双去重：回调层 + 看门狗层）
_last_field_color_value = None
_last_zone_retry_value = None

def field_color_callback(data: bytes):  # 0x78
    """
    红蓝场决定指令回调
    帧格式：0xFF 0x78 [场地决定帧] 0xFF (4 字节)
    场地决定帧：0x01 = 蓝场，0x00 = 红场
    """
    global _last_field_color_value
    if not data or len(data) != 2:
        return
    if data[1] != 0xFF:
        return

    field = data[0]
    if field not in (0x00, 0x01):
        print(f"场地决定：未知场地码 0x{field:02X}")
        return
    # 回调层去重：数据内容没变则不重复设 flag
    if field == _last_field_color_value:
        return
    _last_field_color_value = field
    if field == 0x01:
        TFManagerInstance.field_color_flag = 1
        TFManagerInstance.sick_direction_flag = 1
        print("场地决定：蓝场")
    elif field == 0x00:
        TFManagerInstance.field_color_flag = 0
        TFManagerInstance.sick_direction_flag = 0
        print("场地决定：红场")


def zone_retry_callback(data: bytes):  # 0x69
    """
    一三区场地重试指令回调
    帧格式：0xFF 0x69 [重试地决定帧] 0xFF (4 字节)
    重试地决定帧：0x01 = 一区重试，0x03 = 三区重试
    注意：R1 与 R2 即使是一三区重启，参数设置也不一样，但可共用一个回调。
    """
    global _last_zone_retry_value
    if not data or len(data) != 2:
        return
    if data[1] != 0xFF:
        return

    zone = data[0]
    if zone not in (0x01, 0x03):
        print(f"场地重试：未知重试区码 0x{zone:02X}")
        return
    # 回调层去重：数据内容没变则不重复设 flag
    if zone == _last_zone_retry_value:
        return
    _last_zone_retry_value = zone
    if zone == 0x01:
        TFManagerInstance.zone_retry_flag = 1
        print("场地重试：一区重试")
    elif zone == 0x03:
        TFManagerInstance.zone_retry_flag = 3
        print("场地重试：三区重试")


def slam_reset_callback(msg: Empty):
    """SLAM Reset 话题回调：监听 /slam_reset (std_msgs/Empty)，当 VoxelSLAM 内部触发 system_reset 时被调用。"""
    print("⚠️  SLAM Reset 话题已触发！VoxelSLAM 内部执行了 system_reset")
    bridge = ros_bridge_module.RosBridgeNodeInstance
    if bridge is None:
        return

    bridge.writeBytes(SLAMRESET)# 发送指令，让下位机知道slam有问题需要重试


def slam_restart_callback(data: bytes):  # 0x13
    """
    SLAM 重启指令回调
    帧格式：0xFF 0x13 0x13 0xFF (4 字节)
    此指令用于重试时由下位机按键触发，重启 SLAM 容器。
    """
    if not data or len(data) != 2:
        return
    if data[0] != 0x13 or data[1] != 0xFF:
        return
    print("SLAM 重启指令已触发，重启 SLAM 容器")
    import subprocess
    try:
        # -t 3: 仅控制关停旧容器的超时(SIGTERM 3s → SIGKILL)，不影响启动过程
        subprocess.run(
            ["docker", "restart", "-t", "3", "voxel_slam_ros2_runtime"],
            check=True,
            timeout=30,
        )
        print("✓ voxel_slam_ros2_runtime 容器重启成功")
    except subprocess.CalledProcessError as e:
        print(f"✗ voxel_slam_ros2_runtime 容器重启失败 (exit={e.returncode}): {e.stderr}")
    except subprocess.TimeoutExpired:
        print("✗ voxel_slam_ros2_runtime 容器重启超时（超过 30 秒），请检查容器状态")
    except FileNotFoundError:
        print("✗ docker CLI 未找到，请确认容器内已安装 docker")
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

def serial_action_return_callback(data: bytes):
    if not build_spear_active.value:
        return
    if len(data) < 4:
        return
    if data[0:2] == b'\xFF\xFF':  # 后面根据帧头改
        return_statu = data[3:4]
        if return_statu == BUILD_SPEAR_ACTION_TYPE:
            build_spear_finish.value = return_statu

def climb_type_callback(data: bytes):
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


def spear_offset_callback(msg):
    if not build_spear_active.value:
        return

    left_mm = float(msg.point.x)
    up_mm = float(msg.point.y)
    if not math.isfinite(left_mm) or not math.isfinite(up_mm):
        return

    bridge = ros_bridge_module.RosBridgeNodeInstance
    if bridge is None:
        return

    bridge.writeBytes(SPEAR_OFFSET_COMMAND + turn_to_bytes([left_mm, up_mm]))


    
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
    

def yolo_classname_callback(msg: String):
    """YOLO 检测类别名回调：根据 class_name 向下位机发送警示/正常帧。

    帧格式：0xFA 0xEA [data_0] [data_1]（0xFA 由 writeBytes 自动添加）
      - class_name 为 R_R1 或 B_R1 → 0x01 0x01（警示）
      - 其他 → 0x00 0x00（正常）
    """
    class_name = msg.data
    if not class_name:
        return

    bridge = ros_bridge_module.RosBridgeNodeInstance
    if bridge is None:
        return

    warn = 0x01 if class_name in ("R_R1", "B_R1") else 0x00
    bridge.writeBytes(YOLO_CLASSNAME_COMMAND + turn_to_bytes([warn, warn]))
    print(f"[YOLO] class_name={class_name}")
