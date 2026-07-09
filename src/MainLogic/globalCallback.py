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
from MainLogic.app.zone2_model_api import generate_actions_from_result, determine_start_position, encode_action_sequence, send_actions, send_r1_nodes, extract_r1_nodes_on_path,send_actions_one_by_one
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
        send_actions(actions)
        # asyncio.run_coroutine_threadsafe(send_actions_one_by_one(actions, timeout=10.0), RosBridgeNodeInstance._loop)
        R1 = extract_r1_nodes_on_path(result)
        send_r1_nodes(R1)
        return True
    except Exception as e:
        print(f"梅林地图编码帧处理错误: {e}")
        return False

'''
全局回调函数串口接收回调和ros2话题回调
'''
import math
import time
from threading import Thread
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
from MainLogic.core.tf_manager import TFManagerInstance
from MainLogic.Lib.bytes import turn_to_bytes
from std_msgs.msg import Empty, Float32, String


SPEAR_OFFSET_COMMAND = b'\xB1'
YOLO_CLASSNAME_COMMAND = b'\xB4'
SICK_LEFT_DISTANCE_TOPIC = '/state/sick_left_distance'
SICK_RIGHT_DISTANCE_TOPIC = '/state/sick_right_distance'

# === R1 二区 KFS 属性帧 (0xC2) ===
# 遥控器或下位机发送方块属性 → 触发路径规划 → 编码0xBA帧 → 串口下发
from MainLogic.core.R1_zone2 import compute_r1_zone2_path, compute_r2_entry_col, encode_zone2_frame

KFS_LABELS = {0: "空", 1: "R1", 2: "R2", 3: "假块"}
zone2_kfs_state: list[int] = [0] * 12

# === R1 ↔ zone2_model KFS 坐标转换 ===
# R1 与 zone2_model 的 KFS 编号体系不同，按红蓝半场做物理位置映射。
#   kfs_raw 字节顺序: 俯视从上到下、从左到右 (12 字节，下标 0-11)
#   红半场: zone2 stake 列序与 R1 相同 (恒等映射)
#   蓝半场: zone2 stake 列序与 R1 相反 (镜像)
_R1_TO_ZONE2_RED  = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
_R1_TO_ZONE2_BLUE = [2, 1, 0, 5, 4, 3, 8, 7, 6, 11, 10, 9]

_STATE_MAP = {0: "EMPTY", 1: "R1", 2: "R2", 3: "FAKE"}


def r1_kfs_to_zone2_states(kfs_raw: list[int], field_color_flag: int) -> list[str]:
    """R1 KFS 原始数据 → zone2_model 状态列表 (stake 1~12 顺序)。

    _R1_TO_ZONE2[z] = zone2 stake (z+1) 对应的 kfs_raw 下标。
    """
    mapping = _R1_TO_ZONE2_RED if field_color_flag == 0 else _R1_TO_ZONE2_BLUE
    return [_STATE_MAP[kfs_raw[mapping[z]]] for z in range(12)]


def zone2_stakes_to_r1_indices(stakes: list[int], field_color_flag: int) -> list[int]:
    """zone2 stake 编号 (1-based) → R1 kfs_raw 下标。

    查 _R1_TO_ZONE2[N-1] 即得 zone2 stake N 对应的 kfs_raw 下标。
    """
    mapping = _R1_TO_ZONE2_RED if field_color_flag == 0 else _R1_TO_ZONE2_BLUE
    return [mapping[n - 1] for n in stakes]


def kfs_callback(data: bytes):
    """解析 0xC2 KFS 属性帧 → 推算R2路径 → R1路径规划(基于R2优先级) → 下发R1帧。

    完整流水线：
      1. 解析12个KFS状态
      2. 调用 zone2_model 推算 R2 最优路径（仅用于优先级参考，不下发）
      3. 提取 R2 路径上的 R1 节点作为 R1 高优先级取块目标
      4. R1 根据 R2 优先级规划自身路径
      5. 下发 R1 0xBA 帧

    方块编号顺序（俯视图，一区面向机器人）：
        三区
        ---------------
        1   2   3
        4   5   6
        7   8   9
       10  11  12
        ---------------
        一区
    KFS 状态约定：0=空, 1=R1, 2=R2, 3=假块
    """
    global zone2_kfs_state

    print(f"[KFS] 原始帧数据 (hex): {data.hex()}")

    if len(data) < 12:
        print(f"[KFS] 帧长度不足: 期望12字节, 实际{len(data)}")
        return

    kfs_raw = list(data[:12])
    has_abnormal = False
    for i, v in enumerate(kfs_raw):
        if v not in (0, 1, 2, 3):
            print(f"[KFS] 方块{i+1}属性值异常: {v}, 终止路径规划")
            has_abnormal = True
    if has_abnormal:
        return

    zone2_kfs_state = kfs_raw

    r1_blocks = [i for i, v in enumerate(kfs_raw) if v == 1]
    r2_blocks = [i for i, v in enumerate(kfs_raw) if v == 2]
    fake_block = [i for i, v in enumerate(kfs_raw) if v == 3]
    print(f"[KFS] 二区属性更新: R1={r1_blocks}, R2={r2_blocks}, Fake={fake_block}")

    if len(fake_block) > 1:
        print("[Zone2] 假块数量超过1个，无法规划")
        return

    # ============================================================
    # Step 1: R2 路径推算 (zone2_model) — 确定 R1 优先级 + R2 实际经过桩位
    # ============================================================
    r1_priority_blocks: list[int] = []
    r2_traversal_kfs: list[int] = []

    if r2_blocks:
        try:
            fc = TFManagerInstance.field_color_flag
            meilin_states = r1_kfs_to_zone2_states(kfs_raw, fc)

            print("[KFS] → 推算 R2 路径用以确定 R1 优先级...")
            r2_result = run_solver_on_states(meilin_states, render_map=True)

            # 提取 R1 优先级：R2 路径上经过的 R1 块
            r1_nodes_on_r2_path = extract_r1_nodes_on_path(r2_result)
            r1_priority_blocks = [
                idx for idx in zone2_stakes_to_r1_indices(r1_nodes_on_r2_path, fc)
                if idx in r1_blocks
            ]
            print(f"[KFS] zone2 stake: {r1_nodes_on_r2_path} → R1优先(kfs下标): {r1_priority_blocks}")

            # 提取 R2 实际经过的桩位 (zone2 stake → kfs 下标)，传给 R1 planner 做避让
            r2_path_nodes = r2_result.get("path", [])
            r2_traversal_stakes = []
            for node in r2_path_nodes:
                try:
                    n = int(node)
                    if 1 <= n <= 12:
                        r2_traversal_stakes.append(n)
                except (ValueError, TypeError):
                    pass
            r2_traversal_kfs = zone2_stakes_to_r1_indices(r2_traversal_stakes, fc)
            print(f"[KFS] R2 实际经过桩位(zone2): {r2_traversal_stakes} → kfs下标: {r2_traversal_kfs}")

        except Exception as e:
            print(f"[KFS] R2路径推算异常，回退到无优先级模式: {e}")

    # ============================================================
    # Step 2: R1 路径规划
    #   传入 zone2_model 算出的 R2 实际路径 (r2_traversal_kfs)
    #   R1 优先取该路径上的 R1 块，再取其余 R1 块
    # ============================================================
    if not r1_blocks:
        print("[Zone2] 未检测到R1方块，跳过R1路径规划")
        return

    # 红蓝半场决定离场过道：红场→11，蓝场→7
    exit_node = 11 if TFManagerInstance.field_color_flag == 0 else 7

    if r2_traversal_kfs:
        auto_mode = 1
        print(f"[KFS] R1 优先级来源: zone2_model R2实际路径 → {r2_traversal_kfs}")
    elif r2_blocks:
        auto_mode = 1
        print(f"[KFS] R1 优先级来源: 格子级 R2 路线 (auto_dog_flag=1)")
    else:
        auto_mode = 0
        print(f"[KFS] 无 R2 块，R1 自由取块")

    result = compute_r1_zone2_path(
        r1_blocks=r1_blocks, r2_blocks=r2_blocks, fake_block=fake_block,
        auto_dog_flag=auto_mode,
        priority_block=r1_priority_blocks,
        r2_traversal=r2_traversal_kfs if r2_traversal_kfs else None,
        start_candidates=[2, 0, 16],
        exit_node=exit_node, verbose=True,
    )

    if not result['success']:
        print(f"[Zone2] R1路径规划失败: {result['error']}")
        return

    r2_entry_col = compute_r2_entry_col(r2_traversal_kfs, TFManagerInstance.field_color_flag)
    print(f"[KFS] R2 入口列编码: {r2_entry_col:02b} (00=中 01=左 10=右)")
    ba_frame = encode_zone2_frame(result['filtered_nodes'], r2_entry_col=r2_entry_col)

    # === 调试打印：动作序列 ===
    _print_action_sequence(ba_frame)

    BA_REPEAT_COUNT = 100       # 重复发送次数
    BA_REPEAT_INTERVAL = 0.1  # 发送间隔（秒）

    def _repeat_send(frame, count, interval):
        for i in range(count):
            ros_bridge_module.RosBridgeNodeInstance.writeBytes(frame)
            print(f"R1 BA帧已下发 ({i+1}/{count})")
            if i < count - 1:
                time.sleep(interval)

    Thread(target=_repeat_send, args=(ba_frame, BA_REPEAT_COUNT, BA_REPEAT_INTERVAL), daemon=True).start()

def _print_action_sequence(frame: bytes):
    """解码 0xBA 帧并打印可读的动作序列（调试用）。"""
    if len(frame) < 2:
        return

    n = frame[1]
    yaw_names = {0b00: "↓(0.0)", 0b01: "→(1.57)", 0b10: "←(-1.57)", 0b11: "↑(3.14)"}

    lines = [f"\n{'='*50}", f" 下发动作序列 ({n} 步)", f"{'='*50}"]
    for i in range(n):
        off = 2 + i * 2
        if off + 1 >= len(frame):
            break
        b1, b2 = frame[off], frame[off + 1]
        seq = b1 & 0b1111
        yaw = (b1 >> 4) & 0b11
        pick = (b1 >> 6) & 0b1
        end = (b1 >> 7) & 0b1
        aisle = (b2 >> 3) & 0b11111

        flags = []
        if pick: flags.append("抓取")
        if end:  flags.append("终点")
        if not flags: flags.append("经过")

        lines.append(
            f"  [{seq:2d}] 过道:{aisle:2d}  朝向:{yaw_names.get(yaw, '?')}  "
            f"{' | '.join(flags)}  "
            f"HEX: {b1:02X} {b2:02X}"
        )
    lines.append(f"{'='*50}\n")
    print('\n'.join(lines))


def mcu_transmit_callback(data: bytes):
    """下位机串口回调：单帧输入模式，完成 odom/sick 的检测与解包，sick纠正指令的回调"""
    _ODOM_FRAME_LEN = 12
    
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


def sick_callback(data: bytes): # 0xAA
    """下位机串口数据帧回调"""
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
            distance = 1.0613*sick_floats[0]-0.0407 # sick校正
            print(id(TFManagerInstance))
            TFManagerInstance.sick(float(distance))
           #print(f"SICK数据解析成功: distance={distance:.3f} m")
        except Exception as e:
            print(f"SICK解析错误: {e}")


def serial_correct_callback(data: bytes): # 0xB2
    """
    correct纠正指令核心处理函数
    帧格式：FF B2 [checksum=0xB2] FF (4 字节)
    """
    # 检查数据长度和格式
    if len(data) < 2:
        print(f"✗ SLAM correct 指令格式错误：数据长度不足，期望≥2，实际{len(data)}")
        return False
    # 检查脱头后的前两个字节：[0xB2, 0xFF]
    if data[0] != 0xB2 or data[1] != 0xFF:
        # print(f"✗ SLAM correct 指令格式错误：期望[0xB2, 0xFF]，实际[{data[0]:02x}, {data[1]:02x}]")
        return False
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

    bridge.writeBytes(SLAMRESET)# 发送 SLAM correct 指令，触发 SICK yaw 纠正


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
    爬墙类型回调函数
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
    """YOLO 检测类别名回调：接收 YOLO_detection 话题的 class_name，下发到下位机。

    仿照 debug_spear_offset_callback 的方式：
    - 从 String 消息中取出类别名
    - 拼装 payload: YOLO_CLASSNAME_COMMAND + class_name.encode('utf-8')
    - 通过 writeBytes 下发（0xFA 帧头由 writeBytes 自动添加）

    串口帧格式：0xFA 0xB4 [class_name UTF-8 bytes]
    """
    class_name = msg.data
    if not class_name:
        return

    bridge = ros_bridge_module.RosBridgeNodeInstance
    if bridge is None:
        return

    payload = YOLO_CLASSNAME_COMMAND + turn_to_bytes([ord(c) for c in class_name])
    frame = b'\xFA' + payload
    bridge.writeBytes(payload)
    print(f"[YOLO] 下发类别名: {class_name}, frame={frame.hex(' ')}")
