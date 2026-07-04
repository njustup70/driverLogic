"""R1 二区路径帧编码器 (0xBA 协议)

帧格式（不含 0xFA 上位机帧头）：
    [0xBA] [序列总数 n] [单步动作1 (2 bytes)] [单步动作2 (2 bytes)] ...

单步动作 2 字节位布局：
    Byte1:
        Bit[0-3]  动作序列号 (1~n)
        Bit[4-5]  Yaw: 00=0.0  01=1.57(右)  10=-1.57(左)  11=3.14(上)
        Bit[6]    is_pick: 1=抓取, 0=不抓取
        Bit[7]    is_end:  1=终点, 0=非终点
    Byte2:
        Bit[0-2]  保留位, 填0
        Bit[3-7]  labels: 节点标签 (过道编号 0~17)
"""
from __future__ import annotations

from typing import List, Dict

# === 帧定义 ===
ZONE2_FRAME_HEADER = 0xBA

def yaw_to_code(yaw):
    """协议 2bit 朝向码
    0.0   → 0b00 (下)
    1.57  → 0b01 (右)
    -1.57 → 0b10 (左)
    3.14  → 0b11 (上)
    """
    if abs(yaw - 0.0) < 0.1:
        return 0b00
    if abs(yaw - 1.57) < 0.1:
        return 0b01
    if abs(yaw + 1.57) < 0.1:
        return 0b10
    if abs(yaw - 3.14) < 0.1:
        return 0b11
    return 0b00

def _encode_single_action(
    seq: int,
    aisle_id: int,
    yaw: float,
    is_pick: bool,
    is_end: bool,
):
    """将单个路径点编码为 2 字节动作。
    Byte1: [is_end(1b)] [is_pick(1b)] [yaw(2b)] [seq(4b)]
    Byte2: [labels(5b)] [reserved(3b)]
    """
    if not (1 <= seq <= 15):
        raise ValueError(f"序列号超出范围: {seq} (有效 1~15)")
    if not (0 <= aisle_id <= 17):
        raise ValueError(f"过道编号超出范围: {aisle_id} (有效 0~17)")

    yaw_bits = yaw_to_code(yaw)

    byte1 = (
        ((1 if is_end else 0) << 7)
        | ((1 if is_pick else 0) << 6)
        | ((yaw_bits & 0b11) << 4)
        | (seq & 0b1111)
    )

    byte2 = ((aisle_id & 0b11111) << 3) | 0b000  # labels[3-7], reserved[0-2]=0

    return bytes([byte1, byte2])


def encode_zone2_frame(filtered_nodes):
    """将 filtered_nodes 编码为 0xBA 完整路径帧。

    Args:
        filtered_nodes: pathai 输出的 filtered_nodes 列表，
                        每个元素包含 id, yaw, is_pick, is_at_point, target_block 等

    Returns:
        完整的 0xBA 帧字节流（不含 0xFA 上位机帧头）
    """
    if not filtered_nodes:
        return b''

    n = len(filtered_nodes)
    if n > 15:
        raise ValueError(f"路径节点数 {n} 超过单帧上限 15")

    payload = bytearray()
    payload.append(n)  # 序列总数

    for i, step in enumerate(filtered_nodes):
        seq = i + 1
        aisle_id = int(step['id'])
        yaw = float(step['yaw'])
        is_pick = bool(step.get('is_pick', False))
        is_end = (i == n - 1)

        action_bytes = _encode_single_action(seq, aisle_id, yaw, is_pick, is_end)
        payload.extend(action_bytes)

    return bytes([ZONE2_FRAME_HEADER]) + bytes(payload)

