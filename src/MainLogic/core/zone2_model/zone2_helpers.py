"""zone2_helpers 模块作用：

集中放置 zone2 领域里会被多个模块复用的基础工具：
- 转向/取块常量
- 节点编码函数
- derived 节点识别与解析函数
- heading 到转向动作的转换函数

这个模块不依赖 API 层，也不负责格式化、编码或发送。
"""
from __future__ import annotations

import re
from typing import Optional

# Turn and pick constants (kept in helpers for reuse)
TURN_ACTION_STRAIGHT = 0x00
TURN_ACTION_LEFT = 0x01
TURN_ACTION_RIGHT = 0x02
TURN_ACTION_UTURN = 0x03

def _turn_action_name(action: int) -> str:
    """把转向动作码转换为中文名字。"""
    return {
        TURN_ACTION_STRAIGHT: "直行",
        TURN_ACTION_LEFT: "左转",
        TURN_ACTION_RIGHT: "右转",
        TURN_ACTION_UTURN: "掉头",
    }.get(action, f"未知({action})")


def _turn_action_from_headings(prev_heading: Optional[str], next_heading: Optional[str]) -> int:
    """根据进入/离开朝向计算转向动作。"""
    if prev_heading is None or next_heading is None:
        return TURN_ACTION_STRAIGHT

    heading_vec = {
        "up": (0, 1),
        "right": (1, 0),
        "down": (0, -1),
        "left": (-1, 0),
    }
    a = heading_vec.get(str(prev_heading))
    b = heading_vec.get(str(next_heading))
    if a is None or b is None:
        return TURN_ACTION_STRAIGHT

    if a == b:
        return TURN_ACTION_STRAIGHT

    ax, ay = a
    bx, by = b
    cross = ax * by - ay * bx
    dot = ax * bx + ay * by
    if dot < 0:
        return TURN_ACTION_UTURN
    if cross > 0:
        return TURN_ACTION_LEFT
    if cross < 0:
        return TURN_ACTION_RIGHT
    return TURN_ACTION_STRAIGHT


def _is_derived_node(node: str) -> bool:
    """判断节点是否为 derived 节点。"""
    node_str = str(node)
    return node_str.startswith("D_") or ("to" in node_str)


def _is_real_node(node: str) -> bool:
    """判断节点是否为真实节点（非 derived，且不是 end）。"""
    node_str = str(node)
    return node_str != "end" and not _is_derived_node(node_str)


def _extract_pick_target_from_derived_node(node: str) -> int:
    """从 derived 节点名中提取物块编号。"""
    node_str = str(node)
    match = re.search(r"(\d+)(?!.*\d)", node_str)
    if match is None:
        return 0
    return int(match.group(1))


def _extract_owner_from_derived_node(node: str) -> str:
    """从 derived 节点中提取其 owner 节点。"""
    node_str = str(node)
    match = re.match(r"^D_(.+?)_to_\d+$", node_str)
    if match is not None:
        return match.group(1)

    match = re.match(r"^(\d+)to\d+$", node_str)
    if match is not None:
        return match.group(1)

    return node_str
