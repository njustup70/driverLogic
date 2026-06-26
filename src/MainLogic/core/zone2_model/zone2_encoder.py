"""zone2_encoder 模块作用：

负责把路径和动作记录编码成二进制帧，供串口或下位机直接消费。

本模块只做"数据编码"，不负责打印和发送；发送逻辑放在 `zone2_sender.py`。
"""
from __future__ import annotations

from MainLogic.core.zone2_model.zone2_helpers import (
    _encode_path_node,
    _is_derived_node,
    _extract_pick_target_from_derived_node,
    _turn_action_from_headings,
)
from MainLogic.core.zone2_model.zone2_format import extract_r1_nodes_on_path

PATH_NODE_FRAME_HEADER = 0xB5
MCU_ACTION_FRAME_HEADER = 0xB8

MCU_ACTION_TYPE_TURN = 0x01
MCU_ACTION_TYPE_PICK = 0x02
MCU_ACTION_TYPE_MOVE = 0x03
MCU_ACTION_TYPE_R1_LIST = 0x04


def _build_action_payload(result: dict, include_r1_list: bool = False) -> tuple[bytes, int]:
    """按 path_steps 直接生成动作序列负载。"""
    path_steps = list(result.get("path_steps", []))
    if not path_steps:
        return b"", 0

    payload = bytearray()
    action_count = 0

    for step in path_steps:
        u = step.get("from")
        v = step.get("to")
        edge_class = str(step.get("edge_class", ""))
        turn_action = _turn_action_from_headings(step.get("heading_in"), step.get("heading_out"))
        turn_cost = float(step.get("turn_cost", 0.0))
        pick_target = _extract_pick_target_from_derived_node(v) if edge_class == "to_derived" or _is_derived_node(v) else 0

        if turn_cost > 0.0 and turn_action != 0 and str(u) != "start":
            u_code = _encode_path_node(str(u))
            payload.append(MCU_ACTION_TYPE_TURN)
            payload.append(u_code)
            payload.append(turn_action)
            action_count += 1

        is_pick_step = edge_class == "to_derived" or _is_derived_node(v)

        if is_pick_step:
            u_code = _encode_path_node(str(u))
            payload.append(MCU_ACTION_TYPE_PICK)
            payload.append(u_code)
            payload.append(pick_target)
            action_count += 1
        else:
            u_code = _encode_path_node(str(u))
            v_code = _encode_path_node(str(v))
            payload.append(MCU_ACTION_TYPE_MOVE)
            payload.append(u_code)
            payload.append(v_code)
            action_count += 1

    if include_r1_list:
        r1_nodes = extract_r1_nodes_on_path(result)
        if r1_nodes:
            if len(r1_nodes) > 255:
                raise ValueError(f"R1 列表过长，数量 {len(r1_nodes)} 超过单帧上限 255")
            payload.append(MCU_ACTION_TYPE_R1_LIST)
            payload.append(len(r1_nodes))
            for nid in r1_nodes:
                payload.append(_encode_path_node(str(nid)))
            action_count += 1

    if action_count > 255:
        raise ValueError(f"动作过多，当前数量 {action_count} 超过单帧上限 255")

    return bytes(payload), action_count


def encode_path_frame(path_nodes: list[str], result: dict = None) -> bytes:
    """把路径节点列表编码成节点帧，并包含R1物块列表。
    
    帧格式：[0xB5, 节点总数, ..., 0x04, R1数量, 节点码1, 节点码2, ...]
    """
    payload = bytearray()
    
    # 编码路径节点
    node_count = len(path_nodes)
    payload.append(node_count)
    
    for node in path_nodes:
        payload.append(_encode_path_node(node))
    
    # 添加R1物块列表分隔符和数据
    payload.append(MCU_ACTION_TYPE_R1_LIST)  # 0x04
    
    # 编码R1物块列表
    if result:
        r1_nodes = extract_r1_nodes_on_path(result)
        if r1_nodes:
            if len(r1_nodes) > 255:
                raise ValueError(f"R1 列表过长，数量 {len(r1_nodes)} 超过单帧上限 255")
            payload.append(len(r1_nodes))
            for nid in r1_nodes:
                payload.append(_encode_path_node(str(nid)))
        else:
            payload.append(0)
    else:
        payload.append(0)
    
    return bytes([PATH_NODE_FRAME_HEADER]) + bytes(payload)


def encode_mcu_action_frame(action_type: int, from_node: str, to_or_code) -> bytes:
    """把单个动作编码成 MCU 二进制帧。
    
    帧格式：[0xB8, 动作类型, 当前节点编码, 目标或代码]
    
    - 移动：(0x03, 当前节点编码, 目标位置编码)
    - 转向：(0x01, 当前节点编码, 转向码)
    - 拾取：(0x02, 当前节点编码, R2物块的位置编码)
    """
    from_node_code = _encode_path_node(from_node)
    
    payload = bytearray()
    payload.append(action_type)
    payload.append(from_node_code)
    payload.append(int(to_or_code))
    
    return bytes([MCU_ACTION_FRAME_HEADER]) + bytes(payload)
