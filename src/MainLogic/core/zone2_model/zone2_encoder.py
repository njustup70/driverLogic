"""zone2_encoder 模块作用：

负责把路径和动作记录编码成二进制帧，供串口或下位机直接消费。

本模块只做“数据编码”，不负责打印和发送；发送逻辑放在 `zone2_sender.py`。
"""
from __future__ import annotations

from MainLogic.core.zone2_model.zone2_helpers import (
    _encode_path_node,
    _encode_path_node_text,
    _is_derived_node,
    _extract_pick_target_from_derived_node,
    _turn_action_from_headings,
)
from MainLogic.core.zone2_model.zone2_format import build_path_step_records, extract_r1_nodes_on_path

PATH_NODE_FRAME_HEADER = 0xB5
PATH_NODE_ACTION_FRAME_HEADER = 0xB6
PATH_FULL_PATH_FRAME_HEADER = 0xB7
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


def encode_path_frame(path_nodes: list[str]) -> bytes:
    """把路径节点列表编码成最基础的节点帧。"""
    if not path_nodes:
        return bytes([PATH_NODE_FRAME_HEADER, 0x00])

    node_bytes = bytearray()
    for node in path_nodes:
        node_bytes.append(_encode_path_node(node))

    if len(node_bytes) > 255:
        raise ValueError(f"路径过长，当前长度 {len(node_bytes)} 超过单帧上限 255")

    return bytes([PATH_NODE_FRAME_HEADER, len(node_bytes)]) + bytes(node_bytes)


def encode_path_action_frame(result: dict) -> bytes:
    """把路径记录编码成“节点 + 动作”的文本友好帧。"""
    records = build_path_step_records(result)

    if not records:
        return bytes([PATH_NODE_ACTION_FRAME_HEADER, 0x00])

    payload = bytearray()
    for record in records:
        payload.extend(_encode_path_node_text(record["node"]))
        payload.append(record["turn_action"])
        payload.append(record["pick_target"])

    pair_count = len(records)
    if pair_count > 255:
        raise ValueError(f"路径过长，当前节点数 {pair_count} 超过单帧上限 255")

    return bytes([PATH_NODE_ACTION_FRAME_HEADER, pair_count]) + bytes(payload)


def encode_full_path_frame(result: dict) -> bytes:
    """把完整路径按动作序列直接拼接后编码成完整路径帧。"""
    payload, action_count = _build_action_payload(result, include_r1_list=False)

    if action_count == 0:
        return bytes([PATH_FULL_PATH_FRAME_HEADER, 0x00])

    return bytes([PATH_FULL_PATH_FRAME_HEADER, action_count]) + payload


def encode_mcu_action_frame(result: dict) -> bytes:
    """把完整动作序列编码成 MCU 二进制帧。"""
    payload, action_count = _build_action_payload(result, include_r1_list=True)

    if action_count == 0:
        return bytes([MCU_ACTION_FRAME_HEADER, 0x00])

    return bytes([MCU_ACTION_FRAME_HEADER, action_count]) + bytes(payload)
