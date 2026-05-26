"""zone2_sender 模块作用：

负责把已经编码好的帧写入 `RosBridgeNodeInstance`。

本模块只负责"发送"和最少量日志，不负责路径计算和二进制编码。
"""
from __future__ import annotations

from MainLogic.core.zone2_model.zone2_encoder import encode_path_frame, encode_mcu_action_frame, MCU_ACTION_TYPE_TURN, MCU_ACTION_TYPE_PICK, MCU_ACTION_TYPE_MOVE
from MainLogic.core.zone2_model.zone2_format import extract_r1_nodes_on_path
from MainLogic.core.zone2_model.zone2_helpers import (
    _is_derived_node,
    _extract_pick_target_from_derived_node,
    _turn_action_from_headings,
)


def _get_ros_bridge_module():
    from MainLogic.core import ros_bridge_node as ros_bridge_module

    return ros_bridge_module


# ========== 方法1：从result中逐个提取动作并分别发送 ==========
def send_mcu_action_frame_to_mcu_batch(result: dict) -> None:
    """从路径结果中逐个提取动作并分别发送到下位机（一个result对应多个帧）。"""
    ros_bridge_module = _get_ros_bridge_module()
    assert ros_bridge_module.RosBridgeNodeInstance is not None, "RosBridgeNodeInstance is not initialized yet!"

    if not result.get("found"):
        ros_bridge_module.RosBridgeNodeInstance.writeBytes(bytes([0xB8, 0x00]))
        print("[zone2_model_api] 没有找到可用路径，已发送空动作帧", flush=True)
        return

    path_steps = list(result.get("path_steps", []))
    if not path_steps:
        ros_bridge_module.RosBridgeNodeInstance.writeBytes(bytes([0xB8, 0x00]))
        print("[zone2_model_api] 没有动作步骤，已发送空动作帧", flush=True)
        return

    action_count = 0
    for step in path_steps:
        u = step.get("from")
        v = step.get("to")
        edge_class = str(step.get("edge_class", ""))
        turn_action = _turn_action_from_headings(step.get("heading_in"), step.get("heading_out"))
        turn_cost = float(step.get("turn_cost", 0.0))
        pick_target = _extract_pick_target_from_derived_node(v) if edge_class == "to_derived" or _is_derived_node(v) else 0

        # 转向动作
        if turn_cost > 0.0 and turn_action != 0 and str(u) != "start":
            frame = encode_mcu_action_frame(MCU_ACTION_TYPE_TURN, str(u), turn_action)
            ros_bridge_module.RosBridgeNodeInstance.writeBytes(frame)
            action_count += 1

        # 取货或移动动作
        is_pick_step = edge_class == "to_derived" or _is_derived_node(v)
        if is_pick_step:
            frame = encode_mcu_action_frame(MCU_ACTION_TYPE_PICK, str(u), pick_target)
            ros_bridge_module.RosBridgeNodeInstance.writeBytes(frame)
            action_count += 1
        else:
            frame = encode_mcu_action_frame(MCU_ACTION_TYPE_MOVE, str(u), str(v))
            ros_bridge_module.RosBridgeNodeInstance.writeBytes(frame)
            action_count += 1

    print(f"[zone2_model_api] 已发送 {action_count} 个动作到下位机", flush=True)


# ========== 方法2：直接接受单个动作参数并发送 ==========
def send_mcu_action_frame_to_mcu(action_type: int, from_node: str, to_or_code) -> None:
    """发送单个 MCU 动作帧到串口/桥接节点。
    
    参数：
        action_type: 0x01(转向), 0x02(取货), 0x03(移动)
        from_node: 当前节点编码
        to_or_code: 目标节点编码或动作码
    """
    ros_bridge_module = _get_ros_bridge_module()
    assert ros_bridge_module.RosBridgeNodeInstance is not None, "RosBridgeNodeInstance is not initialized yet!"

    frame = encode_mcu_action_frame(action_type, from_node, to_or_code)
    ros_bridge_module.RosBridgeNodeInstance.writeBytes(frame)
    
    action_name = {
        MCU_ACTION_TYPE_TURN: "转向",
        MCU_ACTION_TYPE_PICK: "取货",
        MCU_ACTION_TYPE_MOVE: "移动",
    }.get(action_type, "未知")
    
    print(
        f"[zone2_model_api] 已发送{action_name}动作到下位机: "
        f"from_node={from_node}, to_or_code={to_or_code}, "
        f"frame_hex={' '.join(f'{b:02X}' for b in frame)}",
        flush=True,
    )


def send_path_result_to_mcu(result: dict) -> None:
    """发送完整路径帧到串口/桥接节点。"""
    ros_bridge_module = _get_ros_bridge_module()
    assert ros_bridge_module.RosBridgeNodeInstance is not None, "RosBridgeNodeInstance is not initialized yet!"

    if not result.get("found"):
        ros_bridge_module.RosBridgeNodeInstance.writeBytes(bytes([0xB5, 0x00, 0x04, 0x00]))
        print("[zone2_model_api] 没有找到可用路径，已发送空路径帧", flush=True)
        return

    # 从result中提取路径节点列表（根据实际数据结构调整）
    path_nodes = result.get("path", [])
    if not path_nodes:
        ros_bridge_module.RosBridgeNodeInstance.writeBytes(bytes([0xB5, 0x00, 0x04, 0x00]))
        print("[zone2_model_api] 路径为空，已发送空路径帧", flush=True)
        return
    
    frame = encode_path_frame(path_nodes, result)
    ros_bridge_module.RosBridgeNodeInstance.writeBytes(frame)
    
    r1_nodes = extract_r1_nodes_on_path(result)
    r1_text = "无" if not r1_nodes else ", ".join(f"{n}号" for n in r1_nodes)
    
    print(
        f"[zone2_model_api] 已发送路径帧到下位机: "
        f"path_length={len(path_nodes)}, "
        f"frame_bytes={len(frame)}, "
        f"R1节点={r1_text}, "
        f"frame_hex={' '.join(f'{b:02X}' for b in frame[:min(30, len(frame))])}{'...' if len(frame) > 30 else ''}",
        flush=True,
    )
