"""zone2_sender 模块作用：

负责把已经编码好的帧写入 `RosBridgeNodeInstance`。

本模块只负责“发送”和最少量日志，不负责路径计算和二进制编码。
"""
from __future__ import annotations

from MainLogic.core.zone2_model.zone2_encoder import encode_full_path_frame, encode_mcu_action_frame
from MainLogic.core.zone2_model.zone2_format import extract_r1_nodes_on_path


def _get_ros_bridge_module():
    from MainLogic.core import ros_bridge_node as ros_bridge_module

    return ros_bridge_module


def send_mcu_action_frame_to_mcu(result: dict) -> None:
    """发送 MCU 动作帧到串口/桥接节点。"""
    ros_bridge_module = _get_ros_bridge_module()
    assert ros_bridge_module.RosBridgeNodeInstance is not None, "RosBridgeNodeInstance is not initialized yet!"

    if not result.get("found"):
        ros_bridge_module.RosBridgeNodeInstance.writeBytes(bytes([0xB8, 0x00]))
        print("[zone2_model_api] 没有找到可用路径，已发送空动作帧", flush=True)
        return

    frame = encode_mcu_action_frame(result)
    ros_bridge_module.RosBridgeNodeInstance.writeBytes(frame)

    r1_nodes = extract_r1_nodes_on_path(result)
    r1_text = "无" if not r1_nodes else ", ".join(f"{n}号" for n in r1_nodes)

    print(
        f"[zone2_model_api] 已发送动作帧到下位机: "
        f"action_count={frame[1]}, "
        f"frame_bytes={len(frame)}, "
        f"R1节点={r1_text}, "
        f"frame_hex={' '.join(f'{b:02X}' for b in frame[:min(30, len(frame))])}{'...' if len(frame) > 30 else ''}",
        flush=True,
    )


def send_path_result_to_mcu(result: dict) -> None:
    """发送完整路径帧到串口/桥接节点。"""
    ros_bridge_module = _get_ros_bridge_module()
    assert ros_bridge_module.RosBridgeNodeInstance is not None, "RosBridgeNodeInstance is not initialized yet!"

    if not result.get("found"):
        ros_bridge_module.RosBridgeNodeInstance.writeBytes(bytes([0xB7, 0x00]))
        print("[zone2_model_api] 没有找到可用路径，已发送空完整路径帧", flush=True)
        return

    frame = encode_full_path_frame(result)
    ros_bridge_module.RosBridgeNodeInstance.writeBytes(frame)
    print("[zone2_model_api] 已发送完整路径到下位机", flush=True)
