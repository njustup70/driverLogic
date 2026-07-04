import asyncio
import threading
from typing import Any, Dict, List, Optional, Tuple

from MainLogic.core.zone2_model.zone2_helpers import (
    _extract_pick_target_from_derived_node,
    _is_derived_node,
    _turn_action_from_headings,
)
from MainLogic.core.zone2_model.zone2_encoder import encode_action_sequence, encode_r1_frame
from MainLogic.core.zone2_model.zone2_format import extract_r1_nodes_on_path

# 动作完成确认事件：action_callback 收到 FF 6F 帧时 set，send_actions_one_by_one 等待此事件
# 使用 threading.Event 而非 asyncio.Event，因为 action_callback 在 ROS2 回调线程中执行，
# threading.Event 天然线程安全，set() 后 run_in_executor 中的 wait() 立即返回，无调度延迟
action_ack_event = threading.Event()

# =============================================================================
# 三维坐标常量
# =============================================================================
STAKE_3D_INFO: Dict[int, Dict[str, float]] = {
    1:  {"x": 3800, "y": 1800, "base_height": 400},
    2:  {"x": 3800, "y": 3000, "base_height": 200},
    3:  {"x": 3800, "y": 4200, "base_height": 400},
    4:  {"x": 5000, "y": 1800, "base_height": 200},
    5:  {"x": 5000, "y": 3000, "base_height": 400},
    6:  {"x": 5000, "y": 4200, "base_height": 600},
    7:  {"x": 6200, "y": 1800, "base_height": 400},
    8:  {"x": 6200, "y": 3000, "base_height": 600},
    9:  {"x": 6200, "y": 4200, "base_height": 400},
    10: {"x": 7400, "y": 1800, "base_height": 200},
    11: {"x": 7400, "y": 3000, "base_height": 400},
    12: {"x": 7400, "y": 4200, "base_height": 200},
}
R2_HEIGHT = 200  # R2 物块额外高度 (mm)

# =============================================================================
# 高度计算
# =============================================================================
def _get_node_total_height(node_str: str, blocks: Dict[int, str]) -> float:
    """计算任意节点的总高度 (mm)。

    规则:
      - start / end → 0
      - 衍生节点 D_{owner}_to_{r2} → 继承 owner 的高度
      - 主桩 → base_height + (R2_HEIGHT if R2 else 0)
    """
    if node_str in ("start", "end"):
        return 0.0

    if node_str.startswith("D_"):
        owner = node_str.split("_")[1]
        return _get_node_total_height(owner, blocks) if owner.isdigit() else 0.0

    if node_str.isdigit():
        stake_id = int(node_str)
        base = STAKE_3D_INFO[stake_id]["base_height"]
        return base + R2_HEIGHT if blocks.get(stake_id) == "R2" else base

    return 0.0


# =============================================================================
# 动作生成
# =============================================================================
def generate_actions_from_result(
    result: dict,
    blocks: Optional[Dict[int, str]] = None,
) -> List[Dict[str, Any]]:
    """根据路径结果生成动作序列。

    Args:
        result: 路径求解结果，需包含 "path_steps" 字段。
        blocks: 桩类型配置 {桩号: "R2" | "fake" | "R1" | "empty"}。

    Returns:
        动作列表，每项为 dict:
          {"type": "turn", "from": "1", "code": 1}
          {"type": "pick", "from": "1", "target": 2, "delta_z": 400}
          {"type": "lift", "from": "1", "to": "4", "delta_z": 200}
          {"type": "move", "from": "1", "to": "4"}
    """
    if blocks is None:
        blocks = {}

    actions: List[Dict[str, Any]] = []
    if not result.get("found"):
        return actions

    path_steps = result.get("path_steps", [])
    if not path_steps:
        return actions

    for step in path_steps:
        u = step.get("from")
        v = step.get("to")
        edge_class = str(step.get("edge_class", ""))
        turn_action = _turn_action_from_headings(
            step.get("heading_in"), step.get("heading_out")
        )
        turn_cost = float(step.get("turn_cost", 0.0))
        is_pick = edge_class == "to_derived" or _is_derived_node(v)
        pick_target = (
            _extract_pick_target_from_derived_node(v) if is_pick else 0
        )

        # 1) 转向（起点不转）
        if turn_cost > 0.0 and turn_action != 0 and str(u) != "start":
            actions.append({"type": "turn", "from": str(u), "code": turn_action})

        # 2) 取块 / 升降+移动
        if is_pick:
            h_current = _get_node_total_height(str(u), blocks)
            h_target = _get_node_total_height(str(pick_target), blocks)
            actions.append({
                "type": "pick",
                "from": str(u),
                "target": pick_target,
                "delta_z": int(round(h_target - h_current)),
            })
        else:
            h_u = _get_node_total_height(str(u), blocks)
            h_v = _get_node_total_height(str(v), blocks)
            delta_z = int(round(h_v - h_u))

            if str(u) == str(v):
                if delta_z != 0:
                    actions.append({"type": "lift", "from": str(u), "delta_z": delta_z})
            elif delta_z != 0:
                actions.append({
                    "type": "lift", "from": str(u), "to": str(v), "delta_z": delta_z,
                })
            else:
                actions.append({"type": "move", "from": str(u), "to": str(v)})

    return actions


# =============================================================================
# 起始位置
# =============================================================================
def determine_start_position(
    actions: List[Dict[str, Any]],
    approach_distance: float = 500.0,
) -> Optional[int]:
    """根据动作序列推算第一个目标桩的编号。

    返回第一个 move/pick 动作的目标桩号，若无法确定则返回 None。
    """
    first_target: Optional[str] = None
    for act in actions:
        if act.get("type") in ("move", "pick", "lift") and str(act.get("from")) == "start":
            first_target = str(act.get("to" if act["type"] in ("move", "lift") else "target"))
            break

    if first_target is None:
        return None

    # 解析桩号（兼容衍生节点 D_1_to_2）
    if first_target.startswith("D_"):
        parts = first_target.split("_")
        stake_id = int(parts[1]) if len(parts) >= 2 and parts[1].isdigit() else None
    elif first_target.isdigit():
        stake_id = int(first_target)
    else:
        stake_id = None

    if stake_id is None or stake_id not in STAKE_3D_INFO:
        return None

    return stake_id


def _stake_id_to_column(stake_id: int) -> int:
    """桩号 → 列编号（1-based）。

    桩位布局: 四行三列，每列 4 个桩。
    列 1={1,2,3,4}, 列 2={5,6,7,8}, 列 3={9,10,11,12}。
    """
    return (stake_id - 1) // 4 + 1


def determine_column_stake_id(actions: List[Dict[str, Any]]) -> Optional[int]:
    """提取第一个 lift 动作的目标桩号。

    扫描整个动作序列，找到第一个 lift 动作，返回其 to 字段对应的桩号（1~12）。
    若无法确定则返回 None。
    """
    for act in actions:
        if act.get("type") != "lift":
            continue
        target = str(act.get("to", ""))
        stake_id: Optional[int] = None
        if target.isdigit():
            stake_id = int(target)
        elif target.startswith("D_"):
            parts = target.split("_")
            stake_id = int(parts[1]) if len(parts) >= 2 and parts[1].isdigit() else None
        if stake_id is not None and stake_id in STAKE_3D_INFO:
            return stake_id
    return None


# =============================================================================
# 发送
# =============================================================================
def send_r1_nodes(r1_nodes: List[int]) -> None:
    """将 R1 物块列表编码后通过串口发送。

    帧格式: [个数, 桩号1, 桩号2, ...]
    """
    from MainLogic.core.ros_bridge_node import RosBridgeNodeInstance

    data = encode_r1_frame([n - 1 for n in r1_nodes])  # 桩号从 1 开始，编码时减 1
    if not data or data[0] == 0:
        print("[send_r1_nodes] R1 列表为空，不发送")
        return

    print(f"[send_r1_nodes] {r1_nodes} → {data.hex(' ')}")
    RosBridgeNodeInstance.writeBytes(data)


def send_actions(actions: List[Dict[str, Any]]) -> None:
    """将动作序列编码后一次性通过串口发送（同步版本，不等待响应）。

    发送链路:
      RosBridgeNodeInstance.writeBytes() → serial_tx 话题 → serial_node → 物理串口
    """
    from MainLogic.core.ros_bridge_node import RosBridgeNodeInstance

    start_stake_id = determine_start_position(actions) or 0
    column_stake_id = determine_column_stake_id(actions) or 0
    data = encode_action_sequence(actions, start_stake_id=start_stake_id, column_stake_id=column_stake_id)
    if not data:
        print("[send_actions] 动作序列为空，不发送")
        return

    print(f"[send_actions] 起始桩={start_stake_id}, 列桩={column_stake_id}, {len(actions)} 个动作 → {data.hex(' ')}")
    RosBridgeNodeInstance.writeBytes(data)


async def send_actions_one_by_one(
    actions: List[Dict[str, Any]],
    timeout: float = 10.0,
) -> bool:
    """逐帧发送动作序列，每发送一帧等待下位机回复 0x6F 后再发下一帧。

    通信协议:
      - 发送帧格式: [1, 功能码]  (每帧一个动作)
      - 下位机成功执行后回复: 0x6F
      - 超时未收到 0x6F 则终止发送并返回 False

    Args:
        actions: 动作列表，每项为 dict:
          {"type": "turn", "from": "1", "code": 1}
          {"type": "pick", "from": "1", "target": 2, "delta_z": 400}
          {"type": "lift", "from": "1", "to": "4", "delta_z": 200}
          {"type": "move", "from": "1", "to": "4"}
        timeout: 每帧等待响应的超时时间（秒）。

    Returns:
        True 表示所有帧均成功发送并收到 0x6F 响应。
        False 表示超时或动作序列为空。
    """
    from MainLogic.core.ros_bridge_node import RosBridgeNodeInstance
    actions_finish_frame = bytes([0x6e])  # 动作序列结束帧
    # 1) 编码整个动作序列，得到 [N, 0x91, start_stake_id, column_stake_id, code1, ..., codeN, 0x6e]
    start_stake_id = determine_start_position(actions) or 0
    column_stake_id = determine_column_stake_id(actions) or 0
    data = encode_action_sequence(actions, start_stake_id=start_stake_id, column_stake_id=column_stake_id)
    if not data or data[0] == 0:
        print("[send_actions] 动作序列为空，不发送")
        return False

    total = data[0]  # 功能码个数
    # 跳过 [N, 0x91, start_stake_id, column_stake_id] 四个字节，提取功能码列表
    codes = data[4:-1]  # 去掉末尾 0x6e

    print(f"[send_actions] 共 {total} 帧，开始逐帧发送...")

    for i, code in enumerate(codes):
        action_ack_event.clear()

        # 构造单帧: [功能码]
        frame = bytes([code])
        print(f"[send_actions] 发送第 {i + 1}/{total} 帧: {frame.hex(' ')}")
        RosBridgeNodeInstance.writeBytes(frame)

        # 等待 action_callback 收到 FF 6F 后 set action_ack_event
        # threading.Event.wait(timeout) 返回 True=已set, False=超时
        loop = asyncio.get_running_loop()
        acked = await loop.run_in_executor(None, action_ack_event.wait, timeout)
        if not acked:
            print(f"[send_actions] 第 {i + 1}/{total} 帧超时 {timeout}s 未收到 FF 6F，终止发送")
            return False
        print(f"[send_actions] 第 {i + 1}/{total} 帧收到 FF 6F ✓")

    RosBridgeNodeInstance.writeBytes(actions_finish_frame)
    print("[send_actions] 所有帧发送完成，动作序列结束帧已发送")
    return True
