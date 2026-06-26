from typing import Any, Dict, List, Optional, Tuple

from MainLogic.core.zone2_model.zone2_helpers import (
    _extract_pick_target_from_derived_node,
    _is_derived_node,
    _turn_action_from_headings,
)

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
# 单字节功能码协议
# =============================================================================
# 升降
FUNC_UP_200   = 0x64
FUNC_DOWN_200 = 0x65
FUNC_UP_400   = 0x66
FUNC_DOWN_400 = 0x67
# 转向
FUNC_TURN_CCW = 0x68  # 逆时针（正方向）
FUNC_TURN_CW  = 0x69  # 顺时针（负方向）
# 取块
FUNC_PICK_UP_200   = 0x6A
FUNC_PICK_UP_400   = 0x6B
FUNC_PICK_DOWN_200 = 0x6C
FUNC_PICK_DOWN_400 = 0x6D

# delta_z → 升降功能码
_LIFT_CODE: Dict[int, int] = {
    200:  FUNC_UP_200,
    -200: FUNC_DOWN_200,
    400:  FUNC_UP_400,
    -400: FUNC_DOWN_400,
}

# delta_z → 取块功能码
_PICK_CODE: Dict[int, int] = {
    200:  FUNC_PICK_UP_200,
    -200: FUNC_PICK_DOWN_200,
    400:  FUNC_PICK_UP_400,
    -400: FUNC_PICK_DOWN_400,
}

# turn code → 转向功能码
_TURN_CODE: Dict[int, int] = {
    1: FUNC_TURN_CCW,  # 逆时针
    2: FUNC_TURN_CW,   # 顺时针
}

# 动作类型 → (参数字段, 映射表)
_ACTION_ENCODERS = {
    "pick": ("delta_z", _PICK_CODE),
    "lift": ("delta_z", _LIFT_CODE),
    "turn": ("code",    _TURN_CODE),
}

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
) -> Tuple[float, float]:
    """根据动作序列推算机器人起始坐标 (x, y)。

    起始点位于第一个目标桩正前方 approach_distance 处。
    """
    first_target: Optional[str] = None
    for act in actions:
        if act.get("type") in ("move", "pick") and str(act.get("from")) == "start":
            first_target = str(act.get("to" if act["type"] == "move" else "target"))
            break

    if first_target is None:
        return (0.0, 0.0)

    # 解析桩号（兼容衍生节点 D_1_to_2）
    if first_target.startswith("D_"):
        parts = first_target.split("_")
        stake_id = int(parts[1]) if len(parts) >= 2 and parts[1].isdigit() else None
    elif first_target.isdigit():
        stake_id = int(first_target)
    else:
        stake_id = None

    if stake_id is None or stake_id not in STAKE_3D_INFO:
        return (0.0, 0.0)

    stake = STAKE_3D_INFO[stake_id]
    return (stake["x"] - approach_distance, stake["y"])


# =============================================================================
# 编码 & 发送
# =============================================================================
def encode_action_sequence(actions: List[Dict[str, Any]]) -> bytes:
    """将动作序列编码为 [长度位 | 功能码...] 的字节串。

    帧格式: [N, code1, code2, ..., codeN]
      N  = 有效功能码个数（不含 move）

    pick  → 0x6A~0x6D  (按 delta_z)
    lift  → 0x64~0x67  (按 delta_z)
    turn  → 0x68~0x69  (按 code)
    move  → 忽略（升降已合并到 lift）
    """
    codes = bytearray()
    for act in actions:
        t = act["type"]
        if t == "move":
            continue  # 纯水平移动无功能码
        if t not in _ACTION_ENCODERS:
            print(f"[encode] 未知动作类型: {t}，跳过")
            continue

        key, mapping = _ACTION_ENCODERS[t]
        val = act.get(key, 0)
        code = mapping.get(val)
        if code is None:
            print(f"[encode] 未知 {t} {key}={val}，跳过")
            continue
        codes.append(code)

    # 前面插入长度位
    return bytes([len(codes)]) + bytes(codes)


def send_actions(actions: List[Dict[str, Any]]) -> None:
    """将动作序列编码后一次性通过串口发送。

    发送链路:
      RosBridgeNodeInstance.writeBytes() → serial_tx 话题 → serial_node → 物理串口
    """
    from MainLogic.core.ros_bridge_node import RosBridgeNodeInstance

    data = encode_action_sequence(actions)
    if not data:
        print("[send_actions] 动作序列为空，不发送")
        return

    print(f"[send_actions] {len(actions)} 个动作 → {data.hex(' ')}")
    RosBridgeNodeInstance.writeBytes(data)