from typing import List, Dict, Optional, Any

# ---------- 三维坐标常量（与之前一致）----------
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
R2_HEIGHT = 200  # R2物块额外高度

# ---------- 辅助函数 ----------
def _get_node_total_height(node_str: str, blocks: Dict[int, str]) -> float:
    """
    计算任意节点（主桩/衍生节点/门户）的总高度（mm）。
    - 主桩：base_height + (R2_HEIGHT if R2 else 0)
    - 衍生节点：继承其 owner 的高度
    - 门户：0
    """
    if node_str in ("start", "end"):
        return 0.0

    # 衍生节点：格式 D_{owner}_to_{r2}
    if node_str.startswith("D_"):
        parts = node_str.split("_")
        owner = parts[1]  # 例如 "5"
        if owner.isdigit():
            return _get_node_total_height(owner, blocks)
        else:
            return 0.0  # owner 为 start/end 时高度为0

    # 主桩节点
    if node_str.isdigit():
        stake_id = int(node_str)
        base = STAKE_3D_INFO[stake_id]["base_height"]
        if blocks.get(stake_id) == "R2":
            return base + R2_HEIGHT
        return base

    return 0.0


def generate_actions_from_result(
    result: dict,
    blocks: Optional[Dict[int, str]] = None
) -> List[Dict[str, Any]]:
    """
    根据路径结果和桩类型配置，生成完整的动作序列（包含升降、移动、转向、取货）。
    
    :param result: 路径求解结果字典，需包含 "path_steps" 字段
    :param blocks: 桩类型配置 {桩号: "R2"/"fake"/"R1"/"empty"}，默认空（全部为 empty）
    :return: 动作列表，每个动作为字典，格式如：
        {"type": "move", "from": "1", "to": "4"}
        {"type": "lift", "from": "1", "delta_z": -200}   # 负值下降，正值上升
        {"type": "turn", "from": "1", "code": 1}         # 转向码由 zone2_helpers 定义
        {"type": "pick", "from": "1", "target": 2}       # 取货目标桩号
    """
    if blocks is None:
        blocks = {}

    actions: List[Dict[str, Any]] = []

    if not result.get("found"):
        return actions  # 无路径，返回空列表

    path_steps = result.get("path_steps", [])
    if not path_steps:
        return actions

    # 导入转向解析函数（假设已存在于 zone2_helpers）
    from MainLogic.core.zone2_model.zone2_helpers import (
        _is_derived_node,
        _extract_pick_target_from_derived_node,
        _turn_action_from_headings,
    )

    for step in path_steps:
        u = step.get("from")
        v = step.get("to")
        edge_class = str(step.get("edge_class", ""))
        turn_action = _turn_action_from_headings(step.get("heading_in"), step.get("heading_out"))
        turn_cost = float(step.get("turn_cost", 0.0))
        pick_target = _extract_pick_target_from_derived_node(v) if (
            edge_class == "to_derived" or _is_derived_node(v)
        ) else 0

        # 1. 转向动作（起点除外）
        if turn_cost > 0.0 and turn_action != 0 and str(u) != "start":
            actions.append({
                "type": "turn",
                "from": str(u),
                "code": turn_action,
            })

        # 2. 取货或移动
        is_pick_step = edge_class == "to_derived" or _is_derived_node(v)
        if is_pick_step:
            actions.append({
                "type": "pick",
                "from": str(u),
                "target": pick_target,
            })
        else:
            # 移动步骤：先处理高度差
            h_u = _get_node_total_height(str(u), blocks)
            h_v = _get_node_total_height(str(v), blocks)
            delta_z = h_v - h_u  # 正值为上升，负值为下降

            if abs(delta_z) > 0.001:
                actions.append({
                    "type": "lift",
                    "from": str(u),
                    "delta_z": int(round(delta_z)),  # 毫米
                })

            # 水平移动
            actions.append({
                "type": "move",
                "from": str(u),
                "to": str(v),
            })

    return actions
