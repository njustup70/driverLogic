"""zone2_format 模块作用：

负责把 `path` / `path_steps` 转换成可读文本，主要提供：
- 生成动作记录（records）
- 将记录格式化成中文步骤说明
- 生成适合终端调试的 MCU 动作列表文本

这个模块不负责二进制编码，也不负责串口发送，只负责“展示层”的字符串拼装。
"""
from __future__ import annotations

from typing import Any
from MainLogic.core.zone2_model.zone2_helpers import (
    _turn_action_name,
    _is_derived_node,
    _is_real_node,
    _extract_pick_target_from_derived_node,
    _extract_owner_from_derived_node,
    _turn_action_from_headings,
)


def extract_r1_nodes_on_path(result: dict) -> list[int]:
    """提取路径上经过的 R1 节点编号（升序）。"""
    path_steps = list(result.get("path_steps", []))
    if not path_steps:
        return []

    preset_nodes = result.get("r1_nodes_on_path")
    if isinstance(preset_nodes, list) and preset_nodes:
        normalized: set[int] = set()
        for value in preset_nodes:
            value_str = str(value)
            if value_str.isdigit():
                normalized.add(int(value_str))
        return sorted(normalized)

    map_data = result.get("map_data") if isinstance(result, dict) else None
    blocks = map_data.get("blocks", {}) if isinstance(map_data, dict) else {}

    r1_nodes: set[int] = set()
    for step in path_steps:
        for node in (step.get("from"), step.get("to")):
            node_str = str(node)
            if not node_str.isdigit():
                continue
            node_id = int(node_str)

            block_value = blocks.get(node_id)
            if block_value is None:
                block_value = blocks.get(node_str)

            if block_value == "R1":
                r1_nodes.add(node_id)

    return sorted(r1_nodes)


def build_path_step_records(result: dict) -> list[dict]:
    """将 path_steps 压缩成按实际节点组织的记录。"""
    path_nodes = list(result.get("path", []))
    path_steps = list(result.get("path_steps", []))

    records: list[dict] = []
    for index, step in enumerate(path_steps):
        from_node = step.get("from")
        to_node = step.get("to")
        if not _is_real_node(from_node):
            continue

        turn_action = _turn_action_from_headings(step.get("heading_in"), step.get("heading_out"))

        edge_class = str(step.get("edge_class", ""))
        if edge_class == "to_derived" or _is_derived_node(to_node):
            pick_target = _extract_pick_target_from_derived_node(to_node)
        else:
            pick_target = 0

        next_real_node = "end"
        for look_ahead in range(index + 1, len(path_nodes)):
            candidate = path_nodes[look_ahead]
            if _is_real_node(candidate):
                next_real_node = str(candidate)
                break

        records.append(
            {
                "node": str(from_node),
                "to": next_real_node,
                "raw_to": str(to_node),
                "turn_action": turn_action,
                "pick_target": pick_target,
                "edge_class": edge_class,
            }
        )

    return records


def _node_display_name(node: object) -> str:
    node_str = str(node)
    if node_str == "start" or node_str == "end":
        return node_str
    if node_str.isdigit():
        return f"{node_str}号"
    return node_str


def _display_node_for_action(node: object) -> str:
    node_str = str(node)
    if _is_derived_node(node_str):
        owner = _extract_owner_from_derived_node(node_str)
        return _node_display_name(owner)
    return _node_display_name(node_str)


def _display_target_for_turn(node: object) -> str:
    """转向文本中显示的“面向目标”：衍生节点显示其 target_r2，而不是 owner。"""
    node_str = str(node)
    if _is_derived_node(node_str):
        target_r2 = _extract_pick_target_from_derived_node(node_str)
        if target_r2 > 0:
            return _node_display_name(target_r2)
    return _display_node_for_action(node_str)


def format_action_chain(records: list[dict]) -> str:
    """把记录列表格式化成更自然的中文动作链。"""
    if not records:
        return "(empty)"

    lines: list[str] = []
    step_no = 1
    for record in records:
        raw_from = record.get("from") if record.get("from") is not None else record.get("node")
        raw_to = record.get("to") if record.get("to") is not None else record.get("raw_to")

        from_node = _display_node_for_action(raw_from)
        to_node = _display_node_for_action(raw_to)

        turn_action = int(record.get("turn_action", 0))
        pick_target = int(record.get("pick_target", 0))
        edge_class = str(record.get("edge_class", ""))

        is_pick_step = edge_class == "to_derived" or _is_derived_node(raw_to)

        if is_pick_step:
            if pick_target > 0:
                lines.append(f"{step_no:02d}. 在 {from_node} 拾取 {pick_target} 号上的 R2 物块")
            else:
                lines.append(f"{step_no:02d}. 在 {from_node} 执行取块动作")
            step_no += 1
        else:
            if turn_action != 0 and str(raw_from) != "start":
                turn_name = _turn_action_name(turn_action)
                turn_to_node = _display_target_for_turn(raw_to)
                lines.append(f"{step_no:02d}. 在 {from_node} 节点{turn_name}，面向 {turn_to_node}")
                step_no += 1

            lines.append(f"{step_no:02d}. 从 {from_node} 节点走到 {to_node} 节点")
            step_no += 1

    return "\n".join(lines)


def format_chronological_steps(result: dict) -> str:
    """按 path_steps 原始顺序输出动作时间线。"""
    path_steps = list(result.get("path_steps", []))
    if not path_steps:
        return "(empty)"

    lines: list[str] = []
    step_no = 1
    for step in path_steps:
        u = step.get("from")
        v = step.get("to")
        edge_class = str(step.get("edge_class", ""))
        from_name = _display_node_for_action(u)
        to_name = _display_node_for_action(v)

        turn_action = _turn_action_from_headings(step.get("heading_in"), step.get("heading_out"))
        turn_cost = float(step.get("turn_cost", 0.0))

        if turn_cost > 0.0 and turn_action != 0 and str(u) != "start":
            turn_to_name = _display_target_for_turn(v)
            lines.append(f"{step_no:02d}. 在 {from_name} 节点{_turn_action_name(turn_action)}，面向 {turn_to_name}")
            step_no += 1

        is_pick_step = edge_class == "to_derived" or _is_derived_node(v)

        if is_pick_step:
            pick_target = _extract_pick_target_from_derived_node(v)
            if pick_target > 0:
                lines.append(f"{step_no:02d}. 在 {from_name} 拾取 {pick_target} 号上的 R2 物块")
            else:
                lines.append(f"{step_no:02d}. 在 {from_name} 执行取块动作")
            step_no += 1
        else:
            lines.append(f"{step_no:02d}. 从 {from_name} 节点走到 {to_name} 节点")
            step_no += 1

    return "\n".join(lines)


def build_action_chain_records(result: dict) -> list[dict]:
    """按原始边语义构造逐边记录，供其它模块复用。"""
    path_steps = list(result.get("path_steps", []))

    records: list[dict] = []
    for step in path_steps:
        from_node = step.get("from")
        to_node = step.get("to")
        edge_class = str(step.get("edge_class", ""))
        pick_target = _extract_pick_target_from_derived_node(to_node) if edge_class == "to_derived" or _is_derived_node(to_node) else 0
        records.append(
            {
                "from": str(from_node),
                "to": str(to_node),
                "turn_action": _turn_action_from_headings(step.get("heading_in"), step.get("heading_out")),
                "turn_cost": float(step.get("turn_cost", 0.0)),
                "pick_target": pick_target,
                "edge_class": edge_class,
            }
        )

    return records


def format_mcu_action_list(result: dict) -> str:
    """生成便于终端查看的 MCU 动作列表文本。"""
    path_steps = list(result.get("path_steps", []))
    if not path_steps:
        return "(empty)"

    lines: list[str] = []
    step_no = 1

    for step in path_steps:
        u = step.get("from")
        v = step.get("to")
        edge_class = str(step.get("edge_class", ""))

        from_name = _display_node_for_action(u)
        to_name = _display_node_for_action(v)

        turn_action = _turn_action_from_headings(step.get("heading_in"), step.get("heading_out"))
        turn_cost = float(step.get("turn_cost", 0.0))

        if turn_cost > 0.0 and turn_action != 0 and str(u) != "start":
            lines.append(f"{step_no:02d}. {from_name} | {_turn_action_name(turn_action)}")
            step_no += 1

        is_pick_step = edge_class == "to_derived" or _is_derived_node(v)

        if is_pick_step:
            pick_target = _extract_pick_target_from_derived_node(v)
            if pick_target > 0:
                lines.append(f"{step_no:02d}. {from_name} | 拾取{pick_target}号")
            else:
                lines.append(f"{step_no:02d}. {from_name} | 执行取块")
            step_no += 1
        else:
            lines.append(f"{step_no:02d}. {from_name} -> {to_name}")
            step_no += 1

    return "\n".join(lines)


def print_path_debug_info(result: dict) -> None:
    chronological = format_chronological_steps(result)
    if not chronological or chronological == "(empty)":
        print("[zone2_model_api] action_chain: (empty)", flush=True)
        return

    print("[zone2_model_api] action_chain:", flush=True)
    print(chronological, flush=True)

    mcu_format = format_mcu_action_list(result)
    if mcu_format and mcu_format != "(empty)":
        print("\n[zone2_model_api] mcu_action_list:", flush=True)
        print(mcu_format, flush=True)

    r1_nodes = extract_r1_nodes_on_path(result)
    r1_text = "无" if not r1_nodes else ", ".join(f"{node}号" for node in r1_nodes)
    print(f"\n[zone2_model_api] path_r1_nodes: {r1_text}", flush=True)
