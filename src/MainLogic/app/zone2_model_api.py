"""Main 层的 Zone 2 梅林接口封装。

这一层位于 MainLogic.core.zone2_model 与具体 async main 之间。
仅保留 main 运行所需的最小接口：
- 5 个可调参数（MOVE/PICK/TURN/R1_REMOVE/REQUIRED_R2_COUNT）
- 随机地图求解并可视化的入口
"""

from __future__ import annotations

import re
from typing import Any, Dict, Optional

from MainLogic.core import ros_bridge_node as ros_bridge_module
from MainLogic.core.zone2_model import (
    MOVE_COST,
    PICK_COST,
    R1_REMOVE_COST,
    REQUIRED_R2_COUNT,
    TURN_COST,
    dijkstra_min_cost_path,
    draw_merlin_model,
    get_merlin_map,
)


PATH_NODE_FRAME_HEADER = 0xB5
PATH_NODE_ACTION_FRAME_HEADER = 0xB6
PATH_FULL_PATH_FRAME_HEADER = 0xB7

PATH_ACTION_MOVE = 0x00
PATH_ACTION_PICK_R2 = 0x01
PATH_ACTION_STOP = 0x02

TURN_ACTION_STRAIGHT = 0x00
TURN_ACTION_LEFT = 0x01
TURN_ACTION_RIGHT = 0x02
TURN_ACTION_UTURN = 0x03

PICK_ACTION_NONE = 0x00
PICK_ACTION_R2 = 0x01


def _turn_action_name(action: int) -> str:
    return {
        TURN_ACTION_STRAIGHT: "直行",
        TURN_ACTION_LEFT: "左转",
        TURN_ACTION_RIGHT: "右转",
        TURN_ACTION_UTURN: "掉头",
    }.get(action, f"未知({action})")


def _pick_action_name(action: int) -> str:
    if action == PICK_ACTION_NONE:
        return "不取块"
    return f"取{action}上的物块"


def _encode_path_node(node: str) -> int:
    node_str = str(node)
    if node_str == "start":
        return 0xFE
    if node_str == "end":
        return 0xFF
    if node_str.isdigit():
        value = int(node_str)
        if not 0 <= value <= 253:
            raise ValueError(f"路径节点超出可编码范围: {node_str}")
        return value
    raise ValueError(f"无法编码的路径节点: {node_str}")


def _encode_path_node_text(node: str) -> bytes:
    """把任意节点名编码成长度前缀 UTF-8 字节串。"""
    node_bytes = str(node).encode("utf-8")
    if len(node_bytes) > 255:
        raise ValueError(f"路径节点文本过长: {node}")
    return bytes([len(node_bytes)]) + node_bytes


def _turn_action_from_headings(prev_heading: Optional[str], next_heading: Optional[str]) -> int:
    """根据进入当前节点前后的朝向，生成转向动作编码。"""
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


def _pick_target_from_node(node: str, step: Optional[dict]) -> int:
    """根据当前节点判断取块目标。

    规则：
    - 只有衍生节点才视为取块点
    - 取块目标取节点名里最后一个数字，比如 1to4 / D_1_to_4 -> 4
    - 非衍生节点返回 0
    """
    node_str = str(node)
    is_derived_node = node_str.startswith("D_") or ("to" in node_str) or (step is not None and step.get("edge_class") == "to_derived")
    if not is_derived_node:
        return PICK_ACTION_NONE

    match = re.search(r"(\d+)(?!.*\d)", node_str)
    if match is None:
        return PICK_ACTION_NONE
    return int(match.group(1))


def _is_derived_node(node: str) -> bool:
    node_str = str(node)
    return node_str.startswith("D_") or ("to" in node_str)


def _is_real_node(node: str) -> bool:
    node_str = str(node)
    return node_str != "end" and not _is_derived_node(node_str)


def _extract_pick_target_from_derived_node(node: str) -> int:
    node_str = str(node)
    match = re.search(r"(\d+)(?!.*\d)", node_str)
    if match is None:
        return PICK_ACTION_NONE
    return int(match.group(1))


def build_path_step_records(result: dict) -> list[dict]:
    """把 path_steps 压缩成只保留实际节点的发送记录。

    规则：
    - 只保留 from 节点是实际节点的步进
    - 如果该步进入衍生节点，则把衍生节点折叠成当前节点上的取块动作
    - 衍生节点本身不单独发送
    """
    path_nodes = list(result.get("path", []))
    path_steps = list(result.get("path_steps", []))

    records: list[dict] = []
    for index, step in enumerate(path_steps):
        from_node = step.get("from")
        to_node = step.get("to")
        if not _is_real_node(from_node):
            continue

        # 计算本记录的转向动作：
        # 在从当前实际节点到下一个实际节点的整段 path_steps 中，
        # 优先找出第一处非直行的转向，并把它归属到当前实际节点上。
        turn_action = TURN_ACTION_STRAIGHT
        for look_idx in range(index, len(path_steps)):
            look_step = path_steps[look_idx]
            ta = _turn_action_from_headings(look_step.get("heading_in"), look_step.get("heading_out"))
            if ta != TURN_ACTION_STRAIGHT:
                turn_action = ta
                break

        edge_class = str(step.get("edge_class", ""))
        if edge_class == "to_derived" or _is_derived_node(to_node):
            pick_target = _extract_pick_target_from_derived_node(to_node)
        else:
            pick_target = PICK_ACTION_NONE

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


def format_path_step_records(records: list[dict]) -> str:
    """把逐步记录格式化成更容易阅读的多行文本。"""
    if not records:
        return "(empty)"

    lines: list[str] = []
    for index, record in enumerate(records, start=1):
        turn_name = _turn_action_name(int(record.get("turn_action", 0)))
        pick_target = int(record.get("pick_target", 0))
        pick_name = _pick_action_name(pick_target)
        lines.append(
            f"{index:02d}. {record.get('node')} -> {record.get('to')} | "
            f"turn={turn_name} | pick={pick_name}"
        )
    return "\n".join(lines)


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
        target = _extract_pick_target_from_derived_node(node_str)
        return f"{target}号衍生节点" if target > 0 else "衍生节点"
    return _node_display_name(node_str)


def format_action_chain(records: list[dict]) -> str:
    """把路径压成一条连续的动作链。"""
    if not records:
        return "(empty)"

    lines: list[str] = []
    step_no = 1
    for record in records:
        # 兼容不同 record 格式：优先使用 'from'，否则使用 'node'
        raw_from = record.get("from") if record.get("from") is not None else record.get("node")
        raw_to = record.get("to") if record.get("to") is not None else record.get("raw_to")

        from_node = _display_node_for_action(raw_from)
        to_node = _display_node_for_action(raw_to)

        turn_action = int(record.get("turn_action", 0))
        pick_target = int(record.get("pick_target", 0))
        edge_class = str(record.get("edge_class", ""))

        # 如果有取块动作，先打印取块再打印转向（用户期望先取块再转向）
        if edge_class == "to_derived" or _is_derived_node(raw_to):
            if pick_target > 0:
                lines.append(f"{step_no:02d}. 在 {from_node} 拾取 {pick_target} 号上的 R2 物块")
                step_no += 1

        # 有转向动作时输出转向（根据 turn_action 判断）
        # 但若起点是 start，则遵循 "start 面向 end" 的约定，不把后续转向计入 start
        if turn_action != TURN_ACTION_STRAIGHT and str(raw_from) != "start":
            turn_name = _turn_action_name(turn_action)
            lines.append(f"{step_no:02d}. 在 {from_node} 节点{turn_name}，面向 {to_node}")
            step_no += 1

        # 最后打印移动动作
        if edge_class == "to_derived" or _is_derived_node(raw_to):
            lines.append(f"{step_no:02d}. 从 {from_node} 节点走到 {to_node} 节点")
        else:
            lines.append(f"{step_no:02d}. 从 {from_node} 节点走到 {to_node} 节点")
        step_no += 1

    return "\n".join(lines)


def build_action_chain_records(result: dict) -> list[dict]:
    """按 plot 的边语义构造动作链记录：每条边对应一个动作。"""
    path_steps = list(result.get("path_steps", []))

    records: list[dict] = []
    for step in path_steps:
        from_node = step.get("from")
        to_node = step.get("to")
        edge_class = str(step.get("edge_class", ""))
        turn_action = _turn_action_from_headings(step.get("heading_in"), step.get("heading_out"))
        pick_target = _extract_pick_target_from_derived_node(to_node) if edge_class == "to_derived" or _is_derived_node(to_node) else PICK_ACTION_NONE
        records.append(
            {
                "from": str(from_node),
                "to": str(to_node),
                "turn_action": turn_action,
                "turn_cost": float(step.get("turn_cost", 0.0)),
                "pick_target": pick_target,
                "edge_class": edge_class,
            }
        )

    return records


def format_chronological_steps(result: dict) -> str:
    """按求解器原始顺序（path_steps）将动作按时间线打印：
    对于每个 step，若存在转向（turn_cost>0 或 heading 变化），先打印转向，
    然后打印该条边对应的动作（移动或取块）。
    这样严格遵循 plot 中的可视化顺序：节点圆圈=转向必须发生在随后的边动作之前。
    """
    path_steps = list(result.get("path_steps", []))
    if not path_steps:
        return "(empty)"

    lines: list[str] = []
    step_no = 1
    for step in path_steps:
        u = step.get("from")
        v = step.get("to")
        edge_class = str(step.get("edge_class", ""))
        turn_action = _turn_action_from_headings(step.get("heading_in"), step.get("heading_out"))
        turn_cost = float(step.get("turn_cost", 0.0))
        pick_target = _extract_pick_target_from_derived_node(v) if edge_class == "to_derived" or _is_derived_node(v) else PICK_ACTION_NONE

        from_name = _node_display_name(u)
        to_name = _display_node_for_action(v)

        # 转向先行（但保留 start 特例）
        if turn_cost > 0.0 and turn_action != TURN_ACTION_STRAIGHT and str(u) != "start":
            lines.append(f"{step_no:02d}. 在 {from_name} 节点{_turn_action_name(turn_action)}，面向 {to_name}")
            step_no += 1

        # 边动作：紫色（to_derived）视为取块动作（移动+取块），红色为普通移动
        if edge_class == "to_derived" or _is_derived_node(v):
            if pick_target > 0:
                lines.append(f"{step_no:02d}. 从 {from_name} 节点走到 {to_name} 并拾取 {pick_target} 号上的 R2 物块")
            else:
                lines.append(f"{step_no:02d}. 从 {from_name} 节点走到 {to_name}")
        else:
            lines.append(f"{step_no:02d}. 从 {from_name} 节点走到 {to_name} 节点")

        step_no += 1

    return "\n".join(lines)


def build_compact_path_records(result: dict) -> list[dict]:
    """把 full path 压缩成只包含实际节点的路径记录。

    每条记录对应一个实际节点，并带有：
    - turn_action：该实际节点离开时的转向动作
    - pick_target：该实际节点后续要取的物块编号，0 表示不取
    """
    path_nodes = list(result.get("path", []))
    path_steps = list(result.get("path_steps", []))

    records: list[dict] = []
    for index, node in enumerate(path_nodes):
        if not _is_real_node(node):
            continue

        step_info = path_steps[index] if index < len(path_steps) else None
        if step_info is not None:
            turn_action = _turn_action_from_headings(step_info.get("heading_in"), step_info.get("heading_out"))
        else:
            turn_action = TURN_ACTION_STRAIGHT

        pick_target = PICK_ACTION_NONE
        for look_ahead in range(index + 1, len(path_nodes)):
            next_node = path_nodes[look_ahead]
            if _is_real_node(next_node):
                break
            if _is_derived_node(next_node):
                pick_target = _extract_pick_target_from_derived_node(next_node)
                break

        records.append(
            {
                "node": str(node),
                "turn_action": turn_action,
                "pick_target": pick_target,
            }
        )

    return records


def encode_path_frame(path_nodes: list[str]) -> bytes:
    """把求解路径编码成适合下位机解析的二进制帧。"""
    if not path_nodes:
        return bytes([PATH_NODE_FRAME_HEADER, 0x00])

    node_bytes = bytearray()
    for node in path_nodes:
        node_bytes.append(_encode_path_node(node))

    if len(node_bytes) > 255:
        raise ValueError(f"路径过长，当前长度 {len(node_bytes)} 超过单帧上限 255")

    return bytes([PATH_NODE_FRAME_HEADER, len(node_bytes)]) + bytes(node_bytes)


def encode_path_action_frame(result: dict) -> bytes:
    """把 path_steps 及每步对应动作编码成二进制帧。"""
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
    """把完整最优路径按 path_steps 编码成二进制帧。

    帧格式：
    - 0xB7 | 节点数 N | [node_len, node_bytes, turn_action, pick_target] * N

    说明：
    - node：每一步的起点节点 from，使用长度前缀 UTF-8 编码
    - turn_action：该步离开当前节点时对应的转向动作
    - pick_target：如果该步进入衍生节点，则把衍生节点替换成取块动作，0 表示不取
    """
    records = build_path_step_records(result)

    if not records:
        return bytes([PATH_FULL_PATH_FRAME_HEADER, 0x00])

    payload = bytearray()
    for record in records:
        payload.extend(_encode_path_node_text(record["node"]))
        payload.append(record["turn_action"])
        payload.append(record["pick_target"])

    node_count = len(records)
    if node_count > 255:
        raise ValueError(f"路径过长，当前节点数 {node_count} 超过单帧上限 255")

    return bytes([PATH_FULL_PATH_FRAME_HEADER, node_count]) + bytes(payload)


def print_path_debug_info(result: dict) -> None:
    """只把求解出来的动作链打印到终端。"""
    # 使用原始的时间顺序（path_steps）打印：转向（节点圆圈）必须在随后边的移动/取块之前
    chronological = format_chronological_steps(result)
    if not chronological or chronological == "(empty)":
        print("[zone2_model_api] action_chain: (empty)")
        return

    print("[zone2_model_api] action_chain:")
    print(chronological)


def send_path_result_to_mcu(result: dict) -> None:
    """把最优路径、每个节点动作、以及转向代价通过串口发给下位机。"""
    assert ros_bridge_module.RosBridgeNodeInstance is not None, "RosBridgeNodeInstance is not initialized yet!"

    if not result.get("found"):
        ros_bridge_module.RosBridgeNodeInstance.writeBytes(bytes([PATH_FULL_PATH_FRAME_HEADER, 0x00]))
        print("[zone2_model_api] 没有找到可用路径，已发送空完整路径帧")
        return

    frame = encode_full_path_frame(result)
    ros_bridge_module.RosBridgeNodeInstance.writeBytes(frame)
    step_records = build_path_step_records(result)
    step_labels = [f"{r['node']}->{r['to']}" for r in step_records]
    # print("[zone2_model_api] path_steps_records:\n" + format_path_step_records(step_records))
    # print(f"[zone2_model_api] frame_hex={' '.join(f'{b:02X}' for b in frame)}")
    # print(
        # f"[zone2_model_api] 已发送完整路径到下位机: "
        # f"steps={step_labels}, "
        # f"step_count={len(result.get('path_steps', []))}, "
        # f"turn_cost={result.get('total_turn_cost', 0.0)}"
    # )


def visualize_path_result(result: dict, save_path: str = "/tmp/merlin_random_demo.png", show: bool = True) -> str:
    """把已求解的路径重新绘制成 GUI 页面。"""
    map_data = result.get("map_data")
    if map_data is None:
        raise ValueError("result 中缺少 map_data，无法绘制 GUI 页面")

    return draw_merlin_model(
        save_path=save_path,
        show=show,
        show_bidirectional_white_arrows=True,
        show_base_edges=True,
        show_optimal_path=True,
        show_turn_markers=True,
        map_data=map_data,
        path_result=result,
    )


class Zone2ModelAPI:
    """面向 main 的最小调用入口。"""

    MOVE_COST = MOVE_COST
    PICK_COST = PICK_COST
    R1_REMOVE_COST = R1_REMOVE_COST
    REQUIRED_R2_COUNT = REQUIRED_R2_COUNT
    TURN_COST = TURN_COST

    def get_merlin_map(self, force_refresh: bool = False, seed: Optional[int] = None) -> dict:
        return get_merlin_map(force_refresh=force_refresh, seed=seed)

    def dijkstra_min_cost_path(
        self,
        start: str = "start",
        end: str = "end",
        normal_cost: Optional[float] = None,
        to_derived_cost: Optional[float] = None,
        required_r2_count: Optional[int] = None,
        enforce_top_entry_after_one_pick: bool = True,
        turn_cost: Optional[float] = None,
        r1_remove_cost: Optional[float] = None,
        turn_free_rules: Optional[set[str]] = None,
        map_data: Optional[dict] = None,
    ) -> Dict[str, Any]:
        use_normal_cost = self.MOVE_COST if normal_cost is None else normal_cost
        use_to_derived_cost = self.PICK_COST if to_derived_cost is None else to_derived_cost
        use_required_r2_count = self.REQUIRED_R2_COUNT if required_r2_count is None else required_r2_count
        use_turn_cost = self.TURN_COST if turn_cost is None else turn_cost
        use_r1_remove_cost = self.R1_REMOVE_COST if r1_remove_cost is None else r1_remove_cost

        return dijkstra_min_cost_path(
            start=start,
            end=end,
            normal_cost=use_normal_cost,
            to_derived_cost=use_to_derived_cost,
            required_r2_count=use_required_r2_count,
            enforce_top_entry_after_one_pick=enforce_top_entry_after_one_pick,
            turn_cost=use_turn_cost,
            r1_remove_cost=use_r1_remove_cost,
            turn_free_rules=turn_free_rules,
            map_data=map_data,
        )

    def demo_visualize_random_map(
        self,
        seed: Optional[int] = None,
        save_path: str = "/tmp/merlin_random_demo.png",
        show: bool = True,
        move_cost: Optional[float] = None,
        pick_cost: Optional[float] = None,
        turn_cost: Optional[float] = None,
        r1_remove_cost: Optional[float] = None,
        required_r2_count: Optional[int] = None,
    ) -> Dict[str, Any]:
        """一键示例：随机生成地图、求解，并把地图和 path 一起画出来。"""
        map_data = self.get_merlin_map(force_refresh=True, seed=seed)
        result = self.dijkstra_min_cost_path(
            map_data=map_data,
            normal_cost=move_cost,
            to_derived_cost=pick_cost,
            turn_cost=turn_cost,
            r1_remove_cost=r1_remove_cost,
            required_r2_count=required_r2_count,
        )
        image_path = draw_merlin_model(
            save_path=save_path,
            show=show,
            show_bidirectional_white_arrows=True,
            show_base_edges=True,
            show_optimal_path=True,
            show_turn_markers=True,
            map_data=map_data,
            path_result=result,
        )
        result["map_data"] = map_data
        result["image_path"] = image_path
        return result

    def send_path_result_to_mcu(self, result: dict) -> None:
        send_path_result_to_mcu(result)

    def print_path_debug_info(self, result: dict) -> None:
        print_path_debug_info(result)

    def visualize_path_result(self, result: dict, save_path: str = "/tmp/merlin_random_demo.png", show: bool = True) -> str:
        return visualize_path_result(result, save_path=save_path, show=show)


zone2_model_api = Zone2ModelAPI()


__all__ = [
    "Zone2ModelAPI",
    "encode_path_frame",
    "encode_path_action_frame",
    "encode_full_path_frame",
    "visualize_path_result",
    "send_path_result_to_mcu",
    "zone2_model_api",
]
