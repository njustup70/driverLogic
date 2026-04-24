"""
Merlin 路径求解器（状态扩展 Dijkstra）
=====================================

一、求解目标
1) 在有向图上求 start -> end 的最小总代价路径。
2) 代价拆分为：
    - 基础边代价（移动/取块）
    - 转向代价（与基础边代价分离）

二、边分类（不依赖 plot 颜色字符串）
1) to_derived：src 非衍生节点，dst 为衍生节点（等价于“指向衍生节点”）。
2) normal：其余所有边。

三、核心状态与规则
1) 状态为 (node, r2_mask, heading)：
    - node：当前节点
    - r2_mask：已获取 R2 的位图集合
    - heading：当前朝向（up/down/left/right）
2) R2 获取规则：
    - 经过 to_derived 边，视为尝试获取该衍生节点对应的 target_r2。
    - 若该 R2 尚未获取，收取取块基础代价并置位。
    - 若该 R2 已获取，再次进入同 R2 的衍生节点，其“取块基础代价”为 0。
3) 转向规则：
    - 按相邻两步 heading 是否变化决定是否加 TURN_COST。
    - 若规则名在 turn_free_rules 中，则该步免转向代价。
4) 顶排约束（可开关）：
    - 若 1/2/3 中存在 R2，则进入 1/2/3 的 empty 前需先获取至少 1 个 R2。
5) 起始取块朝向约束：
    - start 及 owner=start 的衍生链路按“面向 end”处理，避免首段取块误计转向。

四、可供修改的参数
A. 全局默认参数（文件内常量）
1) MOVE_COST：normal 边基础代价（默认 1.0）
2) PICK_COST：to_derived 边基础代价（默认 2.0）
3) TURN_COST：转向代价（默认 0.5）
4) REQUIRED_R2_COUNT：到达 end 前至少获取的不同 R2 数量（默认 3）

B. dijkstra_min_cost_path(...) 调用参数
1) start / end：起终点（默认 start/end）
2) normal_cost / to_derived_cost：覆盖基础边代价
3) required_r2_count：到达 end 前至少获取的不同 R2 数量
4) enforce_top_entry_after_one_pick：是否启用顶排进入约束
5) turn_cost：覆盖转向代价
6) turn_free_rules：免转向规则集合（如爬坡类 rule）
7) map_data：外部地图数据注入

五、输出结果（关键字段）
1) cost：总最小代价
2) path / path_edges / path_steps：节点路径、边路径、分步明细
3) total_turn_cost / total_move_cost / total_pick_cost：分项代价
4) collected_r2 / collected_r2_count：已获取 R2 信息
"""

from typing import Dict, List, Optional, Tuple, Any, Set
import heapq

from MainLogic.core.zone2_model.merlin_model import build_merlin_model


ArrowClass = str
EdgeTuple = Tuple[str, str, str]
WeightedEdge = Tuple[str, str, float, str, ArrowClass]

# ===== 全局代价（可按业务直接修改） =====
# normal: 非“指向衍生节点”的边（对应 plot 中红色箭头性质）
# to_derived: 指向衍生节点的边（对应 plot 中紫色箭头性质）
# 这里先把它们作为“基础动作代价”使用，真正的转向代价在状态搜索中单独叠加。
MOVE_COST: float = 1.0   # 红色边基础代价
PICK_COST: float = 2.0   # 紫色边基础代价
TURN_COST: float = 0.5            # 转向代价（与边基础代价分离）
REQUIRED_R2_COUNT: int = 3        # 到达 end 前至少获取的不同 R2 数量（默认 3）


def _fixed_stake_pos() -> Dict[str, Tuple[float, float]]:
    """固定桩位坐标：四行三列 + start/end。"""
    pos: Dict[str, Tuple[float, float]] = {}
    for i in range(1, 13):
        r = (i - 1) // 3
        c = (i - 1) % 3
        pos[str(i)] = (float(c), float(3 - r))

    x2, y2 = pos["2"]
    x11, y11 = pos["11"]
    pos["start"] = (x2, y2 + 1.0)
    pos["end"] = (x11, y11 - 1.0)
    return pos


def _build_layout_positions(model: dict) -> Dict[str, Tuple[float, float]]:
    """用于转向判断的简化布局：基础桩位固定，衍生节点放在 owner 和 target 的中点。"""
    pos = _fixed_stake_pos()
    graph_nodes: Dict[str, dict] = model.get("graph_nodes", {})

    for node_id, meta in graph_nodes.items():
        if meta.get("kind") != "derived":
            continue
        owner = str(meta.get("owner", ""))
        target_r2 = str(meta.get("target_r2", ""))
        if owner not in pos or target_r2 not in pos:
            continue
        ox, oy = pos[owner]
        tx, ty = pos[target_r2]
        pos[str(node_id)] = ((ox + tx) / 2.0, (oy + ty) / 2.0)

    return pos


def _edge_heading(src: str, dst: str, pos: Dict[str, Tuple[float, float]]) -> str:
    """把一条边归约成 4 向朝向。"""
    sx, sy = pos[str(src)]
    dx, dy = pos[str(dst)]
    vx, vy = dx - sx, dy - sy
    if abs(vx) >= abs(vy):
        return "right" if vx >= 0 else "left"
    return "up" if vy >= 0 else "down"


def _is_end_facing_node(node_id: str) -> bool:
    """需要在该节点落点强制朝向 end 的节点。"""
    return str(node_id) in {"start", "1", "2", "3", "10", "11", "12", "end"}


def _node_heading_override(node_id: str, pos: Dict[str, Tuple[float, float]]) -> Optional[str]:
    """节点级朝向覆盖：这些节点在落点上统一按 end 方向处理。"""
    if _is_end_facing_node(node_id):
        return "down"
    return None


def classify_edge_by_arrow_property(src: str, dst: str, graph_nodes: Dict[str, dict]) -> ArrowClass:
	"""按箭头性质分类：
	- to_derived: 指向衍生节点（dst derived 且 src 非 derived）
	- normal: 其他
	"""
	src_meta = graph_nodes.get(str(src), {})
	dst_meta = graph_nodes.get(str(dst), {})
	src_is_derived = src_meta.get("kind") == "derived"
	dst_is_derived = dst_meta.get("kind") == "derived"

	if dst_is_derived and not src_is_derived:
		return "to_derived"
	return "normal"


def extract_edges_by_arrow_property(model: dict) -> Dict[str, List[EdgeTuple]]:
    """按箭头性质提取边：to_derived / normal。"""
    graph_nodes: Dict[str, dict] = model["graph_nodes"]
    to_derived_edges: List[EdgeTuple] = []
    normal_edges: List[EdgeTuple] = []

    for e in model["edges"]:
        src = str(e["from"])
        dst = str(e["to"])
        rule = str(e.get("rule", ""))
        edge_class = classify_edge_by_arrow_property(src, dst, graph_nodes)
        if edge_class == "to_derived":
            to_derived_edges.append((src, dst, rule))
        else:
            normal_edges.append((src, dst, rule))

    return {
        "to_derived": to_derived_edges,
        "normal": normal_edges,
    }


def build_weighted_edges(
    normal_cost: Optional[float] = None,
    to_derived_cost: Optional[float] = None,
    map_data: Optional[dict] = None,
) -> Dict[str, Any]:
    """
    按箭头性质映射代价。

    返回：
    - model: 原模型
    - to_derived_edges / normal_edges: 按箭头性质分组
    - weighted_edges: (src, dst, cost, rule, edge_class)
    """
    normal_cost = MOVE_COST if normal_cost is None else normal_cost
    to_derived_cost = PICK_COST if to_derived_cost is None else to_derived_cost

    model = build_merlin_model(map_data=map_data)
    graph_nodes: Dict[str, dict] = model["graph_nodes"]
    grouped = extract_edges_by_arrow_property(model)

    weighted_edges: List[WeightedEdge] = []
    for e in model["edges"]:
        src = str(e["from"])
        dst = str(e["to"])
        rule = str(e.get("rule", ""))
        edge_class = classify_edge_by_arrow_property(src, dst, graph_nodes)
        cost = to_derived_cost if edge_class == "to_derived" else normal_cost
        weighted_edges.append((src, dst, cost, rule, edge_class))

    return {
        "model": model,
        "to_derived_edges": grouped["to_derived"],
        "normal_edges": grouped["normal"],
        "weighted_edges": weighted_edges,
        "costs": {
            "normal": normal_cost,
            "to_derived": to_derived_cost,
        },
    }


def build_weighted_adjacency(
    normal_cost: Optional[float] = None,
    to_derived_cost: Optional[float] = None,
	map_data: Optional[dict] = None,
) -> Dict[str, List[Tuple[str, float, str, ArrowClass]]]:
	"""输出用于最短路的加权邻接表：node -> [(next, cost, rule, edge_class), ...]。"""
	data = build_weighted_edges(
		normal_cost=normal_cost,
		to_derived_cost=to_derived_cost,
		map_data=map_data,
	)
	adj: Dict[str, List[Tuple[str, float, str, ArrowClass]]] = {}

	for src, dst, cost, rule, edge_class in data["weighted_edges"]:
		adj.setdefault(src, []).append((dst, cost, rule, edge_class))

	return adj


# 兼容旧参数名（red/purple），但内部仍按“箭头性质”分类。
def build_weighted_edges_by_plot_semantic(
    red_cost: Optional[float] = None,
    purple_cost: Optional[float] = None,
	map_data: Optional[dict] = None,
) -> Dict[str, Any]:
	return build_weighted_edges(
		normal_cost=red_cost,
		to_derived_cost=purple_cost,
		map_data=map_data,
	)


def dijkstra_min_cost_path(
    start: str = "start",
    end: str = "end",
    normal_cost: Optional[float] = None,
    to_derived_cost: Optional[float] = None,
    required_r2_count: int = REQUIRED_R2_COUNT,
    enforce_top_entry_after_one_pick: bool = True,
    turn_cost: Optional[float] = None,
    turn_free_rules: Optional[Set[str]] = None,
    map_data: Optional[dict] = None,
) -> Dict[str, Any]:
    """
    在当前模型上用状态扩展 Dijkstra 计算 start->end 最小代价路径。

    规则：
    1) 经过“指向衍生节点”的边（to_derived）视为获取一次该衍生节点对应的 R2。
    2) 若该 R2 已经获取过，则再次经过指向同一 R2 的衍生边代价为 0。
    3) 仅当到达 end 且已获取的不同 R2 数量 >= required_r2_count 时，才算有效终点。
    4) 转向代价从边代价中独立出来，作为相邻两步之间的状态代价单独叠加。
    """
    turn_cost = TURN_COST if turn_cost is None else turn_cost
    turn_free_rules = set() if turn_free_rules is None else set(turn_free_rules)

    weighted = build_weighted_edges(
        normal_cost=normal_cost,
        to_derived_cost=to_derived_cost,
        map_data=map_data,
    )
    adjacency = build_weighted_adjacency(
        normal_cost=weighted["costs"]["normal"],
        to_derived_cost=weighted["costs"]["to_derived"],
        map_data=map_data,
    )

    start = str(start)
    end = str(end)
    required_r2_count = max(0, int(required_r2_count))

    model = weighted["model"]
    graph_nodes: Dict[str, dict] = model["graph_nodes"]
    stake_kinds: Dict[int, str] = model.get("stake_kinds", {})
    layout_pos = _build_layout_positions(model)

    # 统计模型中的所有 R2（由衍生节点 target_r2 定义）并构建位图索引
    r2_values = sorted(
        {
            str(meta.get("target_r2"))
            for meta in graph_nodes.values()
            if isinstance(meta, dict) and meta.get("kind") == "derived" and meta.get("target_r2") is not None
        }
    )
    r2_to_bit: Dict[str, int] = {r2: i for i, r2 in enumerate(r2_values)}
    max_collectable = len(r2_values)
    if required_r2_count > max_collectable:
        required_r2_count = max_collectable

    def _mask_count(mask: int) -> int:
        return mask.bit_count()

    # 业务约束：
    # 若 1/2/3 中存在 R2，则进入 1/2/3 中的 empty 节点前，必须已获取至少 1 个 R2。
    top_row_ids = (1, 2, 3)
    top_has_r2 = any(stake_kinds.get(i) == "R2" for i in top_row_ids)
    top_empty_targets: Set[str] = {str(i) for i in top_row_ids if stake_kinds.get(i) == "empty"}

    def _is_transition_allowed(v: str, current_mask: int) -> bool:
        if not enforce_top_entry_after_one_pick:
            return True
        if not top_has_r2:
            return True
        if v in top_empty_targets and _mask_count(current_mask) < 1:
            return False
        return True

    def _edge_step_cost_and_mask(
        u: str,
        v: str,
        edge_class: ArrowClass,
        current_mask: int,
        base_cost: float,
        prev_heading: Optional[str],
        rule: str,
    ) -> Tuple[float, int, str, float]:
        """返回在 current_mask 下经过边 u->v 的实际代价、新mask、当前朝向、转向代价。"""
        edge_heading = _edge_heading(u, v, layout_pos)
        node_override = _node_heading_override(v, layout_pos)
        if node_override is not None:
            edge_heading = node_override

        # start 顶排取块约束：
        # 从 start 出发取 1/2/3（含 start 的衍生节点链路）默认都视为朝向 end，
        # 避免把这段动作误计为转向。
        u_meta = graph_nodes.get(str(u), {})
        v_meta = graph_nodes.get(str(v), {})
        u_is_start_derived = u_meta.get("kind") == "derived" and str(u_meta.get("owner")) == "start"
        v_is_start_derived = v_meta.get("kind") == "derived" and str(v_meta.get("owner")) == "start"
        if str(u) == "start" or u_is_start_derived or v_is_start_derived:
            edge_heading = "down"

        step_turn_cost = 0.0
        if prev_heading is not None and prev_heading != edge_heading and rule not in turn_free_rules:
            step_turn_cost = turn_cost

        if edge_class != "to_derived":
            return base_cost + step_turn_cost, current_mask, edge_heading, step_turn_cost

        dst_meta = graph_nodes.get(str(v), {})
        target_r2 = dst_meta.get("target_r2")
        if target_r2 is None:
            return base_cost + step_turn_cost, current_mask, edge_heading, step_turn_cost

        r2_key = str(target_r2)
        bit = r2_to_bit.get(r2_key)
        if bit is None:
            return base_cost + step_turn_cost, current_mask, edge_heading, step_turn_cost

        bit_mask = 1 << bit
        if current_mask & bit_mask:
            # 同一 R2 已获取，再进入指向该 R2 的衍生节点代价为 0
            return step_turn_cost, current_mask, edge_heading, step_turn_cost

        return base_cost + step_turn_cost, (current_mask | bit_mask), edge_heading, step_turn_cost

    # 扩展状态：(node, r2_mask, heading)
    start_state = (start, 0, None)
    dist: Dict[Tuple[str, int], float] = {start_state: 0.0}
    # prev[(node, mask, heading)] = ((pre_node, pre_mask, pre_heading), step_cost, rule, edge_class, turn_cost)
    prev: Dict[Tuple[str, int, Optional[str]], Tuple[Tuple[str, int, Optional[str]], float, str, ArrowClass, float]] = {}

    pq: List[Tuple[float, str, int, Optional[str]]] = [(0.0, start, 0, None)]
    visited: set[Tuple[str, int, Optional[str]]] = set()

    best_end_state: Optional[Tuple[str, int, Optional[str]]] = None

    while pq:
        cur_cost, u, mask, heading = heapq.heappop(pq)
        state = (u, mask, heading)
        if state in visited:
            continue
        visited.add(state)

        if u == end and _mask_count(mask) >= required_r2_count:
            best_end_state = state
            break

        for v, base_w, rule, edge_class in adjacency.get(u, []):
            if base_w < 0:
                raise ValueError("Dijkstra 仅适用于非负权重，请检查代价设置")
            if not _is_transition_allowed(v, mask):
                continue
            step_cost, next_mask, next_heading, applied_turn = _edge_step_cost_and_mask(
                u, v, edge_class, mask, base_w, heading, rule
            )
            new_cost = cur_cost + step_cost
            next_state = (v, next_mask, next_heading)
            if next_state not in dist or new_cost < dist[next_state]:
                dist[next_state] = new_cost
                prev[next_state] = (state, step_cost, rule, edge_class, applied_turn)
                heapq.heappush(pq, (new_cost, v, next_mask, next_heading))

    if best_end_state is None:
        return {
            "found": False,
            "start": start,
            "end": end,
            "cost": float("inf"),
            "path": [],
            "path_edges": [],
            "path_steps": [],
            "costs": weighted["costs"],
            "required_r2_count": required_r2_count,
            "collected_r2_count": 0,
            "collected_r2": [],
            "total_turn_cost": 0.0,
            "total_move_cost": 0.0,
            "total_pick_cost": 0.0,
            "top_entry_constraint_active": bool(enforce_top_entry_after_one_pick and top_has_r2),
        }

    # 回溯路径
    end_node, end_mask, end_heading = best_end_state
    path_nodes: List[str] = [end_node]
    path_edges_rev: List[Tuple[str, str, float, str, ArrowClass]] = []
    path_steps_rev: List[Dict[str, Any]] = []
    cur_state = best_end_state
    while cur_state != start_state:
        pre_state, edge_cost, rule, edge_class, applied_turn = prev[cur_state]
        pre_node, _, pre_heading = pre_state
        cur_node, _, cur_heading = cur_state
        path_edges_rev.append((pre_node, cur_node, edge_cost, rule, edge_class))
        path_steps_rev.append({
            "from": pre_node,
            "to": cur_node,
            "step_cost": edge_cost,
            "base_cost": edge_cost - applied_turn,
            "turn_cost": applied_turn,
            "rule": rule,
            "edge_class": edge_class,
            "heading_in": pre_heading,
            "heading_out": cur_heading,
        })
        path_nodes.append(pre_node)
        cur_state = pre_state

    path_nodes.reverse()
    path_edges = list(reversed(path_edges_rev))
    path_steps = list(reversed(path_steps_rev))

    collected_r2 = [r2 for r2, bit in r2_to_bit.items() if end_mask & (1 << bit)]
    collected_r2.sort(key=lambda x: int(x) if str(x).isdigit() else x)

    total_turn_cost = sum(step["turn_cost"] for step in path_steps)
    total_base_cost = sum(step["base_cost"] for step in path_steps)
    total_pick_cost = 0.0
    for step in path_steps:
        if step["edge_class"] == "to_derived":
            total_pick_cost += step["base_cost"]
    total_move_cost = total_base_cost - total_pick_cost

    return {
        "found": True,
        "start": start,
        "end": end,
        "cost": dist[best_end_state],
        "path": path_nodes,
        "path_edges": path_edges,
        "path_steps": path_steps,
        "costs": weighted["costs"],
        "required_r2_count": required_r2_count,
        "collected_r2_count": len(collected_r2),
        "collected_r2": collected_r2,
        "total_turn_cost": total_turn_cost,
        "total_move_cost": total_move_cost,
        "total_pick_cost": total_pick_cost,
        "top_entry_constraint_active": bool(enforce_top_entry_after_one_pick and top_has_r2),
    }

