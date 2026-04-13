"""
基于 merlin_model 的“箭头性质”代价提取。

这里不依赖 plot 颜色本身，而是使用与 plot 一致的箭头性质定义：
1) 指向衍生节点的箭头：dst 是 derived 且 src 不是 derived。
2) 其他箭头：除上面以外的全部边（基础边 + 从衍生节点发出的边等）。

可把这两类箭头映射为两个代价，用于路径求解。
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
TURN_AND_MOVE_COST: float = 1.0   # 红色：转身 + 移动 代价
TURN_AND_PICK_COST: float = 2.0   # 紫色：转身 + 取块 代价


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
    normal_cost = TURN_AND_MOVE_COST if normal_cost is None else normal_cost
    to_derived_cost = TURN_AND_PICK_COST if to_derived_cost is None else to_derived_cost

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
    required_r2_count: int = 2,
    enforce_top_entry_after_one_pick: bool = True,
    map_data: Optional[dict] = None,
) -> Dict[str, Any]:
    """
    在当前模型上用状态扩展 Dijkstra 计算 start->end 最小代价路径。

    规则：
    1) 经过“指向衍生节点”的边（to_derived）视为获取一次该衍生节点对应的 R2。
    2) 若该 R2 已经获取过，则再次经过指向同一 R2 的衍生边代价为 0。
    3) 仅当到达 end 且已获取的不同 R2 数量 >= required_r2_count 时，才算有效终点。
    """
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

    def _edge_step_cost_and_mask(u: str, v: str, edge_class: ArrowClass, current_mask: int, base_cost: float) -> Tuple[float, int]:
        """返回在 current_mask 下经过边 u->v 的实际代价与新mask。"""
        if edge_class != "to_derived":
            return base_cost, current_mask

        dst_meta = graph_nodes.get(str(v), {})
        target_r2 = dst_meta.get("target_r2")
        if target_r2 is None:
            return base_cost, current_mask

        r2_key = str(target_r2)
        bit = r2_to_bit.get(r2_key)
        if bit is None:
            return base_cost, current_mask

        bit_mask = 1 << bit
        if current_mask & bit_mask:
            # 同一 R2 已获取，再进入指向该 R2 的衍生节点代价为 0
            return 0.0, current_mask

        return base_cost, (current_mask | bit_mask)

    # 扩展状态：(node, r2_mask)
    start_state = (start, 0)
    dist: Dict[Tuple[str, int], float] = {start_state: 0.0}
    # prev[(node, mask)] = ((pre_node, pre_mask), edge_cost, rule, edge_class)
    prev: Dict[Tuple[str, int], Tuple[Tuple[str, int], float, str, ArrowClass]] = {}

    pq: List[Tuple[float, str, int]] = [(0.0, start, 0)]
    visited: set[Tuple[str, int]] = set()

    best_end_state: Optional[Tuple[str, int]] = None

    while pq:
        cur_cost, u, mask = heapq.heappop(pq)
        state = (u, mask)
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
            step_cost, next_mask = _edge_step_cost_and_mask(u, v, edge_class, mask, base_w)
            new_cost = cur_cost + step_cost
            next_state = (v, next_mask)
            if next_state not in dist or new_cost < dist[next_state]:
                dist[next_state] = new_cost
                prev[next_state] = (state, step_cost, rule, edge_class)
                heapq.heappush(pq, (new_cost, v, next_mask))

    if best_end_state is None:
        return {
            "found": False,
            "start": start,
            "end": end,
            "cost": float("inf"),
            "path": [],
            "path_edges": [],
            "costs": weighted["costs"],
            "required_r2_count": required_r2_count,
            "collected_r2_count": 0,
            "collected_r2": [],
            "top_entry_constraint_active": bool(enforce_top_entry_after_one_pick and top_has_r2),
        }

    # 回溯路径
    end_node, end_mask = best_end_state
    path_nodes: List[str] = [end_node]
    path_edges_rev: List[Tuple[str, str, float, str, ArrowClass]] = []
    cur_state = best_end_state
    while cur_state != start_state:
        pre_state, edge_cost, rule, edge_class = prev[cur_state]
        pre_node, _ = pre_state
        cur_node, _ = cur_state
        path_edges_rev.append((pre_node, cur_node, edge_cost, rule, edge_class))
        path_nodes.append(pre_node)
        cur_state = pre_state

    path_nodes.reverse()
    path_edges = list(reversed(path_edges_rev))

    collected_r2 = [r2 for r2, bit in r2_to_bit.items() if end_mask & (1 << bit)]
    collected_r2.sort(key=lambda x: int(x) if str(x).isdigit() else x)

    return {
        "found": True,
        "start": start,
        "end": end,
        "cost": dist[best_end_state],
        "path": path_nodes,
        "path_edges": path_edges,
        "costs": weighted["costs"],
        "required_r2_count": required_r2_count,
        "collected_r2_count": len(collected_r2),
        "collected_r2": collected_r2,
        "top_entry_constraint_active": bool(enforce_top_entry_after_one_pick and top_has_r2),
    }

