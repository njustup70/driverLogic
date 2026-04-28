"""
梅林模型建模逻辑总览（基于 merlin_map）
======================================

一、建模对象
1) 主区域桩位：1~12。
2) 门户桩位：start / end。
3) 桩位类型：empty / R2 / fake。

二、Step 1 基础边规则（仅 1~12）
1) empty 与相邻 empty 建立有向边（遍历后形成双向）。
2) R2 只允许指向相邻 empty（R2 -> empty）。
3) fake 完全隔离，不产生任何入边/出边。

三、Step 2 衍生节点规则
1) 若某来源桩 src 与 R2 相邻，不直接连到 R2，而是：
    src -> D_src_to_r2 -> r2。
2) 衍生节点继承 src 的邻接特性时，遵循“单向继承”：
    - 只继承 src 的出边方向（D_src_to_r2 -> x）；
    - 不继承反向入边（不会添加 x -> D_src_to_r2）。
3) 继承过滤：排除 r2 本身、背后节点、上方节点、fake 节点。
4) 同一 owner 若有多个衍生节点，则这些衍生节点两两双向互连。

四、start / end 硬约束
1) start 不能有入边；只允许向 1/2/3（或对应 R2 的 start 衍生节点）出边。
2) end 不能有出边；允许 10/11/12（非 fake）及其衍生节点指向 end。
3) 对 10/11/12 中非 fake 节点，强制补齐到 end 的入边。
4) 若 10/11/12 为非 fake，则它们所属衍生节点也继承到 end 的出边特性。
5) start 的衍生节点继承 start 到 1/2/3 中非 R2 非 fake 节点的出边特性。

五、R2 强制衍生补强
1) 对每个 R2，所有相邻且非 fake 的 1~12 桩都必须具备衍生链：
    src -> D_src_to_r2 -> r2。
2) 衍生继承仍保持“仅继承 src 出边方向”。

六、最终收敛
1) 再次执行 fake 隔离（删去 fake 相关全部边）。
2) 再次补强同 owner 衍生节点互连。
3) 输出 graph_nodes、edges、derived_by_owner 等结构供绘图与求解使用。

补充说明：地图中可生成 4 个 R2 物块，建模时会按实际 blocks 自动处理。
"""

from typing import Dict, List, Optional, Set, Tuple, Any
from MainLogic.core.zone2_model.merlin_map import get_merlin_map


def _node_kind(blocks: Dict[int, str], stake_id: int) -> str:
    t = blocks.get(stake_id)
    if t == "R2":
        return "R2"
    if t == "R1":
        return "R1"
    if t == "fake":
        return "fake"
    return "empty"


def _node_rc(node_id: int) -> Tuple[int, int]:
    """把 1~12 的节点编号转换成 4x3 网格行列坐标。"""
    idx = node_id - 1
    return idx // 3, idx % 3


def _opposite_neighbor(src: int, target: int) -> Optional[int]:
    """
    返回 src 相对 target 的“背后节点”。
    例如 5 -> 8 时，背后节点是 2。
    如果两点不是上下左右相邻，则返回 None。
    """
    sr, sc = _node_rc(src)
    tr, tc = _node_rc(target)
    dr, dc = tr - sr, tc - sc
    if abs(dr) + abs(dc) != 1:
        return None

    br, bc = sr - dr, sc - dc
    if 0 <= br < 4 and 0 <= bc < 3:
        return br * 3 + bc + 1
    return None


def _upper_neighbor(node_id: int) -> Optional[int]:
    """返回当前桩位正上方桩位（编号-3），若不存在则返回 None。"""
    up = node_id - 3
    if 1 <= up <= 12:
        return up
    return None


def build_merlin_model(map_data: Optional[dict] = None) -> dict:
    if map_data is None:
        map_data = get_merlin_map()

    adjacency: Dict[Any, List[Any]] = map_data["adjacency"]
    blocks: Dict[int, str] = map_data["blocks"]

    stakes = list(range(1, 13))  # 主区域
    portal_nodes = ["start", "end"]  # 也参与 Step2 的“来源节点”
    kinds = {s: _node_kind(blocks, s) for s in stakes}

    graph_nodes: Dict[str, dict] = {}
    for s in stakes:
        graph_nodes[str(s)] = {"kind": "stake", "stake_kind": kinds[s]}
    for p in portal_nodes:
        graph_nodes[p] = {"kind": "stake", "stake_kind": "portal"}

    edge_set: Set[Tuple[str, str, str]] = set()

    def add_edge(src: str, dst: str, rule: str) -> None:
        edge_set.add((src, dst, rule))

    def has_directed_edge(src: str, dst: str) -> bool:
        return any(esrc == src and edst == dst for esrc, edst, _ in edge_set)

    # Step 1（保持只在 1~12 内）
    for s in stakes:
        s_kind = kinds[s]
        neigh = [n for n in adjacency.get(s, []) if isinstance(n, int) and 1 <= n <= 12]

        if s_kind == "fake":
            # fake 孤立：不产生任何边
            continue

        if s_kind in {"empty", "R1"}:
            # empty 与 R1 拓扑上完全等价：都能与相邻的 empty/R1 建边
            for n in neigh:
                if kinds[n] in {"empty", "R1"}:
                    add_edge(str(s), str(n), "step1_empty_like_adjacent")

        elif s_kind == "R2":
            # R2 -> empty / R1
            for n in neigh:
                if kinds[n] in {"empty", "R1"}:
                    add_edge(str(s), str(n), "step1_R2_to_empty_like")



    # Step 2（仅允许 src 来自 1~12）
    derived_by_owner: Dict[str, List[str]] = {}
    modeled_sources = set(stakes)

    r2_stakes = [s for s in stakes if kinds[s] == "R2"]
    for r2 in r2_stakes:
        neigh = [n for n in adjacency.get(r2, []) if n in modeled_sources]

        for src in neigh:
            if isinstance(src, int) and kinds[src] == "fake":
                continue

            owner_key = str(src)
            dnode = f"D_{owner_key}_to_{r2}"
            graph_nodes[dnode] = {
                "kind": "derived",
                "owner": owner_key,
                "target_r2": r2,
                "owner_kind": (kinds[src] if isinstance(src, int) else "portal"),
            }

            # src -> derived -> r2
            add_edge(owner_key, dnode, "step2_owner_to_derived")
            add_edge(dnode, str(r2), "step2_derived_to_R2")
            derived_by_owner.setdefault(owner_key, []).append(dnode)

            # 继承 owner 的相邻特性（仅继承 owner 的“出边”方向）
            src_neigh = [x for x in adjacency.get(src, []) if x in modeled_sources]
            back_node = _opposite_neighbor(src, r2)
            up_node = _upper_neighbor(src)
            for x in src_neigh:
                if x == r2:
                    continue
                if back_node is not None and x == back_node:
                    continue
                if up_node is not None and x == up_node:
                    continue
                if isinstance(x, int) and kinds[x] == "fake":
                    continue
                if not has_directed_edge(owner_key, str(x)):
                    continue
                add_edge(dnode, str(x), "step2_derived_inherit_adj")

    # 同 owner 的多个衍生节点互连（双向）
    for owner, dnodes in derived_by_owner.items():
        if len(dnodes) >= 2:
            for i in range(len(dnodes)):
                for j in range(i + 1, len(dnodes)):
                    add_edge(dnodes[i], dnodes[j], "step2_derived_peer_link")
                    add_edge(dnodes[j], dnodes[i], "step2_derived_peer_link")

    # ---- start 特殊规则 ----
    # 初始基础节点共 14 个：1~12 + start + end。
    # start 只允许有 3 条向外连接（对应 1/2/3）：
    # - 若目标是 empty：start -> 目标
    # - 若目标是 R2：start -> D_start_to_x -> 目标
    for t in (1, 2, 3):
        if kinds[t] == "fake":
            continue

        if kinds[t] == "R2":
            dnode = f"D_start_to_{t}"
            if dnode not in graph_nodes:
                graph_nodes[dnode] = {
                    "kind": "derived",
                    "owner": "start",
                    "target_r2": t,
                    "owner_kind": "portal",
                }
            add_edge("start", dnode, "start_to_derived_for_r2")
            add_edge(dnode, str(t), "derived_to_r2")
            derived_by_owner.setdefault("start", []).append(dnode)
        else:
            add_edge("start", str(t), "start_direct_to_adjacent")

    # start 衍生节点继承 start 的单向出边特性：
    # D_start_to_r2 -> {1,2,3 中非 R2 且非 fake 的节点}
    start_inherit_targets = [
        str(t)
        for t in (1, 2, 3)
        if kinds[t] != "R2" and kinds[t] != "fake" and has_directed_edge("start", str(t))
    ]
    for dnode in sorted(set(derived_by_owner.get("start", []))):
        for t in start_inherit_targets:
            add_edge(dnode, t, "start_derived_inherit_adj")

    # ---- start/end 硬约束过滤（最终兜底）----
    # start：
    # 1) 不能有入边
    # 2) 只能有 3 条对外连接（对应 1/2/3 的直连或到衍生节点）
    allowed_start_out_targets = set()
    for t in (1, 2, 3):
        if kinds[t] == "R2":
            allowed_start_out_targets.add(f"D_start_to_{t}")
        else:
            allowed_start_out_targets.add(str(t))

    filtered_after_start: Set[Tuple[str, str, str]] = set()
    for src, dst, rule in edge_set:
        # start 不允许任何入边
        if dst == "start":
            continue
        # start 只允许指向 1/2/3（或对应衍生节点）
        if src == "start" and dst not in allowed_start_out_targets:
            continue
        filtered_after_start.add((src, dst, rule))
    edge_set = filtered_after_start

    # end：
    # 1) end 不允许任何出边
    # 2) 只允许 10/11/12（非 fake）及其衍生节点指向 end
    # 3) 若其中有 fake，则 fake 本体不能连 end，最终为 2 或 3 条本体入边
    filtered_end: Set[Tuple[str, str, str]] = set()
    for src, dst, rule in edge_set:
        if src == "end":
            continue
        if dst == "end":
            if src not in {"10", "11", "12"}:
                continue
            src_id = int(src)
            if kinds[src_id] == "fake":
                continue
        filtered_end.add((src, dst, rule))
    edge_set = filtered_end

    # 强制补齐 end 入边：10/11/12 中非 fake 的节点必须直连 end
    for n in (10, 11, 12):
        if kinds[n] != "fake":
            add_edge(str(n), "end", "forced_to_end")

    # ---- R2 衍生节点强制规则（在高优先级约束之后执行）----
    # 若某个桩是 R2，则其相邻的所有“非 fake 梅林桩(1~12)”都必须衍生节点并指向该 R2。
    # start 的情况已由 start 特殊规则处理；end 不允许出边，因此不参与该规则。
    for r2 in r2_stakes:
        for src in adjacency.get(r2, []):
            if not isinstance(src, int):
                continue
            if not (1 <= src <= 12):
                continue
            if kinds[src] == "fake":
                continue

            back_node = _opposite_neighbor(src, r2)
            up_node = _upper_neighbor(src)

            dnode = f"D_{src}_to_{r2}"
            if dnode not in graph_nodes:
                graph_nodes[dnode] = {
                    "kind": "derived",
                    "owner": str(src),
                    "target_r2": r2,
                    "owner_kind": kinds[src],
                }
            add_edge(str(src), dnode, "step2_owner_to_derived")
            add_edge(dnode, str(r2), "step2_derived_to_R2")
            derived_by_owner.setdefault(str(src), []).append(dnode)

            # 继承 src 的相邻特性时，排除背后节点（仅继承 src 的出边方向）
            for x in adjacency.get(src, []):
                if not isinstance(x, int):
                    continue
                if not (1 <= x <= 12):
                    continue
                if x == r2:
                    continue
                if back_node is not None and x == back_node:
                    continue
                if up_node is not None and x == up_node:
                    continue
                if kinds[x] == "fake":
                    continue
                if not has_directed_edge(str(src), str(x)):
                    continue
                add_edge(dnode, str(x), "step2_derived_inherit_adj")


    # 若 10/11/12 为非 fake，则其衍生节点也可直接指向 end。
    for owner in ("10", "11", "12"):
        if kinds[int(owner)] == "fake":
            continue
        for dnode in sorted(set(derived_by_owner.get(owner, []))):
            add_edge(dnode, "end", "derived_owner_to_end")

    # ---- fake 最高优先级隔离 ----
    # fake 物块独立：不允许任何输入/输出边。
    fake_nodes = {str(i) for i in stakes if kinds[i] == "fake"}
    if fake_nodes:
        edge_set = {
            (src, dst, rule)
            for (src, dst, rule) in edge_set
            if src not in fake_nodes and dst not in fake_nodes
        }

    # ---- 同一节点的多个衍生节点互连（最终补强）----
    # 若某个 owner 拥有 >=2 个衍生节点，则这些衍生节点两两双向连接。
    for owner, dnodes in derived_by_owner.items():
        uniq_dnodes = sorted(set(dnodes))
        if len(uniq_dnodes) < 2:
            continue
        for i in range(len(uniq_dnodes)):
            for j in range(i + 1, len(uniq_dnodes)):
                add_edge(uniq_dnodes[i], uniq_dnodes[j], "step2_derived_peer_link")
                add_edge(uniq_dnodes[j], uniq_dnodes[i], "step2_derived_peer_link")

    edges = [
        {"from": src, "to": dst, "rule": rule}
        for (src, dst, rule) in sorted(edge_set, key=lambda x: (x[0], x[1], x[2]))
    ]

    return {
        "name": "merlin_model",
        "initial_node_count": 14,
        "stake_kinds": kinds,
        "graph_nodes": graph_nodes,
        "edges": edges,
        "derived_by_owner": derived_by_owner,
    }


def render_merlin_model(model: dict) -> str:
    lines: List[str] = []
    lines.append("========== MERLIN MODEL ==========")
    lines.append("stake kinds:")
    for s in range(1, 13):
        lines.append(f"  {s}: {model['stake_kinds'][s]}")

    lines.append("")
    lines.append("derived nodes by owner:")
    if model["derived_by_owner"]:
        def _owner_sort_key(item):
            k = item[0]
            return (0, int(k)) if k.isdigit() else (1, k)
        for owner, dnodes in sorted(model["derived_by_owner"].items(), key=_owner_sort_key):
            lines.append(f"  {owner} -> {dnodes}")
    else:
        lines.append("  (none)")

    lines.append("")
    lines.append("edges:")
    for e in model["edges"]:
        lines.append(f"  {e['from']} -> {e['to']}  [{e['rule']}]")

    lines.append("==================================")
    return "\n".join(lines)


def print_merlin_model(model: Optional[dict] = None) -> None:
    if model is None:
        model = build_merlin_model()
    print(render_merlin_model(model))


def main() -> None:
    model = build_merlin_model()
    print_merlin_model(model)


if __name__ == "__main__":
    main()