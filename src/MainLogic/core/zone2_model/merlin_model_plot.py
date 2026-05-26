"""
根据 merlin_model 生成有向图（规则化布局）
- 1~12 固定为 4x3 网格
- start 在 2 号正上方
- end 在 11 号正下方
- 衍生节点大小为桩节点的一半
"""

from typing import Dict, Tuple, List, Set, Any
from MainLogic.core.zone2_model.merlin_model import build_merlin_model
from MainLogic.core.zone2_model.path_solver import dijkstra_min_cost_path
import math


TOP_STAKES_FOR_ARROW_OVERLAY: Set[str] = {"1", "2", "3"}


def _direction_vector_from_local(
    direction: str,
    ux: float,
    uy: float,
    nx: float,
    ny: float,
) -> Tuple[float, float]:
    d = str(direction).lower()
    if d == "left":
        return nx, ny
    if d == "right":
        return -nx, -ny
    if d == "up":
        return ux, uy
    if d == "down":
        return -ux, -uy
    # 默认 left，保证结果稳定
    return nx, ny


def _lateral_vector_for_local_direction(
    direction: str,
    ux: float,
    uy: float,
    nx: float,
    ny: float,
) -> Tuple[float, float]:
    d = str(direction).lower()
    # left/right 以法向为主，切向做微调；up/down 以切向为主，法向做微调
    if d in {"left", "right"}:
        return ux, uy
    return nx, ny


def _fixed_stake_pos() -> Dict[str, Tuple[float, float]]:
    """固定桩位坐标：四行三列 + start/end"""
    pos: Dict[str, Tuple[float, float]] = {}

    # 1~12：四行三列（从上到下 y=3,2,1,0；从左到右 x=0,1,2）
    for i in range(1, 13):
        r = (i - 1) // 3
        c = (i - 1) % 3
        x = float(c)
        y = float(3 - r)
        pos[str(i)] = (x, y)

    # start 在 2 正上方；end 在 11 正下方
    x2, y2 = pos["2"]
    x11, y11 = pos["11"]
    pos["start"] = (x2, y2 + 1.0)
    pos["end"] = (x11, y11 - 1.0)

    return pos


def _enforce_min_distance(
    pos: Dict[str, Tuple[float, float]],
    fixed_nodes: Set[str],
    min_dist: float = 0.22,
    iterations: int = 160,
    derived_side_constraints: Dict[str, Tuple[float, float, float, float, float]] | None = None,
) -> Dict[str, Tuple[float, float]]:
    """
    简单的最小距离迭代：
    - fixed_nodes 不移动（1~12, start, end）
    - 其他节点自动避让
    """
    keys = list(pos.keys())

    def _project_if_needed(node: str) -> None:
        if not derived_side_constraints:
            return
        c = derived_side_constraints.get(node)
        if not c:
            return
        mx, my, bx, by, eps = c
        px, py = pos[node]
        dot = (px - mx) * bx + (py - my) * by
        if dot < eps:
            # 投影回约束半平面，避免跨侧
            corr = (eps - dot)
            pos[node] = (px + bx * corr, py + by * corr)

    for _ in range(iterations):
        moved = False
        for i in range(len(keys)):
            a = keys[i]
            ax, ay = pos[a]
            for j in range(i + 1, len(keys)):
                b = keys[j]
                bx, by = pos[b]

                dx, dy = bx - ax, by - ay
                dist = math.hypot(dx, dy)

                if dist == 0:
                    dx, dy = 1e-6, 0.0
                    dist = 1e-6

                if dist < min_dist:
                    overlap = min_dist - dist
                    ux, uy = dx / dist, dy / dist

                    a_fixed = a in fixed_nodes
                    b_fixed = b in fixed_nodes

                    if a_fixed and b_fixed:
                        continue
                    elif a_fixed and not b_fixed:
                        pos[b] = (bx + ux * overlap, by + uy * overlap)
                        _project_if_needed(b)
                    elif b_fixed and not a_fixed:
                        pos[a] = (ax - ux * overlap, ay - uy * overlap)
                        _project_if_needed(a)
                    else:
                        half = overlap * 0.5
                        pos[a] = (ax - ux * half, ay - uy * half)
                        pos[b] = (bx + ux * half, by + uy * half)
                        _project_if_needed(a)
                        _project_if_needed(b)
                    moved = True

        if not moved:
            break

    return pos


def _point_to_segment_distance(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    abx, aby = bx - ax, by - ay
    apx, apy = px - ax, py - ay
    ab2 = abx * abx + aby * aby
    if ab2 == 0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, (apx * abx + apy * aby) / ab2))
    cx, cy = ax + t * abx, ay + t * aby
    return math.hypot(px - cx, py - cy)


def _place_derived_nodes_avoiding_edges(
    pos: Dict[str, Tuple[float, float]],
    graph_nodes: Dict[str, dict],
    edges: List[dict],
    stakes_fixed: Set[str],
) -> Tuple[Dict[str, Tuple[float, float]], Dict[str, Tuple[float, float, float, float, float]]]:
    """
    给衍生节点选位置：不压到其他边段上（尽量远离）
    """
    # 先收集“基础边段”（不包含任何衍生节点参与的边）
    base_segments: List[Tuple[float, float, float, float]] = []
    # 衍生节点侧约束：node -> (mx, my, bx, by, eps)
    # 要求 (p-m)·b >= eps，保证最小间距后处理不跨侧
    derived_side_constraints: Dict[str, Tuple[float, float, float, float, float]] = {}
    for e in edges:
        u, v = str(e["from"]), str(e["to"])
        if u not in pos or v not in pos:
            continue
        mu = graph_nodes.get(u, {})
        mv = graph_nodes.get(v, {})
        if mu.get("kind") == "derived" or mv.get("kind") == "derived":
            continue
        ux, uy = pos[u]
        vx, vy = pos[v]
        base_segments.append((ux, uy, vx, vy))

    # 按 owner 分组放置
    owner_to_derived: Dict[str, List[str]] = {}
    for nid, meta in graph_nodes.items():
        if meta.get("kind") == "derived":
            owner_to_derived.setdefault(str(meta.get("owner")), []).append(str(nid))

    for owner, dnodes in owner_to_derived.items():
        for idx, dn in enumerate(sorted(dnodes)):
            meta = graph_nodes[dn]
            src = str(meta.get("owner"))
            dst = str(meta.get("target_r2"))
            if src not in pos or dst not in pos:
                continue

            sx, sy = pos[src]
            tx, ty = pos[dst]
            mx, my = (sx + tx) / 2.0, (sy + ty) / 2.0
            dx, dy = tx - sx, ty - sy
            norm = math.hypot(dx, dy) or 1.0
            ux, uy = dx / norm, dy / norm
            nx, ny = -uy, ux  # 法向量

            bx, by = _direction_vector_from_local("left", ux, uy, nx, ny)
            lx, ly = _lateral_vector_for_local_direction("left", ux, uy, nx, ny)

            # 候选点：优先落在局部方向指定的一侧，层级决定离中点的距离
            layer = idx + 1
            base_offset = 0.16 * layer

            candidates: List[Tuple[float, float]] = []
            # 主候选：严格落在目标半平面
            candidates.append((mx + bx * base_offset, my + by * base_offset))
            candidates.append((mx + bx * (base_offset + 0.08), my + by * (base_offset + 0.08)))
            candidates.append((mx + bx * (base_offset + 0.16), my + by * (base_offset + 0.16)))

            # 备选：沿横向轻微微调，避免重叠到边上
            for shift in (-0.08, 0.08):
                candidates.append((mx + bx * base_offset + lx * shift, my + by * base_offset + ly * shift))
                candidates.append((mx + bx * (base_offset + 0.08) + lx * shift, my + by * (base_offset + 0.08) + ly * shift))

            # 选分数最高：离基础边越远越好、离其它已放衍生节点越远越好
            best_p = (mx, my)
            best_score = -1e9
            for cx, cy in candidates:
                side_dot = (cx - mx) * bx + (cy - my) * by
                if side_dot <= 0.0:
                    # 硬约束：不允许落在目标侧反面
                    continue

                min_edge_dist = 1e9
                for ax, ay, sx2, sy2 in base_segments:
                    d = _point_to_segment_distance(cx, cy, ax, ay, sx2, sy2)
                    if d < min_edge_dist:
                        min_edge_dist = d

                min_node_dist = 1e9
                for k, (px, py) in pos.items():
                    if k == src or k == dst:
                        continue
                    d = math.hypot(cx - px, cy - py)
                    if d < min_node_dist:
                        min_node_dist = d

                # 优先远离边，其次远离节点
                score = min_edge_dist * 3.0 + min_node_dist + side_dot * 0.2
                if score > best_score:
                    best_score = score
                    best_p = (cx, cy)

            pos[dn] = best_p
            derived_side_constraints[dn] = (mx, my, bx, by, 0.02)

    return pos, derived_side_constraints


def _bidirectional_edge_pairs(g) -> Set[Tuple[str, str]]:
    """找出图中互为反向的边对，返回需要单独绘制的有向边集合。"""
    bidirectional_edges: Set[Tuple[str, str]] = set()
    for u, v in g.edges():
        if u == v:
            continue
        if g.has_edge(v, u):
            bidirectional_edges.add((u, v))
    return bidirectional_edges


def _edge_connectionstyle(u: str, v: str, data: dict, graph_nodes: Dict[str, dict], pos: Dict[str, Tuple[float, float]]) -> str:
    """为单条边返回合适的弧度，尽量绕开衍生节点。"""
    rule = str(data.get("rule", ""))
    u_meta = graph_nodes.get(str(u), {})
    v_meta = graph_nodes.get(str(v), {})
    u_is_derived = u_meta.get("kind") == "derived"
    v_is_derived = v_meta.get("kind") == "derived"

    # 衍生边：优先使用更大的弧度，避免压在节点上
    if u_is_derived or v_is_derived:
        if rule == "step2_derived_inherit_adj":
            derived_node = str(u) if u_is_derived else str(v)
            derived_meta = graph_nodes.get(derived_node, {})
            owner = str(derived_meta.get("owner", ""))
            target = str(derived_meta.get("target_r2", ""))
            return "arc3,rad=0.0"

        # owner -> derived / derived -> R2
        if rule in {"step2_owner_to_derived", "step2_derived_to_R2"}:
            return "arc3,rad=0.0"

        return "arc3,rad=0.0"

    # 普通边稍微弯一点即可
    return "arc3,rad=0.0"


def _edge_color_by_direction(u: str, v: str, graph_nodes: Dict[str, dict]) -> str:
    """边颜色规则：
    - 基础边与从衍生节点发出的箭头：红色
    - 指向衍生节点的箭头：紫色
    """
    u_meta = graph_nodes.get(str(u), {})
    v_meta = graph_nodes.get(str(v), {})
    u_is_derived = u_meta.get("kind") == "derived"
    v_is_derived = v_meta.get("kind") == "derived"

    if v_is_derived and not u_is_derived:
        return "#7B2CBF"  # 紫色：指向衍生节点
    return "#D00000"      # 红色：基础边 + 从衍生节点发出


def _edge_zorder_for_target(v: str, graph_nodes: Dict[str, dict]) -> float:
    """指向 1/2/3 号桩的箭头放到桩层之上。"""
    v_str = str(v)
    v_meta = graph_nodes.get(v_str, {})
    if v_str in TOP_STAKES_FOR_ARROW_OVERLAY and v_meta.get("kind") == "stake":
        return 5.0
    return 2.0


def _set_edge_artists_zorder(artists: Any, z: float) -> None:
    if artists is None:
        return
    if isinstance(artists, list):
        for a in artists:
            if hasattr(a, "set_zorder"):
                a.set_zorder(z)
        return
    if hasattr(artists, "set_zorder"):
        artists.set_zorder(z)


def _signed_bidir_connectionstyle(u: str, v: str) -> str:
    """双向边使用相反弧度，避免重叠遮挡箭头。"""
    rad = 0.12 if str(u) < str(v) else -0.12
    return f"arc3,rad={rad}"


def draw_merlin_model(
    save_path: str = "/tmp/merlin_model.png",
    show: bool = False,
    show_bidirectional_white_arrows: bool = False,
    show_base_edges: bool = False,
    show_optimal_path: bool = True,
    show_turn_markers: bool = True,
    optimal_path_width: float = 3.0,
    map_data: dict | None = None,
    path_result: dict | None = None,
) -> str:
    try:
        import networkx as nx  # type: ignore[reportMissingImports]
        import matplotlib.pyplot as plt  # type: ignore[reportMissingImports]
    except ImportError as e:
        raise RuntimeError("缺少依赖，请先安装: pip3 install networkx matplotlib") from e

    model = build_merlin_model(map_data=map_data)
    edges = model["edges"]
    graph_nodes = model["graph_nodes"]

    g = nx.DiGraph()
    for node_id, meta in graph_nodes.items():
        g.add_node(str(node_id), **meta)
    for e in edges:
        g.add_edge(str(e["from"]), str(e["to"]), rule=e["rule"])

    # 1) 固定桩位
    pos = _fixed_stake_pos()

    # 2) 放置衍生节点（避开已有边）
    pos, derived_side_constraints = _place_derived_nodes_avoiding_edges(
        pos=pos,
        graph_nodes=graph_nodes,
        edges=edges,
        stakes_fixed={str(i) for i in range(1, 13)} | {"start", "end"},
    )

    # 3) 再做一次最小间距约束（可选）
    pos = _enforce_min_distance(
        pos,
        fixed_nodes={str(i) for i in range(1, 13)} | {"start", "end"},
        min_dist=0.24,
        iterations=200,
        derived_side_constraints=derived_side_constraints,
    )

    # 4) 颜色和大小映射
    node_color_map: Dict[str, str] = {}
    node_sizes: List[int] = []

    for n in g.nodes():
        meta = graph_nodes.get(str(n), {})
        kind = meta.get("kind")
        stake_kind = meta.get("stake_kind")

        if kind == "derived":
            node_color_map[str(n)] = "#FFD166"   # 衍生节点
            node_sizes.append(600)               # 一半大小
        elif str(n) == "start":
            node_color_map[str(n)] = "#4CC9F0"
            node_sizes.append(1200)
        elif str(n) == "end":
            node_color_map[str(n)] = "#F72585"
            node_sizes.append(1200)
        elif stake_kind == "R2":
            node_color_map[str(n)] = "#90BE6D"
            node_sizes.append(1200)
        elif stake_kind == "R1":
            node_color_map[str(n)] = "#FF0000"   # R1 物块：红色
            node_sizes.append(1200)
        elif stake_kind == "fake":
            node_color_map[str(n)] = "#6C757D"
            node_sizes.append(1200)
        else:
            node_color_map[str(n)] = "#DEE2E6"   # empty
            node_sizes.append(1200)

    plt.figure(figsize=(9, 10))
    nx.draw_networkx_nodes(
        g,
        pos,
        node_color=[node_color_map[n] for n in g.nodes()],
        node_size=node_sizes,
        linewidths=1.2,
        edgecolors="#333",
    )
    nx.draw_networkx_labels(g, pos, font_size=8)

    bidirectional_edges = _bidirectional_edge_pairs(g)

    derived_edges = []
    normal_edges = []
    for u, v, data in g.edges(data=True):
        if graph_nodes.get(str(u), {}).get("kind") == "derived" or graph_nodes.get(str(v), {}).get("kind") == "derived":
            derived_edges.append((u, v, data))
        else:
            normal_edges.append((u, v, data))

    # 普通边：统一细线样式（仅使用红/紫两种颜色）
    if show_base_edges and normal_edges:
        for u, v, data in normal_edges:
            base_color = _edge_color_by_direction(str(u), str(v), graph_nodes)
            edge_z = _edge_zorder_for_target(str(v), graph_nodes)
            connectionstyle = (
                _signed_bidir_connectionstyle(str(u), str(v))
                if show_bidirectional_white_arrows and (u, v) in bidirectional_edges
                else "arc3,rad=0.0"
            )
            artists = nx.draw_networkx_edges(
                g,
                pos,
                edgelist=[(u, v)],
                arrowstyle="->",
                arrows=True,
                arrowsize=11,
                width=1.0,
                edge_color=[base_color],
                alpha=0.95,
                connectionstyle=connectionstyle,
            )
            _set_edge_artists_zorder(artists, edge_z)

    # 衍生相关边：统一细线样式（与基础红线一致，仅颜色由规则区分红/紫）
    for u, v, data in derived_edges:
        edge_z = _edge_zorder_for_target(str(v), graph_nodes)
        base_cs = _edge_connectionstyle(str(u), str(v), data, graph_nodes, pos)
        connectionstyle = (
            _signed_bidir_connectionstyle(str(u), str(v))
            if show_bidirectional_white_arrows and (u, v) in bidirectional_edges
            else base_cs
        )
        artists = nx.draw_networkx_edges(
            g,
            pos,
            edgelist=[(u, v)],
            arrowstyle="->",
            arrows=True,
            arrowsize=11,
            width=1.0,
            edge_color=[_edge_color_by_direction(str(u), str(v), graph_nodes)],
            alpha=0.95,
            connectionstyle=connectionstyle,
        )
        _set_edge_artists_zorder(artists, edge_z)

    # 最优路径叠加高亮（Dijkstra: start -> end）
    if show_optimal_path:
        if path_result is None:
            path_result = dijkstra_min_cost_path(start="start", end="end", map_data=map_data)
        if path_result.get("found"):
            turn_nodes: Set[str] = set()
            if show_turn_markers:
                for step in path_result.get("path_steps", []):
                    if step.get("turn_cost", 0.0) > 0:
                        turn_nodes.add(str(step.get("from")))

            for u, v, _, rule, _ in path_result.get("path_edges", []):
                edge_z = 7.0
                color = _edge_color_by_direction(str(u), str(v), graph_nodes)
                is_derived_edge = (
                    graph_nodes.get(str(u), {}).get("kind") == "derived"
                    or graph_nodes.get(str(v), {}).get("kind") == "derived"
                )

                if show_bidirectional_white_arrows and (u, v) in bidirectional_edges:
                    connectionstyle = _signed_bidir_connectionstyle(str(u), str(v))
                elif is_derived_edge:
                    connectionstyle = _edge_connectionstyle(
                        str(u), str(v), {"rule": rule}, graph_nodes, pos
                    )
                else:
                    connectionstyle = "arc3,rad=0.0"

                artists = nx.draw_networkx_edges(
                    g,
                    pos,
                    edgelist=[(u, v)],
                    arrowstyle="->",
                    arrows=True,
                    arrowsize=13,
                    width=optimal_path_width,
                    edge_color=[color],
                    alpha=1.0,
                    connectionstyle=connectionstyle,
                )
                _set_edge_artists_zorder(artists, edge_z)

            if turn_nodes:
                nx.draw_networkx_nodes(
                    g,
                    pos,
                    nodelist=list(turn_nodes),
                    node_size=980,
                    node_color="none",
                    edgecolors="#FF8C00",
                    linewidths=2.8,
                )

            plt.title(
                f"Merlin Directed Graph (Fixed Layout) | Cost: {path_result.get('cost', 0.0):.2f} "
                f"| Turn: {path_result.get('total_turn_cost', 0.0):.2f} "
                f"| Move: {path_result.get('total_move_cost', 0.0):.2f} "
                f"| Pick: {path_result.get('total_pick_cost', 0.0):.2f}"
            )
        else:
            plt.title("Merlin Directed Graph (Fixed Layout) | Optimal Path Not Found")
    else:
        plt.title("Merlin Directed Graph (Fixed Layout)")
    plt.axis("off")
    plt.tight_layout()
    plt.savefig(save_path, dpi=200)

    if show:
        plt.show()
    else:
        plt.close()

    return save_path


def main() -> None:
    path_without_base_edges = draw_merlin_model(
        save_path="/tmp/merlin_model_without_base_edges.png",
        show_bidirectional_white_arrows=True,
        show_base_edges=False,
    )
    print(f"图已生成(不含基础边): {path_without_base_edges}")

    path_with_base_edges = draw_merlin_model(
        save_path="/tmp/merlin_model_with_base_edges.png",
        show_bidirectional_white_arrows=True,
        show_base_edges=True,
    )
    print(f"图已生成(含基础边): {path_with_base_edges}")


if __name__ == "__main__":
    main()