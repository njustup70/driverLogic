"""
梅林地图与物块生成（内存缓存版，不保存本地）
梅林: 4行3列，编号 1~12（从左到右、从上到下）
额外桩: start, end
"""

import random
from typing import Dict, List, Union, Optional

Node = Union[int, str]

# 模块内缓存（仅内存）
_MAP_CACHE: Optional[dict] = None


def _build_adjacency() -> Dict[Node, List[Node]]:
    # 先给 1~12 初始化
    adj: Dict[Node, List[Node]] = {i: [] for i in range(1, 13)}

    rows, cols = 4, 3

    def rc_to_id(r: int, c: int) -> int:
        return r * cols + c + 1  # r,c 从 0 开始 => id 从 1 开始

    # 网格上下左右邻接
    for r in range(rows):
        for c in range(cols):
            cur = rc_to_id(r, c)
            if r > 0:
                adj[cur].append(rc_to_id(r - 1, c))
            if r < rows - 1:
                adj[cur].append(rc_to_id(r + 1, c))
            if c > 0:
                adj[cur].append(rc_to_id(r, c - 1))
            if c < cols - 1:
                adj[cur].append(rc_to_id(r, c + 1))

    # 加 start / end
    adj["start"] = [1, 2, 3]
    adj["end"] = [10, 11, 12]

    # 双向补充：1/2/3 连 start，10/11/12 连 end
    for n in (1, 2, 3):
        adj[n].append("start")
    for n in (10, 11, 12):
        adj[n].append("end")

    return adj


def _generate_blocks(rng: random.Random) -> Dict[int, str]:
    """
    返回: {桩号: 物块类型}
    共 5 个物块：4 个 R2 + 1 个 fake
    fake 不会在 1/2/3
    """
    all_slots = list(range(1, 13))
    fake_slot = rng.choice(list(range(4, 13)))  # 4~12
    remain = [x for x in all_slots if x != fake_slot]
    r2_slots = rng.sample(remain, 4)

    blocks: Dict[int, str] = {s: "R2" for s in r2_slots}
    blocks[fake_slot] = "fake"
    return blocks


def get_merlin_map(force_refresh: bool = False, seed: Optional[int] = None) -> dict:
    """
    获取梅林地图（缓存版）
    - force_refresh=True 时重新生成
    - seed 可控随机（便于复现）
    """
    global _MAP_CACHE
    if _MAP_CACHE is not None and not force_refresh:
        return _MAP_CACHE

    rng = random.Random(seed)
    _MAP_CACHE = {
        "name": "merlin",
        "shape": {"rows": 4, "cols": 3},
        "nodes": ["start"] + list(range(1, 13)) + ["end"],
        "adjacency": _build_adjacency(),
        "blocks": _generate_blocks(rng),
    }
    return _MAP_CACHE


def clear_merlin_map_cache() -> None:
    """清空缓存，下次 get_merlin_map 会重新生成。"""
    global _MAP_CACHE
    _MAP_CACHE = None


def render_merlin_map(data: Optional[dict] = None) -> str:
    """
    生成梅林地图的终端文本图。
    - 横向相邻: --
    - 纵向相邻: |
    - start 在 2 正上方
    - end 在 11 正下方
    """
    if data is None:
        data = get_merlin_map()

    blocks = data.get("blocks", {})

    CELL_W = 10      # 每个格子的固定宽度，便于对齐
    LINK_H = " -- "  # 横向连接符

    def raw_cell_text(node_id: int) -> str:
        t = blocks.get(node_id)
        if t == "R2":
            return f"{node_id:>2}[R2]"
        if t == "fake":
            return f"{node_id:>2}[FAKE]"
        return f"{node_id:>2}[  ]"

    def cell(node_id: int) -> str:
        return f"{raw_cell_text(node_id):<{CELL_W}}"

    total_w = CELL_W * 3 + len(LINK_H) * 2
    center_col2 = (CELL_W + len(LINK_H)) + CELL_W // 2  # 第二列中心位置

    def vline_three_cols() -> str:
        chars = [" "] * total_w
        for i in range(3):
            pos = i * (CELL_W + len(LINK_H)) + CELL_W // 2
            chars[pos] = "|"
        return "".join(chars).rstrip()

    lines = []
    lines.append("========== MERLIN MAP ==========")

    # start 在 2 的正上方
    start_pos = max(0, center_col2 - len("start") // 2)
    lines.append(" " * start_pos + "start")
    lines.append(" " * center_col2 + "|")

    # 4行3列，横向 --，纵向 |
    for r in range(4):
        base = r * 3 + 1
        row_nodes = [base, base + 1, base + 2]
        row_line = LINK_H.join(cell(n) for n in row_nodes).rstrip()
        lines.append(row_line)
        if r < 3:
            lines.append(vline_three_cols())

    # end 在 11 的正下方
    lines.append(" " * center_col2 + "|")
    end_pos = max(0, center_col2 - len("end") // 2)
    lines.append(" " * end_pos + "end")

    lines.append("================================")
    return "\n".join(lines)


def print_merlin_map(data: Optional[dict] = None) -> None:
    """直接在终端打印梅林地图。"""
    print(render_merlin_map(data))


def main() -> None:
    """脚本入口：生成并打印梅林地图。"""
    data = get_merlin_map()
    print_merlin_map(data)


if __name__ == "__main__":
    main()