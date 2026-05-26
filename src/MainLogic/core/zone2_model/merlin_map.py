"""
梅林地图与物块生成。

功能：
1) 支持随机生成地图并做单次内存缓存。
2) 自动把最近 10 次随机生成的地图保存到本地缓冲区。
3) 提供读取历史地图的接口，便于调试路径求解算法。

梅林: 4行3列，编号 1~12（从左到右、从上到下）
额外桩: start, end
"""

import json
import random
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Union, Optional

Node = Union[int, str]

# 模块内缓存（仅内存）
_MAP_CACHE: Optional[dict] = None

# 最近 10 次随机地图的本地缓冲区
_MAP_BUFFER_LIMIT = 10
_MAP_BUFFER_DIR = Path(__file__).resolve().parent / "_merlin_map_buffer"
_MAP_INDEX_FILE = _MAP_BUFFER_DIR / "index.json"


def _ensure_buffer_dir() -> None:
    _MAP_BUFFER_DIR.mkdir(parents=True, exist_ok=True)


def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _resolve_seed(seed: Optional[int]) -> int:
    if seed is not None:
        return int(seed)
    return random.SystemRandom().randint(0, 2**32 - 1)


def _load_buffer_index() -> List[dict]:
    if not _MAP_INDEX_FILE.exists():
        return []
    try:
        with _MAP_INDEX_FILE.open("r", encoding="utf-8") as f:
            data = json.load(f)
        if isinstance(data, list):
            return [x for x in data if isinstance(x, dict)]
    except Exception:
        pass
    return []


def _save_buffer_index(entries: List[dict]) -> None:
    _ensure_buffer_dir()
    with _MAP_INDEX_FILE.open("w", encoding="utf-8") as f:
        json.dump(entries, f, ensure_ascii=False, indent=2)


def _map_snapshot_path(map_id: str) -> Path:
    return _MAP_BUFFER_DIR / f"{map_id}.json"


def _normalize_blocks_for_storage(blocks: Dict[int, str]) -> Dict[str, str]:
    return {str(k): str(v) for k, v in blocks.items()}


def _restore_blocks_from_storage(blocks: Dict[str, str]) -> Dict[int, str]:
    restored: Dict[int, str] = {}
    for k, v in blocks.items():
        try:
            restored[int(k)] = str(v)
        except (TypeError, ValueError):
            continue
    return restored


def _persist_map_snapshot(map_data: dict) -> None:
    """把随机生成的地图保存到本地缓冲区，并裁剪到最近 10 条。"""
    _ensure_buffer_dir()
    map_id = str(map_data.get("map_id") or f"merlin_{map_data.get('seed', 'unknown')}_{int(random.random() * 1e9)}")
    snapshot = {
        "name": map_data.get("name", "merlin"),
        "shape": map_data.get("shape", {"rows": 4, "cols": 3}),
        "seed": map_data.get("seed"),
        "map_id": map_id,
        "generated_at": map_data.get("generated_at", _utc_now_iso()),
        "blocks": _normalize_blocks_for_storage(map_data.get("blocks", {})),
    }

    snapshot_path = _map_snapshot_path(map_id)
    with snapshot_path.open("w", encoding="utf-8") as f:
        json.dump(snapshot, f, ensure_ascii=False, indent=2)

    index = _load_buffer_index()
    index = [entry for entry in index if entry.get("map_id") != map_id]
    index.append(
        {
            "map_id": map_id,
            "seed": snapshot["seed"],
            "generated_at": snapshot["generated_at"],
            "file": snapshot_path.name,
        }
    )

    # 仅保留最近 10 条
    while len(index) > _MAP_BUFFER_LIMIT:
        removed = index.pop(0)
        old_file = _MAP_BUFFER_DIR / str(removed.get("file", ""))
        try:
            if old_file.exists():
                old_file.unlink()
        except OSError:
            pass

    _save_buffer_index(index)


def _load_map_snapshot(snapshot_path: Path) -> dict:
    with snapshot_path.open("r", encoding="utf-8") as f:
        data = json.load(f)

    blocks = _restore_blocks_from_storage(data.get("blocks", {}))
    return {
        "name": data.get("name", "merlin"),
        "shape": data.get("shape", {"rows": 4, "cols": 3}),
        "nodes": ["start"] + list(range(1, 13)) + ["end"],
        "adjacency": _build_adjacency(),
        "blocks": blocks,
        "seed": data.get("seed"),
        "map_id": data.get("map_id"),
        "generated_at": data.get("generated_at"),
    }


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
    共 8 个物块：3 个 R1 + 4 个 R2 + 1 个 fake
    R1 物块только能放在外圈（1,2,3,4,6,7,9,10,11,12）
    fake 不会在 1/2/3
    """
    # 外圈位置：上下两行（1,2,3,10,11,12）+ 左右两列边界（4,6,7,9）
    outer_ring = [1, 2, 3, 4, 6, 7, 9, 10, 11, 12]
    # 内圈位置：5, 8
    inner_positions = [5, 8]
    
    # 选择 fake 位置（不在 1/2/3）
    fake_slot = rng.choice(list(range(4, 13)))  # 4~12
    
    # 从外圈选择 3 个 R1 物块位置（不包括 fake）
    available_outer = [x for x in outer_ring if x != fake_slot]
    r1_slots = rng.sample(available_outer, 3)
    
    # 剩余位置用来放 R2（不是 R1、fake 的位置）
    used_slots = set(r1_slots) | {fake_slot}
    all_slots = set(range(1, 13))
    r2_candidates = list(all_slots - used_slots)
    r2_slots = rng.sample(r2_candidates, 4)  # 4 个 R2
    
    blocks: Dict[int, str] = {}
    for s in r1_slots:
        blocks[s] = "R1"
    for s in r2_slots:
        blocks[s] = "R2"
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
        if seed is None or _MAP_CACHE.get("seed") == seed:
            return _MAP_CACHE

    actual_seed = _resolve_seed(seed)
    rng = random.Random(actual_seed)
    _MAP_CACHE = {
        "name": "merlin",
        "shape": {"rows": 4, "cols": 3},
        "nodes": ["start"] + list(range(1, 13)) + ["end"],
        "adjacency": _build_adjacency(),
        "blocks": _generate_blocks(rng),
        "seed": actual_seed,
        "map_id": f"merlin_{actual_seed}_{int(random.random() * 1e9)}",
        "generated_at": _utc_now_iso(),
    }

    _persist_map_snapshot(_MAP_CACHE)
    return _MAP_CACHE


def list_saved_merlin_maps() -> List[dict]:
    """列出最近保存的梅林地图缓冲条目（按时间顺序）。"""
    return _load_buffer_index()


def load_saved_merlin_map(index: int = -1) -> dict:
    """读取缓冲区里的历史地图。

    参数：
        index: 缓冲条目索引，支持负数；默认 -1 表示最新一条。
    """
    entries = _load_buffer_index()
    if not entries:
        raise FileNotFoundError("没有可用的梅林地图缓冲记录，请先生成随机地图。")

    try:
        entry = entries[index]
    except IndexError as exc:
        raise IndexError(f"缓冲索引超出范围：{index}, 当前共有 {len(entries)} 条") from exc

    snapshot_path = _MAP_BUFFER_DIR / str(entry.get("file", ""))
    if not snapshot_path.exists():
        raise FileNotFoundError(f"缓冲文件不存在：{snapshot_path}")

    return _load_map_snapshot(snapshot_path)


def load_saved_merlin_map_by_filename(filename: str) -> dict:
    """按缓冲文件名直接读取历史地图。"""
    if not filename:
        raise ValueError("filename 不能为空")

    snapshot_path = _MAP_BUFFER_DIR / str(filename)
    if not snapshot_path.exists():
        raise FileNotFoundError(f"缓冲文件不存在：{snapshot_path}")

    return _load_map_snapshot(snapshot_path)


def clear_merlin_map_buffer() -> None:
    """清空本地地图缓冲区。"""
    global _MAP_CACHE
    _MAP_CACHE = None
    if _MAP_INDEX_FILE.exists():
        try:
            _MAP_INDEX_FILE.unlink()
        except OSError:
            pass
    if _MAP_BUFFER_DIR.exists():
        for child in _MAP_BUFFER_DIR.glob("*.json"):
            try:
                child.unlink()
            except OSError:
                pass


def clear_merlin_map_cache() -> None:
    """清空内存缓存，下次 get_merlin_map 会重新生成。"""
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
        if t == "R1":
            return f"{node_id:>2}[R1]"
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