"""梅林地图历史缓冲区调试入口。

用途：
1) 列出最近保存的随机地图。
2) 选择某一张历史地图。
3) 用该地图运行路径求解算法，检查结果是否异常。

示例：
    python3 -m MainLogic.app.merlin_map_solver_debug --list
    python3 -m MainLogic.app.merlin_map_solver_debug --index 0
    python3 -m MainLogic.app.merlin_map_solver_debug
"""

from __future__ import annotations

import argparse
from typing import Any, Dict, List, Optional

from MainLogic.app.zone2_model_api import (
    dijkstra_min_cost_path,
    draw_merlin_model,
    list_saved_merlin_maps,
    load_saved_merlin_map,
    load_saved_merlin_map_by_filename,
    print_merlin_map,
    print_path_debug_info,
    solve_route,
)


_STATE_TO_BLOCK = {
    "EMPTY": "empty",
    "empty": "empty",
    "R1": "R1",
    "r1": "R1",
    "R2": "R2",
    "r2": "R2",
    "FAKE": "fake",
    "fake": "fake",
}


def _build_fixed_adjacency() -> Dict[Any, List[Any]]:
    """构造梅林 4x3 固定邻接关系，和地图状态无关。"""
    adjacency: Dict[Any, List[Any]] = {i: [] for i in range(1, 13)}

    rows, cols = 4, 3

    def rc_to_id(r: int, c: int) -> int:
        return r * cols + c + 1

    for r in range(rows):
        for c in range(cols):
            cur = rc_to_id(r, c)
            if r > 0:
                adjacency[cur].append(rc_to_id(r - 1, c))
            if r < rows - 1:
                adjacency[cur].append(rc_to_id(r + 1, c))
            if c > 0:
                adjacency[cur].append(rc_to_id(r, c - 1))
            if c < cols - 1:
                adjacency[cur].append(rc_to_id(r, c + 1))

    adjacency["start"] = [1, 2, 3]
    adjacency["end"] = [10, 11, 12]

    for n in (1, 2, 3):
        adjacency[n].append("start")
    for n in (10, 11, 12):
        adjacency[n].append("end")

    return adjacency


def build_map_data_from_states(states12: List[str], *, map_id: Optional[str] = None, seed: Optional[int] = None) -> dict:
    """把解析后的 12 个桩位状态组装成求解器需要的 map_data。"""
    if len(states12) != 12:
        raise ValueError(f"状态数量必须是 12 个，当前是 {len(states12)} 个")

    blocks: Dict[int, str] = {}
    for idx, state in enumerate(states12, start=1):
        block = _STATE_TO_BLOCK.get(str(state))
        if block is None:
            raise ValueError(f"未知的桩位状态: index={idx}, value={state!r}")
        blocks[idx] = block

    return {
        "name": "merlin",
        "shape": {"rows": 4, "cols": 3},
        "nodes": ["start"] + list(range(1, 13)) + ["end"],
        "adjacency": _build_fixed_adjacency(),
        "blocks": blocks,
        "map_id": map_id,
        "seed": seed,
    }


def _print_map_entries() -> None:
    entries = list_saved_merlin_maps()
    if not entries:
        print("[merlin_map_debug] 当前没有可用的历史地图。请先生成随机地图。", flush=True)
        return

    print("[merlin_map_debug] 最近保存的地图（按时间顺序）：", flush=True)
    for idx, entry in enumerate(entries):
        print(
            f"  [{idx}] map_id={entry.get('map_id')} | seed={entry.get('seed')} | "
            f"generated_at={entry.get('generated_at')} | file={entry.get('file')}",
            flush=True,
        )


def _choose_index_interactively(default: int = -1) -> int:
    entries = list_saved_merlin_maps()
    if not entries:
        raise FileNotFoundError("没有可用的历史地图。")

    _print_map_entries()
    raw = input(f"请输入要运行的地图索引（默认 {default}）：").strip()
    if raw == "":
        return default
    return int(raw)


def _choose_file_interactively() -> str:
    entries = list_saved_merlin_maps()
    if not entries:
        raise FileNotFoundError("没有可用的历史地图。")

    _print_map_entries()
    raw = input("请输入要运行的地图文件名（例如 merlin_xxx.json）：").strip()
    if not raw:
        raise ValueError("文件名不能为空")
    return raw


def run_solver_on_saved_map(
    index: int = -1,
    filename: Optional[str] = None,
    render_map: bool = True,
    save_image: Optional[str] = None,
) -> dict:
    if filename:
        map_data = load_saved_merlin_map_by_filename(filename)
    else:
        map_data = load_saved_merlin_map(index=index)
    result = solve_route(strategy="dijkstra", map_data=map_data)
    result["map_data"] = map_data

    print(
        f"[merlin_map_debug] 已加载地图: map_id={map_data.get('map_id')} | seed={map_data.get('seed')} | "
        f"generated_at={map_data.get('generated_at')}",
        flush=True,
    )

    if render_map:
        print("\n[merlin_map_debug] 地图内容：", flush=True)
        print_merlin_map(map_data)

    print("\n[merlin_map_debug] 求解结果：", flush=True)
    print(
        f"found={result.get('found')} | cost={result.get('cost')} | "
        f"collected_r2_count={result.get('collected_r2_count')} | collected_r2={result.get('collected_r2')}",
        flush=True,
    )

    print_path_debug_info(result)

    if save_image:
        image_path = draw_merlin_model(
            save_path=save_image,
            show=False,
            show_bidirectional_white_arrows=True,
            show_base_edges=True,
            show_optimal_path=True,
            show_turn_markers=True,
            map_data=map_data,
            path_result=result,
        )
        print(f"\n[merlin_map_debug] 图像已保存到: {image_path}", flush=True)

    return result


def run_solver_on_states(
    states12: List[str],
    render_map: bool = True,
    save_image: Optional[str] = None,
    map_id: Optional[str] = None,
    seed: Optional[int] = None,
) -> dict:
    """直接把 12 个状态组装成 map_data 后求解。"""
    map_data = build_map_data_from_states(states12, map_id=map_id, seed=seed)
    result = solve_route(strategy="straight", map_data=map_data)
    result["map_data"] = map_data

    if render_map:
        print("\n[merlin_map_debug] 地图内容：", flush=True)
        print_merlin_map(map_data)

    print_path_debug_info(result)

    if save_image:
        image_path = draw_merlin_model(
            save_path=save_image,
            show=False,
            show_bidirectional_white_arrows=True,
            show_base_edges=True,
            show_optimal_path=True,
            show_turn_markers=True,
            map_data=map_data,
            path_result=result,
        )
        print(f"\n[merlin_map_debug] 图像已保存到: {image_path}", flush=True)

    return result


def main() -> None:
    parser = argparse.ArgumentParser(description="梅林地图缓冲区路径求解调试工具")
    parser.add_argument("--list", action="store_true", help="只列出缓冲区里的历史地图")
    parser.add_argument("--index", type=int, default=None, help="选择缓冲区地图索引（支持负数）")
    parser.add_argument("--file", type=str, default=None, help="直接指定缓冲区里的地图文件名")
    parser.add_argument("--no-render", action="store_true", help="不打印地图文本")
    parser.add_argument("--save-image", type=str, default=None, help="把结果图保存到指定路径")
    args = parser.parse_args()

    if args.list:
        _print_map_entries()
        return

    filename = args.file
    index = args.index

    if filename is None and index is None:
        choice = input("请选择方式：输入 i 按索引选择，输入 f 按文件名选择（默认 i）：").strip().lower()
        if choice == "f":
            filename = _choose_file_interactively()
        else:
            index = _choose_index_interactively(default=-1)

    if filename is None and index is None:
        index = -1

    run_solver_on_saved_map(
        index=index,
        filename=filename,
        render_map=not args.no_render,
        save_image=args.save_image,
    )


if __name__ == "__main__":
    main()
