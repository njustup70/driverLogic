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
from typing import Optional

from MainLogic.core.zone2_model import (
    dijkstra_min_cost_path,
    draw_merlin_model,
    list_saved_merlin_maps,
    load_saved_merlin_map,
    print_merlin_map,
)
from MainLogic.core.zone2_model.merlin_map import load_saved_merlin_map_by_filename
from MainLogic.core.zone2_model.zone2_format import print_path_debug_info


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
    result = dijkstra_min_cost_path(map_data=map_data)
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
