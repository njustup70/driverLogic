"""Main 层的 Zone 2 梅林接口封装。

这一层位于 MainLogic.core.zone2_model 与具体 async main 之间。
仅保留 main 运行所需的最小接口：
- 4 个可调参数（MOVE/PICK/TURN/REQUIRED_R2_COUNT）
- 随机地图求解并可视化的入口
"""

from __future__ import annotations

from typing import Any, Dict, Optional

from MainLogic.core.zone2_model import (
    MOVE_COST,
    PICK_COST,
    REQUIRED_R2_COUNT,
    TURN_COST,
    dijkstra_min_cost_path,
    draw_merlin_model,
    get_merlin_map,
)


class Zone2ModelAPI:
    """面向 main 的最小调用入口。"""

    MOVE_COST = MOVE_COST
    PICK_COST = PICK_COST
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
        turn_free_rules: Optional[set[str]] = None,
        map_data: Optional[dict] = None,
    ) -> Dict[str, Any]:
        use_normal_cost = self.MOVE_COST if normal_cost is None else normal_cost
        use_to_derived_cost = self.PICK_COST if to_derived_cost is None else to_derived_cost
        use_required_r2_count = self.REQUIRED_R2_COUNT if required_r2_count is None else required_r2_count
        use_turn_cost = self.TURN_COST if turn_cost is None else turn_cost

        return dijkstra_min_cost_path(
            start=start,
            end=end,
            normal_cost=use_normal_cost,
            to_derived_cost=use_to_derived_cost,
            required_r2_count=use_required_r2_count,
            enforce_top_entry_after_one_pick=enforce_top_entry_after_one_pick,
            turn_cost=use_turn_cost,
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
        required_r2_count: Optional[int] = None,
    ) -> Dict[str, Any]:
        """一键示例：随机生成地图、求解，并把地图和 path 一起画出来。"""
        map_data = self.get_merlin_map(force_refresh=True, seed=seed)
        result = self.dijkstra_min_cost_path(
            map_data=map_data,
            normal_cost=move_cost,
            to_derived_cost=pick_cost,
            turn_cost=turn_cost,
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


zone2_model_api = Zone2ModelAPI()


__all__ = [
    "Zone2ModelAPI",
    "zone2_model_api",
]
