"""zone2_model_api 模块作用：

这是 Main 层对外的汇总入口，只做三件事：
1. 汇总导入 zone2 的辅助模块（helpers / format / encoder / sender）
2. 提供 `Zone2ModelAPI` 作为 main 调用的稳定门面
3. 暴露少量常量和对象，方便旧代码平滑迁移

真正的功能实现已经拆分到：
- `zone2_helpers.py`：基础工具和共享常量
- `zone2_format.py`：动作链与调试文本格式化
- `zone2_encoder.py`：二进制帧编码
- `zone2_sender.py`：串口/桥接发送
"""

from __future__ import annotations

from typing import Any, Dict, Optional

from MainLogic.core.zone2_model import (
    MOVE_COST,
    PICK_COST,
    R1_REMOVE_COST,
    REQUIRED_R2_COUNT,
    TURN_COST,
    dijkstra_min_cost_path,
    draw_merlin_model,
    get_merlin_map,
<<<<<<< HEAD
    solve_route,
)
=======
    list_saved_merlin_maps,
    load_saved_merlin_map,
    print_merlin_map,
    solve_route,
)
from MainLogic.core.zone2_model.merlin_map import load_saved_merlin_map_by_filename
>>>>>>> Zone2

from MainLogic.core.zone2_model.zone2_helpers import *
from MainLogic.core.zone2_model.zone2_format import *
from MainLogic.core.zone2_model.zone2_encoder import *
from MainLogic.core.zone2_model.zone2_sender import *


def visualize_path_result(result: dict, save_path: str = "/tmp/merlin_random_demo.png", show: bool = True) -> str:
    """把已求解路径重新绘制成 GUI 页面。"""
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
        map_frame: Optional[Any] = None,
        map_data: Optional[dict] = None,
    ) -> Dict[str, Any]:
        use_normal_cost = self.MOVE_COST if normal_cost is None else normal_cost
        use_to_derived_cost = self.PICK_COST if to_derived_cost is None else to_derived_cost
        use_required_r2_count = self.REQUIRED_R2_COUNT if required_r2_count is None else required_r2_count
        use_turn_cost = self.TURN_COST if turn_cost is None else turn_cost
        use_r1_remove_cost = self.R1_REMOVE_COST if r1_remove_cost is None else r1_remove_cost

        return solve_route(
            strategy="dijkstra",
            start=start,
            end=end,
            normal_cost=use_normal_cost,
            to_derived_cost=use_to_derived_cost,
            required_r2_count=use_required_r2_count,
            enforce_top_entry_after_one_pick=enforce_top_entry_after_one_pick,
            turn_cost=use_turn_cost,
            r1_remove_cost=use_r1_remove_cost,
            turn_free_rules=turn_free_rules,
            map_frame=map_frame,
            map_data=map_data,
        )

    def solve_route(
        self,
        strategy: str = "dijkstra",
        start: str = "start",
        end: str = "end",
        normal_cost: Optional[float] = None,
        to_derived_cost: Optional[float] = None,
        required_r2_count: Optional[int] = None,
        enforce_top_entry_after_one_pick: bool = True,
        turn_cost: Optional[float] = None,
        r1_remove_cost: Optional[float] = None,
        turn_free_rules: Optional[set[str]] = None,
        map_frame: Optional[Any] = None,
        map_data: Optional[dict] = None,
    ) -> Dict[str, Any]:
        use_normal_cost = self.MOVE_COST if normal_cost is None else normal_cost
        use_to_derived_cost = self.PICK_COST if to_derived_cost is None else to_derived_cost
        use_required_r2_count = self.REQUIRED_R2_COUNT if required_r2_count is None else required_r2_count
        use_turn_cost = self.TURN_COST if turn_cost is None else turn_cost
        use_r1_remove_cost = self.R1_REMOVE_COST if r1_remove_cost is None else r1_remove_cost

        return solve_route(
            strategy=strategy,
            start=start,
            end=end,
            normal_cost=use_normal_cost,
            to_derived_cost=use_to_derived_cost,
            required_r2_count=use_required_r2_count,
            enforce_top_entry_after_one_pick=enforce_top_entry_after_one_pick,
            turn_cost=use_turn_cost,
            r1_remove_cost=use_r1_remove_cost,
            turn_free_rules=turn_free_rules,
            map_frame=map_frame,
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
        result = self.solve_route(
            strategy="dijkstra",
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

<<<<<<< HEAD
    def send_path_result_to_mcu(self, result: dict) -> None:
        send_path_result_to_mcu(result)

=======
>>>>>>> Zone2
    def print_path_debug_info(self, result: dict) -> None:
        print_path_debug_info(result)

    def format_mcu_action_list(self, result: dict) -> str:
        return format_mcu_action_list(result)

<<<<<<< HEAD
    def encode_mcu_action_frame(self, result: dict) -> bytes:
        return encode_mcu_action_frame(result)

    def send_mcu_action_frame_to_mcu(self, result: dict) -> None:
        send_mcu_action_frame_to_mcu(result)

=======
>>>>>>> Zone2
    def visualize_path_result(self, result: dict, save_path: str = "/tmp/merlin_random_demo.png", show: bool = True) -> str:
        return visualize_path_result(result, save_path=save_path, show=show)


zone2_model_api = Zone2ModelAPI()


__all__ = [
    "Zone2ModelAPI",
    "zone2_model_api",
    "visualize_path_result",
<<<<<<< HEAD
    "send_path_result_to_mcu",
    "send_mcu_action_frame_to_mcu",
    "encode_path_frame",
    "encode_path_action_frame",
    "encode_full_path_frame",
    "encode_mcu_action_frame",
=======
>>>>>>> Zone2
    "format_mcu_action_list",
    "print_path_debug_info",
    "build_path_step_records",
    "format_action_chain",
    "format_chronological_steps",
<<<<<<< HEAD
=======
    "generate_actions_from_result",
    "determine_start_position",
    "encode_action_sequence",
    "send_actions",
    "send_r1_nodes",
    "extract_r1_nodes_on_path",
    "list_saved_merlin_maps",
    "load_saved_merlin_map",
    "load_saved_merlin_map_by_filename",
    "print_merlin_map",
    "dijkstra_min_cost_path",
    "draw_merlin_model",
    "solve_route",
>>>>>>> Zone2
]
