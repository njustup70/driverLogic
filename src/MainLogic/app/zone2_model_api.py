"""Main 层的 Zone 2 梅林接口封装。

这一层位于 MainLogic.core.zone2_model 与具体 async main 之间。
仅保留 main 运行所需的最小接口：
- 5 个可调参数（MOVE/PICK/TURN/R1_REMOVE/REQUIRED_R2_COUNT）
- 随机地图求解并可视化的入口
"""

from __future__ import annotations

from typing import Any, Dict, Optional

from MainLogic.core import ros_bridge_node as ros_bridge_module
from MainLogic.core.zone2_model import (
    MOVE_COST,
    PICK_COST,
    R1_REMOVE_COST,
    REQUIRED_R2_COUNT,
    TURN_COST,
    dijkstra_min_cost_path,
    draw_merlin_model,
    get_merlin_map,
)


def encode_path_frame(path_nodes: list[str]) -> bytes:
    """把求解路径编码成适合下位机解析的二进制帧。"""
    if not path_nodes:
        return b"\xB5\x00"

    node_bytes = bytearray()
    for node in path_nodes:
        node_str = str(node)
        if node_str == "start":
            node_bytes.append(0xFE)
        elif node_str == "end":
            node_bytes.append(0xFF)
        elif node_str.isdigit():
            value = int(node_str)
            if not 0 <= value <= 253:
                raise ValueError(f"路径节点超出可编码范围: {node_str}")
            node_bytes.append(value)
        else:
            raise ValueError(f"无法编码的路径节点: {node_str}")

    if len(node_bytes) > 255:
        raise ValueError(f"路径过长，当前长度 {len(node_bytes)} 超过单帧上限 255")

    return b"\xB5" + bytes([len(node_bytes)]) + bytes(node_bytes)


def send_path_result_to_mcu(result: dict) -> None:
    """把求解出来的路径通过串口发给下位机。"""
    assert ros_bridge_module.RosBridgeNodeInstance is not None, "RosBridgeNodeInstance is not initialized yet!"

    if not result.get("found"):
        ros_bridge_module.RosBridgeNodeInstance.writeBytes(b"\xB5\x00")
        print("[zone2_model_api] 没有找到可用路径，已发送空路径帧")
        return

    path_nodes = result.get("path", [])
    frame = encode_path_frame(path_nodes)
    ros_bridge_module.RosBridgeNodeInstance.writeBytes(frame)
    print(f"[zone2_model_api] 已发送路径到下位机: {path_nodes}")


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
        map_data: Optional[dict] = None,
    ) -> Dict[str, Any]:
        use_normal_cost = self.MOVE_COST if normal_cost is None else normal_cost
        use_to_derived_cost = self.PICK_COST if to_derived_cost is None else to_derived_cost
        use_required_r2_count = self.REQUIRED_R2_COUNT if required_r2_count is None else required_r2_count
        use_turn_cost = self.TURN_COST if turn_cost is None else turn_cost
        use_r1_remove_cost = self.R1_REMOVE_COST if r1_remove_cost is None else r1_remove_cost

        return dijkstra_min_cost_path(
            start=start,
            end=end,
            normal_cost=use_normal_cost,
            to_derived_cost=use_to_derived_cost,
            required_r2_count=use_required_r2_count,
            enforce_top_entry_after_one_pick=enforce_top_entry_after_one_pick,
            turn_cost=use_turn_cost,
            r1_remove_cost=use_r1_remove_cost,
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
        r1_remove_cost: Optional[float] = None,
        required_r2_count: Optional[int] = None,
    ) -> Dict[str, Any]:
        """一键示例：随机生成地图、求解，并把地图和 path 一起画出来。"""
        map_data = self.get_merlin_map(force_refresh=True, seed=seed)
        result = self.dijkstra_min_cost_path(
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

    def send_path_result_to_mcu(self, result: dict) -> None:
        send_path_result_to_mcu(result)


zone2_model_api = Zone2ModelAPI()


__all__ = [
    "Zone2ModelAPI",
    "encode_path_frame",
    "send_path_result_to_mcu",
    "zone2_model_api",
]
