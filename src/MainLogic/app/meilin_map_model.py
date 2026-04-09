from dataclasses import dataclass


@dataclass
class Pile:
    """桩工具类：按 id 管理状态，位置由外部传入。"""

    id: int
    position: tuple
    height: int = 200
    block_type: str = "EMPTY"
    cost: float = 1.0

    VALID_BLOCK_TYPES = {"EMPTY", "R1", "R2", "FAKE"}

    def _set_block_type(self, block_type, r1_penalty, normal_cost):
        if block_type not in self.VALID_BLOCK_TYPES:
            raise ValueError(f"桩 {self.id} 的物块类型非法: {block_type}")
        self.block_type = block_type
        if block_type == "R1":
            self.cost = float(r1_penalty)
        elif block_type == "FAKE":
            self.cost = float("inf")
        else:
            self.cost = float(normal_cost)


class MeilinMap:
    """地图容器：维护 half_flag 与 12 个桩实例。"""

    def __init__(self, half_flag="blue", coordinates=None, location=None, heights=None, r1_penalty=1.1, normal_cost=1):
        if half_flag not in ("blue", "red"):
            raise ValueError("half_flag 仅支持 'blue' 或 'red'")
        self.half_flag = half_flag
        self.r1_penalty = r1_penalty
        self.normal_cost = normal_cost
        self.heights = heights or self._default_heights()
        self.location = coordinates or location or self._default_location(half_flag)
        self.piles = {}
        for pile_id in range(1, 13):
            position = self.location.get(pile_id, (0.0, 0.0))
            height = self.heights.get(pile_id, 200)
            pile = Pile(id=pile_id, position=position, height=height)
            pile._set_block_type("EMPTY", self.r1_penalty, self.normal_cost)
            self.piles[pile_id] = pile

    def _default_heights(self):
        # 2,4,10,12为200/1,3,5,7,9,11为400/6,8为600
        return {
            1: 400, 2: 200, 3: 400,
            4: 200, 5: 400, 6: 600,
            7: 400, 8: 600, 9: 400,
            10: 200, 11: 400, 12: 200,
        }

    def _default_location(self, half_flag):
        # 填写红蓝半场对应各id的实际坐标(x,y)
        if half_flag == "blue":
            return {
                1: (0.0, 0.0), 2: (1.0, 0.0), 3: (2.0, 0.0),
                4: (0.0, 1.0), 5: (1.0, 1.0), 6: (2.0, 1.0),
                7: (0.0, 2.0), 8: (1.0, 2.0), 9: (2.0, 2.0),
                10: (0.0, 3.0), 11: (1.0, 3.0), 12: (2.0, 3.0),
            }
        if half_flag == "red":
            return {
                1: (2.0, 0.0), 2: (1.0, 0.0), 3: (0.0, 0.0),
                4: (2.0, 1.0), 5: (1.0, 1.0), 6: (0.0, 1.0),
                7: (2.0, 2.0), 8: (1.0, 2.0), 9: (0.0, 2.0),
                10: (2.0, 3.0), 11: (1.0, 3.0), 12: (0.0, 3.0),
            }

    def apply_block_types(self, id_to_block_type):
        """外部入口：批量传入 {id: block_type} 并更新到对应桩。"""
        if not isinstance(id_to_block_type, dict):
            raise ValueError("id_to_block_type 必须是字典，例如 {1: 'R2', 2: 'FAKE'}")
        for pile_id, block_type in id_to_block_type.items():
            if pile_id in self.piles:
                self.piles[pile_id]._set_block_type(block_type, self.r1_penalty, self.normal_cost)

    def _set_block_type_by_id(self, pile_id, block_type):
        if pile_id not in self.piles:
            raise ValueError(f"无效的 pile_id: {pile_id}，必须在 1..12 之间")
        self.piles[pile_id]._set_block_type(block_type, self.r1_penalty, self.normal_cost)

    def count_types(self):
        counts = {"R2": 0, "R1": 0, "FAKE": 0}
        for pile in self.piles.values():
            if pile.block_type in counts:
                counts[pile.block_type] += 1
        return counts