import asyncio

from MainLogic.Lib.AsyncTools import async_property
from MainLogic.app.meilin_map_model import MeilinMap
from MainLogic.app.meilin_solver import SolverConfigError, solve_route


STATE_CODE_TO_NAME = {
    0: "EMPTY",
    1: "R2",
    2: "R1",
    3: "FAKE",
}


class MeilinManager:
    """梅林任务状态与求解编排。"""

    pile_states = async_property(lambda: ["EMPTY"] * 12)
    solver_mode = async_property(lambda: "MASTER")
    solve_request_seq = async_property(lambda: 0)
    latest_route = async_property(lambda: [])
    latest_cost = async_property(lambda: float("inf"))
    solve_busy = async_property(lambda: False)
    last_error = async_property(lambda: "")

    def update_pile_states(self, states12):
        """更新 12 桩状态。"""
        if len(states12) != 12:
            raise ValueError(f"桩状态数量必须为 12，当前为 {len(states12)}")

        normalized_states = []
        for state in states12:
            if state not in ("EMPTY", "R1", "R2", "FAKE"):
                raise ValueError(f"未知桩状态: {state}")
            normalized_states.append(state)

        self.pile_states = normalized_states
        print(f"梅林桩状态已更新: {normalized_states}")

    def request_solve(self):
        """触发一次重算请求。"""
        self.solve_request_seq = self.solve_request_seq.value + 1

    def _build_map(self):
        map_model = MeilinMap()
        for index, state in enumerate(self.pile_states.value, start=1):
            map_model._set_block_type_by_id(index, state)
        return map_model

    async def solve_loop(self):
        """等待重算请求并执行求解。"""
        last_seen_seq = self.solve_request_seq.value
        while True:
            await self.solve_request_seq
            current_seq = self.solve_request_seq.value
            if current_seq == last_seen_seq:
                continue
            if self.solve_busy.value:
                last_seen_seq = current_seq
                continue

            self.solve_busy = True
            try:
                map_model = self._build_map()
                route, total_cost = solve_route(self.solver_mode.value, map_model.piles)
                if not route:
                    raise SolverConfigError("当前布局无可行路径")
                self.latest_route = route
                self.latest_cost = total_cost
                self.last_error = ""
                print(f"梅林求解完成: mode={self.solver_mode.value}, cost={total_cost}, route_len={len(route)}")
            except Exception as exc:
                self.latest_route = []
                self.latest_cost = float("inf")
                self.last_error = str(exc)
                print(f"梅林求解失败: {exc}")
            finally:
                self.solve_busy = False
                last_seen_seq = current_seq


MeilinManagerInstance = MeilinManager()