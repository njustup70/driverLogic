import asyncio
from enum import Enum
from typing import Dict, Tuple, Optional, Callable, Any, Coroutine

class MachineState(str, Enum):
    IDLE = "IDLE"
    PLACE_LAYER2 = "PLACE_LAYER2"
    PLACE_LAYER3 = "PLACE_LAYER3"
    INTERRUPTED = "INTERRUPTED"

class Signal(str, Enum):
    TO_L2 = "TO_L2"
    TO_L3 = "TO_L3"
    FINISH = "FINISH"

ActionHandler = Callable[[], Coroutine[Any, Any, Optional[Signal]]]

class StateMachine:
    def __init__(self) -> None:
        self.state: MachineState = MachineState.IDLE
        self._current_task: Optional[asyncio.Task] = None

        # 状态转移表：(当前状态, 产生的信号) -> (目标状态, 下一个动作)
        self._table: Dict[Tuple[MachineState, Signal], Tuple[MachineState, ActionHandler]] = {
            # IDLE 状态下产生的信号决定去哪
            (MachineState.IDLE, Signal.TO_L2): (MachineState.PLACE_LAYER2, self._run_place_layer2_flow),
            (MachineState.IDLE, Signal.TO_L3): (MachineState.PLACE_LAYER3, self._run_place_layer3_flow),
            
            # 流程完成后回到 IDLE
            (MachineState.PLACE_LAYER2, Signal.FINISH): (MachineState.IDLE, self._idle_flow),
            (MachineState.PLACE_LAYER3, Signal.FINISH): (MachineState.IDLE, self._idle_flow),
        }

    async def start(self):
        """外部唯一启动入口：固定从 IDLE 开始"""
        if self._current_task and not self._current_task.done():
            print("[拒绝] 流程已经在运行中")
            return

        # 初始状态设为 IDLE
        self.state = MachineState.IDLE
        # 初始逻辑：由 IDLE 流程产生第一个驱动信号
        current_signal = await self._idle_flow()

        try:
            while True:
                # 1. 查表获取下一步
                entry = self._table.get((self.state, current_signal))
                if not entry:
                    print(f">>> 链路中断：状态 {self.state} 无法识别信号 {current_signal}")
                    break
                
                next_state, handler = entry
                self.state = next_state
                
                # 2. 执行动作并获取下一个信号
                self._current_task = asyncio.create_task(handler())
                current_signal = await self._current_task
                
                # 如果动作没返回信号，说明流程需要停下来等待或彻底结束
                if current_signal is None:
                    break

        except asyncio.CancelledError:
            print(f"<!> 流程在 {self.state} 被外部强制重置")
        except Exception as e:
            print(f"<!> 异常退出: {e}")
            self.state = MachineState.INTERRUPTED

    def reset(self):
        """暴力中断：保持原写法"""
        if self._current_task and not self._current_task.done():
            self._current_task.cancel()
        self.state = MachineState.INTERRUPTED
        print("[RESET] 系统已强行切至中断态")

    # --- 业务处理器 ---

    async def _idle_flow(self) -> Signal:
        """
        IDLE 动作：现在它决定了系统的去向
        你可以根据传感器、任务队列等逻辑返回不同的信号
        """
        print("-> [IDLE] 检查任务队列...")
        await asyncio.sleep(1)
        
        # 模拟内部决策逻辑
        import random
        choice = random.choice([Signal.TO_L2, Signal.TO_L3])
        
        if choice:
            print(f"-> [IDLE] 决策完成，准备执行: {choice}")
        else:
            print("-> [IDLE] 无新任务，停止循环")
        return choice

    async def _run_place_layer2_flow(self) -> Signal:
        print("-> [L2] 正在放置二层...")
        await asyncio.sleep(2)
        return Signal.FINISH

    async def _run_place_layer3_flow(self) -> Signal:
        print("-> [L3] 正在放置三层...")
        await asyncio.sleep(2)
        return Signal.FINISH
# --- 测试代码 ---
async def main():
    sm = StateMachine()

    # 1. 正常触发
    print(f"当前状态: {sm.state}")
    # await sm.trigger("layer2")
    
    await asyncio.sleep(0.5)
    print(f"执行中状态: {sm.state}")

    # 2. 模拟中途打断
    await asyncio.sleep(0.5)
    # await sm.interrupt()
    
    print(f"中断后状态: {sm.state}")
    
    # 3. 尝试在中断态再次触发（应该失败）
    # success = await sm.trigger("layer2")
    # print(f"中断态触发结果: {success}")
    await sm.start()
    while True:
        await asyncio.sleep(1)
if __name__ == "__main__":
    asyncio.run(main())