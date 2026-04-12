import asyncio
from enum import Enum
from typing import Dict, Tuple, Callable, Awaitable, Optional,Coroutine, Any

class MachineState(str, Enum):
    IDLE = "IDLE"
    PLACE_LAYER2 = "PLACE_LAYER2"
    PLACE_LAYER3 = "PLACE_LAYER3"
    INTERRUPTED = "INTERRUPTED"
    _FINISHED = "FINISH"  # 可选：表示流程正常结束的状态


# 定义流程函数的类型
FlowHandler = Callable[[], Coroutine[Any, Any, None]]

class StateMachine:
    """
    精简版状态机：
    1. 靠 state + table 实现天然锁，无需额外 busy 标志。
    2. 利用 Task.cancel() 实现即时打断，无需业务代码配合检查。
    """

    def __init__(self) -> None:
        self.state: MachineState = MachineState.IDLE
        self._current_task: Optional[asyncio.Task] = None

        # 状态转移表：(当前状态, 命令) -> (目标状态, 执行流程)
        self._table: Dict[Tuple[MachineState, str], Tuple[MachineState, FlowHandler]] = {
            (MachineState.IDLE, "layer2"): (MachineState.PLACE_LAYER2, self._run_place_layer2_flow),
            (MachineState.IDLE, "layer3"): (MachineState.PLACE_LAYER3, self._run_place_layer3_flow),
            (MachineState.PLACE_LAYER2,"FINISH"): (MachineState.IDLE, self._idle_flow),
            (MachineState.PLACE_LAYER3,"FINISH"): (MachineState.IDLE, self._idle_flow),
            # 可以根据需要添加更多状态和命令
        }
    async def trigger(self, command: str) -> bool:
        """统一触发入口"""
        # 1. 查表：如果当前状态不对应，或者命令不存在，直接拒绝
        entry = self._table.get((self.state, command))
        if not entry:
            print(f"[拒绝] 当前状态 {self.state} 无法处理命令: {command}")
            return False

        next_state, handler = entry
        
        # 2. 切换状态（此时 state 变为非 IDLE，后续 trigger 会被查表逻辑挡掉）
        self.state = next_state
        
        # 3. 创建并执行异步任务
        self._current_task = asyncio.create_task(handler())
        
        try:
            await self._current_task
            return True
        except asyncio.CancelledError:
            # 当调用 interrupt 时，await 处会立即抛出此异常
            print(f"[中断] {next_state} 流程被强行终止")
            return False
        except Exception as e:
            print(f"[错误] 流程执行异常: {e}")
            return False
        finally:
            # 4. 善后：如果不是被打断进入了 INTERRUPTED 态，则恢复 IDLE
            if self.state != MachineState.INTERRUPTED:
                # await self.trigger(self.state)
                self.state = MachineState.IDLE

    def interrupt(self):
        """外部中断接口：瞬间杀死正在跑的流程"""
        if self._current_task and not self._current_task.done():
            self.state = MachineState.INTERRUPTED
            self._current_task.cancel()
            print("正在触发中断...")
        else:
            self.state = MachineState.INTERRUPTED
            print("当前无活跃流程，已直接切至中断态")

    # --- 业务流程（这里面不需要写任何中断检查，保持逻辑纯粹） ---
    async def _idle_flow(self) -> None:
        #模拟回到空闲点
        await asyncio.sleep(4)  # 模拟空闲状态的等待
        print("进入空闲状态，等待命令...")
    async def _run_place_layer2_flow(self) -> None:
        print("-> 启动放二层流程")
        await asyncio.sleep(1)  # 模拟通信
        print("-> 正在移动机械臂")
        await asyncio.sleep(2)  # 模拟运动
        print("-> 放置动作完成")
        self.state = MachineState._FINISHED
    async def _run_place_layer3_flow(self) -> None:
        print("-> 启动放三层流程")
        await asyncio.sleep(3)
        print("-> 放置动作完成")
        self.state = MachineState._FINISHED
# --- 测试代码 ---
async def main():
    sm = StateMachine()

    # 1. 正常触发
    print(f"当前状态: {sm.state}")
    await sm.trigger("layer2")
    
    await asyncio.sleep(0.5)
    print(f"执行中状态: {sm.state}")

    # 2. 模拟中途打断
    await asyncio.sleep(0.5)
    # await sm.interrupt()
    
    print(f"中断后状态: {sm.state}")
    
    # 3. 尝试在中断态再次触发（应该失败）
    success = await sm.trigger("layer2")
    print(f"中断态触发结果: {success}")
    while True:
        await asyncio.sleep(1)
if __name__ == "__main__":
    asyncio.run(main())