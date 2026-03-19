import asyncio
from MainLogic.Lib.AsyncTools import async_property


serial_action_finish = async_property(bytes)
async def check_finish(action_type , timeout = 500):
    check_time = 0
    while serial_action_finish != action_type:
        if check_time > timeout:
            print(f"等待动作完成超时: {action_type}")
            return
        await asyncio.sleep(0.01)  # 每100ms检查一次状态
        check_time += 1