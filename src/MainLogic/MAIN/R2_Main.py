'''
异步主逻辑和其他函数
'''
import asyncio
import os
from MainLogic.app.zone2_model_api import zone2_model_api
# from MainLogic.core.serial_node import start_serial_process

ZONE2_DEMO_SEED = os.getenv("ZONE2_DEMO_SEED")


async def async_main():
    """融合后的 main：先跑 zone2 演示，再执行原 ROS 回调与动作逻辑。"""
    seed = None if ZONE2_DEMO_SEED is None else int(ZONE2_DEMO_SEED)

    move_cost = 1.0
    pick_cost = 2.0
    turn_cost = 0.5
    required_r2_count = 3

    result = zone2_model_api.demo_visualize_random_map(
        seed=seed,
        show=True,
        move_cost=move_cost,
        pick_cost=pick_cost,
        turn_cost=turn_cost,
        required_r2_count=required_r2_count,
    )
    print(f"[R2_Main] zone2 demo finished: found={result.get('found')} cost={result.get('cost')} image={result.get('image_path')}")

    # serial_port = '/dev/ttyACM0'
    # baudrate = 115200
    # start_serial_process(serial_port=serial_port, baudrate=baudrate)

    while True:
        await asyncio.sleep(1)
