'''
异步主逻辑和其他函数
'''
import asyncio
import os
from MainLogic.core import ros_bridge_node as ros_bridge_module
from MainLogic.core.serial_node import start_serial_process
from MainLogic.app.zone2_model_api import zone2_model_api

ZONE2_DEMO_SEED = os.getenv("ZONE2_DEMO_SEED")
SERIAL_PORT = os.getenv("SERIAL_PORT", "/dev/ttyACM0")
SERIAL_BAUDRATE = int(os.getenv("SERIAL_BAUDRATE", "115200"))
async def async_main():
    
    """融合后的 main：先跑 zone2 演示，再执行原 ROS 回调与动作逻辑。"""
    # start_serial_process(serial_port=SERIAL_PORT, baudrate=SERIAL_BAUDRATE)

    seed = None if ZONE2_DEMO_SEED is None else int(ZONE2_DEMO_SEED)

    move_cost = 1.0
    pick_cost = 2.0
    turn_cost = 0.5
    r1_remove_cost = 0.01
    required_r2_count = 3

    result = zone2_model_api.demo_visualize_random_map(
        seed=seed,
        show=False,
        move_cost=move_cost,
        pick_cost=pick_cost,
        turn_cost=turn_cost,
        r1_remove_cost=r1_remove_cost,
        required_r2_count=required_r2_count,
    )
    print(f"[R2_Main] zone2 demo finished: found={result.get('found')} cost={result.get('cost')} image={result.get('image_path')}")
    zone2_model_api.print_path_debug_info(result)
    
    zone2_model_api.visualize_path_result(result, show=True)
    zone2_model_api.send_mcu_action_frame_to_mcu(result)
    while True:
        await asyncio.sleep(1)
