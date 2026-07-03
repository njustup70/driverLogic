import argparse
import asyncio
import importlib
import inspect
import multiprocessing
import os
import threading
import traceback
import rclpy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from MainLogic.core import ros_bridge_node as ros_bridge_module

# 设置多进程启动方法为 'spawn'，确保子进程完全独立，不共享 ROS2 上下文
multiprocessing.set_start_method('spawn', force=True)

# 注意：不要在全局作用域调用 rclpy.init()，因为主进程和子进程需要各自初始化
# 重要全局变量
asyncioEventLoop = asyncio.new_event_loop()
asyncioEventLoop = asyncio.get_event_loop()

# ---- R1 专用：只允许加载 R1 系列模块 ----
_R1_ALLOWED_MODULES = {
    'R1_Main',
    'R1n_Main',
}


def _load_async_entry(main_module: str, main_func: str):
    """动态加载 MAIN 下的目标模块与协程函数（仅限 R1 模块）。"""
    if main_module not in _R1_ALLOWED_MODULES:
        raise ValueError(
            f"[Main_R1] 不允许加载非 R1 模块: '{main_module}'。"
            f" 允许的模块: {sorted(_R1_ALLOWED_MODULES)}"
        )
    module_name = f"MainLogic.MAIN.{main_module}"
    module = importlib.import_module(module_name)
    entry = getattr(module, main_func, None)
    if entry is None:
        raise AttributeError(f"{module_name} does not contain function '{main_func}'")
    if not inspect.iscoroutinefunction(entry):
        raise TypeError(f"{module_name}.{main_func} must be an async function")
    return entry


def _report_async_future_result(fut):
    """打印后台协程失败堆栈，避免静默退出。"""
    try:
        fut.result()
    except Exception as e:
        print(f"[Main_R1] async entry crashed: {e!r}")
        traceback.print_exc()


def main():
    parser = argparse.ArgumentParser(description='MainLogic R1 entry selector')
    parser.add_argument(
        '--main-module',
        default=os.getenv('MAIN_MODULE_R1', 'R1n_Main'),
    )
    parser.add_argument(
        '--main-func',
        default=os.getenv('MAIN_FUNC_R1', 'async_main'),
    )
    args, _ = parser.parse_known_args()

    entry_func = _load_async_entry(args.main_module, args.main_func)

    # 在主进程中初始化 ROS2
    rclpy.init()

    # 创建并启动 asyncio 的后台线程
    t = threading.Thread(target=asyncioEventLoop.run_forever, daemon=True)
    t.start()

    # 创建ROS2节点与单线程执行器
    executor = SingleThreadedExecutor()
    ros_bridge_module.RosBridgeNodeInstance.init()
    ros_bridge_module.RosBridgeNodeInstance.register_event_loop(asyncioEventLoop)
    # 只把 bridge 节点加入主进程的 executor
    executor.add_node(ros_bridge_module.RosBridgeNodeInstance)

    # 注册异步任务（确保 RosBridgeNodeInstance 已经初始化）
    main_future = asyncio.run_coroutine_threadsafe(entry_func(), asyncioEventLoop)
    main_future.add_done_callback(_report_async_future_result)
    for i in range(5):
        print(f"\033[95m[Main_R1] running MAIN.{args.main_module}.{args.main_func}\033[0m")

    try:
        # 主线程被 ROS 2 占据，负责处理所有传感器/通信回调
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # 停止通信层
        asyncioEventLoop.call_soon_threadsafe(asyncioEventLoop.stop)
        executor.shutdown()

        # 清理所有子进程
        for p in multiprocessing.active_children():
            p.terminate()
            p.join(timeout=2)
            if p.is_alive():
                p.kill()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
