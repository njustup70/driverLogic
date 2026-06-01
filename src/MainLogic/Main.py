'''
代码的总入口，负责创建ROS2节点，异步主函数，订阅话题，并将数据传递给异步任务。
'''
import argparse
import asyncio
import importlib
import inspect
import multiprocessing
import os
import threading
import traceback
import rclpy
from rclpy.executors import MultiThreadedExecutor,SingleThreadedExecutor
from MainLogic.core import ros_bridge_node as ros_bridge_module

# 设置多进程启动方法为 'spawn'，确保子进程完全独立，不共享 ROS2 上下文
multiprocessing.set_start_method('spawn', force=True)

# 注意：不要在全局作用域调用 rclpy.init()，因为主进程和子进程需要各自初始化
# 重要全局变量
asyncioEventLoop = asyncio.new_event_loop() 
# User used get_event_loop() but usually new_event_loop() or get_running_loop() is safer in modern python, 
# but sticking to pattern, but cleaning variable name.
# Wait, original code: asyncio_event_loop = asyncio.get_event_loop(). 
# I will use asyncio.get_event_loop() to maintain logic, just rename variable.
asyncioEventLoop = asyncio.get_event_loop()


def _load_async_entry(main_module: str, main_func: str):
    """动态加载 MAIN 下的目标模块与协程函数。"""
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
        print(f"[Main] async entry crashed: {e!r}")
        traceback.print_exc()


def main():
    parser = argparse.ArgumentParser(description='MainLogic entry selector')
    parser.add_argument('--main-module', default=os.getenv('MAIN_MODULE', 'R1n_Main'))
    parser.add_argument('--main-func', default=os.getenv('MAIN_FUNC', 'async_main'))
    args, _ = parser.parse_known_args()

    entry_func = _load_async_entry(args.main_module, args.main_func)

    # 在主进程中初始化 ROS2
    rclpy.init()
    
    # 创建并启动 asyncio 的后台线程
    t = threading.Thread(target=asyncioEventLoop.run_forever, daemon=True)
    t.start()
    #创建ROS2节点与多线程执行器
    executor = SingleThreadedExecutor()
    '''需要用命名空间来保证修改修改的是全局变量'''
    '''原来的from MainLogic.core.ros_bridge_node import rosBridgeNode,RosBridgeNodeInstance'''
    '''是在本地命名空间里创建了RosBridgeNodeInstance,修改的是本地的RosBridgeNodeInstance,而不是全局的RosBridgeNodeInstance'''
    '''另外如果在本地命名空间创建全局变量要赋值的话,需要global关键字声明'''
    # ros_bridge_module.RosBridgeNodeInstance = ros_bridge_module.rosBridgeNode()
    ros_bridge_module.RosBridgeNodeInstance.init()
    ros_bridge_module.RosBridgeNodeInstance.register_event_loop(asyncioEventLoop)
    # 只把 bridge 节点加入主进程的 executor
    executor.add_node(ros_bridge_module.RosBridgeNodeInstance)

    # 注册异步任务（确保 RosBridgeNodeInstance 已经初始化）
    main_future = asyncio.run_coroutine_threadsafe(entry_func(), asyncioEventLoop)
    main_future.add_done_callback(_report_async_future_result)
    for i in range(5):
        print(f"\033[95m[Main] running MAIN.{args.main_module}.{args.main_func}\033[0m")

    try:
        # 3. 主线程被 ROS 2 占据，负责处理所有传感器/通信回调
        # rclpy.spin(mainNode)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # 1. 停止通信层
        asyncioEventLoop.call_soon_threadsafe(asyncioEventLoop.stop)
        executor.shutdown()
        
        # 2. 简洁处理所有子进程
        for p in multiprocessing.active_children():
            p.terminate()
            p.join(timeout=2)
            if p.is_alive(): p.kill() # Python 3.7+ 
            
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
