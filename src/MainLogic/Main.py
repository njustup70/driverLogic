'''
代码的总入口，负责创建ROS2节点，异步主函数，订阅话题，并将数据传递给异步任务。
'''
import asyncio, threading
import multiprocessing, os, signal, time
import rclpy, rclpy.time
from rclpy.executors import MultiThreadedExecutor
import Lib.rosBridgeNode as ros_bridge_module
import Lib.rosSerialNode as ros_serial_module
import asyncMain

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

def main():
    # 在主进程中初始化 ROS2
    rclpy.init()
    
    # 创建并启动 asyncio 的后台线程
    t = threading.Thread(target=asyncioEventLoop.run_forever, daemon=True)
    t.start()
    # rclpy.init()
    # 注册异步任务
    asyncio.run_coroutine_threadsafe(asyncMain.async_main(), asyncioEventLoop)
    #创建ROS2节点与多线程执行器
    executor = MultiThreadedExecutor()
    '''需要用命名空间来保证修改修改的是全局变量'''
    '''原来的from Lib.rosBridgeNode import rosBridgeNode,RosBridgeNodeInstance'''
    '''是在本地命名空间里创建了RosBridgeNodeInstance,修改的是本地的RosBridgeNodeInstance,而不是全局的RosBridgeNodeInstance'''
    '''另外如果在本地命名空间创建全局变量要赋值的话,需要global关键字声明'''
    # ros_bridge_module.RosBridgeNodeInstance = ros_bridge_module.rosBridgeNode()
    ros_bridge_module.RosBridgeNodeInstance=ros_bridge_module.rosBridgeNode()
    ros_bridge_module.RosBridgeNodeInstance.register_event_loop(asyncioEventLoop)
    # ros_serial_module.RosSerialNodeInstance = ros_serial_module.SerialNode()
    # 只把 bridge 节点加入主进程的 executor
    executor.add_node(ros_bridge_module.RosBridgeNodeInstance)

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
            
        rclpy.shutdown()

if __name__ == '__main__':
    main()
