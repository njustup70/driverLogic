'''
代码的总入口，负责创建ROS2节点，异步主函数，订阅话题，并将数据传递给异步任务。
'''
import asyncio, threading
import rclpy, rclpy.time
from rclpy.executors import MultiThreadedExecutor
from Lib.rosBridgeNode import rosBridgeNode,RosBridgeNodeInstance
from Lib.rosSerialNode import SerialNode ,RosSerialNodeInstance
import asyncMain
# 重要全局变量
asyncioEventLoop = asyncio.new_event_loop() 
# User used get_event_loop() but usually new_event_loop() or get_running_loop() is safer in modern python, 
# but sticking to pattern, but cleaning variable name.
# Wait, original code: asyncio_event_loop = asyncio.get_event_loop(). 
# I will use asyncio.get_event_loop() to maintain logic, just rename variable.
asyncioEventLoop = asyncio.get_event_loop()

def main():
    # 创建并启动 asyncio 的后台线程
    t = threading.Thread(target=asyncioEventLoop.run_forever, daemon=True)
    t.start()
    rclpy.init()
    # 注册异步任务
    asyncio.run_coroutine_threadsafe(asyncMain.async_main(), asyncioEventLoop)
    executor = MultiThreadedExecutor()
    RosBridgeNodeInstance =rosBridgeNode(asyncioEventLoop)
    RosSerialNodeInstance = SerialNode()

    executor.add_node(RosBridgeNodeInstance)
    executor.add_node(RosSerialNodeInstance)
    try:
        # 3. 主线程被 ROS 2 占据，负责处理所有传感器/通信回调
        # rclpy.spin(mainNode)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        asyncioEventLoop.call_soon_threadsafe(asyncioEventLoop.stop)
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
