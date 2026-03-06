'''
代码的总入口，负责创建ROS2节点，异步主函数，订阅话题，并将数据传递给异步任务。
'''
import asyncio, threading
import rclpy, rclpy.time
#需要在最开始初始化ROS2,不然节点初始化会报错
rclpy.init()
from rclpy.executors import MultiThreadedExecutor
import Lib.rosBridgeNode as ros_bridge_module
import Lib.rosSerialNode as ros_serial_module
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
    # rclpy.init()
    # 注册异步任务
    asyncio.run_coroutine_threadsafe(asyncMain.async_main(), asyncioEventLoop)
    #创建ROS2节点与多线程执行器
    executor = MultiThreadedExecutor()
    '''需要用命名空间来保证修改修改的是全局变量'''
    '''原来的from Lib.rosBridgeNode import rosBridgeNode,RosBridgeNodeInstance'''
    '''是在本地命名空间里创建了RosBridgeNodeInstance,修改的是本地的RosBridgeNodeInstance,而不是全局的RosBridgeNodeInstance'''
    '''另外如果在本地命名空间创建全局变量要赋值的话,需要global关键字声明'''
    ros_bridge_module.RosBridgeNodeInstance = ros_bridge_module.rosBridgeNode()
    ros_bridge_module.RosBridgeNodeInstance.register_event_loop(asyncioEventLoop)
    ros_serial_module.RosSerialNodeInstance = ros_serial_module.SerialNode()

    executor.add_node(ros_bridge_module.RosBridgeNodeInstance)
    executor.add_node(ros_serial_module.RosSerialNodeInstance)
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
