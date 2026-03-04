'''
代码的总入口，负责创建ROS2节点，异步主函数，订阅话题，并将数据传递给异步任务。
'''
import asyncio,threading
import rclpy,rclpy.time
from roslib.rosBridgeNode import MainNode 
import async_main
#重要全局变量
asyncio_event_loop = asyncio.get_event_loop()
mainNode= MainNode(asyncio_event_loop)
def main():
    #创建并启动 asyncio 的后台线程
    t=threading.Thread(target=asyncio_event_loop.run_forever, daemon=True)
    t.start()
    rclpy.init()
    #注册异步任务
    asyncio.run_coroutine_threadsafe(async_main.async_main(), asyncio_event_loop)
    try:
        # 3. 主线程被 ROS 2 占据，负责处理所有传感器/通信回调
        rclpy.spin(mainNode)
    except KeyboardInterrupt:
        pass
    finally:
        asyncio_event_loop.call_soon_threadsafe(asyncio_event_loop.stop)
        mainNode.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()