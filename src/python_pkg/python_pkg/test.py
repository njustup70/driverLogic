import asyncio
from rclpy.node import Node
import rclpy
from std_msgs.msg import String

class AsyncNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.create_subscription(String, '/test', self.msg_callback, 10)
        self.event=asyncio.Event()
        self.pub=self.create_publisher(String,'/test_out',10)
        # self.tim=self.create_timer(1.0,self.timer_callback)
    # def timer_callback(self):
    #     msg=String()
    #     msg.data='Hello from AsyncNode'
    #     self.pub.publish(msg)
    async def msg_callback(self, msg):
        print(f'Message received: {msg.data}')
        self.event.set()
    async def asynctask(self):
        while not self.event.is_set():
            
            await asyncio.sleep(1)

rclpy.init()
node = AsyncNode('async_node')
async def main():
    print('Node started.')
    print('Listening to topic /test...')
    await node.asynctask()
    print('异步任务完成')
async def ros_loop():
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0)
        await asyncio.sleep(1e-5)
if __name__ == "__main__":
    future = asyncio.wait([ros_loop(), main()])
    asyncio.get_event_loop().run_until_complete(future)