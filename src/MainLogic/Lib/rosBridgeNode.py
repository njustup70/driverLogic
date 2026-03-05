from typing import Optional

from rclpy.node import Node
import asyncio, threading,json
import rclpy, rclpy.time
from std_msgs.msg import String
# 导入ros2坐标管理依赖
from tf2_ros import TransformListener, Buffer
# 导入驱动
from app.TFManager import TFManagerInstance
# 导入Odom类
from Lib.odomVec import Odom
from Lib.bytes import DataFrame
class rosBridgeNode(Node):
    '''
    ros2耦合节点,从ros2话题获得数据,传给类或者队列函数，需要耦合TFManager管理坐标
    '''
    def __init__(self, loop):
        super().__init__('main_node')
        print("MainNode initialized")
        # 话题订阅
        assert isinstance(loop, asyncio.events.AbstractEventLoop), "传入的 loop 必须是 asyncio 的事件循环"
        self._serial_tx_pub=self.create_publisher(String, 'serial_tx', 10)
        self._loop: asyncio.events.AbstractEventLoop = loop
        self._subPool=[]
        # 创建tf2坐标管理器
        self._tfBuffer = Buffer()
        self._tfListener = TransformListener(self._tfBuffer, self)
        self.create_timer(0.01, self.tf_timer_callback)  # 定时器回调，频率为100Hz
        self._tfOffline = False
        self._serial_callbacks=[]
    def register_ros2_sub(self,topic_name,callback):
        #注册ros2话题回调
        sub=self.create_subscription(String, topic_name, callback, 10)
        self._subPool.append(sub)
    def register_serial_sub(self, callback):
        #注册串口数据回调
        self._serial_callbacks.append(callback)
    def writeBytes(self, data: bytes):
        #给下位机发送数据
        dataFrame = DataFrame(data)
        dataFrame.data = b'\xFA' + dataFrame.data
        json_str = json.dumps(dataFrame.to_dict())
        msg = String()
        msg.data = json_str
        self._serial_tx_pub.publish(msg)

    def _serial_rx_callback(self, msg: String):
        # 内部函数
        # 从串口收到数据的回调，将数据转发给注册的串口回调函数
        try:
            data_dict = json.loads(msg.data)
            dataFrame = DataFrame.from_dict(data_dict)
            if self._serial_callbacks:
                for callback in self._serial_callbacks:
                    callback(dataFrame.data)
        except Exception as e:
            self.get_logger().warning(f"Failed to parse JSON from serial data: {e}")
    def tf_timer_callback(self):
        #坐标变换原始数据回调
        try:
            # 尝试获取从 "base_link" 到 "odom" 的坐标变换
            transform = self._tfBuffer.lookup_transform("map", "base_link", rclpy.time.Time())
            transTuple = (transform.transform.translation.x, transform.transform.translation.y, transform.transform.rotation.z)
            TFManagerInstance.baseLinkOdom = Odom(*transTuple)
            if self._tfOffline:
                self.get_logger().info("TF is back online!")
                self._tfOffline = False
        except Exception as e:
            if not self._tfOffline:
                self.get_logger().warn(f"TF lookup failed: {e}")
                self._tfOffline = True
            return
#声明类，不初始化，在Main.py中初始化
#因为需要rclpy.init之后才能创建Node实例
RosBridgeNodeInstance: Optional[rosBridgeNode] = None