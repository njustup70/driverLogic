from typing import Optional
from rclpy.node import Node
import asyncio, threading
import math
import rclpy, rclpy.time
from std_msgs.msg import UInt8MultiArray, String
import json
# 导入ros2坐标管理依赖
from tf2_ros import TransformListener, Buffer
# 导入驱动
import app.TFManager as tf_manager_module
# 导入Odom类
from Lib.odomVec import Odom
from Lib.bytes import turn_to_bytes
class rosBridgeNode(Node):
    '''
    ros2耦合节点,从ros2话题获得数据,传给类或者队列函数，需要耦合TFManager管理坐标
    '''
    def __init__(self):
        super().__init__('main_node')
        print("MainNode initialized")
        # 话题订阅
        self._serial_tx_pub = self.create_publisher(UInt8MultiArray, 'serial_tx', 10)
        self._serial_rx_sub = self.create_subscription(UInt8MultiArray, 'serial_rx', self._serial_rx_callback, 10)
        self._location_pub = self.create_publisher(String, 'location', 10)
        self._loop: asyncio.events.AbstractEventLoop 
        self._subPool = []
        self._pubPool = []
        # 创建tf2坐标管理器
        self._tfBuffer = Buffer()
        self._tfListener = TransformListener(self._tfBuffer, self)
        self.create_timer(0.01, self.tf_timer_callback)  # 定时器回调，频率为100Hz
        self._tfOffline = False
        self._serial_callbacks = []

    def register_event_loop(self, loop: asyncio.events.AbstractEventLoop):
        assert isinstance(loop, asyncio.events.AbstractEventLoop), "传入的 loop 必须是 asyncio 的事件循环"
        self._loop = loop

    def register_ros2_sub(self, topic_name, callback, type=UInt8MultiArray):
        # 注册ros2话题回调
        sub = self.create_subscription(type, topic_name, callback, 10)
        self._subPool.append(sub)

    def register_serial_sub(self, callback):
        # 注册串口数据回调
        self._serial_callbacks.append(callback)

    def writeBytes(self, data: bytes):
        # 给下位机发送数据，增加 \xFA 帧头，直接使用原始字节
        self.get_logger().info(f"发送数据:  {data.hex()}")
        msg = UInt8MultiArray()
        msg.data = list(b'\xFA' + data)
        self._serial_tx_pub.publish(msg)

    def _serial_rx_callback(self, msg: UInt8MultiArray):
        # 内部函数
        # 从串口收到数据的回调，将数据转发给注册的串口回调函数
        if self._serial_callbacks:
            data = bytes(msg.data)
            for callback in self._serial_callbacks:
                callback(data)

    def tf_timer_callback(self):
        # 坐标变换原始数据回调
        try:
            # 尝试获取从 "base_link" 到 "odom" 的坐标变换
            transform = self._tfBuffer.lookup_transform("map", "base_link", rclpy.time.Time())
            yaw = math.atan2(2 * (transform.transform.rotation.w * transform.transform.rotation.z + transform.transform.rotation.x * transform.transform.rotation.y), 1 - 2 * (transform.transform.rotation.z**2))
            transTuple = (transform.transform.translation.x, transform.transform.translation.y, yaw)
            tf_manager_module.TFManagerInstance.baseLinkOdom = Odom(*transTuple)
            transTuple_odom = Odom(*transTuple)
            send_tf = turn_to_bytes([transTuple_odom.x, transTuple_odom.y, transTuple_odom.yaw])
            #self.get_logger().info(f"发送数据:  {send_tf.hex()}")
            self.writeBytes(b'\xA0' + send_tf)
            pub_msg = String()
            pub_msg.data = json.dumps([transTuple_odom.x, transTuple_odom.y, transTuple_odom.yaw])
            self.publish_ros2('location', pub_msg) 
            if self._tfOffline:
                self.get_logger().info("TF is back online!")
                self._tfOffline = False

        except Exception as e:
            if not self._tfOffline:
                self.get_logger().warn(f"TF lookup failed: {e}")
                self._tfOffline = True
            return
    def register_ros2_pub(self, topic_name, msg_type):
        # 注册ros2话题发布
        pub = self.create_publisher(msg_type, topic_name, 10)
        self._pubPool.append(pub)
        
    def publish_ros2(self, topic_name, msg):
        # 发布ros2话题
        for pub in self._pubPool:
            if pub.topic_name == topic_name:
                msg = msg if isinstance(msg, pub.msg_type) else pub.msg_type(data=msg)
                pub.publish(msg)
                break

# 声明类，不初始化，在Main.py中初始化
# 因为需要rclpy.init之后才能创建Node实例
RosBridgeNodeInstance: rosBridgeNode = rosBridgeNode()
