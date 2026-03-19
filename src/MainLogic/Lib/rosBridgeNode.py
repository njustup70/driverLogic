from rclpy.node import Node
import asyncio
from std_msgs.msg import UInt8MultiArray, String
# 导入ros2坐标管理依赖
from tf2_ros import TransformListener, Buffer, TransformBroadcaster, StaticTransformBroadcaster
# 导入Odom类
from MainLogic.Lib.odomVec import Odom

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
        self._loop: asyncio.events.AbstractEventLoop 
        self._subPool = []
        self._pubPool = []
        # 创建tf2坐标管理器
        self._tfBuffer = Buffer()
        self._tfListener = TransformListener(self._tfBuffer, self)
        self._tfBroadcaster = TransformBroadcaster(self)
        self._staticTfBroadcaster = StaticTransformBroadcaster(self)
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
        #self.get_logger().info(f"发送数据:  {data.hex()}")
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

    def publish_dynamic_tf(self, parent_frame: str, child_frame: str, odom: Odom):
        """发布动态坐标变换（会持续覆盖同名 child_frame 的最新值）。"""
        tf_msg = odom.to_transform_stamped(
            parent_frame=parent_frame,
            child_frame=child_frame,
            stamp=self.get_clock().now().to_msg(),
        )
        self._tfBroadcaster.sendTransform(tf_msg)

    def publish_static_tf(self, parent_frame: str, child_frame: str, odom: Odom):
        """发布静态坐标变换（通常只需在启动或参数更新时发布）。"""
        tf_msg = odom.to_transform_stamped(
            parent_frame=parent_frame,
            child_frame=child_frame,
            stamp=self.get_clock().now().to_msg(),
        )
        self._staticTfBroadcaster.sendTransform(tf_msg)

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
RosBridgeNodeInstance: rosBridgeNode 
