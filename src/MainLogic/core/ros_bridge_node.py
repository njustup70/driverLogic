from rclpy.node import Node
import asyncio
from std_msgs.msg import UInt8MultiArray, String
from tf2_ros import TransformListener, Buffer, TransformBroadcaster, StaticTransformBroadcaster
from MainLogic.Lib.odomVec import Odom


class rosBridgeNode(Node):
    '''
    ros2耦合节点,从ros2话题获得数据,传给类或者队列函数，需要耦合TFManager管理坐标
    '''

    def __init__(self):
        # 注意：这里不启动 rosSerialNode 的功能，而是提供一个接口让外部启动并注册回调
        print('Initializing RosBridgeNode...')

    def init(self):
        super().__init__('main_node')
        print('MainNode initialized')
        # 话题桥接：serial_tx 发给下位机，serial_rx 收下位机上传
        self._serial_tx_pub = self.create_publisher(UInt8MultiArray, 'serial_tx', 10)
        self._serial_rx_sub = self.create_subscription(UInt8MultiArray, 'serial_rx', self._serial_rx_callback, 10)
        self._loop: asyncio.events.AbstractEventLoop
               # 将 pub 和 sub 改为字典形式：{topic_name: publisher/subscription}
        self._pubDict = {}  # {topic_name: publisher}
        self._subDict = {}  # {topic_name: (subscription, msg_type)}
        # 创建 tf2 坐标管理器（Buffer + Listener + Broadcaster）
        self._tfBuffer = Buffer()
        self._tfListener = TransformListener(self._tfBuffer, self)
        self._tfBroadcaster = TransformBroadcaster(self)
        self._staticTfBroadcaster = StaticTransformBroadcaster(self)
        self._serial_callbacks = []

    def register_event_loop(self, loop: asyncio.events.AbstractEventLoop):
        assert isinstance(loop, asyncio.events.AbstractEventLoop), '传入的 loop 必须是 asyncio 的事件循环'
        self._loop = loop

    def register_ros2_sub(self, topic_name, callback, type=UInt8MultiArray):
        # 注册 ros2 话题回调
               # 注册 ros2 话题回调（字典形式）
               if topic_name not in self._subDict:
                   sub = self.create_subscription(type, topic_name, callback, 10)
                   self._subDict[topic_name] = (sub, type)
               else:
                   print(f"Warning: Topic '{topic_name}' already registered as subscriber")

    def register_serial_sub(self, callback):
        # 注册串口数据回调
        self._serial_callbacks.append(callback)

    def writeBytes(self, data: bytes):
        # 给下位机发送数据，增加 \xFA 帧头，直接使用原始字节
        msg = UInt8MultiArray()
        msg.data = list(b'\xFA' + data)
        self._serial_tx_pub.publish(msg)

    def _serial_rx_callback(self, msg: UInt8MultiArray):
        # 从串口收到数据后，转发给所有注册回调
        if self._serial_callbacks:
            data = bytes(msg.data)
            for callback in self._serial_callbacks:
                callback(data)

    def publish_dynamic_tf(self, parent_frame: str, child_frame: str, odom: Odom):
        tf_msg = odom.to_transform_stamped(
            parent_frame=parent_frame,
            child_frame=child_frame,
            stamp=self.get_clock().now().to_msg(),
        )
        self._tfBroadcaster.sendTransform(tf_msg)

    def publish_static_tf(self, parent_frame: str, child_frame: str, odom: Odom):
        tf_msg = odom.to_transform_stamped(
            parent_frame=parent_frame,
            child_frame=child_frame,
            stamp=self.get_clock().now().to_msg(),
        )
        self._staticTfBroadcaster.sendTransform(tf_msg)

    def register_ros2_pub(self, topic_name, msg_type):
        # 注册 ros2 话题发布器
        # 注册 ros2 话题发布器（字典形式）
        if topic_name not in self._pubDict:
            pub = self.create_publisher(msg_type, topic_name, 10)
            self._pubDict[topic_name] = pub
        else:
            # print(f"Warning: Topic '{topic_name}' already registered as publisher")
            raise ValueError(f"Topic '{topic_name}' already registered as publisher")
    def publish_ros2(self, topic_name, msg):
        # 按 topic_name 定位发布器并发布消息
        # 按 topic_name 定位发布器并发布消息
        # 如果话题未注册，自动从 msg 推导类型并注册
        if topic_name not in self._pubDict:
            msg_type = type(msg)
            self.register_ros2_pub(topic_name, msg_type)
       
        pub = self._pubDict[topic_name]
        # 检查 msg 类型是否匹配
        if not isinstance(msg, pub.msg_type):
            raise TypeError(f"Message type {type(msg)} does not match publisher type {pub.msg_type} for topic '{topic_name}'")
        pub.publish(msg)


# 声明类实例引用（在 Main.py 中调用 init 初始化）
RosBridgeNodeInstance: rosBridgeNode = rosBridgeNode()
