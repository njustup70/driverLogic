from rclpy.node import Node
import asyncio
from std_msgs.msg import UInt8MultiArray, String
from tf2_ros import TransformListener, Buffer, TransformBroadcaster, StaticTransformBroadcaster
from MainLogic.Lib.odomVec import Odom


class rosBridgeNode(Node):
    """ROS2 bridge node for serial and TF related communication."""

    def __init__(self):
        super().__init__('main_node')
        print('MainNode initialized')
        self._serial_tx_pub = self.create_publisher(UInt8MultiArray, 'serial_tx', 10)
        self._serial_rx_sub = self.create_subscription(UInt8MultiArray, 'serial_rx', self._serial_rx_callback, 10)
        self._loop: asyncio.events.AbstractEventLoop
        self._subPool = []
        self._pubPool = []
        self._tfBuffer = Buffer()
        self._tfListener = TransformListener(self._tfBuffer, self)
        self._tfBroadcaster = TransformBroadcaster(self)
        self._staticTfBroadcaster = StaticTransformBroadcaster(self)
        self._serial_callbacks = []

    def register_event_loop(self, loop: asyncio.events.AbstractEventLoop):
        assert isinstance(loop, asyncio.events.AbstractEventLoop), '传入的 loop 必须是 asyncio 的事件循环'
        self._loop = loop

    def register_ros2_sub(self, topic_name, callback, type=UInt8MultiArray):
        sub = self.create_subscription(type, topic_name, callback, 10)
        self._subPool.append(sub)

    def register_serial_sub(self, callback):
        self._serial_callbacks.append(callback)

    def writeBytes(self, data: bytes):
        msg = UInt8MultiArray()
        msg.data = list(b'\xFA' + data)
        self._serial_tx_pub.publish(msg)

    def _serial_rx_callback(self, msg: UInt8MultiArray):
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
        pub = self.create_publisher(msg_type, topic_name, 10)
        self._pubPool.append(pub)

    def publish_ros2(self, topic_name, msg):
        for pub in self._pubPool:
            if pub.topic_name == topic_name:
                msg = msg if isinstance(msg, pub.msg_type) else pub.msg_type(data=msg)
                pub.publish(msg)
                break


RosBridgeNodeInstance: rosBridgeNode
