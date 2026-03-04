from rclpy.node import Node
import asyncio, threading
import rclpy, rclpy.time
from std_msgs.msg import String
# 导入ros2坐标管理依赖
from tf2_ros import TransformListener, Buffer
# 导入驱动
from data import chassicInstance
# 导入Odom类
from Lib.odomVec import Odom

class MainNode(Node):
    '''
    ros2耦合节点,从ros2话题获得数据,传给类或者队列函数
    '''
    def __init__(self, loop):
        super().__init__('main_node')
        print("MainNode initialized")
        # 话题订阅
        self.sub = self.create_subscription(String, 'trigger', self.listener_callback, 10)
        assert isinstance(loop, asyncio.events.AbstractEventLoop), "传入的 loop 必须是 asyncio 的事件循环"
        self._loop: asyncio.events.AbstractEventLoop = loop
        # 创建tf2坐标管理器
        self._tfBuffer = Buffer()
        self._tfListener = TransformListener(self._tfBuffer, self)
        self.create_timer(0.01, self.tf_timer_callback)  # 定时器回调，频率为100Hz
        self._tfOffline = False

    def listener_callback(self, msg):
        self.get_logger().info(f"Received ROS message: {msg.data}")

    def tf_timer_callback(self):
        try:
            # 尝试获取从 "base_link" 到 "odom" 的坐标变换
            transform = self._tfBuffer.lookup_transform("map", "base_link", rclpy.time.Time())
            transTuple = (transform.transform.translation.x, transform.transform.translation.y, transform.transform.rotation.z)
            chassicInstance.odom = Odom(*transTuple)
            if self._tfOffline:
                self.get_logger().info("TF is back online!")
                self._tfOffline = False
        except Exception as e:
            if not self._tfOffline:
                self.get_logger().warn(f"TF lookup failed: {e}")
                self._tfOffline = True
            return
