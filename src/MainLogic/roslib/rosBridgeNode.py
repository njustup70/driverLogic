from rclpy.node import Node
import asyncio,threading
import rclpy,rclpy.time
from std_msgs.msg import String
import async_main

#导入ros2坐标管理依赖
from tf2_ros import TransformListener, Buffer
#导入驱动
from data import chassic_instance
from mathlib.odomvec import odom
class MainNode(Node):
    '''
    ros2耦合节点,从ros2话题获得数据,传给类或者队列函数
    '''
    def __init__(self, loop):
        super().__init__('main_node')
        print("MainNode initialized")
        #话题订阅
        self.sub = self.create_subscription(String, 'trigger', self.listener_callback, 10)
        assert isinstance(loop, asyncio.events.AbstractEventLoop), "传入的 loop 必须是 asyncio 的事件循环"
        self.loop :asyncio.events.AbstractEventLoop= loop
        #创建tf2坐标管理器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.01, self.tf_timer_callback)  # 定时器回调，频率为100Hz
        self.tf_offline=False
    def listener_callback(self, msg):
        self.get_logger().info(f"Received ROS message: {msg.data}")
    def tf_timer_callback(self):
        try:
            # 尝试获取从 "base_link" 到 "odom" 的坐标变换
            transform = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            tuple=(transform.transform.translation.x, transform.transform.translation.y, transform.transform.rotation.z)
            chassic_instance.odom=odom(*tuple)
            if(self.tf_offline):
                self.get_logger().info("TF is back online!")
                self.tf_offline=False
        except Exception as e:
            if(self.tf_offline==False):
                self.get_logger().warn(f"TF lookup failed: {e}")
                self.tf_offline=True
            return