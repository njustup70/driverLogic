#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
import numpy as np
from collections import deque
import threading
import time

class UniversalSensorMonitor(Node):
    def __init__(self):
        super().__init__('universal_sensor_monitor')
        
        # 参数声明与获取
        self.declare_parameter('topic_name', '/test_image')
        self.declare_parameter('window_size', 100)
        self.declare_parameter('print_interval', 1.0)
        
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.window_size = self.get_parameter('window_size').get_parameter_value().integer_value
        print_interval = self.get_parameter('print_interval').get_parameter_value().double_value
        
        if not topic_name:
            self.get_logger().error('必须指定 topic_name 参数！')
            raise ValueError('topic_name 参数必须指定')
        
        # 核心统计变量
        self.topic_name = topic_name
        self.receive_times = deque(maxlen=self.window_size)
        self.delays = deque(maxlen=self.window_size)
        self.msg_count = 0
        self.last_msg_time = time.time()
        self.lock = threading.Lock()
        
        # 话题发现相关
        self.subscription = None
        self.topic_discovered = False
        self.discovery_timer = self.create_timer(1.0, self.discover_topic)
        
        # 统计打印定时器
        self.print_timer = self.create_timer(print_interval, self.print_statistics)
        
        self.get_logger().info(f"传感器监控节点已启动，等待话题: {topic_name}")
    
    def discover_topic(self):
        """发现话题并创建对应的订阅者"""
        if self.topic_discovered:
            return
            
        # 获取所有话题和类型
        for topic_name, topic_types in self.get_topic_names_and_types():
            if topic_name == self.topic_name and topic_types:
                self.create_universal_subscription(topic_types[0])
                self.topic_discovered = True
                self.destroy_timer(self.discovery_timer)
                break
    
    def create_universal_subscription(self, msg_type_str):
        """动态创建订阅者"""
        self.get_logger().info(f"发现话题: {self.topic_name}, 类型: {msg_type_str}")
        
        try:
            # 动态导入消息类型
            import importlib
            parts = msg_type_str.split('/')
            if len(parts) != 3:
                self.get_logger().warning(f"消息类型格式不标准: {msg_type_str}")
                return
            
            package, folder, msg_class = parts
            module = importlib.import_module(f"{package}.{folder}")
            msg_type = getattr(module, msg_class)
            
            # 创建订阅者
            from rclpy.qos import qos_profile_sensor_data
            self.subscription = self.create_subscription(
                msg_type,
                self.topic_name,
                self.callback,
                qos_profile_sensor_data,
                callback_group=ReentrantCallbackGroup()
            )
            self.get_logger().info(f"成功订阅话题: {self.topic_name}")
            
        except Exception as e:
            self.get_logger().error(f"创建订阅者失败: {e}")
    
    def callback(self, msg):
        """通用回调函数"""
        try:
            receive_time = time.time()
            
            # 提取消息时间戳
            timestamp = None
            if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            elif hasattr(msg, 'stamp') and hasattr(msg.stamp, 'sec'):
                timestamp = msg.stamp.sec + msg.stamp.nanosec * 1e-9
            else:
                timestamp = receive_time - 0.001  # 备用时间戳
            
            with self.lock:
                # 计算并记录延迟
                if timestamp:
                    self.delays.append(receive_time - timestamp)
                
                # 记录接收时间和消息计数
                self.receive_times.append(receive_time)
                self.msg_count += 1
                self.last_msg_time = receive_time
                
        except Exception as e:
            self.get_logger().warning(f"处理消息时出错: {e}")
    
    def print_statistics(self):
        """打印精简的统计信息"""
        with self.lock:
            if self.msg_count == 0:
                return
            
            # 计算当前频率
            frequency = 0.0
            if len(self.receive_times) >= 2:
                time_diffs = np.diff(list(self.receive_times))
                frequency = 1.0 / np.mean(time_diffs) if time_diffs.size > 0 else 0.0
            
            # 计算延迟统计
            avg_delay = np.mean(self.delays) * 1000 if self.delays else 0.0
            max_delay = np.max(self.delays) * 1000 if self.delays else 0.0
            min_delay = np.min(self.delays) * 1000 if self.delays else 0.0
            delay_std = np.std(self.delays) * 1000 if self.delays else 0.0
            
            # 最后消息间隔
            last_msg_interval = time.time() - self.last_msg_time
            
            # 打印格式化信息
            stats = f"""
==========================================
传感器话题: {self.topic_name}
总消息数: {self.msg_count}
------------------------------------------
频率统计:
  当前频率: {frequency:.2f} Hz
------------------------------------------
延迟统计 (ms):
  平均延迟: {avg_delay:.2f}
  最大延迟: {max_delay:.2f}
  最小延迟: {min_delay:.2f}
  延迟标准差: {delay_std:.2f}
------------------------------------------
最后消息接收: {last_msg_interval:.2f}s 前
==========================================
"""
            self.get_logger().info(stats)

def main(args=None):
    rclpy.init(args=args)
    node = UniversalSensorMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("接收到键盘中断，退出节点")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()