import os
import time
import shutil
import threading  # 【新增】引入线程锁
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.serialization import serialize_message
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from rosidl_runtime_py.utilities import get_message


class SmartBagRecorder(Node):
    def __init__(self):
        super().__init__('smart_bag_recorder')

        # 【新增】创建互斥锁，保护非线程安全的 SequentialWriter
        self.write_lock = threading.Lock()
        self.is_recording = True  # 录制状态标志位

        # 声明ROS参数
        self.declare_parameter('max_size_gb', 10.0)
        self.declare_parameter('max_folder_num', 10)
        self.declare_parameter('mcap', True)
        self.declare_parameter('topic_blacklist', [
            "*/image_raw*",       
            "*/compressed_image*",
            "/camera/*",          
            "depth_image*",       
        ])

        # 参数解析
        self.max_size_bytes: int = int((self.get_parameter('max_size_gb').value or 0) * 1024 ** 3)
        self.max_folder_num = self.get_parameter('max_folder_num').value or 4
        self.topic_blacklist: list[str] = self.get_parameter('topic_blacklist').value or []
        self.record_dir_root = os.path.abspath(os.path.join(
            os.path.expanduser('~'), "ros2_ws/rosbag_record"))
        self.bag_path = self.prepare_record_path()
        print(f'\033[95m📁 Recording to: {self.bag_path}\033[0m')

        # 初始化writer
        self.writer = SequentialWriter()
        if self.get_parameter('mcap').value:
            storage_options = StorageOptions(uri=self.bag_path, storage_id='mcap')
            converter_options = ConverterOptions('cdr', 'cdr')
            self.writer.open(storage_options, converter_options)
            print(f'\033[95m📦 Using MCAP format for recording\033[0m')
        else:
            storage_options = StorageOptions(uri=self.bag_path, storage_id='sqlite3')
            converter_options = ConverterOptions('', '')
            self.writer.open(storage_options, converter_options)
            print(f'\033[95m📦 Using SQLite3 format for recording\033[0m')

        # 初始化变量
        self.subscribers = []
        self.subscribed_topics = set()
        self.blacklisted_topics_checked = set()
        self.blacklist_rules = [self._compile_rule(rule) for rule in self.topic_blacklist]

        # 创建定时器
        self.timer = self.create_timer(3.0, self.timer_callback)
        print(f'\033[92m📋 Topic blacklist (gitignore style): {self.topic_blacklist}\033[0m')

    def stop_recording(self):
        """【新增】安全关闭写入器，确保元数据和索引安全写入磁盘"""
        with self.write_lock:
            if not self.is_recording:
                return
            self.is_recording = False
            print("\033[93m💾 Flushing data and closing rosbag writer...\033[0m")
            # 显式删除或关闭 writer，触发底层 C++ 析构函数写入 MCAP footer 或 SQLite 索引
            if hasattr(self, 'writer') and self.writer is not None:
                del self.writer
                self.writer = None
            print("\033[92m✅ Rosbag safely closed.\033[0m")

    def destroy_node(self):
        """重写 destroy_node，确保节点销毁时关闭录制"""
        self.stop_recording()
        super().destroy_node()

    def _compile_rule(self, rule):
        rule_parts = [p for p in rule.split('/') if p]
        def matcher(topic):
            topic_parts = [p for p in topic.split('/') if p]
            if rule.startswith('/'):
                if len(topic_parts) < len(rule_parts):
                    return False
                for r_part, t_part in zip(rule_parts, topic_parts[:len(rule_parts)]):
                    if not self._wildcard_match(r_part, t_part):
                        return False
                return True
            else:
                for i in range(len(topic_parts) - len(rule_parts) + 1):
                    match = True
                    for j in range(len(rule_parts)):
                        if not self._wildcard_match(rule_parts[j], topic_parts[i + j]):
                            match = False
                            break
                    if match:
                        return True
                return False
        return matcher

    def _wildcard_match(self, pattern, text):
        parts = pattern.split('*')
        if not parts:
            return text == ''
        if parts[0] and not text.startswith(parts[0]):
            return False
        if parts[-1] and not text.endswith(parts[-1]):
            return False
        current = parts[0]
        remaining = text[len(current):] if current else text
        for part in parts[1:-1]:
            if not part:
                continue
            idx = remaining.find(part)
            if idx == -1:
                return False
            remaining = remaining[idx + len(part):]
        return True

    def prepare_record_path(self):
        os.makedirs(self.record_dir_root, exist_ok=True)
        record_dirs = sorted(
            [d for d in os.listdir(self.record_dir_root) if os.path.isdir(os.path.join(self.record_dir_root, d))],
            key=lambda d: os.path.getctime(os.path.join(self.record_dir_root, d))
        )
        if len(record_dirs) >= self.max_folder_num:
            old_path = os.path.join(self.record_dir_root, record_dirs[0])
            print(f'\033[91m📦 Removing oldest folder: {old_path}\033[0m')
            shutil.rmtree(old_path, ignore_errors=True)
        
        file_name = time.strftime("%m-%d-%H-%M", time.localtime())
        file_path = os.path.join(self.record_dir_root, file_name)
        
        if os.path.exists(file_path):
            print(f'\033[93m⚠️ Existing path detected, removing: {file_path}\033[0m')
            shutil.rmtree(file_path, ignore_errors=True)

        return file_path

    def create_callback(self, topic_name):
        def callback(msg):
            # 【修改】获取锁后执行写入，且检查是否仍在录制
            with self.write_lock:
                if not self.is_recording or self.writer is None:
                    return
                try:
                    self.writer.write(topic_name, serialize_message(msg), self.get_clock().now().nanoseconds)
                except Exception as e:
                    print(f'\033[91m⚠️ Error writing message from {topic_name}: {e}\033[0m')
        return callback

    def subscribe_topic(self, topic_name, msg_type_str, qos_profile: QoSProfile = None):
        if topic_name in self.subscribed_topics or topic_name in self.blacklisted_topics_checked:
            return

        for matcher in self.blacklist_rules:
            if matcher(topic_name):
                print(f'\033[93m🔇 Skipping blacklisted topic: {topic_name}\033[0m')
                self.blacklisted_topics_checked.add(topic_name)
                return

        try:
            msg_type = get_message(msg_type_str)
            if qos_profile is None:
                qos_profile = QoSProfile(
                    depth=10,
                    reliability=QoSReliabilityPolicy.BEST_EFFORT
                )
            
            topic_info = TopicMetadata(name=topic_name, type=msg_type_str, serialization_format='cdr')

            # 【修改】使用锁，并且必须【先】调用 create_topic，【再】创建订阅
            with self.write_lock:
                if not self.is_recording or self.writer is None:
                    return
                self.writer.create_topic(topic_info)

            sub = self.create_subscription(
                msg_type,
                topic_name,
                self.create_callback(topic_name),
                qos_profile
            )
            
            self.subscribers.append(sub)
            self.subscribed_topics.add(topic_name)
            print(f'\033[95m✅ Recording topic: {topic_name} [{msg_type_str}]\033[0m')
        except Exception as e:
            print(f'\033[91m⛔ Failed to subscribe to {topic_name}: {e}\033[0m')

    def check_new_topics(self):
        if not self.is_recording:
            return
        topic_names_and_types = self.get_topic_names_and_types()
        for topic_name, types in topic_names_and_types:
            if not types:
                continue
            msg_type_str = types[0]
            
            if topic_name == '/tf_static':
                qos_profile = QoSProfile(
                    depth=10,
                    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                    reliability=QoSReliabilityPolicy.RELIABLE
                )
                self.subscribe_topic(topic_name, msg_type_str, qos_profile)
            else:
                self.subscribe_topic(topic_name, msg_type_str)

    def check_size_limit(self):
        if not os.path.exists(self.bag_path) or not self.is_recording:
            return

        total_size = 0
        for root, dirs, files in os.walk(self.bag_path):
            for f in files:
                total_size += os.path.getsize(os.path.join(root, f))
        
        if total_size > self.max_size_bytes:
            print(f'\033[91m🚫 Reached size limit ({self.max_size_bytes / 1024**3:.2f} GB). Stopping recording...\033[0m')
            # 【修改】不要直接在定时器线程里野蛮 shutdown，先安全停止录制
            self.stop_recording()
            rclpy.shutdown()

    def timer_callback(self):
        self.check_size_limit()
        self.check_new_topics()


def main(args=None):
    rclpy.init(args=args)
    exe = rclpy.executors.SingleThreadedExecutor()
    recorder = SmartBagRecorder()
    exe.add_node(recorder)
    
    try:
        exe.spin()
    except KeyboardInterrupt:
        print("\033[92m🛑 Recording stopped by user.\033[0m")
    finally:
        # 【修改】确保退出时明确关闭写入器并销毁节点
        recorder.stop_recording()
        exe.remove_node(recorder)
        recorder.destroy_node()
        exe.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()