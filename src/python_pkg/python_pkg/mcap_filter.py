import os
import subprocess
import rclpy
from rclpy.node import Node
from collections.abc import Iterable,Sequence
class FilterMcapNode(Node):
    def __init__(self):
        super().__init__('filter_mcap_node')

        self.declare_parameter('rosbag_root_path', '/home/Elaina/ros2_ws/bag_play/mcap_filter')
        self.declare_parameter('whitelist', ['/livox/lidar/pc'])
        self.declare_parameter('start_time', '')
        self.declare_parameter('end_time', '')

        self.rosbag_root_path = self.get_parameter('rosbag_root_path').value or ''
        self.whitelist = self.get_parameter('whitelist').get_parameter_value().string_array_value
        self.start_time = self.get_parameter('start_time').value
        self.end_time = self.get_parameter('end_time').value

        self.find_all_and_filter()

    def find_all_and_filter(self):
        any_found = False

        for root, dirs, files in os.walk(self.rosbag_root_path):
            dirs.sort()
            dirs[:] = [d for d in dirs if not d.endswith('_filter')]

            mcap_files = [f for f in files if f.endswith('.mcap')]

            for mcap_file in mcap_files:
                self.mcap_path = os.path.join(root, mcap_file)
                bag_dir = root

                bag_name = os.path.basename(self.mcap_path)
                bag_base, _ = os.path.splitext(bag_name)

                # 输出文件夹
                parent_dir = os.path.dirname(bag_dir)
                folder_name = os.path.basename(bag_dir)
                output_dir = os.path.join(parent_dir, f"{folder_name}_filter")
                os.makedirs(output_dir, exist_ok=True)

                output_path = os.path.join(output_dir, f"{bag_base}_filtered.mcap")

                if os.path.exists(output_path):
                    self.get_logger().info(f'已处理过，跳过: {output_path}')
                    continue
                print(f'\033[95m 找到mcap文件: {self.mcap_path} \033[0m')
                # self.get_logger().info(f'找到 mcap 文件: {self.mcap_path}')
                self.do_filter(bag_dir, output_path)
                any_found = True

        if not any_found:
            raise FileNotFoundError('没有找到任何 .mcap 文件')

    def do_filter(self, bag_dir, output_path):
        cmd = [
            'mcap', 'filter',
            self.mcap_path,
            '-o', output_path,
            '--include-metadata',
            '--include-attachments'
        ]
        assert isinstance(self.whitelist, Iterable) and isinstance(self.whitelist, Sequence)
        if len(self.whitelist) == 0 or (len(self.whitelist) == 1 and self.whitelist[0] == ''):
            self.get_logger().warn('白名单为空，将不过滤话题！')
            return
        else:
            cleaned_whitelist = [t for t in self.whitelist if t.strip()]
            if not cleaned_whitelist:
                self.get_logger().warn('白名单为空，将不过滤话题！')
                return

            for topic in cleaned_whitelist:
                cmd.extend(['-y', topic])
            # print(f'\033[95m 找到mcap文件: {self.mcap_path} \033[0m')
            print(f'\033[95m 保留话题{cleaned_whitelist} \033[0m')
            # self.get_logger().info(f'保留话题: {cleaned_whitelist}')

        if self.start_time:
            cmd.extend(['--start', self.start_time])
        if self.end_time:
            cmd.extend(['--end', self.end_time])

        self.get_logger().info(f'输出到: {output_path}')
        self.get_logger().info(f'执行命令: {" ".join(cmd)}')

        result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

        if result.returncode == 0:
            self.get_logger().info(f'过滤完成: {output_path}')
        else:
            self.get_logger().error(f'过滤失败:\n{result.stderr}')


def main(args=None):
    rclpy.init(args=args)
    node = FilterMcapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('过滤操作被用户中断')
    finally:
        node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
