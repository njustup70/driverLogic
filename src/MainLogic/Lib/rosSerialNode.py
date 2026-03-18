from typing import Optional
from rclpy.node import Node
from Lib.mySerial import AsyncSerial_t
from std_msgs.msg import UInt8MultiArray

class SerialNode(Node):
    '''
    ros2耦合节点,负责上下位机通信,从串口获得数据,转发给rosBridgeNode
    '''
    def __init__(self, serial_port: str = '/dev/ttyACM0', baudrate: int = 115200):
        super().__init__('serial_node')
        self._serial=AsyncSerial_t(serial_port, baudrate)
        self._serial.register_callback(self._serial_rx_callback)
        self._serial_pub = self.create_publisher(UInt8MultiArray, 'serial_rx', 10)
        self._serial_sub=self.create_subscription(UInt8MultiArray, 'serial_tx', self._serial_tx_callback, 10)

    def _serial_rx_callback(self, data_bytes: bytes):
        # 将串口受到的bytes直接转发到ros2话题，使用UInt8MultiArray
        msg = UInt8MultiArray()
        msg.data = list(data_bytes)  # ROS2 UInt8MultiArray.data expects a list of integers or bytes-like object
        self._serial_pub.publish(msg)

    def _serial_tx_callback(self, msg: UInt8MultiArray):
        # 将ros2话题收到的UInt8MultiArray数据直接发送到串口
        try:
            # 在ROS2 Python中，UInt8MultiArray.data 通常可以直接用于串口写入，或是转为bytes
            self._serial.write(bytes(msg.data))
        except Exception as e:
            self.get_logger().error(f"Failed to send data to serial: {e}")

    def write(self, data: bytes):
        self._serial.write(data)

# 声明类的实例引用（暂不初始化）
# ROS2 节点需要在 rclpy.init() 之后才能创建
import rclpy


def main(serial_port: str='/dev/ttyACM0', baudrate: int = 115200):
    # 当作为独立进程运行时，需要在子进程中初始化 ROS2
    rclpy.init()
    #如果有serial_port和baudrate参数，可以从命令行传入
    import argparse
    node = SerialNode(serial_port=serial_port, baudrate=baudrate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到键盘中断信号，关闭节点...')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


import multiprocessing

SerialProcess: multiprocessing.Process | None = None


def start_serial_process(serial_port: str, baudrate: int) -> None:
    """Start rosSerialNode in a separate process without blocking the asyncio loop."""
    global SerialProcess
    if SerialProcess is not None and SerialProcess.is_alive():
        return

    SerialProcess = multiprocessing.Process(
        target=main,
        args=(serial_port, baudrate),
        name='ros_serial_process',
        daemon=True,
    )
    SerialProcess.start()