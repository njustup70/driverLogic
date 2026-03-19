from rclpy.node import Node
from MainLogic.core.my_serial import AsyncSerial_t
from std_msgs.msg import UInt8MultiArray


class SerialNode(Node):
    """ROS2 serial bridge node forwarding bytes between ROS topics and serial."""

    def __init__(self, serial_port: str = '/dev/ttyACM0', baudrate: int = 115200):
        super().__init__('serial_node')
        self._serial = AsyncSerial_t(serial_port, baudrate)
        self._serial.register_callback(self._serial_rx_callback)
        self._serial_pub = self.create_publisher(UInt8MultiArray, 'serial_rx', 10)
        self._serial_sub = self.create_subscription(UInt8MultiArray, 'serial_tx', self._serial_tx_callback, 10)

    def _serial_rx_callback(self, data_bytes: bytes):
        msg = UInt8MultiArray()
        msg.data = list(data_bytes)
        self._serial_pub.publish(msg)

    def _serial_tx_callback(self, msg: UInt8MultiArray):
        try:
            self._serial.write(bytes(msg.data))
        except Exception as e:
            self.get_logger().error(f'Failed to send data to serial: {e}')

    def write(self, data: bytes):
        self._serial.write(data)


import multiprocessing
import rclpy


SerialProcess: multiprocessing.Process | None = None


def main(serial_port: str = '/dev/ttyACM0', baudrate: int = 115200):
    rclpy.init()
    node = SerialNode(serial_port=serial_port, baudrate=baudrate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到键盘中断信号，关闭节点...')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def start_serial_process(serial_port: str, baudrate: int) -> None:
    """Start serial ROS node in a separate process without blocking asyncio loop."""
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
