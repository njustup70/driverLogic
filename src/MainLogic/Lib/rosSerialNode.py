from typing import Optional
from rclpy.node import Node
from Lib.mySerial import AsyncSerial_t
from std_msgs.msg import UInt8MultiArray

class SerialNode(Node):
    '''
    ros2耦合节点,负责上下位机通信,从串口获得数据,转发给rosBridgeNode
    '''
    def __init__(self):
        super().__init__('serial_node')
        self._serial=AsyncSerial_t('/dev/ttyUSB0', 115200)
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

# 声明类，不初始化，在Main.py中初始化
# 因为需要rclpy.init之后才能创建Node实例
RosSerialNodeInstance: SerialNode = SerialNode()
