from typing import Optional

from rclpy.node import Node
from Lib.mySerial import AsyncSerial_t
from std_msgs.msg import String
from Lib.bytes import DataFrame
import json
class SerialNode(Node):
    '''
    ros2耦合节点,负责上下位机通信,从串口获得数据,转发给rosBridgeNode
    '''
    def __init__(self):
        super().__init__('serial_node')
        self._serial=AsyncSerial_t('/dev/ttyUSB0', 115200)
        self._serial.register_callback(self._serial_rx_callback)
        self._serial_pub = self.create_publisher(String, 'serial_rx', 10)
        self._serial_sub=self.create_subscription(String, 'serial_tx', self._serial_tx_callback, 10)
    def  _serial_rx_callback(self, bytes:bytes):
        #将串口受到的bytes转发到ros2话题
        dataFrame=DataFrame(bytes)
        json_str=json.dumps(dataFrame.to_dict())
        msg=String()
        msg.data=json_str
        self._serial_pub.publish(msg)
    def _serial_tx_callback(self, msg:String):
        #将ros2话题收到的字符串转为bytes,发送到串口
        try:
            data_dict=json.loads(msg.data)
            dataFrame=DataFrame.from_dict(data_dict)
            self._serial.write(dataFrame.data)
        except Exception as e:
            self.get_logger().error(f"Failed to parse JSON or send data: {e}")
    def write(self, data:bytes):
        self._serial.write(data)
#声明类，不初始化，在Main.py中初始化
#因为需要rclpy.init之后才能创建Node实例
RosSerialNodeInstance:Optional[SerialNode]=None