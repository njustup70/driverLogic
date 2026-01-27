#!/usr/bin/env python3
from get_dispose_serial.myserial import AsyncSerial_t
import struct
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

FRAME_LEN = 36  # 帧长度

class SickGetNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.declare_parameter('port', '/dev/ttyACM0')
        port_name = self.get_parameter('port').value

        self.get_logger().info(f'Node {node_name} 启动.')
        self.publisher_ = self.create_publisher(Float32, 'sick_data', 10)

        serial = AsyncSerial_t(port_name, 230400)
        serial.startListening(callback = self.parse_frame)

    def parse_frame(self, frame: bytes):
        if len(frame) != FRAME_LEN:
            print("长度错误:", len(frame))
            return None

        # ------------- 提取字段 ---------------
        header = frame[0]
        frame_type = frame[1]
        data_len = frame[2]
        payload = frame[3:35]      # 8 * float = 32B
        tail = frame[35]

        # ----------- 校验头部和长度 ----------
        if header != tail:
            print(f"头尾不匹配：header={header:02x}, tail={tail:02x}")
            return None

        # ----------- 校验（uint16 求和）---------
        calc_sum = sum(frame[1:35]) & 0xFF
        recv_sum = tail

        if calc_sum != recv_sum:
            print(f"校验失败：calc={calc_sum:04x}, recv={recv_sum:04x}")
            return None

        # ----------- 解 8 个 float --------------
        floats = struct.unpack("<8f", payload)

        # 8 个 float 分别表示：（可能，未找到官方数据）
        # 通道    名称	                    说明
        # 0	    Distance	            实际测距值（单位：米）精度0.1mm，实测大致为2mm
        # 1	    RSSI（光强）	          光返回信号强度，内部归一化
        # 2	    Amplitude（幅值）	      光返波幅度
        # 3	    Pulse width / Energy	脉冲宽度或能量
        # 4	    Noise level	            噪声基线
        # 5	    AGC（增益控制值）	        自动增益状态
        # 6	    Temperature compensation value	温漂补偿值
        # 7	    Confidence / Quality	测距置信度或内部质量参数

        distance = 1.0667 * floats[0] - 0.0533
        # self.get_logger().info(f"distance: {distance:.4f}")
        msg = Float32()

        msg.data = distance
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = SickGetNode('sick_get_node')
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()