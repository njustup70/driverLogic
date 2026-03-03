import rclpy
import struct
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
import time
import sys
import numpy as np
from rclpy.guard_condition import GuardCondition
from queue import Queue
import json
from get_dispose_serial.myserial import AsyncSerial_t 
class SerialDisposeNode(Node):
    def __init__(self):
        super().__init__('serial_dispose_node')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.serial = AsyncSerial_t(
            port=self.get_parameter('port').get_parameter_value().string_value,
            baudrate=self.get_parameter('baudrate').get_parameter_value().integer_value,
        )
        self.serial.startListening(lambda data: self.serial_callback(data))
        self.publisher_ = self.create_publisher(Vector3Stamped, 'lidar_position', 10)
        self.publisher_odom = self.create_publisher(Vector3Stamped, 'odom_data', 10)
        self.publisher_exec_callback = self.create_publisher(String, 'exec_callback', 10)
        self.subscription_control = self.create_subscription(String,'control',self.control_callback,10)
        self.subscription_location = self.create_subscription(String,'location',self.location_callback,10)
        self.rx_queue = Queue()
        self.guard = self.create_guard_condition(self.process_rx_queue)
        self.out_first_frame = b'\xFA'
        self.out_frame_list = {
            "cmd_vel": {
                "id": b'\xA0',
                "fmt": "<fff"
            },
            "location": {
                "id": b'\xA1',
                "fmt": "<fff"
            },
        }
        self.in_first_frame = b'\xFF'
        self.in_frame_list = {
            b'\xAA': {
                "name": "odom_frame",
                "dispose": self.dispose_odom,
                "send": self.publisher_odom,
                "fmt": "<fff"
            },
            b'\xAB': {
                "name": "exec_callback_frame",
                "dispose": self.dispose_exec_callback,
                "send": self.publisher_exec_callback,   
                "fmt": "<B"
            },
        }
    def control_callback(self, msg):
        msg_data = json.loads(msg.data)
        if msg_data["topic_name"] not in self.out_frame_list:
            self.get_logger().warn(f"未知topic: {msg_data['topic_name']}")
            return
        try:
            struct.pack(self.out_frame_list[msg_data["topic_name"]]["fmt"], *msg_data["data"])
            print("格式匹配")
        except struct.error as error:
            print("格式不匹配:", error)
            return
        data = struct.pack(self.out_frame_list[msg_data["topic_name"]]["fmt"], *msg_data["data"])
        send = self.out_first_frame + self.out_frame_list[msg_data["topic_name"]]["id"] + data
        self.serial.write(send)
    def location_callback(self, msg):
        msg_data = json.loads(msg.data)
        self.get_logger().info(f"接收到位置信息: {msg_data}")
        data = struct.pack("<fff", *msg_data)
        send = self.out_first_frame + self.out_frame_list["location"]["id"] + data
        self.serial.write(send)
    def serial_callback(self, data): 
        print("RX:", data.hex())
        self.rx_queue.put(data)
        self.guard.trigger()
    def process_rx_queue(self):
        while not self.rx_queue.empty():
            data = self.rx_queue.get()
            if len(data) < 2:
                continue
            if data[0:1] != self.in_first_frame:
                continue
            frame_id = data[1:2]
            frame_data = self.in_frame_list.get(frame_id)
            if frame_data is None:
                continue
            else :
                if len(data) < 2 + struct.calcsize(frame_data["fmt"]):
                    continue
                get_data = frame_data["dispose"](data[2:2 + struct.calcsize(frame_data["fmt"])],frame_data["fmt"])
                frame_data["send"].publish(get_data)

    def dispose_odom(self, data, fmt):
        x, y, z = struct.unpack(fmt, data)
        msg = Vector3Stamped()
        msg.vector.x = x
        msg.vector.y = y
        msg.vector.z = z
        return msg
    def dispose_exec_callback(self, data, fmt):
        for name, config in self.out_frame_list.items():
            if config["id"] == data:
                msg = String()
                msg.data = name
                return msg
        self.get_logger().warn(f"未知回调ID: {data}")
        return String(data="Unknown frame")
        
def main(args=None):
    rclpy.init(args=args)
    serial_dispose_node = SerialDisposeNode()
    rclpy.spin(serial_dispose_node)
    serial_dispose_node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()