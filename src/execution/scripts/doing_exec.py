#!/usr/bin/env python3
import rclpy
import json
import asyncio
from typing import List
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
import tf2_ros
import math
from typing import Optional

class DoingExec(Node):
    def __init__(self,loop):
        super().__init__('doing_exec')
        self.loop = loop
        self.declare_parameter('error_value_xy', 0.1)
        self.declare_parameter('error_value_yaw', 0.1)
        self.declare_parameter('R2place1_exec_list', '[]')
        R2place1_exec_list_str = self.get_parameter('R2place1_exec_list').get_parameter_value().string_value
        self.R2place1_exec_list = json.loads(R2place1_exec_list_str)
        self.error_value_xy = self.get_parameter('error_value_xy').get_parameter_value().double_value
        self.error_value_yaw = self.get_parameter('error_value_yaw').get_parameter_value().double_value
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.control_pub = self.create_publisher(String, 'control', 10)
        self.location_pub = self.create_publisher(String, 'location', 10)
        self.exec_callback_sub = self.create_subscription(String, 'exec_callback', self.exec_callback, 10)
        #self.pose_queue = asyncio.Queue()
        self.move_event = asyncio.Event()
        self.turn_event = asyncio.Event()
        self.doing_event = asyncio.Event()
        self.doing_event.set() # 初始状态为可以执行
        self.new_pose_xy = None
        self.new_pose_yaw = None
        self.callback_queue = asyncio.Queue()
        self.last_stamp = None
        self.moving = False
        self.turning = False
        self.last_yaw = 0.0
        self.timer = self.create_timer(0.1, self.tf_update_wrapper)
    async def doing_exec(self):
        await asyncio.sleep(1.0)
        for item in self.R2place1_exec_list:
            await self.doing_event.wait()
            self.doing_event.clear()
            cmd = item[0]
            args = item[1:]
            if hasattr(self, cmd):
                func = getattr(self, cmd)
                await func(*args)
            else:
                msg = String()
                msg.data = json.dumps({
                    "topic_name": cmd,
                    "data": args
                })
                self.control_pub.publish(msg)
                while True:
                    callback = await self.callback_queue.get()
                    if callback == cmd:
                        self.doing_event.set()
                        break
                    elif callback == "Unknown frame":
                        self.get_logger().warn(f"未知返回: {callback}")
                    elif callback == "cmd_vel":
                        self.get_logger().info(f"奇怪的返回: {callback}")
    def exec_callback(self, msg):
        self.callback_queue.put_nowait(msg.data)
    async def move(self, x, y):
        self.get_logger().info(f"Start move to {x}, {y}")
        self.moving = True
        while True:
            await self.move_event.wait()
            self.move_event.clear()
            current_x, current_y = self.new_pose_xy
            dx = x - current_x
            dy = y - current_y
            dist = math.sqrt(dx * dx + dy * dy)
            self.get_logger().info(f"移动偏差值: {dist}")
            msg = String()
            msg.data = json.dumps({
                "topic_name": "cmd_vel",
                "data": [dx,dy,self.last_yaw],
            })
            self.control_pub.publish(msg)
            if dist < self.error_value_xy:
                self.get_logger().info("结束移动")
                break
        self.moving = False
        self.doing_event.set()
    async def turn(self, yaw):
        self.get_logger().info(f"Start turn to {yaw}")
        self.last_yaw = yaw
        self.turning = True
        while True:
            await self.turn_event.wait()
            self.turn_event.clear()
            current_yaw = self.new_pose_yaw
            dyaw = math.atan2(math.sin(yaw - current_yaw), math.cos(yaw - current_yaw))
            self.get_logger().info(f"旋转偏差值: {dyaw}")
            msg = String()
            msg.data = json.dumps({
                "topic_name": "cmd_vel",
                "data": [0, 0, dyaw],
            })
            self.control_pub.publish(msg)
            if abs(dyaw) < self.error_value_yaw:
                self.get_logger().info("结束转动")
                break
        self.turning = False
        self.doing_event.set()
    def tf_update_wrapper(self):
        self.get_logger().info("timer tick")
        #asyncio.create_task(self.tf_update())
        self.loop.call_soon_threadsafe(self.tf_update)
    
    def tf_update(self):#这一部分采用了轮询，可能会出现更新幅度频率不稳定的情况，以后再改
        self.get_logger().info("更新 TF")
        tf_msg = self.read_base_to_map_tf()
        if tf_msg is None:
            return 
        # if self.last_stamp == tf_msg.header.stamp:
        #     return
        self.last_stamp = tf_msg.header.stamp
        x = tf_msg.transform.translation.x
        y = tf_msg.transform.translation.y
        yaw = self.quaternion_to_yaw(tf_msg.transform.rotation)
        if x == None or y == None or yaw == None:
            self.get_logger().warn(f"无效的 TF 值: x={x}, y={y}, yaw={yaw}")
            return
        self.get_logger().info(f"当前位姿: x={x}, y={y}, yaw={yaw}")
        # 只在 move/turn运行时才推送
        if self.moving:
            self.new_pose_xy = (x, y)
            self.move_event.set()
        if self.turning:
            self.new_pose_yaw = yaw
            self.turn_event.set()
        self.location_pub.publish(String(data=json.dumps([x, y, yaw])))
    def quaternion_to_yaw(self, q):
        x = q.x
        y = q.y
        z = q.z
        w = q.w
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    def read_base_to_map_tf(self) -> Optional[TransformStamped]:
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                Time(seconds=0)
            )
            self.get_logger().info(f"获取 TF 成功: x={tf_msg.transform.translation.x}, y={tf_msg.transform.translation.y}, yaw={self.quaternion_to_yaw(tf_msg.transform.rotation)}")
            return tf_msg
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f"获取 TF 失败: {e}")
            return None
def main():
    rclpy.init()

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    node = DoingExec(loop)   # 把 loop 传进去

    try:
        spin_task = loop.run_in_executor(None, rclpy.spin, node)
        main_task = loop.create_task(node.doing_exec())
        loop.run_until_complete(main_task)

    finally:
        node.destroy_node()
        rclpy.shutdown()
        loop.stop()
        loop.close()


if __name__ == '__main__':
    main()
