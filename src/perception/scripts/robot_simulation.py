#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped, TransformStamped
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster
import math

class SimAllInOne(Node):
    def __init__(self):
        super().__init__('sim_all_in_one')
        
        # ==================== 1. 参数接口定义 (开关) ====================
        # 你可以在运行节点时通过 --ros-args -p enable_xxx:=false 来关闭特定功能
        self.declare_parameter('enable_odom', True)
        self.declare_parameter('enable_sick', False)
        self.declare_parameter('enable_slam_tf', False)
        
        # 读取参数状态
        self.do_odom = self.get_parameter('enable_odom').value
        self.do_sick = self.get_parameter('enable_sick').value
        self.do_slam = self.get_parameter('enable_slam_tf').value

        # ==================== 2. 初始化发布者与广播器 ====================
        # Odom (Vector3Stamped)
        if self.do_odom:
            self.odom_pub = self.create_publisher(Vector3Stamped, '/odom', 10)
            self.get_logger().info("[Enabled] Odom Simulation (Topic: /odom)")
        
        # Sick (Float32)
        if self.do_sick:
            self.sick_pub = self.create_publisher(Float32, '/sick_data', 10)
            self.get_logger().info("[Enabled] Sick Simulation (Topic: /sick_data, Value: 2.0m)")

        # SLAM TF (TransformBroadcaster)
        if self.do_slam:
            self.tf_broadcaster = TransformBroadcaster(self)
            self.get_logger().info("[Enabled] SLAM TF Simulation (camera_init -> aft_mapped)")

        # ==================== 3. 运动状态初始化 ====================
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        # 运动参数：模拟直线缓慢行驶
        self.vx = 0.002    # 线速度 m/s
        self.vy = 0.001    # 漂移速度 m/s
        self.v_yaw = 0.0   # 角速度

        # 定时器 50Hz
        self.last_time = self.get_clock().now()
        self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # 1. 物理积分 (更新位置)
        self.x += (self.vx * math.cos(self.yaw) - self.vy * math.sin(self.yaw)) * dt
        self.y += (self.vx * math.sin(self.yaw) + self.vy * math.cos(self.yaw)) * dt
        self.yaw += self.v_yaw * dt

        # ---------------------------------------------------------
        # 模块 A: 发布 Odom (Vector3Stamped)
        # ---------------------------------------------------------
        if self.do_odom:
            msg = Vector3Stamped()
            msg.header.stamp = current_time.to_msg()
            msg.header.frame_id = "odom_wheel"
            msg.vector.x = self.x
            msg.vector.y = self.y
            msg.vector.z = self.yaw
            self.odom_pub.publish(msg)

        # ---------------------------------------------------------
        # 模块 B: 发布 Sick (Float32)
        # ---------------------------------------------------------
        if self.do_sick:
            msg = Float32()
            msg.data = 0.9  # 模拟离墙 2 米
            self.sick_pub.publish(msg)

        # ---------------------------------------------------------
        # 模块 C: 发布 SLAM TF (camera_init -> aft_mapped)
        # ---------------------------------------------------------
        if self.do_slam:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = 'camera_init'
            t.child_frame_id = 'aft_mapped'

            # 模拟 SLAM 的轨迹 (为了演示融合效果，这里让 SLAM 和 Odom 保持一致)
            # 在实际测试中，FusionNode 会计算这两者的差异
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0

            # 欧拉角转四元数
            q = self.euler_to_quat(0, 0, self.yaw)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            self.tf_broadcaster.sendTransform(t)

    def euler_to_quat(self, roll, pitch, yaw):
        """辅助函数：欧拉角转四元数"""
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = SimAllInOne()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()