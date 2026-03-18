'''
坐标管理类
'''
import asyncio
import math
from typing import cast

import rclpy.time

from Lib.odomVec import Odom
from Lib.bytes import turn_to_bytes
import Lib.rosBridgeNode as ros_bridge_module
from Lib.AsyncTools import async_property


class TFManager:
    #map->base_link 位姿，由 slam 融合计算得到，供上层异步逻辑使用
    baseLinkOdom = async_property(Odom)
    #odom->base_link 位姿，由码盘输入更新
    wheelOdom = async_property(Odom)
    #slam_init->base_link 位姿，由 slam 直接测量得到，供 sick 修正使用
    slamBaseOdom = async_property(Odom)

    def __init__(self):
        # 坐标系固定配置（不使用 ROS2 参数）
        self.map_frame = 'map'
        self.slam_init_frame = 'slam_init'
        self.odom_frame = 'odom'
        self.base_frame = 'base_link'
        self.slam_odom_frame = 'camera_init'
        self.slam_base_frame = 'aft_mapped'
        self.wheelOdom = Odom(0.0, 0.0, 0.0)  # 里程计原始数据，单位米和弧度
        # [x, y, yaw_deg]
        self.laser_to_base = Odom(0.0, 0.390, 0.0)
        # map -> slam_init（默认对齐）
        self.mapToSlamInit = Odom(0.0, 0.0, 0.0)
        # sick -> base_link
        self.sickToBaseLink = Odom(0.0, 0.390, 0.0)
        # slam_init -> odom
        self.slamInitToOdom = Odom(0.0, 0.0, 0.0)
        # 控制标志
        self._tf_chain_registered = False
        self._has_slam_pose = False
        # sick 修正缓存
        self.sick_lateral_offset = 0.0
        self.sick_buffer_size = 10
        self.sick_buffer: list[float] = []

    def register_tf_chain(self):
        """初始化 TF 链并发布 map->slam_init 初始静态变换。"""
        self._publish_map_to_slam_init_static()
        self._tf_chain_registered = True

    def odom(self, x: float, y: float, yaw: float):
        """码盘数据入口：更新 odom->base_link。"""
        self.wheelOdom = Odom(x, y, yaw)

    def sick(self, sick_y: float):
        """SICK 数据入口：输入侧向测距值（单位米）。"""
        self.sick_buffer.append(float(sick_y) + self.sick_lateral_offset)
        if len(self.sick_buffer) > self.sick_buffer_size:
            self.sick_buffer.pop(0)

    def reset_slam_correction(self):
        """重置 sick yaw 修正。"""
        self.mapToSlamInit = Odom(0.0, 0.0, 0.0)
        self._publish_map_to_slam_init_static()

    def apply_sick_initial_yaw_correction(self) -> bool:
        """使用 sick 缓存值修正 map->slam_init 的初始 yaw。"""
        if not self.sick_buffer or not self._has_slam_pose:
            return False

        slam_pose = cast(Odom, self.slamBaseOdom)
        sick_y = sum(self.sick_buffer) / len(self.sick_buffer)
        yaw_correction = math.atan2(slam_pose.y - sick_y, slam_pose.x)
        self.mapToSlamInit = Odom(0.0, 0.0, yaw_correction)
        self._publish_map_to_slam_init_static()
        self.sick_buffer.clear()
        return True

    def odom_10ms(self):
        """10ms 更新：发布 odom/base, map/odom, 计算 map/base 并下发到下位机。"""
        if not self._tf_chain_registered:
            return

        wheel_pose = cast(Odom, self.wheelOdom)
        self._publish_dynamic_tf(self.odom_frame, self.base_frame, wheel_pose)

        map_to_odom = self.mapToSlamInit @ self.slamInitToOdom
        self._publish_dynamic_tf(self.map_frame, self.odom_frame, map_to_odom)

        fused_base = map_to_odom @ wheel_pose
        self.baseLinkOdom = fused_base
        self._send_base_pose_to_lower(fused_base)

    def slam_100ms(self):
        """100ms 更新：读取 SLAM TF 并更新 slam_init->odom。"""
        node = ros_bridge_module.RosBridgeNodeInstance
        if not self._tf_chain_registered or node is None:
            return

        try:
            tf_msg = node._tfBuffer.lookup_transform(
                self.slam_odom_frame,
                self.slam_base_frame,
                rclpy.time.Time(),
            )
        except Exception:
            return

        slam_sensor_pose = Odom.from_transform_stamped(tf_msg)
        slam_base_pose = slam_sensor_pose @ self.laser_to_base
        self.slamBaseOdom = slam_base_pose
        self._has_slam_pose = True

        wheel_pose = cast(Odom, self.wheelOdom)
        self.slamInitToOdom = slam_base_pose @ wheel_pose.inverse()

    async def tf_update_loop(self):
        """统一更新任务：10ms 执行 odom 更新，每 100ms 执行一次 slam 更新。"""
        tick_10ms = 0
        while True:
            try:
                self.odom_10ms()
                if tick_10ms % 10 == 0:
                    self.slam_100ms()
            except Exception:
                pass
            tick_10ms = (tick_10ms + 1) % 10
            await asyncio.sleep(0.01)
    def _publish_dynamic_tf(self, parent_frame: str, child_frame: str, pose: Odom):
        node = ros_bridge_module.RosBridgeNodeInstance
        if node is None:
            return
        node.publish_dynamic_tf(parent_frame, child_frame, pose)
    def _publish_map_to_slam_init_static(self):
        node = ros_bridge_module.RosBridgeNodeInstance
        if node is None:
            return
        node.publish_static_tf(self.map_frame, self.slam_init_frame, self.mapToSlamInit)
    def _send_base_pose_to_lower(self, pose: Odom):
        """下发融合后的 map->base_link 到下位机，并发布 location。"""
        node = ros_bridge_module.RosBridgeNodeInstance
        node.writeBytes(b'\xA0' + turn_to_bytes([pose.x, pose.y, pose.yaw]))
async def move_to(x, y, yaw):
    targetOdom = Odom(x, y, yaw)
    # 给电控发坐标指令
    assert ros_bridge_module.RosBridgeNodeInstance is not None, "RosBridgeNodeInstance is not initialized yet!"
    ros_bridge_module.RosBridgeNodeInstance.writeBytes(b'\xA1' + turn_to_bytes([x, y, yaw]))
    while True:
        ros_bridge_module.RosBridgeNodeInstance.writeBytes(b'\xA1' + turn_to_bytes([x, y, yaw]))
        # 等待baseLinkOdom更新
        current_odom = cast(Odom, TFManagerInstance.baseLinkOdom)
        dx = targetOdom - current_odom
        # 距离小于1cm且角度误差小于0.05rad就认为到达目标了
        if dx.dist < 0.01 and abs(dx.yaw) < 0.05:
            print("Arrived at target!")
            break
        await asyncio.sleep(0.01)


TFManagerInstance = TFManager()
