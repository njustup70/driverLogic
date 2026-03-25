'''
坐标管理类
'''

import asyncio
import math
from typing import cast

import rclpy.time

from MainLogic.Lib.odomVec import Odom
from MainLogic.Lib.bytes import turn_to_bytes
from MainLogic.Lib.AsyncTools import AsyncVariable
from MainLogic.core import ros_bridge_node as ros_bridge_module


class TFManager:
    # map->base_link 位姿，由 slam 融合计算得到，供上层异步逻辑使用
    

    def __init__(self):
        self.baseLinkOdom: AsyncVariable[Odom] = AsyncVariable(Odom(0.0, 0.0, 0.0))
        self.baseLinkOdom.value = Odom(0.0, 0.0, 0.0)
        # 坐标系固定配置（不使用 ROS2 参数）
        self.map_frame = 'map'
        self.slam_init_frame = 'slam_init'
        self.odom_frame = 'odom'
        self.base_frame = 'base_link'
        self.slam_odom_frame = 'camera_init'
        self.slam_base_frame = 'aft_mapped'
        # odom->base_link 位姿，由码盘输入更新
        self._odomToBase = Odom(0.0, 0.0, 0.0)
        # slam_init->base_link 位姿，由 slam 直接测量得到，供 sick 修正使用
        self.laser_to_base = Odom(0.0, -0.390, 0.0)
        # map -> slam_init（默认对齐）
        self.mapToBaseInit = Odom(0.250, 0.250, 0.0)
        self._mapToSlamInit = Odom(0.0, 0.0, 0.0)
        # sick -> base_link
        self.sickToBaseLink = Odom(0.0, 0.390, 0.0)
        # slam_init -> odom
        self._slamInitToOdom = Odom(0.0, 0.0, 0.0)
        self._mapToBase = Odom(0.0, 0.0, 0.0)
        # 控制标志
        self._tf_chain_registered = False
        self._has_slam_pose = False
        # sick 修正缓存
        self.sick_lateral_offset = 0.0
        self.sick_buffer_size = 10
        self.sick_buffer: list[float] = []
        # 含sick修正的 map->slam_init 位姿中间变量，其中包含了地图原点到车体中心偏移
        self._mapToSlamInitNominal = Odom(0.0, 0.0, 0.0)
        # 存储了sick修正增量的变量，用于连续修正时的撤销与更新逻辑
        self._sickYawCorrection = 0.0

    def register_tf_chain(self,sick2Base: Odom,map2BaseInit: Odom,laser2Base: Odom):
        self.rosBridge = ros_bridge_module.RosBridgeNodeInstance
        assert sick2Base is not None and map2BaseInit is not None and laser2Base is not None, 'TFManager register_tf_chain requires all TFs to be provided!'
        self.laser_to_base = laser2Base
        self.mapToBaseInit = map2BaseInit
        self.sickToBaseLink = sick2Base
        assert self.rosBridge is not None, 'RosBridgeNodeInstance is not initialized yet!'
        # 从 map->base_link_init 推导出 map->slam_init，并发布静态坐标
        # 公式：map->slam_init = map->base_link @ base_link->slam_init
        self._mapToSlamInitNominal = self.mapToBaseInit @ self.laser_to_base.inverse()
        self._mapToSlamInit = self._mapToSlamInitNominal
        self._sickYawCorrection = 0.0
        self.rosBridge.publish_static_tf(self.map_frame, self.slam_init_frame, self._mapToSlamInit)
        self._tf_chain_registered = True

    def odom(self, x: float, y: float, yaw: float):
        """码盘数据入口：更新 odom->base_link。"""
        self._odomToBase = Odom(x, y, yaw)

    def sick(self, sick_y: float):
        """SICK 数据入口：输入侧向测距值（单位米）。"""
        self.sick_buffer.append(float(sick_y) + self.sick_lateral_offset)
        if len(self.sick_buffer) > self.sick_buffer_size:
            self.sick_buffer.pop(0)

    def apply_sick_initial_yaw_correction(self) -> bool:
        """使用 sick 缓存值修正 map->slam_init 的初始 yaw（增量更新，可撤销前次修正）。"""
        if not self.sick_buffer or not self._has_slam_pose:
            return False
        sick_y = sum(self.sick_buffer) / len(self.sick_buffer)

        # 先撤销上一轮修正，再基于未修正状态计算本轮修正量。
        base_without_prev = Odom(
            self._mapToBase.x,
            self._mapToBase.y,
            self._mapToBase.yaw - self._sickYawCorrection,
        )
        sick_pose = base_without_prev @ self.sickToBaseLink
        new_yaw_correction = math.atan2(sick_pose.y - sick_y, sick_pose.x)

        # 从当前 map->slam_init 中撤销旧修正，再应用新修正。
        nominal_yaw = self._mapToSlamInit.yaw - self._sickYawCorrection
        self._mapToSlamInit = Odom(
            self._mapToSlamInit.x,
            self._mapToSlamInit.y,
            nominal_yaw + new_yaw_correction,
        )
        self._sickYawCorrection = new_yaw_correction

        if self.rosBridge is not None:
            self.rosBridge.publish_static_tf(self.map_frame, self.slam_init_frame, self._mapToSlamInit)
        self.sick_buffer.clear()
        return True

    def odom_10ms(self):
        """10ms 更新：发布 odom/base, map/odom, 计算 map/base 并下发到下位机。"""
        if not self._tf_chain_registered or self.rosBridge is None:
            print(f"[DEBUG] odom_10ms skip: _tf_chain_registered={self._tf_chain_registered}, rosBridge={self.rosBridge is not None}")
            return
        # odom->base_link
        wheel_pose = cast(Odom, self._odomToBase)
        self.rosBridge.publish_dynamic_tf(self.odom_frame, self.base_frame, wheel_pose)
        fused_base = self._mapToSlamInit @ self._slamInitToOdom @ wheel_pose
        self._mapToBase = fused_base
        self.baseLinkOdom.value = fused_base
        print(f"is:{fused_base.x}")
        self.rosBridge.writeBytes(b'\xA0' + turn_to_bytes([fused_base.x, fused_base.y, fused_base.yaw]))

    def slam_100ms(self):
        """100ms 更新：读取 SLAM TF 并更新 slam_init->odom。"""
        if not self._tf_chain_registered or self.rosBridge is None:
            return
        try:
            tf_msg = self.rosBridge._tfBuffer.lookup_transform(
                self.slam_odom_frame,
                self.slam_base_frame,
                rclpy.time.Time(),
            )
        except Exception:
            return
        # slam_init->laser
        slam_sensor_pose = Odom.from_transform_stamped(tf_msg)
        # slam_init->base_link = slam_init->laser @ laser->base
        slam_base_pose = slam_sensor_pose @ self.laser_to_base
        self._slamBaseOdom = slam_base_pose
        self._has_slam_pose = True
        wheel_pose = cast(Odom, self._odomToBase)
        # slam_init->odom = slam_init->base_link @ base_link->odom
        self._slamInitToOdom = slam_base_pose @ wheel_pose.inverse()
        self.rosBridge.publish_static_tf(self.slam_init_frame, self.odom_frame, self._slamInitToOdom)
        self.rosBridge.publish_static_tf(self.map_frame, self.slam_init_frame, self._mapToSlamInit)
    async def tf_update_loop(self):
        """统一更新任务：10ms 执行 odom 更新，每 100ms 执行一次 slam 更新。"""
        tick_10ms = 0
        while True:
            assert self._tf_chain_registered, 'TF chain is not registered yet!'
            try:
                self.odom_10ms()
                if tick_10ms % 10 == 0:
                    self.slam_100ms()
            except Exception as e:
                print(e)
            tick_10ms = (tick_10ms + 1) % 10
            await asyncio.sleep(0.01)


async def move_to(x, y, yaw):
    targetOdom = Odom(x, y, yaw)
    # 给电控发坐标指令
    assert TFManagerInstance.rosBridge is not None, 'rosBridge is not initialized yet!'
    TFManagerInstance.rosBridge.writeBytes(b'\xA1' + turn_to_bytes([x, y, yaw]))
    while True:
        TFManagerInstance.rosBridge.writeBytes(b'\xA1' + turn_to_bytes([x, y, yaw]))
        # print("发送移动指令")
        # 等待 baseLinkOdom 更新
        current_odom = await TFManagerInstance.baseLinkOdom
        # print("位置更新完成")
        dx = targetOdom - current_odom
        # 距离小于1cm且角度误差小于0.05rad就认为到达目标
        if dx.dist < 0.01 and abs(dx.yaw) < 0.05:
            print('Arrived at target!')
            break
        await asyncio.sleep(0.01)


TFManagerInstance = TFManager()
