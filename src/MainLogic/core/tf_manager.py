"""Core TF manager and motion helper."""

import asyncio
import math
from typing import cast

import rclpy.time

from MainLogic.Lib.odomVec import Odom
from MainLogic.Lib.bytes import turn_to_bytes
from MainLogic.Lib.AsyncTools import async_property
from MainLogic.core import ros_bridge_node as ros_bridge_module


class TFManager:
    baseLinkOdom = async_property(Odom)

    def __init__(self):
        self.map_frame = 'map'
        self.slam_init_frame = 'slam_init'
        self.odom_frame = 'odom'
        self.base_frame = 'base_link'
        self.slam_odom_frame = 'camera_init'
        self.slam_base_frame = 'aft_mapped'
        self._odomToBase = Odom(0.0, 0.0, 0.0)
        self.laser_to_base = Odom(0.0, -0.390, 0.0)
        self.mapToBaseInit = Odom(0.250, 0.250, 0.0)
        self._mapToSlamInit = Odom(0.0, 0.0, 0.0)
        self.sickToBaseLink = Odom(0.0, 0.390, 0.0)
        self._slamInitToOdom = Odom(0.0, 0.0, 0.0)
        self._mapToBase = Odom(0.0, 0.0, 0.0)
        self._tf_chain_registered = False
        self._has_slam_pose = False
        self.sick_lateral_offset = 0.0
        self.sick_buffer_size = 10
        self.sick_buffer: list[float] = []

    def register_tf_chain(self):
        self.rosBridge = ros_bridge_module.RosBridgeNodeInstance
        assert self.rosBridge is not None, 'RosBridgeNodeInstance is not initialized yet!'
        self._mapToSlamInit = self.mapToBaseInit @ self.laser_to_base.inverse()
        self.rosBridge.publish_static_tf(self.map_frame, self.slam_init_frame, self._mapToSlamInit)
        self._tf_chain_registered = True

    def odom(self, x: float, y: float, yaw: float):
        self._odomToBase = Odom(x, y, yaw)

    def sick(self, sick_y: float):
        self.sick_buffer.append(float(sick_y) + self.sick_lateral_offset)
        if len(self.sick_buffer) > self.sick_buffer_size:
            self.sick_buffer.pop(0)

    def apply_sick_initial_yaw_correction(self) -> bool:
        if not self.sick_buffer or not self._has_slam_pose:
            return False
        sick_pose = self._mapToBase @ self.sickToBaseLink
        sick_y = sum(self.sick_buffer) / len(self.sick_buffer)
        yaw_correction = math.atan2(sick_pose.y - sick_y, sick_pose.x)
        self._mapToSlamInit = Odom(0.0, 0.0, yaw_correction)
        if self.rosBridge is not None:
            self.rosBridge.publish_static_tf(self.map_frame, self.slam_init_frame, self._mapToSlamInit)
        self.sick_buffer.clear()
        return True

    def odom_10ms(self):
        if not self._tf_chain_registered or self.rosBridge is None:
            return
        wheel_pose = cast(Odom, self._odomToBase)
        self.rosBridge.publish_dynamic_tf(self.odom_frame, self.base_frame, wheel_pose)
        fused_base = self._mapToSlamInit @ self._slamInitToOdom @ wheel_pose
        self.mapToBase = fused_base
        self.baseLinkOdom = fused_base
        self.rosBridge.writeBytes(b'\xA0' + turn_to_bytes([fused_base.x, fused_base.y, fused_base.yaw]))

    def slam_100ms(self):
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
        slam_sensor_pose = Odom.from_transform_stamped(tf_msg)
        slam_base_pose = slam_sensor_pose @ self.laser_to_base
        self._slamBaseOdom = slam_base_pose
        self._has_slam_pose = True
        wheel_pose = cast(Odom, self._odomToBase)
        self._slamInitToOdom = slam_base_pose @ wheel_pose.inverse()
        self.rosBridge.publish_static_tf(self.slam_init_frame, self.odom_frame, self._slamInitToOdom)

    async def tf_update_loop(self):
        tick_10ms = 0
        while True:
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
    assert TFManagerInstance.rosBridge is not None, 'rosBridge is not initialized yet!'
    TFManagerInstance.rosBridge.writeBytes(b'\xA1' + turn_to_bytes([x, y, yaw]))
    while True:
        TFManagerInstance.rosBridge.writeBytes(b'\xA1' + turn_to_bytes([x, y, yaw]))
        current_odom = cast(Odom, TFManagerInstance.baseLinkOdom)
        dx = targetOdom - current_odom
        if dx.dist < 0.01 and abs(dx.yaw) < 0.05:
            print('Arrived at target!')
            break
        await asyncio.sleep(0.01)


TFManagerInstance = TFManager()
