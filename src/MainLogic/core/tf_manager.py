'''
坐标管理类
'''

import asyncio
import math
from typing import cast
import numpy as np
from scipy.optimize import fsolve
import rclpy.time
from geometry_msgs.msg import Vector3Stamped,Vector3

from MainLogic.Lib.odomVec import Odom,SE3
from MainLogic.Lib.bytes import turn_to_bytes
from MainLogic.Lib.AsyncTools import AsyncVariable

from MainLogic.core import ros_bridge_node as ros_bridge_module
from MainLogic.Lib.Visual import PathVisualInstance
BASE_LINK_ODOM_TOPIC = '/state/base_link_odom'

class TFManager:
    _instance = None  # 存放唯一实例的私有类属性
    # map->base_link 位姿，由 slam 融合计算得到，供上层异步逻辑使用
    def __new__(cls, *args, **kwargs):
        # 如果实例不存在，则创建一个新的
        if cls._instance is None:
            # 调用父类的 __new__ 来分配内存
            cls._instance = super().__new__(cls)
            # 在这里可以加一个初始化标志，防止 __init__ 被重复调用
            cls._instance._is_initialized = False 
        # 如果实例已存在，直接返回旧的内存地址
        return cls._instance

    def __init__(self):
        if getattr(self, '_is_initialized', False):
                    return
        self._is_initialized = True
        # sick纠正场地分类Flag
        self.flag = 0 #（0表示红场，1表示蓝场）
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
        # 注册 Vector3Stamped 发布者
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
        base_without_prev = Odom(0,0,-self._sickYawCorrection) @ self.baseLinkOdom.value
        sick_pose = base_without_prev @ self.sickToBaseLink.inverse()
        print(sick_pose.x,sick_pose.y,sick_pose.yaw)
        if self.flag==0:
            # 解超越方程tan（new_yaw_correction） = （sick_pose.y-（a/sin（new_yaw_correction）-sick_y））/sick_pose.x 
            new_yaw_correction = min([fsolve(lambda theta: np.tan(theta) - (sick_pose.y - (6 / np.cos(theta) - sick_y)) / sick_pose.x, guess)[0] for guess in (-0.5, 0.5)], key=abs)    
        if self.flag==1:
            new_yaw_correction = math.atan2(sick_pose.y - sick_y, sick_pose.x)
        # 从当前 map->slam_init 中撤销旧修正，再应用新修正。
        nominal_yaw = self._mapToSlamInit.yaw - self._sickYawCorrection
        print(self._mapToSlamInit.x,self._mapToSlamInit.y,self._mapToSlamInit.yaw)
        self._mapToSlamInit = Odom(0,0,-self._sickYawCorrection) @ Odom(0,0,nominal_yaw+new_yaw_correction) @ self._mapToSlamInit 
        print(self._mapToSlamInit.x,self._mapToSlamInit.y,self._mapToSlamInit.yaw)

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
        # print(f"is:{fused_base.x}")
        self.rosBridge.writeBytes(b'\xA0' + turn_to_bytes([fused_base.x, fused_base.y, fused_base.yaw]))
        # 发布 Vector3Stamped 话题
        odom_raw=Vector3(x=wheel_pose.x, y=wheel_pose.y, z=wheel_pose.yaw)
        odom_msg = Vector3(x=fused_base.x, y=fused_base.y, z=fused_base.yaw)
        self.rosBridge.publish_ros2(BASE_LINK_ODOM_TOPIC, odom_msg)
        self.rosBridge.publish_ros2('/state/odom_raw',odom_raw)
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
        PathVisualInstance.add_point("/state/base_link_path",self.baseLinkOdom.value)
    async def tf_update_loop(self):
        """统一更新任务：10ms 执行 odom 更新，每 100ms 执行一次 slam 更新。"""
        tick_10ms = 0
        while True:
            assert self._tf_chain_registered, 'TF chain is not registered yet!'
            try:
                
                if tick_10ms % 10 == 0:
                    self.slam_100ms()
                    self.odom_10ms()
                else:
                    self.odom_10ms()
            except Exception as e:
                print(e)
            tick_10ms = (tick_10ms + 1) % 10
            await asyncio.sleep(0.01)

import numpy as np
class TFOdin:
    _instance = None  # 存放唯一实例的私有类属性
    def __new__(cls, *args, **kwargs):
        # 如果实例不存在，则创建一个新的
        if cls._instance is None:
            # 调用父类的 __new__ 来分配内存
            cls._instance = super().__new__(cls)
            # 在这里可以加一个初始化标志，防止 __init__ 被重复调用
            cls._instance._is_initialized = False 
        # 如果实例已存在，直接返回旧的内存地址
        return cls._instance

        
    def __init__(self):
        if getattr(self, '_is_initialized', False):
                    return
        self._is_initialized = True

        # sick纠正场地分类Flag
        self.flag = 0 #（0表示红场，1表示蓝场）
        self.baseLinkOdom: AsyncVariable[Odom] = AsyncVariable(Odom(0.0, 0.0, 0.0))
        self.baseLinkOdom.value = Odom(0.0, 0.0, 0.0)

        # 坐标系固定配置（不使用 ROS2 参数）
        self.map_frame = 'rc_map'
        # map 到 odom 含有Odin 刷新 重定位矩阵 和 固定偏置M矩阵
        self.base_frame = 'base_link'

        self.odin_map_frame = 'map'
        self.odin_odom_frame = 'odom'
        self.odin_base_frame = 'odin1_base_link'
        
        # map -> slam_init（默认对齐）
        self._mapToBase = Odom(0.0, 0.0, 0.0)

        # 控制标志
        self._tf_chain_registered = False
        self._has_slam_pose = False
        self._is_relocalization = False
        self._transSE=SE3(np.array([0.0,0.0,0.0]))

        # sick 修正缓存
        self.sick_lateral_offset = 0.0
        self.sick_buffer_size = 10
        self.sick_buffer: list[float] = []

        # 存储了sick修正增量的变量，用于连续修正时的撤销与更新逻辑
        self._sickYawCorrection = 0.0

    def register_tf_chain(self,Base2odin: Odom,Base2sick: Odom,Map2Base: Odom,Trans:SE3):
        '''
        param Base2odin: 车体中心到odin坐标
        '''
        self.rosBridge = ros_bridge_module.RosBridgeNodeInstance
        assert Base2odin is not None and Base2sick is not None and Trans is not None, 'TFManager register_tf_chain requires all TFs to be provided!'
        self._odin_to_base = Base2odin.inverse()
        self._sick_to_base = Base2sick.inverse()
        self._map_to_base = Map2Base
        self._transSE=Trans
        assert self.rosBridge is not None, 'RosBridgeNodeInstance is not initialized yet!'
        # 从 map->base_link_init 推导出 map->slam_init，并发布静态坐标
        # 公式：map->slam_init = map->base_link @ base_link->slam_init
        self._mapToOdinInit = self._map_to_base @ Base2odin
        self._sickYawCorrection = 0.0
        self.rosBridge.publish_static_tf(self.odin_map_frame, self.map_frame, self._transSE.inverse())
        # 注册 Vector3Stamped 发布者
        self._tf_chain_registered = True

    def sick(self, sick_y: float):
        """SICK 数据入口：输入侧向测距值（单位米）。"""
        self.sick_buffer.append(float(sick_y) + self.sick_lateral_offset)
        if len(self.sick_buffer) > self.sick_buffer_size:
            self.sick_buffer.pop(0)

    def apply_sick_initial_yaw_correction(self) -> bool:
        """使用 sick 缓存值修正 map->slam_init 的初始 yaw（增量更新，可撤销前次修正）。"""
        if not self.sick_buffer:
            return False
        sick_y = sum(self.sick_buffer) / len(self.sick_buffer)

        # 先撤销上一轮修正，再基于未修正状态计算本轮修正量。
        base_without_prev = Odom(
            self.baseLinkOdom.value.x,
            self.baseLinkOdom.value.y,
            self.baseLinkOdom.value.yaw - self._sickYawCorrection,
        )
        sick_pose = base_without_prev @ self._sick_to_base
        new_yaw_correction = math.atan2(sick_pose.y - sick_y, sick_pose.x)

        # 从当前 map->slam_init 中撤销旧修正，再应用新修正。
        nominal_yaw = self._mapToOdinInit.yaw - self._sickYawCorrection
        self._mapToOdinInit = Odom(
            self._mapToOdinInit.x,
            self._mapToOdinInit.y,
            nominal_yaw + new_yaw_correction,
        )
        self._sickYawCorrection = new_yaw_correction

        if self.rosBridge is not None:
            pass
        self.sick_buffer.clear()
        return True
    def sickInitYCorrect(self):
        '''
        sick初始值修正，直接把sick测量的y值作为车体中心到地图原点的y偏移，适用于车体中心在地图原点的情况
        '''
        if not self.sick_buffer:
            return False
        sick_y = sum(self.sick_buffer) / len(self.sick_buffer)
        #先堆屎，sick装左边的情况下场地有一个12cm的初始偏移
        map2sick_y=6.0-sick_y-0.12
        self._mapToOdinInit = Odom(
            self._mapToOdinInit.x,map2sick_y+self._sick_to_base.y,self._mapToOdinInit.yaw)
        
    def odom_10ms(self):
        """10ms 更新：发布 odom/base, map/odom, 计算 map/base 并下发到下位机。"""
        assert self._tf_chain_registered, 'TF chain is not registered yet!'
        #从slam_odom->slam_base的TF中获取slam_init->base_link
        try:
            # tf_reloc_msg = self.rosBridge._tfBuffer.lookup_transform(
            #     self.odin_map_frame,
            #     self.odin_odom_frame,
            #     rclpy.time.Time(),
            # )
            # tf_odom_msg = self.rosBridge._tfBuffer.lookup_transform(
            #     self.odin_odom_frame,
            #     self.odin_base_frame,
            #     rclpy.time.Time(),
            # )
            tf_map_base_odin=self.rosBridge._tfBuffer.lookup_transform(
                self.odin_map_frame,
                self.odin_base_frame,
                rclpy.time.Time(),
            )
            self._is_relocalization = True
        except Exception:
            self._is_relocalization = False
            # return
        # 若未能成功获取重定位完整TF，则为SLAM模式
        if not self._is_relocalization:
            try:
                tf_odom_base_odin = self.rosBridge._tfBuffer.lookup_transform(
                    self.odin_odom_frame,
                    self.odin_base_frame,
                    rclpy.time.Time(),
                )
            except Exception:
                return
        
        if(self._is_relocalization):
            raw_SE3=SE3.from_transform_stamped(tf_map_base_odin)
            # raw_odom=SE3.from_transform_stamped(tf_odom_msg)
            #========== 坐标变换逻辑 ============
            # 场景坐标系 ——> Odin 建图起点坐标系 ——> 此次定位起点坐标系 ——> Odin里程计坐标系 ——> 车体中心坐标系
            #抓换到ref座标系(地图座标系)
            # 场景坐标系 ——> Odin 建图起点坐标系 ——> 此次定位起点坐标系
            ref_SE3=self._transSE@raw_SE3
            map_to_odin = ref_SE3.to_odom()

            baselink = (map_to_odin @ self._odin_to_base)
            # ========== 发布 TF 树 ============
            # 发布 map_odin -> odom_odin
            # self.rosBridge.publish_dynamic_tf(self.map_frame, self.odom_frame, map_to_odom)
            # 发布 odom_odin -> base_link_


        else:
            raw_SE3=SE3.from_transform_stamped(tf_odom_base_odin)
            odom_to_base = raw_SE3.to_odom()
            baselink = self._mapToOdinInit @ odom_to_base @ self._odin_to_base
            if(self._mapToOdinInit!=Odom(0.0,0.0,0.0)):
                print(f"mapToOdinInit: x={self._mapToOdinInit.x:.3f}, y={self._mapToOdinInit.y:.3f}, yaw={self._mapToOdinInit.yaw:.3f}")
                
        self.rosBridge.publish_dynamic_tf(self.map_frame, self.base_frame, baselink)
        self.baseLinkOdom.value = baselink
        odom_msg = Vector3(x=baselink.x, y=baselink.y, z=baselink.yaw)
        self.rosBridge.publish_ros2(BASE_LINK_ODOM_TOPIC, odom_msg)
        self.rosBridge.writeBytes(b'\xA0' + turn_to_bytes([baselink.x, baselink.y, baselink.yaw]))
    async def tf_update_loop(self):
        """统一更新任务：10ms 执行 odom 更新"""
        while True:
            assert self._tf_chain_registered, 'TF chain is not registered yet!'
            self.odom_10ms()
            await asyncio.sleep(0.01)
TFManagerInstance = TFManager()
TFOdinInstance=TFOdin()