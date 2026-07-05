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

        # 三个场地标志位
        # sick纠正选择墙体分边Flag
        self.sick_direction_flag = 0 #（0表示左侧场，1表示右侧场）
        # 场地红蓝场标志位
        self.field_color_flag = 0 #（0表示红场，1表示蓝场）
        # 一三区重启标志位
        self.zone_retry_flag = 1 #（1表示一区，3表示三区）

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
        # 场地/区域看门狗：追踪上次已应用的组合状态
        self._last_applied_field_color = None
        self._last_applied_zone_retry = None
        self._field_zone_config_applied = False
        # sick 修正缓存
        self.sick_lateral_offset = 0.0
        self.sick_buffer_size = 10
        self.right_sick_buffer: list[float] = []
        self.left_sick_buffer: list[float] = []
        # 含sick修正的 map->slam_init 位姿中间变量，其中包含了地图原点到车体中心偏移
        # 存储了sick修正增量的变量，用于连续修正时的撤销与更新逻辑

    def register_tf_chain(self,sick2Base: Odom,map2BaseInit: Odom,Base2laser: Odom,sick_correct_width: float =0.0):
        self.rosBridge = ros_bridge_module.RosBridgeNodeInstance
        assert sick2Base is not None and map2BaseInit is not None and Base2laser is not None, 'TFManager register_tf_chain requires all TFs to be provided!'
        self.laser_to_base = Base2laser.inverse()
        self.mapToBaseInit = map2BaseInit
        self.sickToBaseLink = sick2Base
        self.sick_correct_width = sick_correct_width
        # 计算 sick 相对车体中心 (base_link) 的坐标和 yaw 角
        # sick2Base 是 sick->base_link 的变换，取逆得到 base_link->sick
        BaseLinktoSick = sick2Base.inverse()
        self.sick_x_in_base = BaseLinktoSick.x
        self.sick_y_in_base = BaseLinktoSick.y
        # sick 坐标向量在右手系下相对于 x 轴正方向的旋转角度
        self.sick_yaw_in_base = math.atan2(BaseLinktoSick.y, BaseLinktoSick.x)
        self.sick_dist_to_base = math.sqrt(BaseLinktoSick.x ** 2 + BaseLinktoSick.y ** 2)
        print(f'{self.sick_x_in_base},{self.sick_y_in_base},{self.sick_yaw_in_base}')
        assert self.rosBridge is not None, 'RosBridgeNodeInstance is not initialized yet!'
        # 从 map->base_link_init 推导出 map->slam_init，并发布静态坐标
        # 公式：map->slam_init = map->base_link @ base_link->slam_init
        self._mapToSlamInit = self.mapToBaseInit @ self.laser_to_base.inverse()
        self._sickYawCorrection = 0.0
        self._baseinitYaw=(self._mapToSlamInit@self.laser_to_base).yaw
        self._slaminitYaw=self._mapToSlamInit.yaw
        self.rosBridge.publish_static_tf(self.map_frame, self.slam_init_frame, self._mapToSlamInit)
        # 注册 Vector3Stamped 发布者
        self._tf_chain_registered = True
        # 同步看门狗状态：记录 register_tf_chain 时的初始 flag 组合
        self._last_applied_field_color = self.field_color_flag
        self._last_applied_zone_retry = self.zone_retry_flag
        self._field_zone_config_applied = True

    # ==================== 场地/区域 看门狗 ====================
    # (field_color_flag, zone_retry_flag) → map2BaseInit 映射表
    # field_color_flag: 0=红场, 1=蓝场
    # zone_retry_flag:  1=一区, 3=三区
    FIELD_ZONE_MAP2BASE_CONFIG = {
        (0, 1): Odom(0.45, 0.45, 0),                       # 红场一区 （现有参数）
        (0, 3): Odom(11.480, 0.45, 0.0),                   # 红场三区 PLACEHOLDER
        (1, 1): Odom(0.45, 6-0.9817+0.45, 0.0),            # 蓝场一区 PLACEHOLDER
        (1, 3): Odom(11.480, 6-0.9817+0.45, 0.0),          # 蓝场三区 PLACEHOLDER
    }

    def _check_and_apply_field_zone_config(self):
        """看门狗检查：若场地/区域组合状态变化，更新 map2BaseInit 并重发静态 TF。

        仅当 (field_color_flag, zone_retry_flag) 组合与上次已应用的不同时才执行：
        1. 用新的 map2BaseInit 覆盖 self.mapToBaseInit
        2. 按 register_tf_chain 相同的公式重算 _mapToSlamInit
        3. 重新发布 map → slam_init 静态 TF
        """
        if not self._tf_chain_registered or self.rosBridge is None:
            return

        fc = self.field_color_flag
        zr = self.zone_retry_flag

        # 看门狗层去重：组合状态没变则跳过
        if self._field_zone_config_applied:
            if fc == self._last_applied_field_color and zr == self._last_applied_zone_retry:
                return

        new_map2base = self.FIELD_ZONE_MAP2BASE_CONFIG.get((fc, zr))
        if new_map2base is None:
            print(f"[FieldZone] 未知场地/区域组合: field={fc}, zone={zr}，跳过")
            return

        field_name = "蓝场" if fc else "红场"
        zone_name = f"{zr}区"
        print(f"[FieldZone] 配置变更: {field_name} {zone_name} → map2BaseInit={new_map2base}")

        self.mapToBaseInit = new_map2base
        self._mapToSlamInit = self.mapToBaseInit @ self.laser_to_base.inverse()
        # 同步所有依赖 mapToBaseInit 的派生变量
        self._sickYawCorrection = 0.0
        self._baseinitYaw = (self._mapToSlamInit @ self.laser_to_base).yaw
        self._slaminitYaw = self._mapToSlamInit.yaw
        self.rosBridge.publish_static_tf(self.map_frame, self.slam_init_frame, self._mapToSlamInit)

        self._last_applied_field_color = fc
        self._last_applied_zone_retry = zr
        self._field_zone_config_applied = True

    def odom(self, x: float, y: float, yaw: float):
        """码盘数据入口：更新 odom->base_link。"""
        self._odomToBase = Odom(x, y, yaw)

    def left_sick(self, sick_y: float):
        """SICK 数据入口：输入侧向测距值（单位米）。"""
        self.left_sick_buffer.append(float(sick_y) + self.sick_lateral_offset)
        if len(self.left_sick_buffer) > self.sick_buffer_size:
            self.left_sick_buffer.pop(0)

    def right_sick(self, sick_y: float):
        """SICK 数据入口：输入侧向测距值（单位米）。"""
        self.right_sick_buffer.append(float(sick_y) + self.sick_lateral_offset)
        if len(self.right_sick_buffer) > self.sick_buffer_size:
            self.right_sick_buffer.pop(0)

    # def sickInitYCorrect(self):
    #     '''
    #     sick初始值修正，直接把sick测量的y值作为车体中心到地图原点的y偏移，适用于车体中心在地图原点的情况
    #     '''
    #     if not self.left_sick_buffer:
    #         return False
    #     left_sick_y = sum(self.left_sick_buffer) / len(self.left_sick_buffer)
    #     # sick装左边的情况下场地有一个12cm的初始偏移
    #     map2sick_y = self.sick_correct_width - sick_y - 0.12
    #     self.mapToBaseInit = Odom(self.mapToBaseInit.x, map2sick_y + self.sickToBaseLink.y, self.mapToBaseInit.yaw)

    #     # self._mapToSlamInit = Odom(
    #     #     self._mapToSlamInit.x,
    #     #     map2sick_y + self.sickToBaseLink.y,
    #     #     self._mapToSlamInit.yaw
    #     # ) @ self.laser_to_base.inverse()

    #     self._mapToSlamInitNominal = self.mapToBaseInit @ self.laser_to_base.inverse()
    #     self._mapToSlamInit = self._mapToSlamInitNominal

    #     self.sick_buffer.clear()
    #     return True

    def apply_sick_initial_yaw_correction(self) -> bool:
        """使用 sick 缓存值修正 map->slam_init 的初始 yaw（增量更新，可撤销前次修正）。"""
        if not self.left_sick_buffer or not self.right_sick_buffer or not self._has_slam_pose:
            return False
        sick_y = sum(self.sick_buffer) / len(self.sick_buffer)
        self.mapToBaseInit=Odom(self.mapToBaseInit.x,self.mapToBaseInit.y,self._baseinitYaw)
        #在当前座标系下求BaseInit->Base的值
        baseinit2base=self.laser_to_base.inverse()@self._slamBaseOdom
        # 先撤销上一轮修正，再基于未修正状态计算本轮修正量。
        #base_without_prev = Odom(0,0,-self._sickYawCorrection) @ self.baseLinkOdom.value
        #sick_pose = base_without_prev @ self.sickToBaseLink.inverse()
        #print(sick_pose.x,sick_pose.y,sick_pose.yaw)
        if self.sick_direction_flag==0:
            # 解超越方程
            # y_real=fsolve
            def calculate_y_real(theta):
                return 0.45
            dyaw =  fsolve(lambda theta:-calculate_y_real(theta)+baseinit2base.x*math.sin(theta+self._baseinitYaw)+baseinit2base.y*math.cos(theta+self._baseinitYaw)+self.mapToBaseInit.y,0)[0]

        if self.sick_direction_flag==1:
            def calculate_y_real(theta):
                return 6-0.9817+0.45
            dyaw = - fsolve(lambda theta:-sick_y*math.cos(theta+self._baseinitodom.yaw)+self._baseinitodom.x*math.sin(theta+self._baseinitYaw)+self._baseinitodom.y*math.cos(theta+self._baseinitYaw)+self.mapToBaseInit.y,0)[0]
        
        print(self._mapToSlamInit)
        #self._mapToSlamInit = Odom(0,0,-self._sickYawCorrection) @ Odom(0,0,nominal_yaw+new_yaw_correction) @ self._mapToSlamInit 
        self.mapToBaseInit=Odom(self.mapToBaseInit.x,self.mapToBaseInit.y,self._baseinitYaw+dyaw)
        self._mapToSlamInit=self.mapToBaseInit@self.laser_to_base.inverse()
        print(self._mapToSlamInit)

        if self.rosBridge is not None:
            self.rosBridge.publish_static_tf(self.map_frame, self.slam_init_frame, self._mapToSlamInit)
        self.left_sick_buffer.clear()
        self.right_sick_buffer.clear()
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
        self.rosBridge.writeBytes(b'\xA0' + turn_to_bytes([fused_base.x, fused_base.y, fused_base.yaw]))
        # 发布 Vector3Stamped 话题
        odom_raw=Vector3(x=wheel_pose.x, y=wheel_pose.y, z=wheel_pose.yaw)
        odom_msg = Vector3(x=fused_base.x, y=fused_base.y, z=fused_base.yaw)
        self.rosBridge.publish_ros2(BASE_LINK_ODOM_TOPIC, odom_msg)
        self.rosBridge.publish_ros2('/state/odom_raw',odom_raw)
        # print(f"{self._baseinitodom.x},{self._baseinitodom.y},{self._baseinitodom.yaw}")
        # print(f"{self._sickYawCorrection},{self._mapToSlamInit.yaw}")
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
        
        self._baseinitodom = self.laser_to_base.inverse() @ slam_sensor_pose @ self.laser_to_base
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
        loop = asyncio.get_running_loop()
        t_next = loop.time()
        while True:
            assert self._tf_chain_registered, 'TF chain is not registered yet!'
            try:
                if tick_10ms % 10 == 0:
                    self.slam_100ms()
                    self._check_and_apply_field_zone_config()
                    self.odom_10ms()
                    # 持续下发场地/区域反馈给下位机（上位机端确认）
                    # 帧格式：0xFA 0x78 [field_color_flag] [zone_retry_flag]
                    # （0xFA 帧头由 writeBytes 自动添加）
                    self.rosBridge.writeBytes(b'\x78' + turn_to_bytes([self.field_color_flag, self.zone_retry_flag]))
                else:
                    self.odom_10ms()
            except Exception as e:
                print(e)
            tick_10ms = (tick_10ms + 1) % 10
            t_next += 0.01
            await asyncio.sleep(max(0, t_next - loop.time()))

TFManagerInstance = TFManager()