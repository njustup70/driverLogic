'''
坐标管理类
'''
import asyncio
from Lib.odomVec import Odom
from Lib.bytes import DataFrame, turn_to_bytes
import Lib.rosBridgeNode as ros_bridge_module
from Lib.AsyncTools import AsyncVariable
class TFManager:
    def __init__(self):
        self._baseLinkOdom = AsyncVariable(Odom())
    
    @property
    def baseLinkOdom(self):
        return self._baseLinkOdom.value
    
    @baseLinkOdom.setter
    def baseLinkOdom(self, value:Odom):
        self._baseLinkOdom.value = value
        
    @property
    def baseLinkOdom_async(self):
        return self._baseLinkOdom

async def move_to(x, y, yaw):
    targetOdom = Odom(x, y, yaw)
    #给电控发坐标指令
    assert ros_bridge_module.RosBridgeNodeInstance is not None, "RosBridgeNodeInstance is not initialized yet!"
    ros_bridge_module.RosBridgeNodeInstance.writeBytes(b'\xA1' + turn_to_bytes([x, y, yaw]))
    #rosBridgeNode.writeBytes(b'\xA1' + list_turn_to_bytes([x, y, yaw]))
    #发送指令代码还没有
    while True:
        # 等待baseLinkOdom更新
        current_odom = await TFManagerInstance.baseLinkOdom_async
        dx = targetOdom - current_odom
        # 距离小于1cm且角度误差小于0.05rad就认为到达目标了
        if dx.dist < 0.01 and abs(dx.yaw) < 0.05:
            print("Arrived at target!")
            break
TFManagerInstance = TFManager()
