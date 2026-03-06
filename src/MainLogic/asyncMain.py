'''
异步主逻辑和其他函数
'''
import asyncio
from Lib.odomVec import Odom
import Lib.rosBridgeNode as ros_bridge_module
from app.TFManager import move_to
import globalCallback as gcb 
from app.TFManager import TFManagerInstance
async def async_main():
    #注册回调
    assert ros_bridge_module.RosBridgeNodeInstance is not None, "RosBridgeNodeInstance is not initialized yet!"
    ros_bridge_module.RosBridgeNodeInstance.register_serial_sub(gcb.example_serial_callback)
    #往下继续注册
    asyncio.create_task(test())
    #逻辑实例...,比如移动到某个坐标
    await move_to(1.0, 1.0, 0.5)
async def test():
    #测试函数
    while True:
        await asyncio.sleep(1)
        TFManagerInstance.baseLinkOdom = Odom(TFManagerInstance.baseLinkOdom.x + 0.1, TFManagerInstance.baseLinkOdom.y, TFManagerInstance.baseLinkOdom.yaw)
# 检测是否到达目标坐标的异步函数,不涉及运动控制

