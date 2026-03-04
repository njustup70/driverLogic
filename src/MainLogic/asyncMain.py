'''
异步主逻辑和其他函数
'''
import asyncio
from Lib.odomVec import Odom
from Lib.rosBridgeNode import RosBridgeNodeInstance
from app.TFManager import move_to
import globalCallback as gcb 
async def async_main():
    #注册回调
    assert RosBridgeNodeInstance is not None, "RosBridgeNodeInstance is not initialized yet!"
    RosBridgeNodeInstance.register_serial_sub(gcb.example_serial_callback)
    #往下继续注册

    #逻辑实例...,比如移动到某个坐标
    await move_to(1.0, 1.0, 0.5)

# 检测是否到达目标坐标的异步函数,不涉及运动控制

