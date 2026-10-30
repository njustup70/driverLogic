'''
Author: Nagisa 2964793117@qq.com
Date: 2026-06-26 11:01:15
LastEditors: Nagisa 2964793117@qq.com
LastEditTime: 2026-06-28 15:14:32
FilePath: \driverLogic\src\MainLogic\MAIN\R2n_Main.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
'''
异步主逻辑和其他函数
'''
from MainLogic.core import ros_bridge_node as ros_bridge_module
from MainLogic.core.serial_node import start_serial_process
import asyncio
from MainLogic.core.tf_manager import TFManagerInstance,TFOdin
import MainLogic.core.tf_manager as tf_manager
from MainLogic.Lib.odomVec import Odom,SE3
import MainLogic.core.Move as Move
import numpy as np
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String, UInt8MultiArray
from MainLogic import globalCallback as gcb

async def continuous_move(vx, vy, wz, duration):
    """保持某个速度行驶一段时间"""
    start_time = asyncio.get_running_loop().time()
    while asyncio.get_running_loop().time() - start_time < duration:
        await Move.send_test_speed(vx, vy, wz)
        await asyncio.sleep(0.02)  # 50Hz
    # 时间到了，立刻刹车
    await Move.send_test_speed(0.0, 0.0, 0.0)

async def async_main():
    serial_port = '/dev/serial_ch340'
    baudrate = 921600
    start_serial_process(serial_port=serial_port, baudrate=baudrate)

    assert ros_bridge_module.RosBridgeNodeInstance is not None
    ros_bridge_module.RosBridgeNodeInstance.register_serial_sub(gcb.mcu_transmit_callback, 0xAA)
    
    await asyncio.sleep(0.5)
    
    # 4. 开始发送速度
    print(">>> 准备发送 0.5m/s 的测试速度... <<<")
    for _ in range(10):
        await continuous_move(0.5, 0.0, 0.0, 2.0)
        await asyncio.sleep(1)
        await continuous_move(0.0, 0.5, 0.0, 2.0)
        await asyncio.sleep(1)
        
    print(">>> 停止发送，发送 0 速... <<<")
    await Move.send_test_speed(0.0, 0.0, 0.0)

    while True:
        await asyncio.sleep(1)
        await asyncio.sleep(1)

async def test():
    #测试函数
    while True:
        await asyncio.sleep(1)
        #这里的baseLinkOdom必须整个重新赋值，如果用baseLinkOdom.x=...是不会触发更新的
        TFManagerInstance.baseLinkOdom.value= Odom(TFManagerInstance.baseLinkOdom.x + 0.1, TFManagerInstance.baseLinkOdom.y, TFManagerInstance.baseLinkOdom.yaw)
        # TFManagerInstance.baseLinkOdom.value.x+=0.1

