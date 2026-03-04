'''
异步主逻辑和其他函数
'''
import asyncio
from Lib.odomVec import Odom
from data import chassicInstance

async def async_main():
    await move_to(1.0, 1.0, 0.5)

# 检测是否到达目标坐标的异步函数,不涉及运动控制
async def move_to(x, y, yaw):
    targetOdom = Odom(x, y, yaw)
    #给电控发坐标指令
    while True:
        await asyncio.sleep(0.01)
        dx = targetOdom - chassicInstance.odom
        # 距离小于1cm且角度误差小于0.05rad就认为到达目标了
        if dx.dist < 0.01 and abs(dx.yaw) < 0.05:
            print("Arrived at target!")
            break
