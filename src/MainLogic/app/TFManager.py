'''
坐标管理类
'''
import asyncio
from Lib.odomVec import Odom
class TFManager:
    def __init__(self):
        self.baseLinkOdom = Odom()
async def move_to(x, y, yaw):
    targetOdom = Odom(x, y, yaw)
    #给电控发坐标指令

    #发送指令代码还没有
    while True:
        await asyncio.sleep(0.01)
        dx = targetOdom - TFManagerInstance.baseLinkOdom
        # 距离小于1cm且角度误差小于0.05rad就认为到达目标了
        if dx.dist < 0.01 and abs(dx.yaw) < 0.05:
            print("Arrived at target!")
            break
TFManagerInstance = TFManager()
