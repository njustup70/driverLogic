'''
坐标管理类
'''
from Lib.odomVec import Odom
class TFManager:
    def __init__(self):
        self.baseLinkOdom = Odom()
TFManagerInstance = TFManager()
