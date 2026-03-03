from mathlib.odomvec import odom
class chassic():
    def __init__(self):
        #初始化当前坐标
        self.odom=odom()
chassic_instance=chassic()