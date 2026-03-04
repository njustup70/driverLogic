'''
坐标管理类
'''
import math

class Odom:
    '''
    2d坐标,包含x,y和yaw
    '''
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = x
        self.y = y
        # 内部yaw初始化
        self._yaw = 0.0
        self.yaw = yaw

    # 使用属性装修yaw,在任意地方处理yaw的范围,保持在[-pi,pi]之间
    @property
    def yaw(self):
        return self._yaw

    @yaw.setter
    def yaw(self, value):
        self._yaw = math.atan2(math.sin(value), math.cos(value))

    def __str__(self):
        return f"Odom(x={self.x}, y={self.y}, yaw={self.yaw/math.pi:.2f}π)"

    @property
    def dist(self):
        """返回当前坐标相对于原点的欧式距离 (位置误差模长)"""
        return math.sqrt(self.x**2 + self.y**2)

    # 重载减法运算符
    def __sub__(self, other):
        if isinstance(other, Odom):
            # 如果是另一个Odom对象，返回一个新的Odom对象，表示坐标差
            return Odom(self.x - other.x, self.y - other.y, self.yaw - other.yaw)
        else:
            raise TypeError("Unsupported operand type(s) for -: 'Odom' and '{}'".format(type(other).__name__))

def test():
    o1 = Odom(1, 2, 0.9 * math.pi)
    print(o1)
    o2 = Odom(4, 6, -0.9 * math.pi)
    print(o2 - o1)

if __name__ == '__main__':
    test()
