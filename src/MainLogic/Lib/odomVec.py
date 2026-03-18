'''
坐标管理类
'''
import math
from typing import Tuple

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

    def as_tuple(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.yaw)

    @property
    def dist(self):
        """返回当前坐标相对于原点的欧式距离 (位置误差模长)"""
        return math.sqrt(self.x**2 + self.y**2)

    # 重载减法运算符
    def __sub__(self, other):
        if isinstance(other, Odom):
            # 如果是另一个Odom对象，返回一个新的Odom对象，表示坐标差
            return Odom(self.x - other.x, self.y - other.y, self.yaw - other.yaw)
            #只表示坐标差，不表示空间变换
            #只表示坐标差，不表示空间变换
            #只表示坐标差，不表示空间变换
        else:
            raise TypeError("Unsupported operand type(s) for -: 'Odom' and '{}'".format(type(other).__name__))

    def relative_to(self, origin: 'Odom') -> 'Odom':
        """计算 self 相对于 origin 的二维位姿变化（在 origin 局部坐标系下）。"""
        dx_world = self.x - origin.x
        dy_world = self.y - origin.y
        c = math.cos(origin.yaw)
        s = math.sin(origin.yaw)
        dx_local = c * dx_world + s * dy_world
        dy_local = -s * dx_world + c * dy_world
        return Odom(dx_local, dy_local, self.yaw - origin.yaw)

    def __matmul__(self, other: 'Odom') -> 'Odom':
        """重载 @ : self @ other == self ⊕ other。"""
        if not isinstance(other, Odom):
            return NotImplemented
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        x_new = self.x + c * other.x - s * other.y
        y_new = self.y + s * other.x + c * other.y
        return Odom(x_new, y_new, self.yaw + other.yaw)

    def __rmatmul__(self, other: 'Odom') -> 'Odom':
        """支持左侧对象触发的 @ : other @ self。"""
        if not isinstance(other, Odom):
            return NotImplemented
        c = math.cos(other.yaw)
        s = math.sin(other.yaw)
        x_new = other.x + c * self.x - s * self.y
        y_new = other.y + s * self.x + c * self.y
        return Odom(x_new, y_new, other.yaw + self.yaw)

    @staticmethod
    def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
        """将二维 yaw 转换为 ROS 四元数 (x, y, z, w)。"""
        half = yaw * 0.5
        return (0.0, 0.0, math.sin(half), math.cos(half))

    @staticmethod
    def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
        """将 ROS 四元数转换为二维 yaw。"""
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def to_transform_stamped(self, parent_frame: str, child_frame: str, stamp=None):
        """将 Odom 转为 ROS2 TransformStamped。"""
        from geometry_msgs.msg import TransformStamped

        tf_msg = TransformStamped()
        tf_msg.header.frame_id = parent_frame
        tf_msg.child_frame_id = child_frame
        if stamp is not None:
            tf_msg.header.stamp = stamp
        tf_msg.transform.translation.x = float(self.x)
        tf_msg.transform.translation.y = float(self.y)
        tf_msg.transform.translation.z = 0.0
        qx, qy, qz, qw = self.yaw_to_quaternion(self.yaw)
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        return tf_msg

    @classmethod
    def from_transform_stamped(cls, transform_stamped) -> 'Odom':
        """从 ROS2 TransformStamped 解析 Odom。"""
        t = transform_stamped.transform.translation
        r = transform_stamped.transform.rotation
        yaw = cls.quaternion_to_yaw(r.x, r.y, r.z, r.w)
        return cls(float(t.x), float(t.y), yaw)

def test():
    #world->A
    o1 = Odom(1, 2, math.pi)
    print(o1)
    #A->B
    o2 = Odom(4, 6, 0)
    #world->B
    print(o1@o2)

if __name__ == '__main__':
    test()
