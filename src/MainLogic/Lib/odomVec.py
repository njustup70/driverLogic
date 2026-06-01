'''
坐标管理类
'''
import math,time
from typing import Tuple
import numpy as np

class Odom:
    '''
    2d坐标,包含x,y和yaw
    '''
    def __init__(self, x=0.0, y=0.0, yaw=0.0,timestamp=None):
        self.x = x
        self.y = y
        # 内部yaw初始化
        self._yaw = 0.0
        self.yaw = yaw
        if timestamp is not None:
            self.timestamp = timestamp #单位为s
        else:
            self.timestamp=time.time()
    # 使用属性装修yaw,在任意地方处理yaw的范围,保持在[-pi,pi]之间
    @property
    def yaw(self):
        return self._yaw

    @yaw.setter
    def yaw(self, value):
        self._yaw = math.atan2(math.sin(value), math.cos(value))

    def __str__(self):
        return f"Odom(x={self.x}, y={self.y}, yaw={self.yaw/math.pi:.2f}π)"

    def __iter__(self):
        yield self.x
        yield self.y
        yield self.yaw
    def __array__(self, dtype=None):
        return np.array([self.x, self.y, self.yaw], dtype=dtype)
    
    
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

    def inverse(self) -> 'Odom':
        """
        返回当前位姿变换的逆变换。

        用法示例:
        1. 已知 A->B, 求 B->A: `b_to_a = a_to_b.inverse()`
        2. 链式求解: `a_to_c = a_to_b @ b_to_c`
        3. 已知 A->C 与 A->B, 求 B->C: `b_to_c = a_to_b.inverse() @ a_to_c`
        """
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        return Odom(
            -(c * self.x + s * self.y),
            -(-s * self.x + c * self.y),
            -self.yaw,
        )

    @staticmethod
    def inv(transform: 'Odom') -> 'Odom':
        """静态便捷调用：`Odom.inv(a_to_b)` 等价于 `a_to_b.inverse()`。"""
        if not isinstance(transform, Odom):
            raise TypeError("transform must be Odom")
        return transform.inverse()

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
    @classmethod
    def from_array(cls, arr) -> 'Odom':
        """从数组或列表创建 Odom，支持任意长度但至少包含 x, y, yaw 三个元素。"""
        if len(arr) < 3:
            raise ValueError("Input array must have at least 3 elements for x, y, yaw")
        return cls(float(arr[0]), float(arr[1]), float(arr[2]))
from scipy.spatial.transform import Rotation as R

class SE3:
    """
    6自由度三维刚体变换类 (Special Euclidean Group 3)
    【纯矩阵核心 + ScipyAPI 驱动】类内隐藏位置与四元数，无欧拉角，逻辑极简。
    """
    def __init__(self, matrix: np.ndarray, timestamp=None):
        self.timestamp = timestamp if timestamp is not None else time.time()
        
        if matrix is None:
            self.matrix = np.eye(4, dtype=np.float64)
            self._x, self._y, self._z = 0.0, 0.0, 0.0
            self._qx, self._qy, self._qz, self._qw = 0.0, 0.0, 0.0, 1.0
            return
        #如果为维数为3，则将x y z生成4*4的齐次矩阵
        if matrix.shape == (3,):
            self.matrix = np.eye(4, dtype=np.float64)
            self.matrix[0:3, 3] = matrix
            self._x, self._y, self._z = float(matrix[0]), float(matrix[1]), float(matrix[2])
            self._qx, self._qy, self._qz, self._qw = 0.0, 0.0, 0.0, 1.0
            return
        elif matrix.shape != (4, 4):
            raise ValueError("齐次变换矩阵必须是 4x4 的 NumPy 数组")
        self.matrix = np.array(matrix, dtype=np.float64)
        
        # 1. 提取平移，保存在类内（不对外公开）
        self._x = float(self.matrix[0, 3])
        self._y = float(self.matrix[1, 3])
        self._z = float(self.matrix[2, 3])
        
        # 2. 调用现成 API 提取四元数并留在类内 (Scipy 返回顺序默认即为 [x, y, z, w])
        quat = R.from_matrix(self.matrix[0:3, 0:3]).as_quat()
        self._qx, self._qy, self._qz, self._qw = quat

    # --- 运算符重载 ---
    def __matmul__(self, other: 'SE3') -> 'SE3':
        if isinstance(other, SE3):
            return SE3(matrix=self.matrix @ other.matrix, timestamp=self.timestamp)
        elif isinstance(other, np.ndarray):
            if other.shape == (4,) or other.shape == (4, 1):
                return self.matrix @ other
            elif other.shape == (3,):
                return (self.matrix @ np.append(other, 1.0))[0:3]
        return NotImplemented

    def __rmatmul__(self, other: np.ndarray) -> 'SE3':
        if isinstance(other, np.ndarray) and other.shape == (4, 4):
            return SE3(matrix=other @ self.matrix, timestamp=self.timestamp)
        return NotImplemented

    def inverse(self) -> 'SE3':
        rot = self.matrix[0:3, 0:3]
        t = self.matrix[0:3, 3]
        inv_mat = np.eye(4, dtype=np.float64)
        inv_mat[0:3, 0:3] = rot.T
        inv_mat[0:3, 3] = -rot.T @ t
        return SE3(matrix=inv_mat, timestamp=self.timestamp)

    def __array__(self, dtype=None):
        return self.matrix.astype(dtype) if dtype else self.matrix

    # --- 互转 2D Odom 类 ---
    def to_odom(self) -> 'Odom':
        """使用类内缓存的 _x, _y 和矩阵自带的旋转直接生成 Odom"""
        yaw = np.arctan2(self.matrix[1, 0], self.matrix[0, 0])
        return Odom(x=self._x, y=self._y, yaw=float(yaw), timestamp=self.timestamp)

    @classmethod
    def from_odom(cls, odom: 'Odom', z=0.0) -> 'SE3':
        """从 2D Odom 升维构建齐次矩阵"""
        c, s = np.cos(odom.yaw), np.sin(odom.yaw)
        mat = np.array([
            [c,  -s,  0.0, float(odom.x)],
            [s,   c,  0.0, float(odom.y)],
            [0.0, 0.0, 1.0, float(z)],
            [0.0, 0.0, 0.0, 1.0]
        ], dtype=np.float64)
        return cls(matrix=mat, timestamp=odom.timestamp)

    # --- 与 ROS2 TF 互转 (得益于类内变量与 API，逻辑被降维打击) ---
    def to_transform_stamped(self, parent_frame: str, child_frame: str, stamp=None):
        from geometry_msgs.msg import TransformStamped
        tf_msg = TransformStamped()
        tf_msg.header.frame_id = parent_frame
        tf_msg.child_frame_id = child_frame
        if stamp is not None:
            tf_msg.header.stamp = stamp

        # 直接使用类内维护的隐藏分量进行赋值，没有任何计算开销
        tf_msg.transform.translation.x = self._x
        tf_msg.transform.translation.y = self._y
        tf_msg.transform.translation.z = self._z

        tf_msg.transform.rotation.x = self._qx
        tf_msg.transform.rotation.y = self._qy
        tf_msg.transform.rotation.z = self._qz
        tf_msg.transform.rotation.w = self._qw
        return tf_msg

    @classmethod
    def from_transform_stamped(cls, transform_stamped) -> 'SE3':
        t = transform_stamped.transform.translation
        r = transform_stamped.transform.rotation
        
        # 1. 抛弃手写公式，直接调用 scipy 用四元数恢复 3x3 旋转矩阵
        rot_mat = R.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
        
        # 2. 拼接 4x4 齐次矩阵
        mat = np.eye(4, dtype=np.float64)
        mat[0:3, 0:3] = rot_mat
        mat[0:3, 3] = [t.x, t.y, t.z]
        
        return cls(matrix=mat, timestamp=transform_stamped.header.stamp)

    def __str__(self):
        return f"SE3 Matrix:\n{self.matrix}"
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
