'''
定义串口数据帧转换相关的函数
'''

import time
import struct
from Lib.odomVec import Odom
from typing import Union, List, Tuple

# 定义类型别名，增加可读性
SupportedType = Union[bool, int, float, list, tuple]

def turn_to_bytes(data: SupportedType) -> bytes:
    """
    将数据转换为字节流。支持嵌套列表和元组。
    """
    # 1. 处理容器类型 (递归)
    if isinstance(data, (list, tuple)):
        # 使用列表推导式配合 b"".join() 是 Python 中处理字节拼接最高效的方式
        return b"".join(turn_to_bytes(item) for item in data)

    # 2. 处理布尔值 (必须放在 int 之前，因为 bool 是 int 的子类)
    if isinstance(data, bool):
        return struct.pack("<B", int(data))

    # 3. 处理整数
    if isinstance(data, int):
        # 1字节 (0~255)
        if 0 <= data <= 255:
            return struct.pack("<B", data)
        # 4字节 (int32)
        return struct.pack("<i", data)

    # 4. 处理浮点数 (float32)
    if isinstance(data, float):
        return struct.pack("<f", data)

    # 5. 异常处理
    raise TypeError(f"不支持的序列化类型: {type(data)}")
