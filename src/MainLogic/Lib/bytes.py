'''
定义串口数据帧的类
'''

import time
import struct
class DataFrame:
    def __init__(self, data: bytes,timestamp=None):
        self.data = data
        # self.timestamp = timestamp 
        self.timestamp = timestamp if timestamp is not None else time.time() #记录数据帧的接收时间，单位为秒
    def to_dict(self):
        # JSON 不支持 bytes，所以必须转为 hex 字符串
        return {
            "data": self.data.hex(),
            "timestamp": self.timestamp
        }
    @classmethod
    def from_dict(cls, d:dict):
        # 反序列化时将 hex 字符串还原为 bytes
        return cls(
            data=bytes.fromhex(d["data"]),
            #如果没有timestamp字段就用当前时间
            timestamp=d.get("timestamp", time.time())
        )
    def __str__(self):
        #返回16进制字符串表示数据帧内容和接收时间
        return f"DataFrame(data={self.data.hex()}, timestamp={self.timestamp:.2f})"
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