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
    def list_turn_to_bytes(self, data):
        if isinstance(data, (list, tuple)):
            data_bytes = b""
            for i in data:
                data_bytes += self.turn_to_bytes(i)
            return data_bytes
        elif isinstance(data, (bool, int, float)):
            return struct.pack("<B", int(data))
    def turn_to_bytes(self, data):
        if isinstance(data, bool):
            data = struct.pack("<B", int(data))
        # byte (0~255)
        elif isinstance(data, int) and 0 <= data <= 255:
            data = struct.pack("<B", data)
        # int32
        elif isinstance(data, int):
            data = struct.pack("<i", data)
        # float -> float32
        elif isinstance(data, float):
            data = struct.pack("<f", data)
        else:
            raise TypeError(f"类型错误: {type(data)}")
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
    