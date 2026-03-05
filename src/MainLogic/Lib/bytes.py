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
    def turn_to_bytes(self, data):
        data_bytes = struct.pack("<" + "f"*len(data), *data)
        data 
        return data_bytes
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
    