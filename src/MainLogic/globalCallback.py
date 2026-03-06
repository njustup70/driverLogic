'''
全局回调函数串口接收回调和ros2话题回调
'''

def example_serial_callback(data: bytes):
    #示例函数
    #检查第一位 非常重要
    if data[0] != 0xAA:
        print(f"Received serial data: {data}")