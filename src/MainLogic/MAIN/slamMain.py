import globalCallback as gcb 
from app.TFManager import TFManagerInstance
from Lib.rosSerialNode import start_serial_process
import asyncio
async def async_main():
    # 启动 rosSerialNode 进程（非阻塞）
    serial_port = '/dev/ttyACM0'  # 可以根据需要修改串口路径
    baudrate = 115200  # 可以根据需要修改波特率
    start_serial_process(serial_port=serial_port, baudrate=baudrate)

    while True:
        #阻塞，无任务
        await asyncio.sleep(1)
        