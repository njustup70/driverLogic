from MainLogic import globalCallback as gcb
from MainLogic.Lib import rosBridgeNode as ros_bridge_module
from MainLogic.Lib.rosSerialNode import start_serial_process
import asyncio
from MainLogic.app.TFManager import TFManagerInstance
async def async_main():
    # 启动 rosSerialNode 进程（非阻塞）
    serial_port = '/dev/ttyACM0'  # 与SICK数据板连接的串口
    baudrate = 115200  # 可以根据需要修改波特率
    start_serial_process(serial_port=serial_port, baudrate=baudrate)

    while ros_bridge_module.RosBridgeNodeInstance is None:
        await asyncio.sleep(0.05)

    ros_bridge_module.RosBridgeNodeInstance.register_serial_sub(gcb.mcu_sensor_callback)
    TFManagerInstance.register_tf_chain()
    asyncio.create_task(TFManagerInstance.tf_update_loop())
    while True:
        #阻塞，无任务
        await asyncio.sleep(1)
        