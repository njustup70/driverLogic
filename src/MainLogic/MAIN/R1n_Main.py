from MainLogic import globalCallback as gcb
from MainLogic.core import ros_bridge_node as ros_bridge_module
from MainLogic.core.serial_node import start_serial_process
import asyncio
from MainLogic.core.tf_manager import TFManagerInstance
from MainLogic.Lib.odomVec import Odom
async def async_main():
    # 启动 rosSerialNode 进程（非阻塞）
    serial_port = '/dev/serial_r1'  # 与SICK数据板连接的串口
    baudrate = 115200  # 可以根据需要修改波特率
    start_serial_process(serial_port=serial_port, baudrate=baudrate)

    while ros_bridge_module.RosBridgeNodeInstance is None:
        await asyncio.sleep(0.05)

    #ros_bridge_module.RosBridgeNodeInstance.register_serial_sub(gcb.mcu_transmit_callback)
    ros_bridge_module.RosBridgeNodeInstance.register_serial_sub(gcb.mcu_transmit_callback, b'\xAA')
    ros_bridge_module.RosBridgeNodeInstance.register_serial_sub(gcb.serial_correct_callback, b'\xB2')
    #ros_bridge_module.RosBridgeNodeInstance.register_serial_sub(gcb.climb_type_callback, b'\xB1')
    sick2Base=Odom(0.0, 0.390, 0.0) # sick底盘
    map2BaseInit=Odom(0.450, 0.450, 0.0) # 地图起点(0.451,0.453)
    laser2Base=Odom(-0.4775,0.345, 0.0) # 雷达底盘
    TFManagerInstance.register_tf_chain(sick2Base, map2BaseInit, laser2Base)
    asyncio.create_task(TFManagerInstance.tf_update_loop())
    while True:
        #阻塞，无任务
        await asyncio.sleep(1)