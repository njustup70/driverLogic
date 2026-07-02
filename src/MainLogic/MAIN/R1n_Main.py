from MainLogic import globalCallback as gcb
from MainLogic.core import ros_bridge_node as ros_bridge_module
from MainLogic.core.serial_node import start_serial_process
import asyncio,math
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
    ros_bridge_module.RosBridgeNodeInstance.register_serial_sub(gcb.sick_callback, b'\xB3')
    ros_bridge_module.RosBridgeNodeInstance.register_serial_sub(gcb.serial_correct_callback, b'\xB2')
    #ros_bridge_module.RosBridgeNodeInstance.register_serial_sub(gcb.climb_type_callback, b'\xB1')
    
    # 注册红蓝场标志位
    TFManagerInstance.start_flag = 1 # 红场=1, 蓝场=-1

    if TFManagerInstance.start_flag == 1:
        TFManagerInstance.sick_flag = 0 # sick纠正方向
    else:
        TFManagerInstance.sick_flag = 1 # sick纠正方向
    
    sf = TFManagerInstance.start_flag

    sick2Base=Odom(-0.3125, -0.495, 0.0) # sick底盘(sick位于底盘的左侧,但在右半场启动)
    if sf == 1:
        map2BaseInit=Odom(0.450, 0.450, 0.0) # 地图起点(0.451,0.453)
    else:
        map2BaseInit=Odom(0.450, -0.5317, 0.0) # 地图起点(0.451,0.453) 0.9817-0.450=0.5317
    laser2Base=Odom(-0.4775,0.345, 0.0) # 雷达底盘
    base2laser=Odom(0.4775,-0.345, 1.6*math.pi/180) # 雷达底盘
    TFManagerInstance.register_tf_chain(sick2Base, map2BaseInit, base2laser)
    asyncio.create_task(TFManagerInstance.tf_update_loop())
    while True:
        #阻塞，无任务
        await asyncio.sleep(1)