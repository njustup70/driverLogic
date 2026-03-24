from MainLogic import globalCallback as gcb
from MainLogic.core import ros_bridge_node as ros_bridge_module
from MainLogic.core.serial_node import start_serial_process
import asyncio
from MainLogic.core.tf_manager import TFManagerInstance
import MainLogic.core.observer as observer_module
from MainLogic.Lib.odomVec import Odom
import MainLogic.core.mpc as mpc
from geometry_msgs.msg import Twist
async def async_main():
    # 启动 rosSerialNode 进程（非阻塞）
    serial_port = '/dev/ttyUSB0'  # 与SICK数据板连接的串口
    baudrate = 115200  # 可以根据需要修改波特率
    start_serial_process(serial_port=serial_port, baudrate=baudrate)

    while ros_bridge_module.RosBridgeNodeInstance is None:
        await asyncio.sleep(0.05)

    ros_bridge_module.RosBridgeNodeInstance.register_serial_sub(gcb.mcu_transmit_callback)
    sick2Base=Odom(0.0, -0.340, 0.0)
    map2BaseInit=Odom(0.390, 0.390, 0.0)
    laser2Base=Odom(0.0, -0.390, 0.0)
    TFManagerInstance.register_tf_chain(sick2Base, map2BaseInit, laser2Base)
    asyncio.create_task(TFManagerInstance.tf_update_loop())
    asyncio.create_task(mpc.mpc_loop())
    asyncio.create_task(observer_module.observer_update())
    import numpy as np
    # path=np.array([[1.0, 1.0]])
    # mpc.MPCPathFollowerInstance.set_target_point(10.0, 10.0, 0.5)
    # 固定终点模式
    mpc.MPCPathFollowerInstance.set_target_point(np.array([1.5, 1.5, 1.0]))
    # mpc.MPCPathFollowerInstance.set_path(path, 0.5)
    while True:
        #阻塞，无任务
        await asyncio.sleep(1)
        