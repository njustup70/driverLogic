from MainLogic import globalCallback as gcb
from MainLogic.core import ros_bridge_node as ros_bridge_module
from MainLogic.core.serial_node import start_serial_process
import asyncio
from MainLogic.core.tf_manager import TFManagerInstance
import MainLogic.core.nav.observer as observer_module
from MainLogic.Lib.odomVec import Odom
import MainLogic.core.nav.mpc as mpc
from geometry_msgs.msg import Twist
import numpy as np
async def async_main():
    # 启动 rosSerialNode 进程（非阻塞）
    serial_port = '/dev/serial_ch340'  # 与SICK数据板连接的串口
    baudrate = 921600  # 可以根据需要修改波特率
    start_serial_process(serial_port=serial_port, baudrate=baudrate)

    while ros_bridge_module.RosBridgeNodeInstance is None:
        await asyncio.sleep(0.05)

    ros_bridge_module.RosBridgeNodeInstance.register_serial_sub(gcb.mcu_transmit_callback, b'\xAA')
    ros_bridge_module.RosBridgeNodeInstance.register_serial_sub(gcb.serial_correct_callback, b'\xB2')
    ros_bridge_module.RosBridgeNodeInstance.register_serial_sub(gcb.climb_type_callback, b'\xB1')
    sick2Base=Odom(0.0, -0.340, 0.0)
    map2BaseInit=Odom(0.390, 0.390, 0.0)
    laser2Base=Odom(0.310, -0.3515, 0.0)
    TFManagerInstance.register_tf_chain(sick2Base, map2BaseInit, laser2Base)
    asyncio.create_task(TFManagerInstance.tf_update_loop())
    asyncio.create_task(mpc.mpc_loop())
    asyncio.create_task(observer_module.observer_update())
    # 固定终点模式
    # paths=np.array([[0.0, 0.0],
    #        [0.5, 0.0],
    #        [1.0, 0.0],
    #        [1.5, 0.0],
    #        [1.5, 0.5],
    #        [1.5, 1.0],
    #        [1.5, 1.5]])
    # target_yaw=1.0
    # mpc.MPCPathFollowerInstance.set_path(paths, target_yaw, ref_speed=0.5)
    mpc.MPCPathFollowerInstance.set_target_point(Odom(0.5, 0.5, 0.5))
    # mpc.MPCPathFollowerInstance.set_path(path, 0.5)
    while True:
        # 阻塞，无任务
        # from MainLogic.Lib.Visual import PathVisualInstance
        # PathVisualInstance.update("/state/base_link_path",Odom(0.0, 0.0, 0.0))
        await asyncio.sleep(1)