from MainLogic import globalCallback as gcb
from MainLogic.core import ros_bridge_node as ros_bridge_module
from MainLogic.core.serial_node import start_serial_process
import asyncio
from MainLogic.core.tf_manager import TFManagerInstance
import MainLogic.core.nav.observer as observer_module
from MainLogic.Lib.odomVec import Odom
import MainLogic.core.nav.mpc as mpc
from geometry_msgs.msg import Twist
import MainLogic.core.Move as Move
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
    # ros_bridge_module.RosBridgeNodeInstance.register_serial_sub(gcb.climb_type_callback, b'\xB1')
    sick2Base=Odom(0.0, -0.340, 0.0)
    map2BaseInit=Odom(0.390, 0.390, 0.0)
    laser2Base=Odom(0.310, -0.3515, 0.0)
    TFManagerInstance.register_tf_chain(sick2Base, map2BaseInit, laser2Base)
    asyncio.create_task(TFManagerInstance.tf_update_loop())
    asyncio.create_task(Move.mpc_control_loop())
    asyncio.create_task(observer_module.observer_update())
    #重要await，让上面的任务先运行起来，等它们都准备好了之后再继续往下走
    await asyncio.sleep(0.0)
    # 固定终点模式
    paths=np.array([[0.5, 0.5],
           [1.5, 1.5],
           [2.0,3.5]])
    target_yaw=3.14
    await Move.mpc_move_to([0.5, 0.5, target_yaw])
    while True:
        # 阻塞，无任务
        # from MainLogic.Lib.Visual import PathVisualInstance
        # PathVisualInstance.update("/state/base_link_path",Odom(0.0, 0.0, 0.0))
        await asyncio.sleep(1)