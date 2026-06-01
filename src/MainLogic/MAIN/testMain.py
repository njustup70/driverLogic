from MainLogic import globalCallback as gcb
from MainLogic.core import ros_bridge_node as ros_bridge_module
from MainLogic.core.serial_node import start_serial_process
import asyncio
from MainLogic.core.tf_manager import TFManagerInstance,TFOdin
import MainLogic.core.nav.observer as observer_module
from MainLogic.Lib.odomVec import Odom,SE3
import MainLogic.core.nav.mpc as mpc
import MainLogic.core.Move as Move
import numpy as np
async def async_main():
    # 启动 rosSerialNode 进程（非阻塞）
    serial_port = '/dev/serial_ch340'  # 与SICK数据板连接的串口
    baudrate = 921600  # 可以根据需要修改波特率
    start_serial_process(serial_port=serial_port, baudrate=baudrate)

    while ros_bridge_module.RosBridgeNodeInstance is None:
        await asyncio.sleep(0.05)

    # ros_bridge_module.RosBridgeNodeInstance.register_serial_sub(gcb.mcu_transmit_callback, b'\xAA')
    ros_bridge_module.RosBridgeNodeInstance.register_serial_sub(gcb.serial_correct_callback, b'\xB2')
    # ros_bridge_module.RosBridgeNodeInstance.register_serial_sub(gcb.climb_type_callback, b'\xB1')
    TFManagerInstance=TFOdin()
    # odin2Base=Odom(-0.310,- 0.3515, 3.14/2)
    base2odin=Odom(-0.371,0.300,3.1415926/2)
    npy_path='/home/Elaina/ros2_ws/src/MainLogic/SE_Trans.npy'
    SE3_map2odin=SE3(matrix=np.load(npy_path))
    TFManagerInstance.register_tf_chain(base2odin,SE3_map2odin)
    asyncio.create_task(TFManagerInstance.tf_update_loop())
    # asyncio.create_task(Move.mpc_control_loop())
    asyncio.create_task(observer_module.observer_update())
    #重要await，让上面的任务先运行起来，等它们都准备好了之后再继续往下走
    await asyncio.sleep(0.0)
    # 固定终点模式

    while True:
        # 阻塞，无任务
        # from MainLogic.Lib.Visual import PathVisualInstance
        TFManagerInstance.baseLinkOdom.value= Odom(TFManagerInstance.baseLinkOdom.x + 0.1, TFManagerInstance.baseLinkOdom.y, TFManagerInstance.baseLinkOdom.yaw)
        # TFManagerInstance.baseLinkOdom.value.x+=0.1
        await asyncio.sleep(1)