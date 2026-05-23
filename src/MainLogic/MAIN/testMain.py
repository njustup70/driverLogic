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
    paths=np.array(
[[0.39000000, 0.39000000],
 [0.44909091, 0.55545455],
 [0.50818182, 0.72090909],
 [0.56727273, 0.88636364],
 [0.62636364, 1.05181818],
 [0.68545455, 1.21727273],
 [0.74454545, 1.38272727],
 [0.80363636, 1.54818182],
 [0.86272727, 1.71363636],
 [0.92181818, 1.87909091],
 [0.98090909, 2.04454545],
 [1.04000000, 2.21000000],
 [1.15454545, 2.34727273],
 [1.26909091, 2.48454545],
 [1.38363636, 2.62181818],
 [1.49818182, 2.75909091],
 [1.61272727, 2.89636364],
 [1.72727273, 3.03363636],
 [1.84181818, 3.17090909],
 [1.95636364, 3.30818182],
 [2.07090909, 3.44545455],
 [2.18545455, 3.58272727],
 [2.30000000, 3.72000000],
 [2.31700000, 3.89800000],
 [2.33400000, 4.07600000],
 [2.35100000, 4.25400000],
 [2.36800000, 4.43200000],
 [2.38500000, 4.61000000],
 [2.40200000, 4.78800000],
 [2.41900000, 4.96600000],
 [2.43600000, 5.14400000],
 [2.45300000, 5.32200000],
 [2.47000000, 5.50000000],
 [2.66375000, 5.50312500],
 [2.85750000, 5.50625000],
 [3.05125000, 5.50937500],
 [3.24500000, 5.51250000],
 [3.43875000, 5.51562500],
 [3.63250000, 5.51875000],
 [3.82625000, 5.52187500],
 [4.02000000, 5.52500000],
 [4.21375000, 5.52812500],
 [4.40750000, 5.53125000],
 [4.60125000, 5.53437500],
 [4.79500000, 5.53750000],
 [4.98875000, 5.54062500],
 [5.18250000, 5.54375000],
 [5.37625000, 5.54687500],
 [5.57000000, 5.55000000],
 [5.76375000, 5.55312500],
 [5.95750000, 5.55625000],
 [6.15125000, 5.55937500],
 [6.34500000, 5.56250000],
 [6.53875000, 5.56562500],
 [6.73250000, 5.56875000],
 [6.92625000, 5.57187500],
 [7.12000000, 5.57500000],
 [7.31375000, 5.57812500],
 [7.50750000, 5.58125000],
 [7.70125000, 5.58437500],
 [7.89500000, 5.58750000],
 [8.08875000, 5.59062500],
 [8.28250000, 5.59375000],
 [8.47625000, 5.59687500],
 [8.67000000, 5.60000000],
 [8.66800000, 5.40640000],
 [8.66600000, 5.21280000],
 [8.66400000, 5.01920000],
 [8.66200000, 4.82560000],
 [8.66000000, 4.63200000],
 [8.65800000, 4.43840000],
 [8.65600000, 4.24480000],
 [8.65400000, 4.05120000],
 [8.65200000, 3.85760000],
 [8.65000000, 3.66400000],
 [8.64800000, 3.47040000],
 [8.64600000, 3.27680000],
 [8.64400000, 3.08320000],
 [8.64200000, 2.88960000],
 [8.64000000, 2.69600000],
 [8.63800000, 2.50240000],
 [8.63600000, 2.30880000],
 [8.63400000, 2.11520000],
 [8.63200000, 1.92160000],
 [8.63000000, 1.72800000],
 [8.62800000, 1.53440000],
 [8.62600000, 1.34080000],
 [8.62400000, 1.14720000],
 [8.62200000, 0.95360000],
 [8.62000000, 0.76000000]]
           )
    target_yaw=0.0
    # await Move.mpc_move_to([0.5, 0.5, target_yaw])
    
    await Move.mpc_by_path_move(paths, target_yaw)
    while True:
        # 阻塞，无任务
        # from MainLogic.Lib.Visual import PathVisualInstance
        # PathVisualInstance.update("/state/base_link_path",Odom(0.0, 0.0, 0.0))
        await asyncio.sleep(1)