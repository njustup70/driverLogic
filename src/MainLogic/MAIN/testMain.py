'''
Author: Nagisa 2964793117@qq.com
Date: 2026-03-24 18:33:24
LastEditors: Nagisa 2964793117@qq.com
LastEditTime: 2026-06-08 17:05:56
FilePath: \driverLogic\src\MainLogic\MAIN\testMain.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''

from MainLogic.core import ros_bridge_node as ros_bridge_module
from MainLogic.core.serial_node import start_serial_process
import asyncio
from MainLogic.core.tf_manager import TFManagerInstance,TFOdin
import MainLogic.core.tf_manager as tf_manager
import MainLogic.core.nav.observer as observer_module
from MainLogic.Lib.odomVec import Odom,SE3
import MainLogic.core.nav.mpc as mpc
import MainLogic.core.Move as Move
import numpy as np
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String, UInt8MultiArray
from MainLogic import globalCallback as gcb
from MainLogic.app.actions import (
    BUILD_SPEAR_ACTION_TYPE,
    build_spear_active,
    build_spear_finish,
    debug_spear_offset_callback,
    build_spear_until_finish,
)

async def async_main():
    # 启动 rosSerialNode 进程（非阻塞）
    serial_port = '/dev/serial_ch340'  # 与SICK数据板连接的串口
    baudrate = 921600  # 可以根据需要修改波特率
    start_serial_process(serial_port=serial_port, baudrate=baudrate)

    while ros_bridge_module.RosBridgeNodeInstance is None:
        await asyncio.sleep(0.05)

    # odin定位需要的部分
    ros_bridge_module.RosBridgeNodeInstance.register_serial_sub(gcb.mcu_transmit_callback, 0xAA)
    ros_bridge_module.RosBridgeNodeInstance.register_serial_sub(gcb.sick_callback, 0xB3)
    ros_bridge_module.RosBridgeNodeInstance.register_serial_sub(gcb.serial_correct_callback, 0xB2)
    # tf_manager.TFManagerInstance=tf_manager.TFOdin()

    tf_manager.TFManagerInstance=tf_manager.TFOdin()
    # print(id(TFManagerInstance))
    TFManagerInstance=tf_manager.TFManagerInstance
    print(id(TFManagerInstance))
    # TFManagerInstance=tf_manager.TFManagerInstance
    base2odin=Odom(-0.336,0.371,3.1415926/2)
    Base2sick=Odom(0.0,0.37116,0.0)
    Map2Base=Odom(0.41,4.6,0.0)
    npy_path='/home/Elaina/ros2_ws/src/MainLogic/SE_Trans.npy'
    # 如果没有预先计算好的SE，先跑一遍对应的标定代码
    import os
    if not os.path.exists(npy_path):
        import MainLogic.core.icp as icp
        icp.main()
    SE3_map2odin=SE3(matrix=np.load(npy_path))
    TFManagerInstance.register_tf_chain(base2odin,Base2sick,Map2Base,SE3_map2odin)
    asyncio.create_task(TFManagerInstance.tf_update_loop())
    # odin定位需要的部分

    # asyncio.create_task(Move.mpc_control_loop())
    asyncio.create_task(observer_module.observer_update())

 # 【优先注册】先把所有通道建好，防止移动过程中漏掉数据
    # 这个回调用来检测build_spear动作是否完成
    ros_bridge_module.RosBridgeNodeInstance.register_serial_sub(gcb.serial_action_return_callback, 0xA3) 
    # 这个回调用来获取二区物块的摆放状态
    ros_bridge_module.RosBridgeNodeInstance.register_ros2_sub("qr_detection_result", gcb.ros_qr_callback, type=String)
    # 暂时看不懂这个回调的作用
    ros_bridge_module.RosBridgeNodeInstance.register_ros2_sub("spear_status", gcb.spear_callback, type=UInt8MultiArray) 
    # 订阅这个话题给下位机发送矛头偏移
    ros_bridge_module.RosBridgeNodeInstance.register_ros2_sub("/arucopnp/offset_mm", debug_spear_offset_callback, type=PointStamped)

    #重要await，让上面的任务先运行起来，等它们都准备好了之后再继续往下走
    # await asyncio.sleep(0.5)
    await asyncio.sleep(0.5)
    # await asyncio.sleep(1.5)
    # 跑到矛头架
    # await Move.mpc_move_to_point([1.45, 5.5, 0.0], ref_speed=0.5)
    print("等待 SICK 数据接入...")
    while len(TFManagerInstance.sick_buffer) == 0:
        await asyncio.sleep(0.1)  # 短暂让出控制权，不阻塞其他协程
    
    # 可选：再等 0.5 秒让 buffer 填满（因为你的 buffer_size 是 10，取均值更准）
    await asyncio.sleep(0.5) 
    
    print("SICK 数据就绪，执行初始 Y 轴修正")
    TFManagerInstance.sickInitYCorrect()

    # 这个异步函数完成用来矛头对齐
    await build_spear_until_finish()

    while True:
        # 阻塞，无任务
        # from MainLogic.Lib.Visual import PathVisualInstance
        # TFManagerInstance.baseLinkOdom.value= Odom(TFManagerInstance.baseLinkOdom.x + 0.1, TFManagerInstance.baseLinkOdom.y, TFManagerInstance.baseLinkOdom.yaw)
        # TFManagerInstance.baseLinkOdom.value.x+=0.1
        await asyncio.sleep(1)