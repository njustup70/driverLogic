'''
异步主逻辑和其他函数
'''
import asyncio
from MainLogic.Lib.odomVec import Odom
from MainLogic.core import ros_bridge_node as ros_bridge_module
from MainLogic.core.tf_manager import move_to, TFManagerInstance
from MainLogic.app.climb_manager import climb
from MainLogic import globalCallback as gcb
from std_msgs.msg import UInt8MultiArray, String
from MainLogic.core.serial_node import start_serial_process

async def async_main():
    # 启动 rosSerialNode 进程（非阻塞）
    serial_port = '/dev/ttyACM0'  # 可以根据需要修改串口路径
    baudrate = 115200  # 可以根据需要修改波特率
    start_serial_process(serial_port=serial_port, baudrate=baudrate)
    #注册回调
    assert ros_bridge_module.RosBridgeNodeInstance is not None, "RosBridgeNodeInstance is not initialized yet!"
    #ros_bridge_module.RosBridgeNodeInstance.register_serial_sub(gcb.example_serial_callback)
    #往下继续注册
    ros_bridge_module.RosBridgeNodeInstance.register_serial_sub(gcb.mcu_transmit_callback)
    sick2Base=Odom(0.0, -0.340, 0.0)
    map2BaseInit=Odom(0.390, 0.390, 0.0)
    laser2Base=Odom(0.0, 0.390, 0.0)
    TFManagerInstance.register_tf_chain(sick2Base, map2BaseInit, laser2Base)
    asyncio.create_task(TFManagerInstance.tf_update_loop())
    while True:
        await asyncio.sleep(1)
    #ros_bridge_module.RosBridgeNodeInstance.register_serial_sub(gcb.serial_action_return_callback)
    ros_bridge_module.RosBridgeNodeInstance.register_serial_sub(gcb.climb_type_callback)
    ros_bridge_module.RosBridgeNodeInstance.register_ros2_sub('qr_detection_result', gcb.ros_qr_callback, type=String)
    ros_bridge_module.RosBridgeNodeInstance.register_ros2_sub('spear_status', gcb.spear_callback, type=UInt8MultiArray)
    #注册话题发布
    ros_bridge_module.RosBridgeNodeInstance.register_ros2_pub('/update_exec_req', String)
    ros_bridge_module.RosBridgeNodeInstance.register_ros2_pub('location', String)

    TFManagerInstance.register_tf_chain()
    asyncio.create_task(TFManagerInstance.tf_update_loop())

    #逻辑实例...,比如移动到某个坐标
    await move_to(1.0,1.0,1.0)
    await climb([0,1], [1,1])
    # await move_to(2.0, 2.5, 1.6) # 矛头位置
    # await move_to(0.5, 0.5, 0.0) # 原点
    # await take_spear_head()
    # await move_to(0.2, 0.2, 0.0) # 矛对接点
    # await build_spear()
    # await move_to(1.0,1.0,1.0)

async def test():
    #测试函数
    while True:
        await asyncio.sleep(1)
        #这里的baseLinkOdom必须整个重新赋值，如果用baseLinkOdom.x=...是不会触发更新的
        TFManagerInstance.baseLinkOdom = Odom(TFManagerInstance.baseLinkOdom.x + 0.1, TFManagerInstance.baseLinkOdom.y, TFManagerInstance.baseLinkOdom.yaw)
        # TFManagerInstance.baseLinkOdom.value.x+=0.1
