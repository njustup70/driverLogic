'''
异步主逻辑和其他函数
'''
import asyncio
from Lib.odomVec import Odom
import Lib.rosBridgeNode as ros_bridge_module
from app.TFManager import move_to, climb
import globalCallback as gcb 
from app.TFManager import TFManagerInstance
from Lib.AsyncTools import AsyncVariable
from app.actions import take_spear_head , build_spear , QR
from std_msgs.msg import UInt8MultiArray, String
from app.actions import QR_recog, QR_recog_off, QRRecogInstance
async def async_main():
    #注册回调
    assert ros_bridge_module.RosBridgeNodeInstance is not None, "RosBridgeNodeInstance is not initialized yet!"
    #ros_bridge_module.RosBridgeNodeInstance.register_serial_sub(gcb.example_serial_callback)
    #往下继续注册
    #ros_bridge_module.RosBridgeNodeInstance.register_serial_sub(gcb.serial_action_return_callback)
    ros_bridge_module.RosBridgeNodeInstance.register_serial_sub(gcb.climb_type_callback)
    ros_bridge_module.RosBridgeNodeInstance.register_ros2_sub('qr_detection_result', gcb.ros_qr_callback, type=String)
    ros_bridge_module.RosBridgeNodeInstance.register_ros2_sub('spear_status', gcb.spear_callback, type=UInt8MultiArray)
    #注册话题发布
    ros_bridge_module.RosBridgeNodeInstance.register_ros2_pub('/update_exec_req', String)
    ros_bridge_module.RosBridgeNodeInstance.register_ros2_pub('location', String)

    asyncio.create_task(test())
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
