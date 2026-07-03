'''
异步主逻辑和其他函数
'''
import asyncio
from MainLogic.Lib.odomVec import Odom
from MainLogic.core import ros_bridge_node as ros_bridge_module
from MainLogic.core.tf_manager import TFManagerInstance
from MainLogic.app.climb_manager import climb
from MainLogic import globalCallback as gcb
from std_msgs.msg import UInt8MultiArray, String
from MainLogic.core.serial_node import start_serial_process

async def async_main():
    # 启动 rosSerialNode 进程（非阻塞）
    serial_port = '/dev/ttyUSB0'  # 可以根据需要修改串口路径
    baudrate = 921600  # 可以根据需要修改波特率
    start_serial_process(serial_port=serial_port, baudrate=baudrate)
    #注册回调
    assert ros_bridge_module.RosBridgeNodeInstance is not None, "RosBridgeNodeInstance is not initialized yet!"

    ros_bridge_module.RosBridgeNodeInstance.register_serial_sub(gcb.meilin_map_frame_callback)
    ros_bridge_module.RosBridgeNodeInstance.register_serial_sub(gcb.action_callback)

