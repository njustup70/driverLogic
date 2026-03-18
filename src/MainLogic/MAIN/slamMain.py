import globalCallback as gcb 
import Lib.rosBridgeNode as ros_bridge_module
from Lib.rosSerialNode import start_serial_process
from std_msgs.msg import Float32
import asyncio
from app.TFManager import TFManagerInstance
async def async_main():
    # 启动 rosSerialNode 进程（非阻塞）
    serial_port = '/dev/ttyACM0'  # 与SICK数据板连接的串口
    baudrate = 115200  # 可以根据需要修改波特率
    start_serial_process(serial_port=serial_port, baudrate=baudrate)
    ros_bridge_module.RosBridgeNodeInstance.register_ros2_pub('/sick_data', Float32)
    # gcb.sick_serial_callback._publish = lambda distance: ros_bridge_module.RosBridgeNodeInstance.publish_ros2('/sick_data', distance)
    ros_bridge_module.RosBridgeNodeInstance.register_serial_sub(gcb.sick_serial_callback)
    TFManagerInstance.register_tf_chain()
    asyncio.create_task(TFManagerInstance.tf_update_loop())
    while True:
        #阻塞，无任务
        await asyncio.sleep(1)
        