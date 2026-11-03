'''
底盘阶段 0：只确认数据链，不发控制。

启动内容：
1. 串口进程；
2. 0xAA odom、0xB3 SICK 回调；
3. TFManager 链 + tf_update_loop。
'''

import asyncio
import os

from MainLogic.Lib.odomVec import Odom
from MainLogic.core.tf_manager import TFManagerInstance
from MainLogic.link import globalCallback as gcb
from MainLogic.link import ros_bridge_node as ros_bridge_module
from MainLogic.link.serial_node import start_serial_process

SERIAL_PORT = os.getenv("SERIAL_PORT", "/dev/serial_ch340")
SERIAL_BAUDRATE = int(os.getenv("SERIAL_BAUDRATE", "921600"))

# 旧车外参占位
SICK2BASE = Odom(0.0, -0.3511, 0.0)
MAP2BASEINIT = Odom(0.0, 0.0, 0.0)
BASE2LASER = Odom(0.10, 0.336, 0.0)
SICK_CORRECT_WIDTH = 0.0

STAGE0_PERIOD_S = 2.0 # 日志间隔
gcb._ODOM_LOG_INTERVAL = STAGE0_PERIOD_S
gcb._SICK_LOG_INTERVAL = STAGE0_PERIOD_S


def _fmt_odom(value) -> str:
    if value is None:
        return "None"
    return f"({value.x:+.3f},{value.y:+.3f},{value.yaw:+.3f})"


def _fmt_sick(value) -> str:
    return "None" if value is None else f"{value:+.3f}"


async def async_main():
    start_serial_process(serial_port=SERIAL_PORT, baudrate=SERIAL_BAUDRATE)

    bridge = ros_bridge_module.RosBridgeNodeInstance
    assert bridge is not None, "RosBridgeNodeInstance 未初始化"

    bridge.register_serial_sub(gcb.mcu_transmit_callback, 0xAA)
    bridge.register_serial_sub(gcb.sick_callback, 0xB3)
    # 阶段 0 先不注册 0xB2：serial_correct_callback 还会走 TFOdin，先避开。

    TFManagerInstance.register_tf_chain(
        SICK2BASE,
        MAP2BASEINIT,
        BASE2LASER,
        SICK_CORRECT_WIDTH,
    )
    asyncio.create_task(TFManagerInstance.tf_update_loop())

    print(
        f"[stage0] 数据链已启动：只读 odom/SICK/SLAM，不发速度，每 {STAGE0_PERIOD_S:.0f}s 打印一次",
        flush=True,
    )

    while True:
        odom = TFManagerInstance._odomToBase
        base = TFManagerInstance.baseLinkOdom.value
        has_slam = TFManagerInstance._has_slam_pose
        slam_base = getattr(TFManagerInstance, "_slamBaseOdom", None)
        sick_l = (
            TFManagerInstance.left_sick_buffer[-1]
            if TFManagerInstance.left_sick_buffer
            else None
        )
        sick_r = (
            TFManagerInstance.right_sick_buffer[-1]
            if TFManagerInstance.right_sick_buffer
            else None
        )

        print("=" * 72, flush=True)
        print(f"[STAGE0] odom={_fmt_odom(odom)}", flush=True)
        print(f"         base={_fmt_odom(base)}", flush=True)
        print(f"         slam={has_slam} slam_base={_fmt_odom(slam_base)}", flush=True)
        print(
            f"         sickL={_fmt_sick(sick_l)} sickR={_fmt_sick(sick_r)}",
            flush=True,
        )
        await asyncio.sleep(STAGE0_PERIOD_S)
