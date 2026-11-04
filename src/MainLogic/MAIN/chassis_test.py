"""底盘分阶段测试入口。

用 CHASSIS_TEST_MODE 选择测试内容：

  stage0     纯数据链：串口/odom/SICK/SLAM，禁用速度发送
  mpc_calc   纯 MPC 计算：假位姿，无串口、TF
  mpc_live   真实位姿 + mpc_loop(send_to_mcu=False)：只算不发
  mpc_drive  真实位姿 + mpc_loop(send_to_mcu=True)：实车短距离移动

示例：
  CHASSIS_TEST_MODE=mpc_calc python3 -m MainLogic.Main \
      --main-module chassis_test --main-func async_main
"""

import asyncio
import os
import time

from MainLogic.Lib.odomVec import Odom
from MainLogic.core.tf_manager import TFManagerInstance
from MainLogic.link import globalCallback as gcb
from MainLogic.link import ros_bridge_node as ros_bridge_module
from MainLogic.link.serial_node import start_serial_process

MODE = os.getenv("CHASSIS_TEST_MODE", "stage0").strip().lower()

SERIAL_PORT = os.getenv("SERIAL_PORT", "/dev/serial_ch340")
SERIAL_BAUDRATE = int(os.getenv("SERIAL_BAUDRATE", "921600"))

# 旧车外参占位
SICK2BASE = Odom(0.0, -0.3511, 0.0)
MAP2BASEINIT = Odom(0.0, 0.0, 0.0)
BASE2LASER = Odom(0.10, 0.336, 0.0)
SICK_CORRECT_WIDTH = 0.0

STAGE0_PERIOD_S = 2.0  # 日志间隔
gcb._ODOM_LOG_INTERVAL = STAGE0_PERIOD_S
gcb._SICK_LOG_INTERVAL = STAGE0_PERIOD_S


def _fmt_odom(value) -> str:
    if value is None:
        return "None"
    return f"({value.x:+.3f},{value.y:+.3f},{value.yaw:+.3f})"


def _fmt_sick(value) -> str:
    return "None" if value is None else f"{value:+.3f}"


def _start_data_chain():
    """启动串口、回调、TFManager；只用于 stage0/mpc_live/mpc_drive。"""
    start_serial_process(serial_port=SERIAL_PORT, baudrate=SERIAL_BAUDRATE)

    bridge = ros_bridge_module.RosBridgeNodeInstance
    assert bridge is not None, "RosBridgeNodeInstance 未初始化"

    bridge.register_serial_sub(gcb.mcu_transmit_callback, 0xAA)
    bridge.register_serial_sub(gcb.sick_callback, 0xB3)

    TFManagerInstance.register_tf_chain(
        SICK2BASE,
        MAP2BASEINIT,
        BASE2LASER,
        SICK_CORRECT_WIDTH,
    )
    asyncio.create_task(TFManagerInstance.tf_update_loop())
    return bridge


async def _wait_for_pose(timeout: float = 10.0) -> bool:
    try:
        await asyncio.wait_for(TFManagerInstance.baseLinkOdom, timeout=timeout)
        return True
    except asyncio.TimeoutError:
        print(f"[chassis_test] 等待 baseLinkOdom 超时（{timeout:.0f}s），先检查串口/odom", flush=True)
        return False


async def run_stage0():
    _start_data_chain()

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


async def run_mpc_calc():
    """假位姿下验证 MPC 求解，无串口、TF、底盘、雷达。"""
    import numpy as np
    from MainLogic.core.mpc import MPCPathFollowerInstance

    mpc = MPCPathFollowerInstance
    mpc.path_visualizer = None
    path = np.asarray([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]], dtype=float)
    mpc.set_path(path, 0.0, ref_speed=0.3)

    print("[MPC-CALC] fake path start -> (2,0), ref_speed=0.3", flush=True)
    for i in range(11):
        x = 0.2 * i
        u = mpc.update(np.asarray([[x], [0.0], [0.0]], dtype=float))
        print(
            f"[MPC-CALC] x={x:.2f} u=({u[0]:+.3f},{u[1]:+.3f},{u[2]:+.3f})",
            flush=True,
        )
        await asyncio.sleep(0.1)

    mpc.clear_path()
    print("[MPC-CALC] done", flush=True)
    while True:
        await asyncio.sleep(1.0)


async def run_mpc_live():
    """真实位姿 + mpc_loop(False)：只发布控制量，不发串口。"""
    import numpy as np
    from geometry_msgs.msg import Vector3
    from MainLogic.core.mpc import MPCPathFollowerInstance, mpc_loop

    bridge = _start_data_chain()
    if not await _wait_for_pose():
        return

    last_print = [0.0]

    def on_mpc(msg: Vector3):
        now = time.monotonic()
        if now - last_print[0] < 0.5:
            return
        last_print[0] = now
        print(
            f"[MPC-LIVE] vx={msg.x:+.3f} vy={msg.y:+.3f} wz={msg.z:+.3f}",
            flush=True,
        )

    bridge.register_ros2_sub("/state/mpc_control", on_mpc, Vector3)
    asyncio.create_task(mpc_loop(send_to_mcu=False))

    current = TFManagerInstance.baseLinkOdom.value
    target = current @ Odom(0.5, 0.0, 0.0)
    MPCPathFollowerInstance.set_path(
        np.asarray([[current.x, current.y], [target.x, target.y]]),
        target.yaw,
        ref_speed=0.2,
    )
    print(
        f"[MPC-LIVE] target=({target.x:.3f},{target.y:.3f},{target.yaw:.3f}) "
        "send_to_mcu=False",
        flush=True,
    )

    while True:
        cur = TFManagerInstance.baseLinkOdom.value
        print(
            f"[MPC-LIVE] base={_fmt_odom(cur)} "
            f"following={MPCPathFollowerInstance.is_following_path}",
            flush=True,
        )
        await asyncio.sleep(2.0)


async def run_mpc_drive():
    """真实位姿 + mpc_loop(True)：实车短距离移动，只用于确认能走。"""
    from MainLogic.app import Move
    from MainLogic.core.mpc import mpc_loop

    _start_data_chain()
    if not await _wait_for_pose():
        return

    print("[MPC-DRIVE] 实车模式：0.5m 直行，速度 0.2m/s", flush=True)
    task = asyncio.create_task(mpc_loop(send_to_mcu=True))
    try:
        await asyncio.sleep(0.5)
        ok = await Move.mpc_move_relative(
            0.5,
            0.5,
            0.0,
            ref_speed=0.2,
            timeout=15.0,
        )
        print(f"[MPC-DRIVE] result={ok}", flush=True)
    finally:
        # 不管正常结束、超时还是异常，都清路径并补一帧 0 速。
        await Move.stop()
        await asyncio.sleep(0.2)
        await Move.send_test_speed(0.0, 0.0, 0.0)
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)
        print("[MPC-DRIVE] final stop sent", flush=True)

    await asyncio.sleep(2.0)

    while True:
        await asyncio.sleep(2.0)


async def async_main():
    if MODE == "stage0":
        await run_stage0()
    elif MODE == "mpc_calc":
        await run_mpc_calc()
    elif MODE == "mpc_live":
        await run_mpc_live()
    elif MODE == "mpc_drive":
        await run_mpc_drive()
    else:
        raise ValueError(
            f"未知 CHASSIS_TEST_MODE={MODE!r}；可选：stage0 / mpc_calc / mpc_live / mpc_drive"
        )
