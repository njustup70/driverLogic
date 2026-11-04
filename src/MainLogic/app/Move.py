"""底盘动作封装

1. 给 MPC 设置目标/路径；
2. 等待到点或超时；
3. 到点后清理路径。

串口 0xBB 由 MainLogic.core.mpc.mpc_loop 统一发送，
Move 不直接写速度，避免和 mpc_loop 抢串口。
"""

import asyncio
import time

import numpy as np

from MainLogic.Lib.odomVec import Odom
from MainLogic.core.mpc import MPCPathFollowerInstance, limit_linear_speed
from MainLogic.core.tf_manager import TFManagerInstance
from MainLogic.link.bytes import turn_to_bytes
from MainLogic.link.ros_bridge_node import RosBridgeNodeInstance

DEFAULT_REF_SPEED = 0.3
DEFAULT_MIN_DISTANCE = 0.05
DEFAULT_TIMEOUT_S = 20.0
POLL_INTERVAL_S = 0.05


def _current_pose() -> Odom:
    return TFManagerInstance.baseLinkOdom.value


def _distance_to(target) -> float:
    current = _current_pose()
    return float(np.hypot(current.x - target[0], current.y - target[1]))


async def _wait_until_reached(
    target,
    min_distance: float = DEFAULT_MIN_DISTANCE,
    timeout: float = DEFAULT_TIMEOUT_S,
) -> bool:
    """轮询 baseLinkOdom，直到进入目标半径或超时。"""
    target = np.asarray(target, dtype=float)
    start = time.monotonic()
    last_report = start

    while True:
        distance = _distance_to(target)
        if distance < min_distance:
            return True

        now = time.monotonic()
        if now - start >= timeout:
            print(
                f"[Move] 到点超时: target=({target[0]:.3f},{target[1]:.3f}) "
                f"distance={distance:.3f} timeout={timeout:.1f}s",
                flush=True,
            )
            return False

        if now - last_report >= 2.0:
            print(
                f"[Move] 前往目标: distance={distance:.3f}m "
                f"({now - start:.1f}s/{timeout:.1f}s)",
                flush=True,
            )
            last_report = now

        await asyncio.sleep(POLL_INTERVAL_S)


async def mpc_move_to_point(
    target,
    ref_speed: float = DEFAULT_REF_SPEED,
    min_distance: float = DEFAULT_MIN_DISTANCE,
    timeout: float = DEFAULT_TIMEOUT_S,
) -> bool:
    """MPC 移动到地图坐标 [x, y, yaw]。"""
    target = np.asarray(target, dtype=float)
    assert target.shape == (3,), "目标必须为 [x, y, yaw]"

    if _distance_to(target) < min_distance:
        return True

    current = _current_pose()
    path = np.asarray(
        [[current.x, current.y], [target[0], target[1]]],
        dtype=float,
    )
    MPCPathFollowerInstance.set_path(path, float(target[2]), ref_speed=ref_speed)

    try:
        return await _wait_until_reached(target, min_distance, timeout)
    finally:
        MPCPathFollowerInstance.clear_path()


async def mpc_move_relative(
    dx: float,
    dy: float,
    dyaw: float = 0.0,
    ref_speed: float = DEFAULT_REF_SPEED,
    min_distance: float = DEFAULT_MIN_DISTANCE,
    timeout: float = DEFAULT_TIMEOUT_S,
) -> bool:
    """按车体相对位移移动：dx 前、dy 左、dyaw 逆时针。"""
    current = _current_pose()
    target = current @ Odom(float(dx), float(dy), float(dyaw))
    return await mpc_move_to_point(
        [target.x, target.y, target.yaw],
        ref_speed=ref_speed,
        min_distance=min_distance,
        timeout=timeout,
    )


async def mpc_by_path_move(
    points,
    yaw: float,
    ref_speed: float = DEFAULT_REF_SPEED,
    min_distance: float = DEFAULT_MIN_DISTANCE,
    timeout: float = 30.0,
) -> bool:
    """沿地图坐标路径点 [[x1,y1], ...] 移动到最后一个点。"""
    points = np.asarray(points, dtype=float)
    assert points.ndim == 2 and points.shape[1] == 2, "路径点必须为 [[x,y], ...]"
    assert len(points) >= 2, "路径至少需要两个点"

    target = [float(points[-1, 0]), float(points[-1, 1]), float(yaw)]
    if _distance_to(target) < min_distance:
        return True

    MPCPathFollowerInstance.set_path(points, float(yaw), ref_speed=ref_speed)

    try:
        return await _wait_until_reached(target, min_distance, timeout)
    finally:
        MPCPathFollowerInstance.clear_path()


async def send_test_speed(vx: float, vy: float, wz: float, verbose: bool = False):
    """直接发一帧 0xBB 速度，仅用于台架/手动测试。

    正常 MPC 移动时串口应由 mpc_loop 统一管理。
    """
    ros_bridge = RosBridgeNodeInstance
    if ros_bridge is None:
        print("[Move] 错误: RosBridgeNode 未初始化")
        return

    vx, vy = limit_linear_speed(float(vx), float(vy))
    ros_bridge.writeBytes(b'\xBB' + turn_to_bytes([vx, vy, wz]))
    if verbose:
        print(
            f"[Move] 测试速度: vx={vx:.2f}, vy={vy:.2f}, wz={wz:.2f}",
            flush=True,
        )


async def stop():
    """停止 MPC 跟随；mpc_loop 会在下一周期发 0 速。"""
    MPCPathFollowerInstance.clear_path()
