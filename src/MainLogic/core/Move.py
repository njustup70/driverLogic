'''
底盘控制耦合代码
'''
from MainLogic.Lib import odomVec
from MainLogic.core.tf_manager import TFManagerInstance
from MainLogic.core.ros_bridge_node import RosBridgeNodeInstance
import numpy as np
import asyncio
_enable_mpc=False


# 🔴不可用🔴
async def mpc_move_to_point(target,ref_speed=1.5,min_distance=0.05):
    '''
    通过 MPC 控制底盘移动到目标位置
    :param target: 目标位置，格式为 [x, y, yaw]
    '''
    # 这里我们假设已经有一个 MPC 控制器实例，命名为 mpc_controller
    assert _enable_mpc, "MPC控制未启用，请设置mpc_control_loop任务"
    target=np.asarray(target, dtype=float)
    assert target.shape ==(3,), "目标位置必须是一个包含 x, y, yaw 的数组"
    #将目标位置和当前位置生成一条路径，传给 MPC 跟随器
    current_odom = TFManagerInstance.baseLinkOdom.value
    ref_path=np.asarray([[current_odom.x, current_odom.y], [target[0], target[1]]])
    # MPCPathFollowerInstance.set_target_point(target)
    # 启动一个异步任务来监听 TF 更新，直到接近目标位置
    await _listen_tf(target,min_distance)

# 🔴不可用🔴
async def mpc_by_path_move(points,yaw,ref_speed=1.5,mindistance=0.05):
    """
    给底盘发位置指令，底盘实现路径
    :param points: 路径点列表，格式为 [[x1, y1], [x2, y2], ...]
    :param yaw: 目标航向角，单位为弧度
    """
    assert _enable_mpc, "MPC控制未启用，请设置mpc_control_loop任务"

# 🔴不可用🔴
async def mpc_move_to_baseodom(target,min_distance=0.05):
    '''
    发布基于局部坐标系的 MPC 目标点，底盘实现位置控制
    '''
    assert _enable_mpc, "MPC控制未启用，请设置mpc_control_loop任务"
    # 将目标点转换到 base_link 坐标系下
    target=np.asarray(target, dtype=float)
    current_odom =TFManagerInstance.baseLinkOdom.value
    target_odombase=odomVec.Odom().from_array(target)
    target_odom_map=target_odombase@current_odom
    # 启动一个异步任务来监听 TF 更新，直到接近目标位置
    await _listen_tf(target_odom_map,min_distance)

# 🔴不可用🔴
from MainLogic.Lib.bytes import turn_to_bytes
async def mpc_control_loop():
    '''
    固定100hz控制频率而不是每次TF更新都发控制指令，避免过度控制
    '''
    serial_cmd_prefix = b'\xBB'
    global _enable_mpc
    _enable_mpc=True
    while True:                   
        current_odom = TFManagerInstance.baseLinkOdom.value
        if current_odom is not None:
            x = np.asarray(current_odom).reshape((3, 1))  # 确保 x 的形状是 (3, 1)
        else:
            print("Waiting for odometry data...")
        await asyncio.sleep(0.01)

# 以下可增加新功能

async def _listen_tf(target,min_distance=0.02):
    '''
    监听 TF 更新，直到机器人接近目标位置
    :param target: 目标位置，格式为 [x, y, yaw]
    :param min_distance: 接近目标的距离阈值，单位为米
    '''
    while True:
        target=np.asarray(target)
        current_odom = await TFManagerInstance.baseLinkOdom
        current_odom=np.array(current_odom)  # 转换为 numpy 数组
        if current_odom is not None:
            current_pos = np.array(current_odom[:2])  # 只考虑 x, y
            target_pos = np.array(target[:2])
            distance = np.linalg.norm(current_pos - target_pos)
            if distance < min_distance:
                # print(f"已接近目标位置，距离: {distance:.3f} 米")
                break

async def chassic_move_to(target,min_distance=0.05):
    """
    给底盘发位置指令，底盘实现位置控制
    :param target: 目标位置，格式为 [x, y, yaw]
    """ 
    target=np.asarray(target, dtype=float)
    assert target.shape ==(3,), "目标位置必须是一个包含 x, y, yaw 的数组"
    TFManagerInstance.rosBridge.writeBytes(b'\xA1' + turn_to_bytes(target.tolist()))
    await _listen_tf(target,min_distance)

async def send_test_speed(vx: float, vy: float, wz: float):
    """
    仅发送固定速度的测试指令
    :param vx: X轴线速度 (m/s)
    :param vy: Y轴线速度 (m/s)
    :param wz: Z轴角速度 (rad/s)
    """
    serial_cmd_prefix = b'\xBB'
    # 使用 turn_to_bytes 将三个 float 转为 12 字节小端浮点数
    speed_bytes = turn_to_bytes([vx, vy, wz])
    
    ros_bridge = RosBridgeNodeInstance
    if ros_bridge is not None:
        # writeBytes 内部会自动加上 0xFA 帧头
        ros_bridge.writeBytes(serial_cmd_prefix + speed_bytes)
        print(f"[Move.py] 发送测试速度: vx={vx:.2f}, vy={vy:.2f}, wz={wz:.2f}")
    else:
        print("[Move.py] 错误: RosBridgeNode 未初始化")