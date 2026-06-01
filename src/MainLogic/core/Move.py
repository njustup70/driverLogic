'''
底盘控制耦合代码
'''
from MainLogic.Lib import odomVec
from MainLogic.core.tf_manager import TFManagerInstance
from MainLogic.core.nav.mpc import MPCPathFollowerInstance
from MainLogic.core.ros_bridge_node import RosBridgeNodeInstance
import numpy as np
import asyncio
_enable_mpc=False
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
    MPCPathFollowerInstance.set_path(ref_path, target[2], ref_speed=ref_speed)
    # MPCPathFollowerInstance.set_target_point(target)
    # 启动一个异步任务来监听 TF 更新，直到接近目标位置
    await _listen_tf(target,min_distance)

async def chassic_move_to(target,min_distance=0.05):
    """
    给底盘发位置指令，底盘实现位置控制
    :param target: 目标位置，格式为 [x, y, yaw]
    """ 
    target=np.asarray(target, dtype=float)
    assert target.shape ==(3,), "目标位置必须是一个包含 x, y, yaw 的数组"
    TFManagerInstance.rosBridge.writeBytes(b'\xA1' + turn_to_bytes(target.tolist()))
    await _listen_tf(target,min_distance)
async def mpc_by_path_move(points,yaw,ref_speed=1.5,mindistance=0.05):
    """
    给底盘发位置指令，底盘实现路径
    :param points: 路径点列表，格式为 [[x1, y1], [x2, y2], ...]
    :param yaw: 目标航向角，单位为弧度
    """
    assert _enable_mpc, "MPC控制未启用，请设置mpc_control_loop任务"


    # 将路径传给 MPC 跟随器（会生成样条并设置终点）
    MPCPathFollowerInstance.set_path(points, yaw, ref_speed=ref_speed)
    # 等待机器人到达 MPC 设置的终点
    await _listen_tf(MPCPathFollowerInstance.end_point, mindistance)
    

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
    MPCPathFollowerInstance.set_target_point(np.array(target_odom_map))
    # 启动一个异步任务来监听 TF 更新，直到接近目标位置
    await _listen_tf(target_odom_map,min_distance)

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
       
from geometry_msgs.msg import Vector3

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
            # u = MPCPathFollowerInstance.update(x)
            u=await MPCPathFollowerInstance.async_update(x)  #如果需要异步版本，改成 await MPCPathFollowerInstance.async_update(x)
            # print(f"MPC output control: {u}")
            
            ros_bridge = RosBridgeNodeInstance
            if ros_bridge is not None:
                state_msg = Vector3(x=float(u[0]), y=float(u[1]), z=float(u[2]))
                # 同步发送到下位机串口
                ros_bridge.writeBytes(serial_cmd_prefix + turn_to_bytes([float(u[0]), float(u[1]), float(u[2])]))
        else:
            print("Waiting for odometry data...")
        await asyncio.sleep(0.01)