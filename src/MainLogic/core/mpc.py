"""MPC 接口/适配层。
解耦 MPC 实现，保留：
1. 将 MPCPathFollower 暴露成 MainLogic 稳定接口；
2. 注入 MainLogic 的路径可视化；
3. 提供依赖 ROS/TF/串口的 mpc_loop。

替换 mpc_lib 时，保证库仍导出 MPCPathFollower / MPC_DT / MPC_TYPE，
"""

import asyncio

import numpy as np
from geometry_msgs.msg import Twist, Vector3

from mpc_lib import MPCPathFollower as _MPCPathFollower
from mpc_lib import MPC_DT, MPC_TYPE, SplinePlanner
from MainLogic.link import ros_bridge_node as ros_bridge_module
from MainLogic.link.bytes import turn_to_bytes

STATE_MPC_CONTROL_TOPIC = '/state/mpc_control'
CMD_VEL_TOPIC = '/cmd_vel'


class MPCPathFollower(_MPCPathFollower):
    """MainLogic 侧的 MPC 适配器。"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.path_visualizer = self._publish_target_path

    @staticmethod
    def _publish_target_path(path_points) -> None:
        try:
            from MainLogic.tools.Visual import PathVisualInstance
            PathVisualInstance.publish_points("/state/target_path", path_points)
        except Exception as exc:
            print(f"[MPC] path visual skipped: {exc}")


MPCPathFollowerInstance = MPCPathFollower(dt=MPC_DT, type=MPC_TYPE)


async def mpc_loop(send_to_mcu: bool = False):
    """MPC 跟随循环。

    send_to_mcu=False 时只发布 /state/mpc_control 和 /cmd_vel，不写串口；
    实车阶段再传 True，才会发 0xBB 速度帧。
    """
    from MainLogic.core.tf_manager import TFManagerInstance
    serial_cmd_prefix = b'\xBB'
    zero_frame = serial_cmd_prefix + turn_to_bytes([0.0, 0.0, 0.0])
    while True:
        ros_bridge = ros_bridge_module.RosBridgeNodeInstance
        if ros_bridge is None:
            await asyncio.sleep(MPCPathFollowerInstance.dt)
            continue

        if not MPCPathFollowerInstance.is_following_path:
            if send_to_mcu:
                ros_bridge.writeBytes(zero_frame)
            await asyncio.sleep(MPCPathFollowerInstance.dt)
            continue

        current_odom = TFManagerInstance.baseLinkOdom.value
        try:
            x = np.asarray(current_odom).reshape((3, 1))
            u = await MPCPathFollowerInstance.async_update(x)
        except Exception as exc:
            print(f"[MPC] solve failed: {exc!r}; stop")
            if send_to_mcu:
                ros_bridge.writeBytes(zero_frame)
            MPCPathFollowerInstance.clear_path()
            await asyncio.sleep(MPCPathFollowerInstance.dt)
            continue

        state_msg = Vector3(x=float(u[0]), y=float(u[1]), z=float(u[2]))
        ros_bridge.publish_ros2(STATE_MPC_CONTROL_TOPIC, state_msg)

        cmd_msg = Twist()
        cmd_msg.linear.x = float(u[0])
        cmd_msg.linear.y = float(u[1])
        cmd_msg.angular.z = float(u[2])
        ros_bridge.publish_ros2(CMD_VEL_TOPIC, cmd_msg)

        if send_to_mcu:
            ros_bridge.writeBytes(
                serial_cmd_prefix
                + turn_to_bytes([float(u[0]), float(u[1]), float(u[2])])
            )
        await asyncio.sleep(MPCPathFollowerInstance.dt)
