import warnings
# 屏蔽所有 UserWarning 警告
warnings.filterwarnings("ignore", category=UserWarning)
import do_mpc
import casadi
from casadi import vertcat, cos, sin
import numpy as np
from MainLogic.Lib.decorder import time_print
from MainLogic.Lib.bytes import turn_to_bytes
import asyncio
from MainLogic.core.linear import SplinePlanner
from MainLogic.core import ros_bridge_node as ros_bridge_module
import MainLogic.Lib.foxgloveTools as foxgloveTools
from geometry_msgs.msg import Twist, Vector3Stamped
class MPCPathFollower:
    def __init__(self, dt,type='swerve'):
        assert type in ['swerve', 'omni'], "type must be 'swerve' or 'omni'"

        # --- 模型配置 ---
        self.drive_type = type
        self.dt = float(dt)
        self.n_horizon = 20
        model_type = 'continuous' if self.drive_type == 'swerve' else 'discrete'

        # --- 路径与运行状态 ---
        self.path_planner = SplinePlanner()
        self.is_following_path = False
        self.ref_speed = 1.0
        self.end_point = np.array([0.0, 0.0, 0.0])
        self.s = 0.0
        self._step_count = 0
        # do-mpc 默认会持续记录运行历史，长时间运行会让 append/内存开销增长。
        self._history_reset_interval = 200
        self.enable_foxglove_stream = True

        self.model = do_mpc.model.Model(model_type)

        # State: [x, y, theta]
        self.state = self.model.set_variable(var_type='_x', var_name='state', shape=(3, 1))
        # Input:
        # - swerve: [v, alpha, vw]
        # - omni:   [vx_body, vy_body, vw]
        self.u_vec = self.model.set_variable(var_type='_u', var_name='u_input', shape=(3, 1))
        ref = self.model.set_variable(var_type='_tvp', var_name='ref', shape=(3, 1))

        # --- 按底盘类型定义动力学 ---
        theta = self.state[2]
        if self.drive_type == 'swerve':
            v = self.u_vec[0]
            alpha = self.u_vec[1]
            vw = self.u_vec[2]

            # 连续系统: x_dot = f(x,u)
            x_dot = v * cos(theta + alpha)
            y_dot = v * sin(theta + alpha)
            theta_dot = vw
            rhs_state = vertcat(x_dot, y_dot, theta_dot)

            rterm = np.array([2.0, 5.0, 10.0])
            lower_u = np.array([[-3.0], [-np.pi], [-2.0]])
            upper_u = np.array([[3.0], [np.pi], [2.0]])
            set_up_settings = {
                'n_horizon': self.n_horizon,
                't_step': self.dt,
                'store_full_solution': False,
                'state_discretization': 'collocation',
                'collocation_deg': 2,
                'nlpsol_opts': {
                    'ipopt.print_level': 0,
                    'print_time': False,
                    'ipopt.max_iter': 30,
                    'ipopt.tol': 1e-3,
                    'ipopt.linear_solver': 'mumps',
                    'ipopt.mu_strategy': 'adaptive',
                },
            }
        else:
            vx_body = self.u_vec[0]
            vy_body = self.u_vec[1]
            vw = self.u_vec[2]

            # 离散系统: x(k+1) = f(x(k),u(k))
            vx_world = cos(theta) * vx_body - sin(theta) * vy_body
            vy_world = sin(theta) * vx_body + cos(theta) * vy_body
            rhs_state = vertcat(
                self.state[0] + self.dt * vx_world,
                self.state[1] + self.dt * vy_world,
                self.state[2] + self.dt * vw,
            )

            rterm = np.array([50.0, 50.0, 50.0])
            lower_u = np.array([[-0.5], [-0.5], [-1.0]])
            upper_u = np.array([[0.5], [0.5], [1.0]])
            set_up_settings = {
                'n_horizon': self.n_horizon,
                't_step': self.dt,
                'store_full_solution': False,
                'nlpsol_opts': {
                    'ipopt.print_level': 0,
                    'print_time': False,
                    'ipopt.max_iter': 30,
                    'ipopt.tol': 1e-3,
                    'ipopt.linear_solver': 'mumps',
                    'ipopt.mu_strategy': 'adaptive',
                },
            }

        assert isinstance(rhs_state, casadi.SX)
        self.model.set_rhs('state', rhs_state)
        self.model.setup()

        pos_err = casadi.sumsqr(self.state[0:2] - ref[0:2])
        # Use cosine yaw cost to avoid angle wrap jump at +-pi.
        # yaw_err = 1.0 - casadi.cos(self.state[2] - ref[2])
        angle_diff = self.state[2] - ref[2]
        wrapped_angle_diff = casadi.atan2(casadi.sin(angle_diff), casadi.cos(angle_diff))

        
        # 直接使用平方误差
        yaw_err = casadi.sumsqr(wrapped_angle_diff)
        #结束代价，符号函数
        mterm = 8.0 * pos_err + 1.0 * yaw_err 
        #过程代价
        lterm = 8.0 * pos_err + 1.0 * yaw_err
       
        self.mpc = do_mpc.controller.MPC(self.model)
        self.mpc.set_rterm(u_input=rterm)
        self.mpc.set_objective(mterm=mterm, lterm=lterm)
        self.mpc.set_param(**set_up_settings)
        self.mpc.bounds['lower', '_u', 'u_input'] = lower_u
        self.mpc.bounds['upper', '_u', 'u_input'] = upper_u

        # 参数注册
        tvp_template=self.mpc.get_tvp_template()
        assert tvp_template is not None
        self.tvp_template = tvp_template
        def tvp_fun(_t_now: float):
            return self.tvp_template

        self.mpc.set_tvp_fun(tvp_fun)
        self.mpc.setup()
        from concurrent.futures import ThreadPoolExecutor
        self.pool= ThreadPoolExecutor(max_workers=1)

    def get_predicted_trajectory(self):
        """
        获取当前最优解的预测轨迹
        返回形状为 (n_horizon+1, 3) 的数组: [x, y, theta]
        """
        # 注意：只有在 make_step 执行后，mpc.opt_x_num 才会更新
        # 我们从计算好的优化变量中提取状态分量 (_x)
        # do-mpc 的 opt_x_num 包含了所有预测步的状态和输入
        try:
            # 提取预测的状态序列
            # x_num 的形状通常是 (n_horizon + 1, n_states)
            pred_x = self.mpc.opt_x_num['_x']
            return np.array(pred_x) 
        except Exception as e:
            return None

    def set_path(self,target_points: np.ndarray,target_yaw: float, ref_speed=None):
        self.is_following_path = True
        #传入的target_points是一个二维数组，形状为 (N, 2)，每行是一个路径点的 (x, y) 坐标
        x_pts = target_points[:, 0]
        y_pts = target_points[:, 1]
        self.path_planner.generate_path(x_pts, y_pts, step_cm=10.0)
        self.end_point=np.array([float(x_pts[-1]), float(y_pts[-1]), float(target_yaw)])
        if ref_speed is not None:
            self.ref_speed = float(ref_speed)
    def _update_prediction_reference(self, x: np.ndarray):
        if self.is_following_path==False:
            return
        # 根据当前位置找到路径上最近点，得到对应的虚拟弧长 s
        self.s = self.path_planner.get_nearest_s(float(x[0, 0]), float(x[1, 0]))
        # 在预测域内，s 按参考速度逐步向前推进，每步推进 ref_speed * dt
        for k in range(self.n_horizon + 1):
            s_k = self.s + self.ref_speed * self.dt * k
            # 插值得到第 k 步的参考点 (x_ref, y_ref, yaw_ref)；超出路径终点后自动钳位
            ref_k= self.path_planner.get_state_by_s(s_k)
            #构造 MPC 需要的参考值格式，并更新到 tvp_template 中
            ref_k[2]=self.end_point[2]  #保持角度参考不变，直接使用终点的角度参考
            #只更改位置参考，保持角度参考不变
            self.tvp_template['_tvp', k, 'ref'] =ref_k 
            # print(ref_k)
    def set_state_init(self, x0):
        '''设置 MPC 的初始状态'''
        self.mpc.x0 = x0
        self.mpc.set_initial_guess()
    def update(self,x):
        assert isinstance(x, np.ndarray) and x.shape == (3, 1)
        self._step_count += 1
        if self._step_count % self._history_reset_interval == 0:
            self.mpc.reset_history()

        self._update_prediction_reference(x)
        u = self.mpc.make_step(x)
        
        # --- 新增：获取预测轨迹并发送到 Foxglove ---
        if self.enable_foxglove_stream:
            pred_traj = self.get_predicted_trajectory()
            if pred_traj is not None:
                # 发送控制量
                foxgloveTools.foxgloveViusalInstance.send(u.flatten(), topic="/mpc/control_input")
                # 发送预测轨迹 (通常需要转换成 Foxglove 支持的 LineStrip 或 Points 格式)
                # 这里假设你的 foxgloveTools 支持发送 numpy 数组或你自定义了转换
                foxgloveTools.foxgloveViusalInstance.send(pred_traj, topic="/mpc/predict_traj")

        # swerve 输出 [v, alpha, vw]，对外统一成 [vx_body, vy_body, vw]
        if self.drive_type == 'swerve':
            vx_body = u[0] * cos(u[1])
            vy_body = u[0] * sin(u[1])
            u[0] = vx_body
            u[1] = vy_body

        # print(self.mpc.data)
        return u.flatten()
    def set_target_point(self, target):
        '''设置 MPC 的目标点'''
        #将target转换成 numpy 数组，并确保它的形状正确
        target = np.asarray(target, dtype=float)
        assert len(target) == 3
        self.is_following_path= False
        self.end_point=target
        for k in range(self.n_horizon + 1):
            self.tvp_template['_tvp', k, 'ref'] = target
    async def async_update(self,x):
        '''异步版本的 update 方法'''
        loop = asyncio.get_event_loop()
        u= await loop.run_in_executor(self.pool, self.update, x)
        return u
MPCPathFollowerInstance = MPCPathFollower(dt=0.1, type='omni')
STATE_MPC_CONTROL_TOPIC = '/state/mpc_control'


async def mpc_loop():
    from MainLogic.core.tf_manager import TFManagerInstance
    # 控制帧类型: A2 + [vx, vy, vw] float32
    serial_cmd_prefix = b'\xBB'
    state_pub_registered = False
    while True:                   
        current_odom = TFManagerInstance.baseLinkOdom.value
        if current_odom is not None:
            x = np.asarray(current_odom).reshape((3, 1))  # 确保 x 的形状是 (3, 1)
            # u = MPCPathFollowerInstance.update(x)
            u=await MPCPathFollowerInstance.async_update(x)  #如果需要异步版本，改成 await MPCPathFollowerInstance.async_update(x)
            # print(f"MPC output control: {u}")

            ros_bridge = ros_bridge_module.RosBridgeNodeInstance
            if ros_bridge is not None:
                if not state_pub_registered:
                    ros_bridge.register_ros2_pub(STATE_MPC_CONTROL_TOPIC, Vector3Stamped)
                    state_pub_registered = True

                state_msg = Vector3Stamped()
                state_msg.header.stamp = ros_bridge.get_clock().now().to_msg()
                state_msg.header.frame_id = 'base_link'
                state_msg.vector.x = float(u[0])
                state_msg.vector.y = float(u[1])
                state_msg.vector.z = float(u[2])
                ros_bridge.publish_ros2(STATE_MPC_CONTROL_TOPIC, state_msg)

                # 发布 ROS2 cmd_vel
                cmd_msg = Twist()
                cmd_msg.linear.x = float(u[0])
                cmd_msg.linear.y = float(u[1])
                cmd_msg.angular.z = float(u[2])
                 

                # 同步发送到下位机串口
                ros_bridge.writeBytes(serial_cmd_prefix + turn_to_bytes([float(u[0]), float(u[1]), float(u[2])]))
        else:
            print("Waiting for odometry data...")
        await asyncio.sleep(0.01)