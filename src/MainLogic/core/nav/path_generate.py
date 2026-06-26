import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
from typing import Optional, Tuple, List

class SplinePlanner:
    def __init__(self):
        self.x_path = np.array([])
        self.y_path = np.array([])
        self.yaw_path = np.array([])
        self.yaw_path_unwrapped = np.array([])
        self.t_samples = np.array([])
        # 真实弧长数组，对应每个采样点沿曲线的累计弧长
        self.s_samples = np.array([])

    def find_nearest_point(self, x: float, y: float) -> Tuple[int, np.ndarray,float]:
        """Find the nearest sampled path point to the query position.

        Returns:
            (index, [x,y,yaw], distance)
        """
        if len(self.x_path) == 0:
            raise ValueError("Path is empty. Call generate_path() first.")

        dx = self.x_path - x
        dy = self.y_path - y
        dist_sq = dx * dx + dy * dy

        idx = int(np.argmin(dist_sq))
        distance = float(np.sqrt(dist_sq[idx]))

        return (
            idx,
            np.array([float(self.x_path[idx]), float(self.y_path[idx]), float(self.yaw_path[idx])]),
            distance
        )   
    def generate_path(
        self, 
        x_pts, 
        y_pts, 
        start_yaw: Optional[float] = None, 
        end_yaw: Optional[float] = None, 
        step_cm: float = 5.0  # 新增参数：固定间距（单位：厘米）
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        
        x_pts = np.array(x_pts)
        y_pts = np.array(y_pts)
        
        # 1. 计算参数 t (累计弦长)
        ds = np.sqrt(np.diff(x_pts)**2 + np.diff(y_pts)**2)
        t_pts = np.concatenate(([0], np.cumsum(ds)))
        
        # 2. 确定边界条件 (保持原有逻辑)
        bc_x, bc_y = 'not-a-knot', 'not-a-knot'
        v_scale = ds[0] if len(ds) > 0 else 1.0 

        if start_yaw is not None and end_yaw is not None:
            v_start = (v_scale * np.cos(start_yaw), v_scale * np.sin(start_yaw))
            v_end = (v_scale * np.cos(end_yaw), v_scale * np.sin(end_yaw))
            bc_x = ((1, v_start[0]), (1, v_end[0]))
            bc_y = ((1, v_start[1]), (1, v_end[1]))
        elif start_yaw is not None:
            v_start = (v_scale * np.cos(start_yaw), v_scale * np.sin(start_yaw))
            bc_x = ((1, v_start[0]), (2, 0.0))
            bc_y = ((1, v_start[1]), (2, 0.0))
        
        # 3. 拟合样条
        cs_x = CubicSpline(t_pts, x_pts, bc_type=bc_x) # type: ignore
        cs_y = CubicSpline(t_pts, y_pts, bc_type=bc_y) # type: ignore

        # --- 核心修改部分：固定间距采样 ---
        total_length = t_pts[-1]           # 路径总长度（米）
        step_m = step_cm / 100.0           # 将厘米转换为米
        
        # 计算采样点数，确保覆盖终点
        num_samples = int(np.floor(total_length / step_m)) + 1
        # 使用 linspace 保证从 0 正好到终点，且间距几乎恒定为 step_m
        self.t_samples = np.linspace(0, total_length, num_samples)
        
        # 4. 插值采样 
        self.x_path = cs_x(self.t_samples)
        self.y_path = cs_y(self.t_samples)
        
        # 5. 计算 Yaw 角
        dx = cs_x(self.t_samples, 1)        
        dy = cs_y(self.t_samples, 1)
        self.yaw_path = np.arctan2(dy, dx)
        self.yaw_path_unwrapped = np.unwrap(self.yaw_path)

        # 6. 计算各采样点的真实弧长（相邻采样点间欧氏距离累积）
        ds_path = np.sqrt(np.diff(self.x_path)**2 + np.diff(self.y_path)**2)
        self.s_samples = np.concatenate(([0.0], np.cumsum(ds_path)))

        return self.x_path, self.y_path, self.yaw_path

    def get_total_length(self) -> float:
        '''返回路径的真实总弧长（单位：米）'''
        if len(self.s_samples) == 0:
            raise ValueError("Path is empty. Call generate_path() first.")
        return float(self.s_samples[-1])

    def get_nearest_s(self, x: float, y: float) -> float:
        '''给定世界坐标 (x, y)，返回路径上距离最近点对应的弧长 s'''
        if len(self.s_samples) == 0:
            raise ValueError("Path is empty. Call generate_path() first.")
        # 先找最近采样点的索引，再查该点的弧长
        idx, _, _ = self.find_nearest_point(x, y)
        return float(self.s_samples[idx])

    def get_state_by_s(self, s_query: float) -> np.ndarray:
        '''给定弧长 s_query，插值返回对应路径点的 (x, y, yaw)。
        s <= 0 时钳位到起点，s >= 总长时钳位到终点（x/y 不动）。
        '''
        if len(self.s_samples) == 0:
            raise ValueError("Path is empty. Call generate_path() first.")

        # 超出起点：直接返回路径起点
        if s_query <= 0.0:
            return np.array([float(self.x_path[0]), float(self.y_path[0]), float(self.yaw_path[0])])

        total_length = float(self.s_samples[-1])
        # 超出终点：x/y 保持不动，返回路径终点
        if s_query >= total_length:
            return np.array([float(self.x_path[-1]), float(self.y_path[-1]), float(self.yaw_path[-1])])

        # 以真实弧长 s_samples 为插值轴，线性插值 x 和 y
        x_ref = float(np.interp(s_query, self.s_samples, self.x_path))
        y_ref = float(np.interp(s_query, self.s_samples, self.y_path))
        # yaw 使用解卷绕后的连续角度插值，避免 ±π 附近的跳变
        yaw_unwrapped = float(np.interp(s_query, self.s_samples, self.yaw_path_unwrapped))
        # 将插值结果重新折叠回 (-π, π]
        yaw_ref = float(np.arctan2(np.sin(yaw_unwrapped), np.cos(yaw_unwrapped)))
        return np.array([x_ref, y_ref, yaw_ref])

    def plot(self):
        """简单的可视化函数"""
        if len(self.x_path) == 0:
            print("No path to plot.")
            return
        
        plt.figure(figsize=(8, 8))
        plt.plot(self.x_path, self.y_path, 'b-', label="Spline Path")
        plt.quiver(self.x_path[::10], self.y_path[::10], 
                   np.cos(self.yaw_path[::10]), np.sin(self.yaw_path[::10]), 
                   color='green', scale=20, width=0.005)
        plt.axis("equal")
        plt.grid(True)
        plt.legend()
        plt.show()

# --- 调用示例 ---
if __name__ == "__main__":
    planner = SplinePlanner()

    # 情况 1: 指定起终点 Yaw (强制掉头)
    print("Generating path with yaw constraints...")
    x1, y1, yaw1 = planner.generate_path(
        x_pts=[0, 2, 4], 
        y_pts=[0, 1, 0], 
        start_yaw=0.0,           # 水平向右出发
        end_yaw=np.deg2rad(180)  # 水平向左结束
    )
    planner.plot()

    # 情况 2: 不指定 Yaw (自由平滑插值)
    print("Generating free path...")
    x2, y2, yaw2 = planner.generate_path(
x_pts=[0.39, 0.44909091, 0.50818182, 0.56727273, 0.62636364, 0.68545455, 0.74454545, 0.80363636, 0.86272727, 0.92181818, 0.98090909, 1.04, 1.15454545, 1.26909091, 1.38363636, 1.49818182, 1.61272727, 1.72727273, 1.84181818, 1.95636364, 2.07090909, 2.18545455, 2.3, 2.34111111, 2.38222222, 2.42333333, 2.46444444, 2.50555556, 2.54666667, 2.58777778, 2.62888889, 2.67, 2.86032258, 3.05064516, 3.24096774, 3.43129032, 3.6216129, 3.81193548, 4.00225806, 4.19258065, 4.38290323, 4.57322581, 4.76354839, 4.95387097, 5.14419355, 5.33451613, 5.52483871, 5.71516129, 5.90548387, 6.09580645, 6.28612903, 6.47645161, 6.66677419, 6.85709677, 7.04741935, 7.23774194, 7.42806452, 7.6183871, 7.80870968, 7.99903226, 8.18935484, 8.37967742, 8.57, 8.572, 8.574, 8.576, 8.578, 8.58, 8.582, 8.584, 8.586, 8.588, 8.59, 8.592, 8.594, 8.596, 8.598, 8.6, 8.602, 8.604, 8.606, 8.608, 8.61, 8.612, 8.614, 8.616, 8.618, 8.62], 
y_pts=[0.39, 0.55545455, 0.72090909, 0.88636364, 1.05181818, 1.21727273, 1.38272727, 1.54818182, 1.71363636, 1.87909091, 2.04454545, 2.21, 2.34727273, 2.48454545, 2.62181818, 2.75909091, 2.89636364, 3.03363636, 3.17090909, 3.30818182, 3.44545455, 3.58272727, 3.72, 3.89555556, 4.07111111, 4.24666667, 4.42222222, 4.59777778, 4.77333333, 4.94888889, 5.12444444, 5.3, 5.30645161, 5.31290323, 5.31935484, 5.32580645, 5.33225806, 5.33870968, 5.34516129, 5.3516129, 5.35806452, 5.36451613, 5.37096774, 5.37741935, 5.38387097, 5.39032258, 5.39677419, 5.40322581, 5.40967742, 5.41612903, 5.42258065, 5.42903226, 5.43548387, 5.44193548, 5.4483871, 5.45483871, 5.46129032, 5.46774194, 5.47419355, 5.48064516, 5.48709677, 5.49354839, 5.5, 5.3104, 5.1208, 4.9312, 4.7416, 4.552, 4.3624, 4.1728, 3.9832, 3.7936, 3.604, 3.4144, 3.2248, 3.0352, 2.8456, 2.656, 2.4664, 2.2768, 2.0872, 1.8976, 1.708, 1.5184, 1.3288, 1.1392, 0.9496, 0.76]
    )
    planner.plot()
