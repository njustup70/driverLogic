import numpy as np

# 1. 输入原始数据点 (x, y)
points = np.array([
    [0.39, 0.39],
    [1.04, 2.21],
    [2.30, 3.72],
    [2.67, 5.30],
    [8.57, 5.50],
    [8.62, 0.76]
])

# 期望的插值点间距
target_dist = 0.2

# 用于存储最终所有点的列表
interpolated_segments = []

# 2. 遍历每两个相邻的点进行插值
for i in range(len(points) - 1):
    p1 = points[i]
    p2 = points[i+1]
    
    # 计算两点之间的欧几里得距离
    total_dist = np.linalg.norm(p2 - p1)
    
    # 根据目标间距 0.2 计算需要分成多少段
    # 使用 round 确保间距尽可能接近 0.2
    num_segments = max(1, int(round(total_dist / target_dist)))
    
    # 在这两点之间生成等间距的比例系数 t，范围从 0 到 1
    # endpoint=False 是为了避免在拼接时，当前段的终点和下一段的起点重复
    t = np.linspace(0, 1, num_segments + 1, endpoint=False)
    
    # 线性插值公式：P(t) = (1-t)*P1 + t*P2
    segment_points = (1 - t)[:, np.newaxis] * p1 + t[:, np.newaxis] * p2
    
    interpolated_segments.append(segment_points)

# 3. 将最后一点（最后一个原始点）手动添加进去，保证轨迹完整
interpolated_segments.append(points[-1:])

# 4. 拼接所有片段，生成最终的 numpy 数组
result_array = np.vstack(interpolated_segments)

# 打印部分结果看看效果
print(f"原始点个数: {len(points)}")
print(f"插值后总点数: {len(result_array)}")
print("\n前10个插值点坐标:")
print(result_array)

# 验证一下相邻两点之间的实际距离是否在 0.2 左右
distances = np.linalg.norm(np.diff(result_array, axis=0), axis=1)
print(f"\n实际平均间距: {np.mean(distances):.4f}")
print(f"最小间距: {np.min(distances):.4f}, 最大间距: {np.max(distances):.4f}")