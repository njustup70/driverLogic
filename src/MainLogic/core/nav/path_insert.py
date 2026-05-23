import numpy as np

# 1. 输入原始数据点 (x, y)
points = np.array([
    [0.39, 0.39],
    [1.04, 2.21],
    [2.30, 3.72],
    [2.47, 5.50],
    [8.67, 5.60],
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
    
    total_dist = np.linalg.norm(p2 - p1)
    num_segments = max(1, int(round(total_dist / target_dist)))
    t = np.linspace(0, 1, num_segments + 1, endpoint=False)
    segment_points = (1 - t)[:, np.newaxis] * p1 + t[:, np.newaxis] * p2
    interpolated_segments.append(segment_points)

# 3. 将最后一点手动添加进去
interpolated_segments.append(points[-1:])

# 4. 拼接所有片段
result_array = np.vstack(interpolated_segments)

# ==================== 修复后的打印部分 ====================

print(f"原始点个数: {len(points)}")
print(f"插值后总点数: {len(result_array)}")

print("\n--- 方式一：带括号和逗号的标准格式（安全兼容版） ---")
# 使用 array2string 来规避 set_printoptions 的版本不兼容问题
formatted_array = np.array2string(
    result_array, 
    separator=', ', 
    formatter={'float_kind': lambda x: f"{x:0.8f}"}
)
print(formatted_array)


print("\n--- 方式二：纯数据格式（最推荐，无括号，方便复制） ---")
for pt in result_array:
    print(f"{pt[0]:.8f}, {pt[1]:.8f}")


print("\n--- 间距验证 ---")
distances = np.linalg.norm(np.diff(result_array, axis=0), axis=1)
print(f"实际平均间距: {np.mean(distances):.4f}")
print(f"最小间距: {np.min(distances):.4f}, 最大间距: {np.max(distances):.4f}")