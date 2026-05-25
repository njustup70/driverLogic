import numpy as np
import cv2
import matplotlib.pyplot as plt

def align_and_evaluate(src_pts, ref_pts):
    """
    使用 OpenCV API 将源点云配准到参考点云，并评估 X-Y 平面上的误差
    """
    num_points = len(src_pts)
    
    # 1. 计算 3D 刚体变换矩阵
    success, M, _ = cv2.estimateAffine3D(src_pts.astype(np.float32), ref_pts.astype(np.float32))
    if not success:
        print("变换矩阵计算失败")
        return None, None

    # 2. 将原始点变换到参考坐标系
    src_pts_homo = np.hstack((src_pts, np.ones((num_points, 1))))
    transformed_pts = (M @ src_pts_homo.T).T

    # 3. 计算误差：仅保留 X, Y 坐标
    errors = np.linalg.norm(transformed_pts[:, :2] - ref_pts[:, :2], axis=1)

    # 4. X-Y 散点图可视化
    plt.figure(figsize=(6, 6))
    plt.scatter(ref_pts[:, 0], ref_pts[:, 1], color='red', label='Reference (Target)', marker='o', s=100) #type:ignore
    plt.scatter(transformed_pts[:, 0], transformed_pts[:, 1], color='blue', label='Transformed (ICP)', marker='x', s=100) #type:ignore

    # 用虚线连接对应的点
    for i in range(num_points):
        plt.plot([ref_pts[i, 0], transformed_pts[i, 0]], [ref_pts[i, 1], transformed_pts[i, 1]], 'k--', alpha=0.5)

    plt.title('X-Y Plane Alignment Scatter Plot')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.tight_layout()
    plt.show()  # 直接弹窗显示

    return errors, transformed_pts

def main():
    # 1. 参考 3D 数据 (对应 5 个点)
    ref_pts = np.array([
        [0, 0, 0],
        [0, 6, 0],
        [8, 0, 0],
        [8, 6, 0],
        [12, 0, 0.6]
    ])

    # 2. 原始座标系数据
    src_pts = np.array([
        [1.694, -1.657, -0.729],
        [-0.657129, -6.945, -0.650],
        [-6.796, 2.038, -0.745],
        [-9.176, -3.330, -0.671],
        [-9.211, 3.047, -0.34]
    ])

    # 3. 调用函数进行配准与评估
    errors, _ = align_and_evaluate(src_pts, ref_pts)

    if errors is not None:
        # 4. 输出每个点的具体 X-Y 误差
        print(f"每个点的 X-Y 配准误差: {np.round(errors, 4)}")
        print(f"平均 X-Y 配准误差: {np.mean(errors):.4f}")

if __name__ == '__main__':
    main()