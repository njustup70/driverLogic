import numpy as np
import cv2
import matplotlib.pyplot as plt

def estimate_rigid_transform_3d(src_pts, ref_pts):
    """
    使用 SVD 求解严格的 3D 刚体变换矩阵 (R, t)，满足 ref = R * src + t
    """
    # 计算质心
    centroid_src = np.mean(src_pts, axis=0)
    centroid_ref = np.mean(ref_pts, axis=0)
    
    # 去质心
    src_centered = src_pts - centroid_src
    ref_centered = ref_pts - centroid_ref
    
    # 计算协方差矩阵
    H = src_centered.T @ ref_centered
    
    # SVD 分解
    U, S, Vt = np.linalg.svd(H)
    
    # 计算旋转矩阵 R
    R = Vt.T @ U.T
    
    # 特殊情况处理：防止反射矩阵
    if np.linalg.det(R) < 0:
        Vt[2, :] *= -1
        R = Vt.T @ U.T
        
    # 计算平移向量 t
    t = centroid_ref - R @ centroid_src
    
    # 构造 4x4 齐次变换矩阵
    M = np.identity(4)
    M[:3, :3] = R
    M[:3, 3] = t
    return M

def align_and_evaluate(src_pts, ref_pts):
    """
    配准源点云到参考点云，并评估 X-Y 平面上的误差
    """
    num_points = len(src_pts)
    
    # 1. 计算严格的 3D 刚体变换矩阵
    M = estimate_rigid_transform_3d(src_pts, ref_pts)

    # 2. 将原始点变换到参考坐标系 (使用齐次坐标)
    src_pts_homo = np.hstack((src_pts, np.ones((num_points, 1))))
    transformed_pts = (M @ src_pts_homo.T).T[:, :3] # 取前三列

    # 3. 计算误差：仅保留 X, Y 坐标
    errors = np.linalg.norm(transformed_pts[:, :2] - ref_pts[:, :2], axis=1)

    # 4. X-Y 散点图可视化
    plt.figure(figsize=(6, 6))
    plt.scatter(ref_pts[:, 0], ref_pts[:, 1], color='red', label='Reference (Target)', marker='o', s=100) #type:ignore
    plt.scatter(transformed_pts[:, 0], transformed_pts[:, 1], color='blue', label='Transformed (Rigid)', marker='x', s=100) #type:ignore

    for i in range(num_points):
        plt.plot([ref_pts[i, 0], transformed_pts[i, 0]], [ref_pts[i, 1], transformed_pts[i, 1]], 'k--', alpha=0.5)

    plt.title('X-Y Plane Alignment Scatter Plot')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.tight_layout()
    plt.show()

    return errors, M

def main():
    # 1. 参考 3D 数据
    ref_pts = np.array([
        [0, 0, 0],
        [0, 6, 0],
        [10, 0, 0],
        [10, 6, 0],
        [12, 0, 0.4]
    ])

    # 2. 原始坐标系数据
    src_pts = np.array([
        [-2.1666,-0.8297, 0.1435],
        [0.4807, 4.4663, 0.1039],
        [6.0277, -4.9484, 0.1014],
        [8.8340, 0.3189, 0.1532],
        [8.4576, -6.1373, 0.5001],
    ])

    # 3. 配准与评估
    errors, M = align_and_evaluate(src_pts, ref_pts)
    
    print(f"计算得到的严格刚体变换矩阵 M (4x4):\n{M}\n")
    if errors is not None:
        print(f"每个点的 X-Y 配准误差: {np.round(errors, 4)}")
        print(f"平均 X-Y 配准误差: {np.mean(errors):.4f}\n")
    
    # 4. 正确地变换第一个点
    # 方式 A：直接用矩阵数学计算 (推荐，不依赖外部库)
    first_pt_homo = np.array([src_pts[0][0], src_pts[0][1], src_pts[0][2], 1.0])
    ref_first_pt_math = (M @ first_pt_homo)[:3]
    print(f"【矩阵计算】第一个点变换后的坐标: {ref_first_pt_math}")

    # 方式 B：如果你必须用你的 SE3 类，请确保通过平移/或者点乘的方法：
    # try:
    #     from MainLogic.Lib.odomVec import SE3
    #     transform_se3 = SE3(matrix=M)
    #     # 注意：具体取决于你的 SE3 库如何支持点变换，通常是 transform_se3.act(src_pts[0]) 或直接乘
    #     # 如果 SE3 只支持 4x4 乘 4x4，你不能把点当矩阵传进去。
    # except Exception as e:
    #     pass

if __name__ == '__main__':
    main()