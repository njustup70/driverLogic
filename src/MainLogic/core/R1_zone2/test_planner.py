"""R1 二区路径规划测试脚本

用法：
    cd /home/Elaina/ros2_ws
    PYTHONPATH=src:$PYTHONPATH python3 src/MainLogic/core/R1_zone2/test_planner.py

修改下方 R1_BLOCKS / R2_BLOCKS / FAKE_BLOCK 即可测试不同配置。
"""

from R1_planner import compute_r1_zone2_path
from encoder import encode_zone2_frame

# ====== 在这里修改配置 ======
R1_BLOCKS = [9, 10, 11]    # R1 要抓的方块
R2_BLOCKS = [1, 3, 4, 5]   # R2 拥有的方块
FAKE_BLOCK = [0]            # 假方块

AUTO_MODE = 1               # 1=自动避让R2, 0=手动优先级
PRIORITY = []               # 手动模式下的优先级，如 [11, 9, 10]
# ============================

if __name__ == "__main__":
    result = compute_r1_zone2_path(
        r1_blocks=R1_BLOCKS,
        r2_blocks=R2_BLOCKS,
        fake_block=FAKE_BLOCK,
        auto_dog_flag=AUTO_MODE,
        priority_block=PRIORITY,
        verbose=True,
    )

    if not result['success']:
        print(f"\n❌ 失败: {result['error']}")
        exit(1)

    frame = encode_zone2_frame(result['filtered_nodes'])

    print()
    print("=" * 50)
    print(f"上位机下发帧 (0xFA 头):")
    print(f"  FA {frame.hex(' ')}")
    print(f"  共 {len(frame)} 字节, {len(result['filtered_nodes'])} 个动作")
    print("=" * 50)
