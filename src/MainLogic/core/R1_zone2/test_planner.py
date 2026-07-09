"""R1 二区 完整链路测试脚本

模拟 kfs_callback 的完整流水线：
  1. 12 字节 KFS 帧 (R1 编号, 0-11)
  2. 按红/蓝半场转换为 zone2_model 状态
  3. zone2_model 推算 R2 最优路径
  4. 提取 R2 路径上的 R1 节点 → R1 优先级
  5. R1 Planner 规划路径
  6. 编码 0xBA 帧

用法：
    cd /home/Elaina/ros2_ws
    PYTHONPATH=src:$PYTHONPATH python3 src/MainLogic/core/R1_zone2/test_planner.py
"""

from R1_planner import compute_r1_zone2_path
from encoder import compute_r2_entry_col, encode_zone2_frame

# ============================================================
# R1 ↔ zone2_model 坐标转换 (同 globalCallback.py)
# ============================================================
_R1_TO_ZONE2_RED  = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
_R1_TO_ZONE2_BLUE = [2, 1, 0, 5, 4, 3, 8, 7, 6, 11, 10, 9]
_STATE_MAP = {0: "EMPTY", 1: "R1", 2: "R2", 3: "FAKE"}


def r1_kfs_to_zone2_states(kfs_raw: list[int], field_color: int) -> list[str]:
    mapping = _R1_TO_ZONE2_RED if field_color == 0 else _R1_TO_ZONE2_BLUE
    return [_STATE_MAP[kfs_raw[mapping[z]]] for z in range(12)]


def zone2_stakes_to_r1_indices(stakes: list[int], field_color: int) -> list[int]:
    mapping = _R1_TO_ZONE2_RED if field_color == 0 else _R1_TO_ZONE2_BLUE
    return [mapping[n - 1] for n in stakes]


# ============================================================
# 测试配置
# ============================================================
# KFS 原始帧: 12 字节, R1 编号 0-11 (俯视从上到下、从左到右)
#   kfs_raw[0]=左上, [1]=中上, [2]=右上, [3]=左二, ..., [11]=右下
# 状态: 0=空  1=R1  2=R2  3=假块
KFS_RAW = [
    2, 0, 0,   # 上行: R2, R2, R1
    0, 0, 1,   # 二行: R1, R2, 空
    0, 3, 0,   # 三行: 假块, 空, R2
    2, 1, 0,   # 下行: 空, R1, 空
]

FIELD_COLOR = 0   # 0=红半场, 1=蓝半场
# ============================================================

if __name__ == "__main__":
    fc = FIELD_COLOR
    field_name = "红半场" if fc == 0 else "蓝半场"

    # 确保 KFS 长度正确
    kfs_raw = list(KFS_RAW[:12])
    assert len(kfs_raw) == 12, f"KFS_RAW 必须为 12 个元素，当前 {len(kfs_raw)}"

    # R1 视角解析
    r1_blocks = [i for i, v in enumerate(kfs_raw) if v == 1]
    r2_blocks = [i for i, v in enumerate(kfs_raw) if v == 2]
    fake_block = [i for i, v in enumerate(kfs_raw) if v == 3]

    print("=" * 60)
    print(f"  R1 二区完整链路测试 — {field_name}")
    print("=" * 60)
    print(f"\n📦 KFS 原始帧 (R1编号):")
    for row in range(4):
        start = row * 3
        labels = [f"{kfs_raw[i]}" for i in range(start, start + 3)]
        print(f"     [{start}] [{start+1}] [{start+2}]  →  {', '.join(labels)}")
    print(f"  R1块(kfs下标): {r1_blocks}")
    print(f"  R2块(kfs下标): {r2_blocks}")
    print(f"  假块(kfs下标): {fake_block}")

    # ----------------------------------------------------------
    # Step 1: R1 → zone2_model 转换 + R2 路径推算
    # ----------------------------------------------------------
    print(f"\n{'─' * 60}")
    print(f"  Step 1: R1 → zone2_model 坐标映射 ({field_name})")
    print(f"  映射表: {_R1_TO_ZONE2_RED if fc == 0 else _R1_TO_ZONE2_BLUE}")
    meilin_states = r1_kfs_to_zone2_states(kfs_raw, fc)
    print(f"  zone2 states: {meilin_states}")

    r1_priority = []
    r2_traversal_kfs = []
    if r2_blocks:
        try:
            # 直接导入核心模块，避免经过 globalCallback → actions 的循环依赖
            from MainLogic.core.zone2_model.merlin_map import get_merlin_map
            from MainLogic.core.zone2_model.path_solver import solve_route
            from MainLogic.core.zone2_model.zone2_format import extract_r1_nodes_on_path

            # 绕过 run_solver_on_states，直接构造 map_data + 求解
            _STATE_TO_BLOCK = {"EMPTY": "empty", "R1": "R1", "R2": "R2", "FAKE": "fake"}
            blocks = {}
            for i, s in enumerate(meilin_states, start=1):
                blocks[i] = _STATE_TO_BLOCK.get(s, s.lower())

            map_data = {
                "name": "merlin",
                "shape": {"rows": 4, "cols": 3},
                "nodes": ["start"] + list(range(1, 13)) + ["end"],
                "adjacency": get_merlin_map()["adjacency"],
                "blocks": blocks,
            }

            r2_result = solve_route(strategy="dijkstra", map_data=map_data, required_r2_count=2)
            r2_result["map_data"] = map_data

            print(f"\n  R2 求解结果: found={r2_result.get('found')}, "
                  f"cost={r2_result.get('cost')}, "
                  f"collected_r2={r2_result.get('collected_r2')}")

            # 输出 R2 路径序列
            r2_path_nodes = r2_result.get("path", [])
            r2_path_steps = r2_result.get("path_steps", [])
            print(f"  R2 路径节点序列 ({len(r2_path_nodes)} 步):")
            print(f"    {' → '.join(str(n) for n in r2_path_nodes)}")
            if r2_path_steps:
                print(f"  R2 路径详细步骤:")
                for i, step in enumerate(r2_path_steps):
                    node = step.get("node", "?")
                    edge = step.get("edge", "")
                    step_cost = step.get("cost", 0)
                    collected = step.get("collected_r2", "")
                    turn = step.get("turn", "")
                    print(f"    [{i}] node={node}, edge={edge}, cost={step_cost}"
                          f"{', turn=' + str(turn) if turn else ''}"
                          f"{', collected=' + str(collected) if collected else ''}")

            r1_on_path = extract_r1_nodes_on_path(r2_result)
            r1_priority = [
                idx for idx in zone2_stakes_to_r1_indices(r1_on_path, fc)
                if idx in r1_blocks
            ]
            print(f"  R2路径上的R1(zone2 stake): {r1_on_path}")
            print(f"  → R1优先级(kfs下标): {r1_priority}")

            # 提取 R2 实际经过的桩位 (zone2 stake → kfs 下标)
            r2_traversal_stakes = []
            for node in r2_path_nodes:
                try:
                    n = int(node)
                    if 1 <= n <= 12:
                        r2_traversal_stakes.append(n)
                except (ValueError, TypeError):
                    pass
            r2_traversal_kfs = zone2_stakes_to_r1_indices(r2_traversal_stakes, fc)
            print(f"  R2 实际经过桩位(zone2): {r2_traversal_stakes} → kfs下标: {r2_traversal_kfs}")
        except Exception as e:
            print(f"  ⚠ zone2_model 求解失败: {e}")
            import traceback; traceback.print_exc()
            print(f"  回退: 不设置优先级")
    else:
        print(f"  无 R2 块，跳过 R2 路径推算")

    # ----------------------------------------------------------
    # Step 2: R1 路径规划 (传入 zone2_model 算出的 R2 实际路径)
    # ----------------------------------------------------------
    if r2_traversal_kfs:
        auto_mode = 1
        print(f"\n  R1 优先级来源: zone2_model R2实际路径 → {r2_traversal_kfs}")
    elif r1_priority:
        auto_mode = 0
        print(f"\n  R1 优先级来源: zone2_model 节点路径 → {r1_priority}")
    elif r2_blocks:
        auto_mode = 1
        print(f"\n  R1 优先级来源: 格子级 R2 路线 (auto_dog_flag=1)")
    else:
        auto_mode = 0
        print(f"\n  无 R2 块，R1 自由取块")

    print(f"  R1块: {r1_blocks}")
    print(f"  R2块: {r2_blocks}")

    if not r1_blocks:
        print("  ⚠ 无 R1 块，跳过")
        exit(0)

    result = compute_r1_zone2_path(
        r1_blocks=r1_blocks,
        r2_blocks=r2_blocks,
        fake_block=fake_block,
        auto_dog_flag=auto_mode,
        priority_block=r1_priority,
        r2_traversal=r2_traversal_kfs if r2_traversal_kfs else None,
        start_candidates=[2, 0, 16],
        exit_node=11 if fc == 0 else 7,
        verbose=True,
    )

    if not result['success']:
        print(f"\n  ❌ R1 规划失败: {result['error']}")
        exit(1)

    # ----------------------------------------------------------
    # 编码输出
    # ----------------------------------------------------------
    r2_entry_col = compute_r2_entry_col(r2_traversal_kfs, fc)
    print(f"\n  R2 入口列编码: {r2_entry_col:02b} (00=中 01=左 10=右)")
    frame = encode_zone2_frame(result['filtered_nodes'], r2_entry_col=r2_entry_col)
    print(f"\n{'=' * 60}")
    print(f"  上位机下发帧 (0xFA 头):")
    print(f"    FA {frame.hex(' ')}")
    print(f"    共 {len(frame)} 字节, {len(result['filtered_nodes'])} 个动作")
    print(f"{'=' * 60}")
