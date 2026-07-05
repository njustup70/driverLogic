"""R1 二区路径规划核心算法
"""

from collections import deque

# === 常量与环境配置 ===
YAW_BOTTOM = 0.0
YAW_LEFT = -1.57
YAW_RIGHT = 1.57
YAW_TOP = 3.14

RING_AISLES = [7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 0, 1, 2, 3, 4, 5, 6]

AISLE_COORDS = {
    7: (-2, 2.5), 8: (-1, 2.5), 9: (0, 2.5), 10: (1, 2.5), 11: (2, 2.5),
    12: (2.5, 1.5), 13: (2.5, 0.5), 14: (2.5, -0.5), 15: (2.5, -1.5),
    16: (2, -2.5), 17: (1, -2.5), 0: (0, -2.5), 1: (-1, -2.5), 2: (-2, -2.5),
    3: (-2.5, -1.5), 4: (-2.5, -0.5), 5: (-2.5, 0.5), 6: (-2.5, 1.5)
}

MERLIN_COORDS = {
    0: (-1, 1.5), 1: (0, 1.5), 2: (1, 1.5),
    3: (-1, 0.5), 4: (0, 0.5), 5: (1, 0.5),
    6: (-1, -0.5), 7: (0, -0.5), 8: (1, -0.5),
    9: (-1, -1.5), 10: (0, -1.5), 11: (1, -1.5)
}


def is_corner(aisle_id):
    return aisle_id in [7, 11, 2, 16]


def get_valid_pickups(merlin_id):
    pickups = {
        0: [(8, YAW_TOP), (6, YAW_LEFT)],
        1: [(9, YAW_TOP)],
        2: [(10, YAW_TOP), (12, YAW_RIGHT)],
        3: [(5, YAW_LEFT)],
        4: [],
        5: [(13, YAW_RIGHT)],
        6: [(4, YAW_LEFT)],
        7: [],
        8: [(14, YAW_RIGHT)],
        9: [(3, YAW_LEFT)],
        10: [(0, YAW_BOTTOM)],
        11: [(15, YAW_RIGHT)]
    }
    return pickups.get(merlin_id, [])


def yaw_to_index(yaw):
    if abs(yaw - YAW_BOTTOM) < 0.1: return 0
    if abs(yaw - YAW_LEFT) < 0.1: return 1
    if abs(yaw - YAW_RIGHT) < 0.1: return 2
    return 3


# === 核心寻路逻辑 ===
def find_shortest_path(start_id, start_yaw, target_id, target_yaw=None):
    came_from = {}
    q = deque()

    start_state = (start_id, yaw_to_index(start_yaw))
    q.append((start_id, start_yaw, 0, -1, 0.0))
    came_from[start_state] = (-1, 0.0)

    found = False
    while q:
        curr_id, curr_yaw, dist, prev_id, prev_yaw = q.popleft()

        if curr_id == target_id and (target_yaw is None or abs(curr_yaw - target_yaw) < 0.1):
            found = True
            if target_yaw is None:
                target_yaw = curr_yaw
            break

        ring_idx = RING_AISLES.index(curr_id)
        left_idx = (ring_idx - 1) % len(RING_AISLES)
        right_idx = (ring_idx + 1) % len(RING_AISLES)
        next_nodes = [RING_AISLES[left_idx], RING_AISLES[right_idx]]

        for nxt in next_nodes:
            state_key = (nxt, yaw_to_index(curr_yaw))
            if state_key not in came_from:
                came_from[state_key] = (curr_id, curr_yaw)
                q.append((nxt, curr_yaw, dist + 1, curr_id, curr_yaw))

        if is_corner(curr_id):
            for n_yaw in [YAW_BOTTOM, YAW_LEFT, YAW_RIGHT, YAW_TOP]:
                state_key = (curr_id, yaw_to_index(n_yaw))
                if state_key not in came_from:
                    came_from[state_key] = (curr_id, curr_yaw)
                    q.append((curr_id, n_yaw, dist + 1, curr_id, curr_yaw))

    path = []
    if found:
        curr_i, curr_y = target_id, target_yaw
        while curr_i != -1:
            path.append({'id': curr_i, 'yaw': curr_y, 'is_pick': False, 'is_at_point': False})
            curr_i, curr_y = came_from[(curr_i, yaw_to_index(curr_y))]
        path.reverse()
        if len(path) > 0:
            path.pop(0)
    return path


def generate_full_route(start_id, start_yaw, r1_blocks, r2_path, exit_id, auto_dog_flag, priority_block):
    full_path = [{'id': start_id, 'yaw': start_yaw, 'is_pick': False, 'is_at_point': False, 'target_block': None}]

    def get_priority(block_id):
        if auto_dog_flag == 1:
            if block_id in r2_path:
                return r2_path.index(block_id)
            return 999
        else:
            if block_id in priority_block:
                return priority_block.index(block_id)
            return 999

    targets = [{'id': b, 'priority': get_priority(b)} for b in r1_blocks]
    targets.sort(key=lambda x: x['priority'])

    curr_id, curr_yaw = start_id, start_yaw

    for tgt in targets:
        options = get_valid_pickups(tgt['id'])
        if not options: continue

        target_aisle, target_yaw = options[0]
        segment = find_shortest_path(curr_id, curr_yaw, target_aisle, target_yaw)

        if not segment and curr_id == target_aisle and abs(curr_yaw - target_yaw) < 0.1:
            full_path[-1]['is_pick'] = True
            full_path[-1]['is_at_point'] = True
            full_path[-1]['target_block'] = tgt['id']
        elif segment:
            segment[-1]['is_pick'] = True
            segment[-1]['is_at_point'] = True
            segment[-1]['target_block'] = tgt['id']
            full_path.extend(segment)
            curr_id, curr_yaw = segment[-1]['id'], segment[-1]['yaw']

    if exit_id is not None:
        exit_segment = find_shortest_path(curr_id, curr_yaw, exit_id, target_yaw=None)
        if exit_segment:
            full_path.extend(exit_segment)

    return full_path


def find_optimal_mission(start_candidates, r1_blocks, r2_path, exit_id, auto_dog_flag, priority_block):
    best_path = None
    best_start = None
    min_steps = float('inf')

    for s_id in start_candidates:
        path = generate_full_route(s_id, YAW_BOTTOM, r1_blocks, r2_path, exit_id, auto_dog_flag, priority_block)
        path_length = len(path)
        if path_length < min_steps:
            min_steps = path_length
            best_path = path
            best_start = s_id

    return best_start, best_path


# === 对外编程接口 ===
def compute_r1_zone2_path(
    r1_blocks: list,
    r2_blocks: list,
    fake_block: list,
    auto_dog_flag: int = 1,
    priority_block: list = None,
    start_candidates: list = None,
    exit_node: int = 11,
    verbose: bool = True,
) -> dict:
    """根据方块配置计算 R1 二区最优路径。

    Args:
        exit_node: 离场过道编号。红半场使用 11，蓝半场使用 7。

    Returns: {success, filtered_nodes, best_start_id, r2_path, error}
    """
    if start_candidates is None:
        start_candidates = [2, 0, 16]
    if priority_block is None:
        priority_block = []

    # R2 最佳路线自动判定
    candidate_paths = [
        [9, 6, 3, 0],
        [10, 7, 4, 1],
        [11, 8, 5, 2],
    ]
    best_path = []
    max_r2_count = -1
    for path in candidate_paths:
        if any(fb in path for fb in fake_block):
            continue
        r2_count = sum(1 for b in path if b in r2_blocks)
        if r2_count > max_r2_count:
            max_r2_count = r2_count
            best_path = path

    if not best_path and auto_dog_flag == 1:
        return {'success': False, 'filtered_nodes': [], 'best_start_id': None,
                'r2_path': [], 'error': '假块导致无法规划R2路线'}

    r2_path = best_path

    if verbose:
        print("\n--- 读取配置与战术判定 ---")
        print(f"R1 块: {r1_blocks}")
        print(f"R2 块: {r2_blocks}")
        print(f"假 块: {fake_block}")
        print(f"R2 路线: {r2_path}")
        print(f"规划模式: {'自动 (避让 R2)' if auto_dog_flag == 1 else '手动 (用户覆盖)'}")
        if auto_dog_flag == 0 and priority_block:
            print(f"指定优先级: {priority_block}")
        print()

    best_start_id, best_path_sequence = find_optimal_mission(
        start_candidates, r1_blocks, r2_path, exit_node, auto_dog_flag, priority_block
    )

    if not best_path_sequence:
        return {'success': False, 'filtered_nodes': [], 'best_start_id': None,
                'r2_path': r2_path, 'error': '未能生成有效路径'}

    filtered_nodes = []
    for i, step in enumerate(best_path_sequence):
        is_start = (i == 0)
        is_end = (i == len(best_path_sequence) - 1)
        is_pick = step.get('is_pick', False)
        is_corner_node = step['id'] in [2, 7, 11, 16]

        if is_start or is_end or is_pick or is_corner_node:
            if i + 1 < len(best_path_sequence) and best_path_sequence[i]['id'] == best_path_sequence[i + 1]['id']:
                best_path_sequence[i + 1]['is_pick'] = best_path_sequence[i + 1].get('is_pick', False) or is_pick
                best_path_sequence[i + 1]['is_at_point'] = \
                    best_path_sequence[i + 1].get('is_at_point', False) or step.get('is_at_point', False)
                if is_pick:
                    best_path_sequence[i + 1]['target_block'] = step.get('target_block')
                continue
            filtered_nodes.append(step)

    if verbose:
        print("--- 最终下发底层序列 (已滤除直道冗余点 & 原地旋转重叠点) ---")
        for step in filtered_nodes:
            if step.get('is_pick'):
                action = f"** 抓取方块 {step['target_block']} **"
            elif step == filtered_nodes[0]:
                action = "起点出发"
            elif step == filtered_nodes[-1]:
                action = "到达终点离场"
            else:
                action = "经过角点 / 转向"
            print(f"过道: {step['id']:2d} | Yaw: {step['yaw']:5.2f} | "
                  f"is_at_point: {step.get('is_at_point', False)} | 动作: {action}")

    return {'success': True, 'filtered_nodes': filtered_nodes,
            'best_start_id': best_start_id, 'r2_path': r2_path, 'error': None}