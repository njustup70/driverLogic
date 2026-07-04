import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.lines as mlines
from collections import deque
import tkinter as tk
from tkinter import messagebox

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
    
    # 优先级判定 (同步 C++ 逻辑)
    def get_priority(block_id):
        if auto_dog_flag == 1:
            # 自动模式
            if block_id in r2_path:
                return r2_path.index(block_id) 
            return 999 
        else:
            # 手动模式
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

# === 绘图可视化 ===
def draw_scene(r1_blocks, r2_blocks, fake_block, r2_path, exit_id, generated_path, auto_dog_flag):
    fig, ax = plt.subplots(figsize=(10, 8)) 
    ax.set_aspect('equal')
    ax.axis('off')

    for mid, (x, y) in MERLIN_COORDS.items():
        color = 'white'
        if mid in r1_blocks: color = '#ff9999'   
        if mid in r2_blocks: color = '#99ccff'   
        if mid in fake_block: color = '#e0e0e0'  
        rect = patches.Rectangle((x - 0.45, y - 0.45), 0.9, 0.9, linewidth=1, edgecolor='black', facecolor=color)
        ax.add_patch(rect)
        ax.text(x, y, str(mid), ha='center', va='center', fontsize=12, fontweight='bold')

    for aid, (x, y) in AISLE_COORDS.items():
        color = '#ffe6cc' if is_corner(aid) else '#f0f0f0'
        circle = patches.Circle((x, y), 0.3, edgecolor='black', facecolor=color)
        ax.add_patch(circle)
        ax.text(x, y, str(aid), ha='center', va='center', fontsize=10)

    if exit_id is not None:
        exit_x, exit_y = AISLE_COORDS[exit_id]
        ax.plot(exit_x, exit_y, marker='*', color='gold', markersize=25, markeredgecolor='black', label='Exit Point')

    r2_x = [MERLIN_COORDS[mid][0] for mid in r2_path]
    r2_y = [MERLIN_COORDS[mid][1] for mid in r2_path]
    ax.plot(r2_x, r2_y, color='blue', linestyle='--', linewidth=3, alpha=0.5, label='R2 Path')
    if len(r2_x) > 1:
        ax.annotate('', xy=(r2_x[-1], r2_y[-1]), xytext=(r2_x[-2], r2_y[-2]),
                    arrowprops=dict(arrowstyle="->", color="blue", lw=3, alpha=0.5))

    for i in range(len(generated_path) - 1):
        p1 = generated_path[i]
        p2 = generated_path[i+1]
        x1, y1 = AISLE_COORDS[p1['id']]
        x2, y2 = AISLE_COORDS[p2['id']]
        ax.annotate('', xy=(x2, y2), xytext=(x1, y1),
                    arrowprops=dict(arrowstyle="->", color="red", lw=2.5, shrinkA=12, shrinkB=12, connectionstyle="arc3,rad=0.15"))

    for p in generated_path:
        if p.get('is_pick'):
            x, y = AISLE_COORDS[p['id']]
            dx, dy = 0, 0
            if abs(p['yaw'] - YAW_BOTTOM) < 0.1: dy = 0.5
            elif abs(p['yaw'] - YAW_LEFT) < 0.1: dx = 0.5
            elif abs(p['yaw'] - YAW_RIGHT) < 0.1: dx = -0.5
            else: dy = -0.5
            
            ax.annotate('', xy=(x+dx, y+dy), xytext=(x, y),
                        arrowprops=dict(facecolor='green', shrink=0, width=4, headwidth=10))
            ax.text(x+dx*1.2, y+dy*1.2, f"Pick {p['target_block']}", color='green', fontweight='bold', ha='center')

    title_mode_str = "AUTO Decision Engine" if auto_dog_flag == 1 else "MANUAL Priority Override"
    plt.title(f"R1 Optimal Path ({title_mode_str})")
    
    r1_patch = patches.Patch(color='#ff9999', label='R1 Blocks')
    r2_patch = patches.Patch(color='#99ccff', label='R2 Blocks')
    fake_patch = patches.Patch(color='#e0e0e0', label='Fake Block')
    exit_marker = mlines.Line2D([], [], color='white', marker='*', markerfacecolor='gold', markeredgecolor='black', markersize=15, label=f'Exit ({exit_id})')
    r2_line = mlines.Line2D([], [], color='blue', linestyle='--', linewidth=3, label='R2 Path (Auto-Generated)')
    r1_line = mlines.Line2D([], [], color='red', marker='>', markersize=8, label='R1 Moving Path')
    
    ax.legend(handles=[r1_patch, r2_patch, fake_patch, r2_line, exit_marker, r1_line], loc='center left', bbox_to_anchor=(1, 0.5))
    plt.show()

# === GUI 界面类 ===
class RoboconSetupUI:
    def __init__(self, root):
        self.root = root
        self.root.title("ROBOCON UP70 - 赛前场控配置")
        self.root.geometry("400x530") 
        
        self.r1_blocks = []
        self.r2_blocks = []
        self.fake_block = []
        self.r2_path = []
        self.auto_dog_flag = 1
        self.priority_block = []
        self.ready = False
        
        self.block_states = {i: 0 for i in range(12)}
        self.colors = {0: "#ffffff", 1: "#ff9999", 2: "#99ccff", 3: "#e0e0e0"}
        self.labels = {0: "空", 1: "R1", 2: "R2", 3: "Fake"}
        
        tk.Label(root, text="点击对应方块切换属性", font=("Arial", 14, "bold")).pack(pady=10)
        
        grid_frame = tk.Frame(root)
        grid_frame.pack(pady=5)
        
        self.buttons = {}
        for i in range(12):
            row = i // 3
            col = i % 3
            btn = tk.Button(grid_frame, text=f"{i}\n{self.labels[0]}", width=8, height=3, 
                            bg=self.colors[0], font=("Arial", 10),
                            command=lambda idx=i: self.toggle_block(idx))
            btn.grid(row=row, column=col, padx=5, pady=5)
            self.buttons[i] = btn
            
        tk.Label(root, text="R1 战术规划模式:", font=("Arial", 11, "bold")).pack(pady=(15, 5))
        
        self.mode_var = tk.IntVar(value=1)
        mode_frame = tk.Frame(root)
        mode_frame.pack()
        
        tk.Radiobutton(mode_frame, text="自动躲避 R2", variable=self.mode_var, value=1, command=self.toggle_mode).pack(side=tk.LEFT, padx=10)
        tk.Radiobutton(mode_frame, text="手动指定顺序", variable=self.mode_var, value=0, command=self.toggle_mode).pack(side=tk.LEFT, padx=10)
        
        self.manual_frame = tk.Frame(root)
        self.manual_frame.pack(pady=5)
        tk.Label(self.manual_frame, text="优先级 (逗号分隔, 如 11,2):").pack(side=tk.LEFT)
        self.manual_entry = tk.Entry(self.manual_frame, width=12, state=tk.DISABLED)
        self.manual_entry.pack(side=tk.LEFT, padx=5)

        submit_btn = tk.Button(root, text="🚀 生成下发序列", font=("Arial", 14, "bold"), bg="#aaffaa", command=self.submit)
        submit_btn.pack(pady=15)

    def toggle_block(self, idx):
        self.block_states[idx] = (self.block_states[idx] + 1) % 4
        state = self.block_states[idx]
        self.buttons[idx].config(text=f"{idx}\n{self.labels[state]}", bg=self.colors[state])

    def toggle_mode(self):
        if self.mode_var.get() == 0:
            self.manual_entry.config(state=tk.NORMAL)
        else:
            self.manual_entry.config(state=tk.DISABLED)

    def submit(self):
        self.r1_blocks = [k for k, v in self.block_states.items() if v == 1]
        self.r2_blocks = [k for k, v in self.block_states.items() if v == 2]
        self.fake_block = [k for k, v in self.block_states.items() if v == 3]
        
        if len(self.fake_block) > 1:
            messagebox.showerror("配置错误", "Fake块数量不能超过1个！")
            return
            
        self.auto_dog_flag = self.mode_var.get()
        if self.auto_dog_flag == 0:
            seq_str = self.manual_entry.get().strip()
            if seq_str:
                try:
                    self.priority_block = [int(x.strip()) for x in seq_str.split(',') if x.strip()]
                    for b in self.priority_block:
                        if b not in self.r1_blocks:
                            messagebox.showwarning("警告", f"输入的方块 {b} 不在当前配置的 R1 块中！")
                            return
                except ValueError:
                    messagebox.showerror("格式错误", "请输入有效的数字序列，并用英文逗号分隔。")
                    return
            else:
                self.priority_block = []
        
        # R2 最佳路线自动判定
        candidate_paths = [
            [9, 6, 3, 0],   
            [10, 7, 4, 1],  
            [11, 8, 5, 2]   
        ]
        
        best_path = []
        max_r2_count = -1
        
        for path in candidate_paths:
            if any(fb in path for fb in self.fake_block):
                continue
            r2_count = sum(1 for b in path if b in self.r2_blocks)
            if r2_count > max_r2_count:
                max_r2_count = r2_count
                best_path = path
                
        if not best_path and self.auto_dog_flag == 1:
            messagebox.showerror("战术无解", "由于 Fake 块的存在或布局问题，无法规划出合法的 R2 路线！")
            return
            
        self.r2_path = best_path
        self.ready = True
        self.root.destroy()

# === 主程序 ===
if __name__ == "__main__":
    
    root = tk.Tk()
    app = RoboconSetupUI(root)
    root.mainloop()
    
    if not app.ready:
        print("配置已取消，程序退出。")
        exit()
        
    start_candidates = [2, 0, 16]
    exit_node = 11
    
    r1_blocks = app.r1_blocks
    r2_blocks = app.r2_blocks
    fake_block = app.fake_block
    r2_path = app.r2_path
    auto_dog_flag = app.auto_dog_flag
    priority_block = app.priority_block
    
    print("\n--- 读取配置与战术判定 ---")
    print(f"R1 块: {r1_blocks}")
    print(f"R2 块: {r2_blocks}")
    print(f"假 块: {fake_block}")
    print(f"R2 路线: {r2_path}")
    print(f"规划模式: {'自动 (避让 R2)' if auto_dog_flag == 1 else '手动 (用户覆盖)'}")
    if auto_dog_flag == 0 and priority_block:
        print(f"指定优先级: {priority_block}")
    print()

    # 运行核心规划算法
    best_start_id, best_path_sequence = find_optimal_mission(
        start_candidates, r1_blocks, r2_path, exit_node, auto_dog_flag, priority_block
    )
    
    if best_path_sequence:
        print("--- 最终下发底层序列 (已滤除直道冗余点 & 原地旋转重叠点) ---")
        
        filtered_nodes = []
        
        # 核心去重逻辑：保留中偏后的节点，继承 is_pick
        for i, step in enumerate(best_path_sequence):
            is_start = (i == 0)
            is_end = (i == len(best_path_sequence) - 1)
            is_pick = step.get('is_pick', False)
            is_corner_node = step['id'] in [2, 7, 11, 16] 
            
            if is_start or is_end or is_pick or is_corner_node:
                
                # 向后看一眼：如果下一个节点的物理坐标（id）一模一样，说明仅仅是原地旋转
                # 舍弃当前节点，并把取块状态继承给下一个节点防止丢失
                if i + 1 < len(best_path_sequence) and best_path_sequence[i]['id'] == best_path_sequence[i+1]['id']:
                    best_path_sequence[i+1]['is_pick'] = best_path_sequence[i+1].get('is_pick', False) or is_pick
                    best_path_sequence[i+1]['is_at_point'] = best_path_sequence[i+1].get('is_at_point', False) or step.get('is_at_point', False)
                    if is_pick:
                        best_path_sequence[i+1]['target_block'] = step.get('target_block')
                    continue
                
                filtered_nodes.append(step)
                
        # 打印底层真正接收到的点
        for step in filtered_nodes:
            if step.get('is_pick'):
                action = f"** 抓取方块 {step['target_block']} **"
            elif step == filtered_nodes[0]:
                action = "起点出发"
            elif step == filtered_nodes[-1]:
                action = "到达终点离场"
            else:
                action = "经过角点 / 转向"
            
            print(f"过道: {step['id']:2d} | Yaw: {step['yaw']:5.2f} | is_at_point: {step.get('is_at_point', False)} | 动作: {action}")
                
        # 绘图还是传入完整序列，保证箭头连贯性
        draw_scene(r1_blocks, r2_blocks, fake_block, r2_path, exit_node, best_path_sequence, auto_dog_flag)
    else:
        print("未能生成有效路径，请检查方块配置是否合理！")