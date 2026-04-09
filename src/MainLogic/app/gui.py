import tkinter as tk
from tkinter import messagebox
import sys
import os

# 确保路径被正确识别，防止 import 报错
current_dir = os.path.dirname(os.path.abspath(__file__))
ws_root = os.path.abspath(os.path.join(current_dir, "../../"))
if ws_root not in sys.path:
    sys.path.insert(0, ws_root)

try:
    from MainLogic.app.meilin_map_generator import generate_initial_map
    import MainLogic.app.meilin_manager as mm
except ImportError as e:
    print(f"导入失败，请检查 PYTHONPATH: {e}")

class MerlinWeightedPathFinder:
    def __init__(self, root, half_flag="blue"):
        self.root = root
        self.root.title("ROBOCON 2026 - 梅林智能寻路")
        self.root.geometry("650x880")

        # 【核心修复】：字体检测和回退机制，确保中文显示
        self.default_font = self._get_available_font()
        self.FONT_NORMAL = (self.default_font, 10)
        self.FONT_BOLD = (self.default_font, 12, "bold")
        self.FONT_TITLE = (self.default_font, 20, "bold")
        self.FONT_SMALL = (self.default_font, 9, "bold")

        self.STATES = ["EMPTY", "R2", "R1", "FAKE"]
        self.COLORS = {"EMPTY": "#f0f0f0", "R2": "#3CB88B", "R1": "#ffb347", "FAKE": "#999999"}
        self.LABELS = {"EMPTY": "", "R2": "R2目标", "R1": "R1阻挡", "FAKE": "假桩 (死路)"}

        # 只用 manager 模块暴露的对象/函数
        self.manager = mm.MeilinManagerInstance
        self.map_model = mm.MeilinMap(half_flag=half_flag)

        self.create_widgets()
        self.load_generated_map_to_gui()
    
    def _get_available_font(self):
        """检测可用的中文字体，返回第一个可用的字体"""
        test_fonts = ["SimHei", "WenQuanYi Micro Hei", "Noto Sans CJK SC", "Heiti TC", "Arial Unicode MS", "Sans"]
        
        # 创建一个临时标签来测试字体
        temp_label = tk.Label(self.root)
        for font in test_fonts:
            try:
                # 尝试设置字体
                temp_label.config(font=(font, 10))
                # 如果没有抛出异常，说明字体可用
                return font
            except:
                continue
        # 如果所有字体都不可用，返回默认字体
        return "Sans"

    def create_widgets(self):
        tk.Label(self.root, text="点击方格切换类型：空地 -> R2 -> R1 -> 假桩", font=self.FONT_NORMAL).pack(pady=5)
        self.canvas = tk.Canvas(self.root, width=400, height=530, bg="white", bd=2, relief="ridge")
        self.canvas.pack(pady=10)

        self.rects, self.texts, self.path_elements = {}, {}, []
        for i in range(1, 13):
            row, col = (i - 1) // 3, (i - 1) % 3
            x1, y1 = 20 + col * 120, 20 + row * 120
            x2, y2 = x1 + 110, y1 + 110

            rid = self.canvas.create_rectangle(x1, y1, x2, y2, fill=self.COLORS["EMPTY"], width=2)
            # 使用修正后的字体
            self.canvas.create_text((x1 + x2) / 2, (y1 + y2) / 2 - 15, text=f"{i}", font=self.FONT_TITLE, fill="#bbb")
            sid = self.canvas.create_text((x1 + x2) / 2, (y1 + y2) / 2 + 15, text="", font=self.FONT_SMALL)

            self.rects[i], self.texts[i] = rid, sid
            for obj in (rid, sid):
                self.canvas.tag_bind(obj, "<Button-1>", lambda e, cid=i: self.cycle_state(cid))

        btn_frame = tk.Frame(self.root)
        btn_frame.pack(pady=10)
        tk.Button(btn_frame, text="重置网格", command=self.reset_grid, font=self.FONT_NORMAL).pack(side=tk.LEFT, padx=10)
        tk.Button(
            btn_frame,
            text="开始寻路",
            command=self.calculate_path,
            bg="#4CAF50",
            fg="white",
            font=self.FONT_BOLD,
        ).pack(side=tk.LEFT, padx=10)

        self.count_label = tk.Label(self.root, text="", font=self.FONT_NORMAL)
        self.count_label.pack()
        self.update_counts()

        self.result_text = tk.StringVar(value="就绪")
        tk.Label(self.root, textvariable=self.result_text, justify=tk.LEFT, wraplength=600, font=self.FONT_NORMAL).pack(pady=10)

    def cycle_state(self, cell_id):
        curr = self.map_model.piles[cell_id].block_type
        new_state = self.STATES[(self.STATES.index(curr) + 1) % len(self.STATES)]
        self.map_model._set_block_type_by_id(cell_id, new_state)
        self.canvas.itemconfig(self.rects[cell_id], fill=self.COLORS[new_state])
        self.canvas.itemconfig(self.texts[cell_id], text=self.LABELS[new_state])
        self.update_counts()
        self.clear_path()

    def update_counts(self):
        c = self.map_model.count_types()
        self.count_label.config(text=f"R2(目标): {c['R2']} | R1(阻挡): {c['R1']} | 假桩: {c['FAKE']}")

    def clear_path(self):
        for item in self.path_elements:
            self.canvas.delete(item)
        self.path_elements.clear()

    def reset_grid(self):
        for i in range(1, 13):
            self.map_model._set_block_type_by_id(i, "EMPTY")
            self.canvas.itemconfig(self.rects[i], fill=self.COLORS["EMPTY"])
            self.canvas.itemconfig(self.texts[i], text="")
        self.update_counts()
        self.clear_path()

    def load_generated_map_to_gui(self):
        """直接调用 map_generator，不保存本地文件。"""
        self.reset_grid()
        try:
            data = generate_initial_map(seed=None)
            piles = data.get("piles", {})

            for i in range(1, 13):
                bt = piles.get(str(i), {}).get("block_type", "EMPTY")
                self.map_model._set_block_type_by_id(i, bt)
                self.canvas.itemconfig(self.rects[i], fill=self.COLORS[bt])
                # 确保这里传入的是本地安全的字典字符串
                self.canvas.itemconfig(self.texts[i], text=self.LABELS[bt])

            self.update_counts()
            self.result_text.set("已加载生成地图（内存直传，未落盘）。")
        except Exception as e:
            self.result_text.set(f"加载地图失败: {str(e)}")

    def calculate_path(self):
        self.clear_path()
        try:
            # 同步到 manager 的状态
            states12 = [self.map_model.piles[i].block_type for i in range(1, 13)]
            self.manager.update_pile_states(states12)

            # 直接调用已有求解器函数
            route, total_cost = mm.solve_route(self.manager.solver_mode.value, self.map_model.piles)
            if not route:
                raise mm.SolverConfigError("当前布局无可行路径")

            self.manager.latest_route = route
            self.manager.latest_cost = total_cost
            self.manager.last_error = ""

            path_str = " -> ".join(f"{nid}[{'抓取' if picked else '路过'}]" for nid, picked in route)
            self.result_text.set(f"最优代价: {total_cost} | 步数: {len(route)-1}\n路径: {path_str}")
            self.draw_path(route)
        except Exception as e:
            self.manager.latest_route = []
            self.manager.latest_cost = float("inf")
            self.manager.last_error = str(e)
            messagebox.showerror("错误", str(e))

    def draw_path(self, path):
        coords = [(20 + (nid - 1) % 3 * 120 + 55, 20 + (nid - 1) // 3 * 120 + 55) for nid, _ in path]
        for i in range(len(coords) - 1):
            x1, y1 = coords[i]
            x2, y2 = coords[i + 1]
            lid = self.canvas.create_line(x1, y1, x2, y2, arrow=tk.LAST, width=3, fill="#2b580c")
            self.path_elements.append(lid)


if __name__ == "__main__":
    root = tk.Tk()
    app = MerlinWeightedPathFinder(root)
    root.mainloop()