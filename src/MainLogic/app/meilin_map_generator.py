import json
import random
from dataclasses import dataclass, asdict
from typing import Dict, Optional
from pathlib import Path

@dataclass
class Pile:
    id: int
    block_type: str
    cost: int


def _is_outer_ring(node_id: int, rows: int = 4, cols: int = 3) -> bool:
    idx = node_id - 1
    r, c = divmod(idx, cols)
    return r == 0 or r == rows - 1 or c == 0 or c == cols - 1


def generate_initial_map(seed: Optional[int] = None) -> Dict:
    rng = random.Random(seed)

    all_nodes = list(range(1, 13))
    outer_nodes = [n for n in all_nodes if _is_outer_ring(n)]
    fake_candidates = [n for n in all_nodes if n not in (1, 2, 3)]

    piles: Dict[int, Pile] = {n: Pile(id=n, block_type="EMPTY", cost=1) for n in all_nodes}

    fake_node = rng.choice(fake_candidates)
    piles[fake_node] = Pile(id=fake_node, block_type="FAKE", cost=999)
    used = {fake_node}

    r2_nodes = rng.sample([n for n in all_nodes if n not in used], 4)
    for n in r2_nodes:
        piles[n] = Pile(id=n, block_type="R2", cost=5)
    used.update(r2_nodes)

    r1_nodes = rng.sample([n for n in outer_nodes if n not in used], 3)
    for n in r1_nodes:
        piles[n] = Pile(id=n, block_type="R1", cost=2)

    return {
        "grid": {"rows": 4, "cols": 3},
        "piles": {str(k): asdict(v) for k, v in piles.items()},
        "piles_list": [asdict(piles[i]) for i in range(1, 13)],
        "meta": {
            "fake_node": fake_node,
            "r2_nodes": sorted(r2_nodes),
            "r1_nodes": sorted(r1_nodes),
        },
    }


def generate_initial_map_json(seed: Optional[int] = None) -> str:
    return json.dumps(generate_initial_map(seed=seed), ensure_ascii=False)


def save_map(output_path: str, seed: Optional[int] = None) -> str:
    data = generate_initial_map(seed=seed)
    out = Path(output_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    return str(out)


if __name__ == "__main__":
    # 默认输出到 app 目录，可直接给 GUI 导入
    output = "/home/Elaina/ros2_ws/src/MainLogic/app/generated_initial_map.json"
    path = save_map(output_path=output, seed=None)  # seed 可填整数用于复现
    print(f"已生成初始 map: {path}")