"""zone2_encoder 模块作用：

负责把动作序列编码成二进制帧，供串口或下位机直接消费。

本模块只做"数据编码"，不负责打印和发送；发送逻辑放在 `zone2_sender.py`。
"""
from typing import Any, Dict, List

# =============================================================================
# 单字节功能码协议
# =============================================================================
# 升降
FUNC_UP_200   = 0x64
FUNC_DOWN_200 = 0x65
FUNC_UP_400   = 0x66
FUNC_DOWN_400 = 0x67
# 转向
FUNC_TURN_CCW = 0x68  # 逆时针（正方向）
FUNC_TURN_CW  = 0x69  # 顺时针（负方向）
# 取块
FUNC_PICK_UP_200   = 0x6A
FUNC_PICK_UP_400   = 0x6B
FUNC_PICK_DOWN_200 = 0x6C
FUNC_PICK_DOWN_400 = 0x6D

# delta_z → 升降功能码
_LIFT_CODE: Dict[int, int] = {
    200:  FUNC_UP_200,
    -200: FUNC_DOWN_200,
    400:  FUNC_UP_400,
    -400: FUNC_DOWN_400,
}

# delta_z → 取块功能码
_PICK_CODE: Dict[int, int] = {
    200:  FUNC_PICK_UP_200,
    -200: FUNC_PICK_DOWN_200,
    400:  FUNC_PICK_UP_400,
    -400: FUNC_PICK_DOWN_400,
}

# turn code → 转向功能码
_TURN_CODE: Dict[int, int] = {
    1: FUNC_TURN_CCW,  # 逆时针
    2: FUNC_TURN_CW,   # 顺时针
}

# 动作类型 → (参数字段, 映射表)
_ACTION_ENCODERS = {
    "pick": ("delta_z", _PICK_CODE),
    "lift": ("delta_z", _LIFT_CODE),
    "turn": ("code",    _TURN_CODE),
}


# =============================================================================
# 编码
# =============================================================================
def encode_r1_frame(r1_nodes: List[int]) -> bytes:
    """编码 R1 物块列表帧: [个数, 桩号1, 桩号2, ...] 每字节一个。"""
    return bytes([len(r1_nodes)]) + bytes(r1_nodes)


def encode_action_sequence(actions: List[Dict[str, Any]]) -> bytes:
    """将动作序列编码为 [长度位 | 功能码...] 的字节串。

    帧格式: [N, code1, code2, ..., codeN]
      N  = 有效功能码个数（不含 move）

    pick  → 0x6A~0x6D  (按 delta_z)
    lift  → 0x64~0x67  (按 delta_z)
    turn  → 0x68~0x69  (按 code)
    move  → 忽略（升降已合并到 lift）
    """
    codes = bytearray()
    for act in actions:
        t = act["type"]
        if t == "move":
            continue  # 纯水平移动无功能码
        if t not in _ACTION_ENCODERS:
            print(f"[encode] 未知动作类型: {t}，跳过")
            continue

        key, mapping = _ACTION_ENCODERS[t]
        val = act.get(key, 0)
        code = mapping.get(val)
        if code is None:
            print(f"[encode] 未知 {t} {key}={val}，跳过")
            continue
        codes.append(code)

    # 前面插入长度位
    return bytes([len(codes)]) + bytes(codes)