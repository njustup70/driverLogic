# 20 单动作回调与协议清单

> 目的：把"可单独下发、单独完成、单独确认"的最小动作找出来，形成一份可以直接发给电控组负责人的清单；同时记录现状协议、历史协议和缺口。
> 基线：`b2e43a2`；历史实现来自 `071fe4c^`（`app/`、`core/zone2_model/` 删除前）。
> **本文第 6 节是草案，未定稿；第 9 节是待电控确认的问题。**

---

## 1. 什么叫"单动作"

满足以下全部条件的最小动作：

1. 一个命令字（或一个 action ID）就能表达；
2. 参数是固定的、可序列化的（没有"先发半帧再发半帧"）；
3. 下位机能独立执行，不依赖上一帧的上下文；
4. 执行完有明确的完成回包（ACK），最好带状态；
5. 上位机可以单独调用、单独超时、单独重试；
6. 不隐含"整场比赛流程"（流程属于决策层）。

反例：`0x91` 整段动作序列、`0x73` R1 列表——它们是"批量/流程"帧，不是单动作。

---

## 2. 上行帧现状（MCU → 上位机）

物理帧格式（`my_serial._parser_loop`）：

```
0xFF | LEN | CODE | BODY
LEN = 1 + len(BODY)
```

解析器回调收到 `0xFF + CODE + BODY`；`ros_bridge_node` 再剥掉 `0xFF/CODE`，把 `BODY` 交给按 `CODE` 注册的回调。

| CODE | 帧示例 | BODY | 含义 | 回调 | 状态 |
|---|---|---|---|---|---|
| `0xAA` | `FF 0D AA <12B>` | 3 × f32 (x,y,yaw) | 里程计 | `mcu_transmit_callback` | ✅ 在用 |
| `0xB3` | `FF 15 B3 <20B>` | header + 2B + 4×f32 + tail | SICK 双测距 | `sick_callback` | ✅ 在用（只取前 2 个 float） |
| `0xB2` | `FF 03 B2 B2 FF` | `B2 FF` | 触发 SICK 初始 yaw 纠偏 | `serial_correct_callback` | ✅ 在用 |
| `0x6F` | `FF 01 6F` | 空 | 单动作完成 ACK | `action_callback` | ❌ 链路断（见第 5 节） |
| `0xB1` | `FF 03 B1 <type> <arm>` | 2B | 攀爬类型/臂状态 | `climb_type_callback` | 🗑 已删除 |
| `0x78` | `FF 03 78 <field> FF` | 2B | 红/蓝场 | `field_color_callback` | 🗑 已删除 |
| `0x69` | `FF 03 69 <zone> FF` | 2B | 一/三区重试 | `zone_retry_callback` | 🗑 已删除 |
| `0x13` | `FF 03 13 13 FF` | 2B | SLAM 容器重启 | `slam_restart_callback` | 🗑 已删除 |
| `0xC2` | 变长 | KFS 属性 | 二区桩位属性 | `kfs_callback` | 🗑 已删除 |

> 注意：`0x6F` 的 `BODY` 为空，`LEN=1`；`ros_bridge_node` 剥完只剩 `b''`，所以现有 `action_callback` 永远收不到 `FF 6F`。

---

## 3. 下行命令现状（上位机 → MCU）

物理帧格式（`writeBytes`）：

```
0xFA | CODE | BODY
```

**没有 LEN、没有 CRC、没有 SEQ**；下位机只能靠 `CODE` 推断 `BODY` 长度。

| CODE | BODY | 含义 | 位置 | ACK | 可用性 |
|---|---|---|---|---|---|
| `0xA0` | 3 × f32 (x,y,yaw) | 融合位姿回传（100 Hz） | `tf_manager.py:210/442` | 无 | ✅ 在用 |
| `0xA1` | 3 × f32 (x,y,yaw) | 底盘到点 | `Move.py:96` | 无（轮询位姿） | ⚠️ 代码在，入口不可用 |
| `0xBB` | 3 × f32 (vx,vy,wz) | 底盘速度 | `Move.py:113` | 无 | ✅ 在用（`R2n_Main`） |
| `0xA2` | 空 | 取矛头 | 历史 `actions.py` | 期望 `A3` | 🗑 已删除 |
| `0xA3` | 空（最新 `actions.py`）；旧版尝试 + f32 矛序 | 对矛 / 触发对矛 | 历史 `actions.py`、`meilin_climb.py` | 期望 `A3` 回包 | 🗑 已删除，且与上行 code 混用 |
| `0xB1` | 2 × f32 (left_mm, up_mm) | 矛头偏移 | 历史 `actions.py`；`globalCallback.py:68` 保留常量 | 无 | ⚠️ 常量在，发送端删除 |
| `0xB1` | 1 B (height code) | 攀爬腿高度档位 | 历史 `climb_manager.py` | 无 | 🗑 已删除，与上一行冲突 |
| `0x64` | 空 | 升降 +200 mm | 历史 `zone2_encoder.py` | `0x6F` | 🗑 仅在序列中使用 |
| `0x65` | 空 | 升降 -200 mm | 同上 | `0x6F` | 🗑 |
| `0x66` | 空 | 升降 +400 mm | 同上 | `0x6F` | 🗑 |
| `0x67` | 空 | 升降 -400 mm | 同上 | `0x6F` | 🗑 |
| `0x68` | 空 | 逆时针转 90° | 同上 | `0x6F` | 🗑 |
| `0x69` | 空 | 顺时针转 90° | 同上 | `0x6F` | 🗑 与上行 `0x69`（重试区）冲突 |
| `0x6A` | 空 | 取块上升 200 mm | 同上 | `0x6F` | 🗑 |
| `0x6B` | 空 | 取块上升 400 mm | 同上 | `0x6F` | 🗑 |
| `0x6C` | 空 | 取块下降 200 mm | 同上 | `0x6F` | 🗑 |
| `0x6D` | 空 | 取块下降 400 mm | 同上 | `0x6F` | 🗑 |
| `0x91` | 起始桩+0x7F, 列桩+0x7F, codes..., `0x6E` | 整段动作序列 | 历史 `zone2_sender.py` | 每帧 `0x6F` | ❌ 编解码与发送器错位（见 5.2） |
| `0x73` | 桩号... | R1 物块列表（无个数，长度靠帧长推断） | 历史 `zone2_sender.py` | 无 | 🗑 |
| `0x52` | 空 | SLAM 重置 | 历史 `globalCallback.py` | 无 | 🗑 |
| `0xB4` | UTF-8（历史版本不一） | YOLO 类别名下发 | 历史 `globalCallback.py` | 无 | 🗑 |
| `0x78` | 2 × u8 (field, zone_retry) | 红蓝场 + 重试区配置 | 历史 R1_new `tf_manager.py` | 无 | 🗑 与上行 `0x78`（红蓝场）冲突 |

### 3.1 并行的第三套协议（未接入主逻辑）

`src/serial_dispose/` 是另一套遗留 ROS2 节点（没有 `package.xml`，但 `src/perception/package.xml` 还 `exec_depend` 它）：

| 方向 | CODE | 含义 | 说明 |
|---|---|---|---|
| 下行 | `0xA0` | `cmd_vel` | 与 MainLogic 的 `0xA0`（融合位姿）**语义冲突** |
| 下行 | `0xA1` | `location` | 与 MainLogic 的 `0xA1`（到点）同名不同义 |
| 上行 | `0xAA` | odom | 与 MainLogic 一致 |
| 上行 | `0xAB` | `exec_callback` | 1 字节动作 ID，映射到 `String` 话题；是"单动作完成"的另一种雏形，但无 SEQ/STATUS |

结论：仓库里至少有三处协议定义（`MainLogic`、`serial_dispose`、旧 `protocol_lib` 串口封装），标准化时必须先收敛到一处。

---

## 4. 历史存在、可以恢复的单动作

如果今年赛题仍需要这些机构，可以直接沿用旧码，减少下位机改动：

| 动作 | 旧码 | 旧载荷 | 完成方式 | 恢复成本 |
|---|---|---|---|---|
| 取矛头 | `0xA2` | 无 | 旧代码期望 `A3`，但 ACK 未闭环 | 低（发送端 3 行） |
| 对矛 | `0xA3` | 空（或 f32 矛序，旧版未跑通） | 旧代码期望 `A3` 回包 | 低，但要先解决 A3 双向冲突 |
| 矛头偏移 | `0xB1` | 2 × f32 (mm) | 无 ACK | 低，但要与攀爬腿分开 |
| 攀爬腿高度 | `0xB1` | 1 B 档位 | 无 ACK | 中（要定义档位表） |
| 升降 ±200/±400 | `0x64-0x67` | 无 | `0x6F` | 中（要能从序列中拆出来单独发） |
| 转向 ±90° | `0x68/0x69` | 无 | `0x6F` | 中（同上） |
| 取块升降 ±200/±400 | `0x6A-0x6D` | 无 | `0x6F` | 中（同上） |
| 红蓝场 | `0x78` | 1 B | 无 | 低（回调可直接恢复） |
| 一/三区重试 | `0x69` | 1 B | 无 | 低 |
| SLAM 重启 | `0x13` | 无 | 无 | 低（但耦合 docker） |
| 通用动作完成 | `0xAB` | 1 B 动作 ID | 无（映射 `exec_callback` 话题） | 低，但属于 `serial_dispose`，不在 MainLogic |

恢复旧回调的入口：

```bash
git show 071fe4c^:src/MainLogic/globalCallback.py
git show 071fe4c^:src/MainLogic/app/actions.py
git show 071fe4c^:src/MainLogic/app/climb_manager.py
git show 071fe4c^:src/MainLogic/core/zone2_model/zone2_encoder.py
```

注意：旧回调都假设"桥接层已经剥掉 `0xFF/CODE`"，恢复时要和当前的剥帧逻辑一起核对，不要直接粘贴。

---

## 5. 缺口与已确认的协议债

### 5.1 完全没有实现的动作

| 动作 | 结论 |
|---|---|
| **夹爪开合** | 全历史 `git grep`（`夹爪/gripper/claw/开合/吸盘/pump/suction`）无任何实现 |
| **夹爪位置/力控** | 无 |
| **升降到绝对高度** | 只有 ±200/±400 档位，没有"到 350 mm"这种绝对目标 |
| **机构回零/找限位** | 无 |
| **急停 / 全机构停止** | 无（只有速度置 0） |
| **心跳 / 看门狗** | 无 |
| **状态查询**（当前高度/夹爪状态/故障码） | 无 |
| **复位/清故障** | 无 |

### 5.2 已确认的协议债（会直接影响单动作）

1. **ACK 不可关联**：`0x6F` 只有 1 字节，没有命令字/序号；连续发两个动作时无法知道是谁完成。
2. **ACK 链路断**：解析器输出 `FF 6F`，桥接层剥成 `b''`；`action_callback` 判断 `FF 6F` 永远失败。
3. **`0x91` 编解码错位**：`encode_action_sequence` 返回 `[0x91, start, column, codes..., 0x6E]`，但 `send_actions_one_by_one` 把 `data[0]` 当"功能码个数"（实际是 `0x91=145`），并用 `codes = data[4:-1]` 跳过第一个功能码；文档写的是 `[N, 0x91, ...]`，代码里没有 `N`。
4. **`0xB1` 冲突**：矛头偏移（float）与攀爬腿（1 字节档位）共用。
5. **`0xA3` 双向混用**：既做下行"对矛"，又做上行"完成"。
6. **`0x69` 双向混用**：下行是 `TURN_CW_90`（`zone2_encoder`），上行是一/三区重试（`zone_retry_callback`）。
7. **`0x78` 双向混用**：上行是红蓝场回调，下行是 R1_new 的 `[field_color_flag, zone_retry_flag]` 配置帧。
8. **`0xA0` 语义漂移**：当前是融合位姿回传；旧 `serial_dispose` 里是 `cmd_vel`。
9. **下行无 CRC**：旧车 921600 波特率下，长线/电机干扰时误帧无法发现。
10. **未知 code 广播**：`ros_bridge_node.py:70-72` 把未知帧广播给所有回调，可能把错误长度数据喂给 odom/sick 解析器。

---

## 6. 建议的最小单动作协议（草案 v0，兼容旧车）

### 6.1 设计目标

- **不改旧车已有命令**（`A0/A1/BB/AA/B3/B2` 原样保留）；
- **只新增一个"单动作包装命令"**，让新机构（夹爪/升降/矛/腿）有统一的请求-应答；
- ACK 带 `命令字 + 序号 + 状态`，可关联、可重试；
- 未来迁移 eRPC 时，这个包装层可以 1:1 映射为 function call。

### 6.2 帧格式

**请求（上位机 → MCU）**

```
0xFA | 0xA5 | SEQ | ACT | ARGS...
```

| 字段 | 长度 | 说明 |
|---|---|---|
| `0xFA` | 1 | 帧头（`writeBytes` 已自动加） |
| `0xA5` | 1 | 单动作包装命令（当前未使用，建议保留） |
| `SEQ` | 1 | 0-255 循环，上位机自增；用于 ACK 关联 |
| `ACT` | 1 | 动作 ID（见 6.3） |
| `ARGS` | 0-N | 小端；f32 / i16 / u8；总长 ≤ 32 B |

**应答（MCU → 上位机）**

```
0xFF | LEN | 0x6F | SEQ_ECHO | ACT_ECHO | STATUS | DETAIL?
LEN = 1 + 3 + len(DETAIL)   # 最少 4
```

| 字段 | 长度 | 说明 |
|---|---|---|
| `SEQ_ECHO` | 1 | 回显请求的 `SEQ` |
| `ACT_ECHO` | 1 | 被确认的动作 ID（回显 `0xA5` 的 `ACT`） |
| `STATUS` | 1 | 0=OK, 1=BUSY, 2=INVALID_ARG, 3=TIMEOUT, 4=HW_FAULT, 5=ESTOP, 6=REJECTED |
| `DETAIL` | 0-N | 可选，故障码/实际高度/耗时 |

> 兼容：旧车若只会回 `FF 01 6F`（无 body），上位机按 `ACT=0, SEQ=0, STATUS=OK` 处理，仅用于旧命令。

### 6.3 动作 ID 草案

> 编号按机构分块，留扩展空间；`0x00-0x0F` 系统，`0x10-0x1F` 底盘，`0x20-0x2F` 升降，`0x30-0x3F` 夹爪，`0x40-0x4F` 矛/末端，`0x50-0x5F` 爬升腿，`0x60-0x6F` 兼容旧单步，`0x70-0x7F` 事件。

| ACT | 名称 | ARGS | 说明 |
|---|---|---|---|
| `0x01` | `HEARTBEAT` | 无 | 上位机 1 Hz；MCU 超时 500 ms 可降级/停车 |
| `0x02` | `RESET_FAULT` | 无 | 清故障、回 idle |
| `0x03` | `ESTOP` | 无 | 最高优先级，立即断使能 |
| `0x10` | `CHASSIS_MOVE_TO` | f32 x, f32 y, f32 yaw | 与 `0xA1` 同义；建议新代码走包装 |
| `0x11` | `CHASSIS_VELOCITY` | f32 vx, f32 vy, f32 wz | 与 `0xBB` 同义 |
| `0x12` | `CHASSIS_STOP` | 无 | 减速停车 |
| `0x20` | `LIFT_TO` | i16 mm | 绝对高度（以回零点为 0） |
| `0x21` | `LIFT_STEP` | i8 dir, u16 mm | 相对升降；兼容 ±200/±400 |
| `0x22` | `LIFT_HOME` | 无 | 找下限位/回零 |
| `0x30` | `GRIPPER_OPEN` | 无 | 张开到限位 |
| `0x31` | `GRIPPER_CLOSE` | 无 | 闭合到限位/夹紧 |
| `0x32` | `GRIPPER_SET` | u8 percent | 0-100% 开度；需要位置闭环 |
| `0x33` | `GRIPPER_HOLD` | u8 force? | 保压/保持（可选） |
| `0x40` | `SPEAR_TAKE` | 无 | 兼容旧 `0xA2` |
| `0x41` | `SPEAR_MATE` | 无 | 兼容旧 `0xA3` |
| `0x42` | `SPEAR_OFFSET` | i16 left_mm, i16 up_mm | 兼容旧 `0xB1` 的 float 语义（改整数更省） |
| `0x50` | `CLIMB_ARM` | u8 front, u8 rear | 兼容旧 `0xB1` 腿档位；档位表待定 |
| `0x60` | `STEP_UP_200` | 无 | 兼容旧 `0x64` |
| `0x61` | `STEP_DOWN_200` | 无 | 兼容旧 `0x65` |
| `0x62` | `STEP_UP_400` | 无 | 兼容旧 `0x66` |
| `0x63` | `STEP_DOWN_400` | 无 | 兼容旧 `0x67` |
| `0x64` | `TURN_CCW_90` | 无 | 兼容旧 `0x68` |
| `0x65` | `TURN_CW_90` | 无 | 兼容旧 `0x69` |
| `0x70` | `FAULT_EVENT` | u8 code, u8 detail | MCU 主动上报（非 ACK） |
| `0x71` | `MECH_STATE` | u8 gripper, i16 lift, u8 arm | MCU 主动上报（1-10 Hz） |

> 为什么用 `0xA5` 包装而不是直接复用 `0x64-0x6D`：旧码没有参数、没有 SEQ，且 `0x69` 与上行重试码冲突；包装层可以一次性解决"参数 + 序号 + 状态"，而旧码保持只读兼容。

### 6.4 交互示例

**夹爪闭合**

```
TX: FA A5 07 31
RX: FF 04 6F 07 31 00          # OK
RX: FF 04 6F 07 31 04          # 硬件故障
```

**升到 350 mm**

```
TX: FA A5 08 20 5E 01          # i16 350 = 0x015E
RX: FF 04 6F 08 20 00
```

**急停**

```
TX: FA A5 09 03
RX: FF 04 6F 09 03 00          # 注意：ESTOP 的 ACK 允许省略（执行即生效）
```

### 6.5 超时与重试建议

| 动作 | 默认超时 | 重试 | 说明 |
|---|---|---|---|
| 夹爪开/合 | 800 ms | 1 次 | 堵转要回 `HW_FAULT`，不要静默超时 |
| 升降到高度 | 3 s | 1 次 | 带限位检测；超时回 `TIMEOUT` |
| 升降回零 | 5 s | 1 次 | 回零失败必须报错 |
| 取矛/对矛 | 3 s | 1 次 | 允许 `BUSY` 后继续等 |
| 底盘到点 | 10 s | 0 次 | 由 `device/chassis` 轮询位姿，不靠 ACK |
| 心跳 | 1 s 周期 | — | 连续 3 次丢失 → 决策层进安全态 |

---

## 7. 建议协议 v1（新车 / eRPC）

如果新车直接走 eRPC，v0 的 `ACT/SEQ/STATUS` 可以原样映射（下面是示意，具体语法以 `erpcgen` 为准）：

```idl
enum Status { OK = 0, BUSY = 1, INVALID_ARG = 2, TIMEOUT = 3,
              HW_FAULT = 4, ESTOP = 5, REJECTED = 6 };

struct ActionResult { Status status; int32 detail; };

interface Robot {
    ActionResult chassisMoveTo(float x, float y, float yaw);
    ActionResult chassisVelocity(float vx, float vy, float wz);
    ActionResult chassisStop();
    ActionResult liftTo(int16 mm);
    ActionResult liftHome();
    ActionResult gripperOpen();
    ActionResult gripperClose();
    ActionResult gripperSet(uint8 percent);
    ActionResult spearTake();
    ActionResult spearMate();
    ActionResult climbArm(uint8 front, uint8 rear);
    ActionResult stopAll();
    oneway void odom(float x, float y, float yaw);      // 100 Hz 通知
    oneway void sick(int32 left_mm, int32 right_mm);    // 通知
    oneway void fault(uint8 code, uint8 detail);        // 事件
};
```

eRPC 自带 6 字节帧头（CRC16 + 长度），**不要再套 `0xFA/0xFF`**；`erpcgen` 生成的 Python client 是阻塞的，要放到独立线程/`run_in_executor`。

---

## 8. 上位机侧的封装形态（目标）

### 8.1 分层

```
tasks/ (决策层)
   └─ await robot.gripper.close()
          │
device/action_client.py
   ├─ seq = next_seq()
   ├─ pending[(ACT, seq)] = Future()
   ├─ transport.send(protocol.encode_action(...))
   ├─ await asyncio.wait_for(future, timeout)
   └─ 返回 ActionResult(status, detail)
          │
protocol/frames.py
   └─ encode/decode（唯一知道 0xFA/0xA5/0x6F 的地方）
          │
transport/serial_transport.py   （未来：erpc_transport.py）
```

### 8.2 目标 API

```python
class ActionResult:
    status: Status
    detail: int
    elapsed: float

class Gripper:
    async def open(self, timeout=0.8) -> ActionResult: ...
    async def close(self, timeout=0.8) -> ActionResult: ...
    async def set(self, percent: int, timeout=0.8) -> ActionResult: ...

class Lift:
    async def to(self, mm: int, timeout=3.0) -> ActionResult: ...
    async def step(self, mm: int, timeout=3.0) -> ActionResult: ...
    async def home(self, timeout=5.0) -> ActionResult: ...

class Chassis:
    async def move_to(self, x: float, y: float, yaw: float, timeout=10.0) -> ActionResult: ...
    async def velocity(self, vx: float, vy: float, wz: float) -> None: ...
    async def stop(self) -> None: ...
```

决策层写法：

```python
await robot.chassis.move_to(2.0, 4.2, 0.0)
await robot.lift.to(400)
await robot.gripper.close()
```

**不允许**再出现 `writeBytes(b'\x...')`、`sleep(1)` 代替 ACK。

### 8.3 回调注册形态

```python
# protocol 层只暴露语义事件
transport.on_frame(protocol.decode)     # bytes -> Message
action_client.on_message(message)       # ACK -> pending Future
state.on_odom(...)                      # 遥测 -> 共享状态
```

`globalCallback.py` 这种"一个函数解析三种协议"的写法不再保留。

---

## 9. 给电控组负责人的确认清单

> 建议按这张表逐项确认；确认后回填"结论"列，再改代码。

| # | 问题 | 需要电控确认 | 结论 |
|---|---|---|---|
| 1 | 新车 MCU 型号、UART 数量、波特率 | 是否仍为 921600 8N1 | |
| 2 | 是否接受 `0xA5` 单动作包装 + `0x6F` 带 `ACT/SEQ/STATUS` 的 ACK | 这是新协议的核心 | |
| 3 | 旧车是否必须保持 `FA CODE BODY` 不变 | 若是，新命令只走 `0xA5` | |
| 4 | 夹爪是开关量、PWM、位置闭环还是力控 | 决定 `GRIPPER_SET` 的 payload | |
| 5 | 升降是否有绝对编码器 / 限位 / 回零 | 决定 `LIFT_TO` 是否可行 | |
| 6 | 升降行程与最大速度 | 决定超时与限幅 | |
| 7 | 矛机构是"取"和"对"两个动作，还是连续流程 | 决定 `SPEAR_TAKE/MATE` | |
| 8 | 爬升腿档位表（旧 `0xB1` 的 0/1/2/4/5/8/10 含义） | 恢复旧协议需要 | |
| 9 | 急停是硬件回路还是软件命令 | 决定 `ESTOP` 语义 | |
| 10 | 心跳/看门狗周期与失联行为 | 建议 1 Hz / 500 ms / 停车断使能 | |
| 11 | 故障码表 | 至少：堵转、超时、限位、过流、通信丢失 | |
| 12 | 是否计划上 eRPC；MCU Flash/RAM/UART 带宽 | 决定 v1 时间表 | |
| 13 | 上行遥测是否允许加 CRC/长度 | 旧车可先不加，新车建议加 | |
| 14 | `0xA0` 位姿回传是否保留 | 当前 100 Hz | |
| 15 | `0xB1` 冲突如何分配（矛偏移 vs 爬升腿） | 建议矛偏移改 `0x42` | |

---

## 10. 落地顺序（与架构审计的阶段对应）

1. **先定表**：本文件第 6/9 节确认后，把 opcode/ACT/STATUS 写进 `protocol/opcodes.py`，成为唯一真源。
2. **再修 ACK**：实现 `ActionClient`（`(ACT, seq)` pending + `Future` + 超时），用 mock transport 单测。
3. **再封装单动作**：`device/gripper.py`、`device/lift.py`、`device/spear.py`，每个方法只调用 `ActionClient`。
4. **最后接决策**：`tasks/` 状态机只调用 `device/*`；入口不再出现字节。
5. **旧车验证**：先只用 `A0/A1/BB/AA/B3/B2` 保证底盘和定位；新机构动作在台架上用 `0xA5` 验证，不和旧车协议混发。

---

## 11. 附：当前代码里可直接复用的片段

| 片段 | 位置 | 复用方式 |
|---|---|---|
| 串口异步收发 + 重连 | `core/my_serial.py` | 保留，抽出 `_parser_loop` 到 `protocol/frames.py` |
| 串口子进程桥接 | `core/serial_node.py` | 保留，未来换 eRPC 时替换 |
| ROS 话题 ↔ 字节 | `core/ros_bridge_node.py` | 保留，去掉协议判断，只做话题转发 |
| 2D 位姿/SE3 | `Lib/odomVec.py` | 直接复用 |
| `turn_to_bytes` | `Lib/bytes.py` | 作为 `protocol` 的底层编码工具，但要明确 int 宽度 |
| 位姿融合 | `core/tf_manager.py` | 拆出 `state/pose.py` + `fusion/` |
| 旧单动作码 | `071fe4c^:app/actions.py`、`app/climb_manager.py`、`core/zone2_model/zone2_encoder.py` | 只作为协议参考，不直接恢复代码 |
