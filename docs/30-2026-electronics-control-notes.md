# 30 2026 R2 电控仓库对照（夹爪/动作控制方法）

> 仓库：<https://github.com/njustup70/KM2026_R2_Action>（注意：不是 `...Actiond`）
> 基线：`6f2907b`（2026-10-22，`main`）
> 目的：确认去年电控怎么控制夹爪/机构，判断上位机能否“复刻”这些动作。

---

## 1. 一句话结论

去年 R2 的**主体决策在 MCU 上**：`SemiAutoGr`、`ChallengeGraph`、`ArenaPK_Graph` 等状态图直接调用 `Jaw`、`VacuumClaw`、`GetBlock`、`Lift_Leg`、`Chassis` 的动作函数。

上位机（`driverLogic`）只负责：

- 发 SLAM 位姿（`0xA0`）
- 发底盘速度（`0xBB`）
- 发红蓝场/一三区（`0x78`，有 bug）
- 发矛头视觉偏移（`0xB1`）
- 收 odom（`0xAA`）、SICK（`0xB3`）、红蓝场选择等

**上位机没有夹爪/升降/泵/阀/腿的动作命令，也没有动作完成 ACK。**
所以要“通过上位机复刻电控的控制”，必须在 MCU 侧新增一个**动作服务**（action server），把这些已有的动作原语暴露给上位机。

---

## 2. 旧 HostPC 实际注册了哪些上位机命令

证据：`Regist(...)` 调用点。

| FuncCode | 值 | 注册位置 | 作用 |
|---|---|---|---|
| `ctrl_spd` | `0xBB` | `ReactorLibs/Users/Chassis/Chassis.cpp:52` | 上位机发底盘速度 |
| `slam_pos` | `0xA0`（旧上位机对应值） | `ReactorLibs/Users/CommCenter/CommCenter.cpp:47` | 上位机发 SLAM 位姿 |
| `field_area_feedback` | `0x78`（旧上位机对应值） | `ReactorLibs/Users/CommCenter/CommCenter.cpp:49` | 上位机发红蓝场/一三区 |
| `dock_err` | `0xB1` | `ReactorLibs/Users/Jaw/Jaw.cpp:51` | 上位机发矛头视觉偏移 |

> `HostPC.hpp` 没有提交到仓库里（`git ls-files` 查不到），所以 `slam_pos` / `field_area_feedback` 的具体枚举值是按旧上位机代码反推的；`ctrl_spd=0xBB`、`dock_err=0xB1` 在 MCU 代码里有直接证据。

MCU → 上位机方向（`HostPC` 的发送函数）：

- `SendOdom`（旧上位机按 `0xAA` 收）
- `SendSickData`（旧上位机按 `0xB3` 收）
- `SendFieldChoice`（红蓝场/一三区）
- `SendKFSData`
- `SendSlamCorrectionCmd`（旧上位机按 `0xB2` 收；代码里被注释）
- `RestartSLAM`

**没有** `SendActionAck`、`SendMechState`、`SendFault` 这类动作反馈。

---

## 3. “夹爪”不是一种机构

2026 R2 至少有三种末端/取块相关机构：

### 3.1 `Jaw`：矛头夹爪，继电器开关

文件：`ReactorLibs/Users/Jaw/Jaw.cpp`

```cpp
void Jaw::OpenClaw()  { relay.On(); }
void Jaw::CloseClaw() { relay.Off(); }
```

- 继电器初始化：`Pin relay_pin = {'I', 2}; relay.Init(relay_pin, Relay::HIGH_ON);`
- 纯开关量，**没有位置/力反馈**。
- `Jaw` 还有三个电机（水平、竖直、Pitch），用于矛头对准：
  - `SetPos(x_mm, y_mm)`
  - `PitchUp()` / `PitchDown()`
  - `SetTrackMode(true)` + 上位机 `0xB1` 偏移，做视觉随动。
- 旧上位机只参与了 `0xB1` 偏移；`OpenClaw/CloseClaw` 是 MCU 状态图直接调的。

### 3.2 `VacuumClaw`：取块夹爪/吸盘，电机 + 泵 + 阀

文件：`ReactorLibs/Users/VacuumClaw/VacuumClaw.cpp`

```cpp
void VacuumClaw::OpenSnap()       { snap_pos = origin - 102000; }
void VacuumClaw::CloseSnap()      { snap_pos = origin - 102000 + 212560; }
void VacuumClaw::CloseSnap_Hard() { snap_pos = origin - 102000 + 272040; }
void VacuumClaw::OpenSnapMini()   { snap_pos = origin + 130000; }
```

- `snap` 是位置控制电机，`OpenSnap/CloseSnap` 只是设定目标位置。
- 真正取块还要配合：
  - `GetBlock::OpenPump()` / `ClosePump()`（负压泵，继电器）
  - `GetBlock::OpenPRV()` / `ClosePRV()`（泄压阀，继电器）
  - `StretchOut/StretchBack`、`PitchDown/PitchOrigin`、`ReturnAndStore`
- 旧 `Area3_Act::FetchBlockOnTheGround()` 的完整顺序（节选）：
  1. `ClosePRV()`
  2. `leg.SetFrontPos/SetBackPos(-100)`，等腿到位
  3. `get_block.LiftUp()`
  4. `vac_claw.PitchDown()`，等 1.5s
  5. 底盘前进
  6. `vac_claw.CloseSnap()`，等 1s
  7. `get_block.OpenPump()`
  8. 底盘再前进一小段
  9. `vac_claw.CloseSnap_Hard()`
  10. 底盘后退
  11. `_StartBelt()`、`StretchSlow(...)`、`PitchOrigin()`、收回
  12. `OpenSnap()` + `ClosePump()` + `OpenPRV()`
  13. `get_block.Store()`

所以 **`gripper.open` 对 VacuumClaw 不是一个动作**，至少是 `snap_open`、`pump_off`、`prv_on` 三个原语，外加前后的伸缩/翻转/收块。

### 3.3 `GetBlock::motor_3508_claw`：声明了但没用

`Get_Block.hpp:54` 有 `MotorDJI motor_3508_claw; // 取块爪机构`，但全仓库只有这一处声明，没有任何调用。**不是当前有效机构。**

---

## 4. MCU 侧可以复用的动作原语

| 机构 | 原语 | 语义 |
|---|---|---|
| `Jaw` | `OpenClaw` / `CloseClaw` | 矛头夹爪继电器开/关 |
| `Jaw` | `SetPos(x_mm,y_mm)` | 矛头水平/竖直位置 |
| `Jaw` | `PitchUp` / `PitchDown` | 矛头翻转 |
| `Jaw` | `SetTrackMode(true)` | 视觉随动模式 |
| `VacuumClaw` | `OpenSnap` / `CloseSnap` / `CloseSnap_Hard` | 取块爪开/合/夹紧 |
| `VacuumClaw` | `StretchOut` / `StretchBack` / `StretchSlow` | 伸缩爪 |
| `VacuumClaw` | `PitchDown` / `PitchOrigin` / `PitchBack` | 翻转 |
| `GetBlock` | `OpenPump` / `ClosePump` | 负压泵 |
| `GetBlock` | `OpenPRV` / `ClosePRV` | 泄压阀 |
| `GetBlock` | `Store` / `Put` | 存块/放块（MCU 内部 FSM） |
| `GetBlock` | `LiftUp` / `LiftDown` / `Push` / `PushBack` | 抬升/送块 |
| `Lift_Leg` | `SetFrontPos` / `SetBackPos`（mm） | 前后腿位置 |
| `Lift_Leg` | `SetFrontPosSlow` / `SetBackPosSlow` | 慢速到位 |
| `Lift_Leg` | `F_Down_200/400`、`B_Down_200/400` | 固定档位 |
| `Chassis` | `Move(Vec3)` | 速度（已有 `0xBB`） |
| `Chassis` | `MoveAt(Vec2)` / `RotateAt(yaw)` | 到点/转向（**没有上位机命令**） |

---

## 5. 对“上位机复刻”的影响

### 5.1 不能直接复刻的部分

- **夹爪开/合**：旧 HostPC 没有对应 FuncCode。
- **升降/腿**：没有对应 FuncCode。
- **泵/阀**：没有对应 FuncCode。
- **`0xA1` 到点**：`Chassis::HostCmdCallback` 只处理 `0xBB`，没有 `0xA1`。旧上位机的 `chassic_move_to` 很可能从未在 MCU 上生效。
- **动作完成 ACK**：没有。旧状态图在 MCU 内部靠 `Seq::Wait` / 电机到位判断，不经过上位机。

### 5.2 可以直接复刻的部分

- 底盘速度 `0xBB`（已通）。
- SLAM 位姿 `0xA0`（已通）。
- 红蓝场/一三区 `0x78`（已通，但 payload 长度有 bug）。
- 矛头视觉偏移 `0xB1`（已通）。

### 5.3 已发现的反馈/协议 bug

1. `field_area_feedback`：`CommCenter::FieldToggleRxCb` 检查 `payload_len < 2`，却读 `payload[2]`；旧上位机只发 2 字节（field + area_retry），`start_at` 越界。
2. `0xB1` 同时被旧上位机用于“矛头偏移”，又被旧攀爬逻辑用于“腿高度”；MCU 这里只按 `dock_err` 处理。
3. `0xA1` 没有 MCU handler（见上）。
4. `0x6F` 动作完成帧没有 MCU 发送端；旧上位机的 `action_callback` 也是断的。
5. `worklist.md` 记录：等爪子关闭用 `WaitUntil` 时，块太大可能一直堵转卡死，后来改成纯延时（仅三区 `GetGroundBlock`）。所以复刻时不要只靠“电机到位”判断，必须加超时/重试。

---

## 6. 建议给电控的动作服务（讨论稿）

目标：不改旧帧格式，先在 MCU 上加一个**动作分发入口**，把第 4 节的已有原语暴露给上位机；以后再升级 ACT+SEQ。

### 6.1 帧

```text
上位机 → MCU：FA A5 ACT ARGS...
MCU → 上位机：FF LEN 6F ACT STATUS
```

- `0xA5` 是一个新的 `HostPC::FuncCode`（或直接叫 `action`），payload 第一字节是 `ACT`，后面是参数。
- 暂时不带 `SEQ`；等 MCU 侧稳定后再在 `ACT` 前加一字节 `SEQ`，ACK 回显。
- 旧 `ctrl_spd/slam_pos/field_area_feedback/dock_err` 保持不动。

### 6.2 建议的动作 ID

| ACT | 动作 | 参数 | MCU 侧调用 |
|---|---|---|---|
| `0x01` | `JAW_OPEN` | 无 | `APP::jaw.OpenClaw()` |
| `0x02` | `JAW_CLOSE` | 无 | `APP::jaw.CloseClaw()` |
| `0x10` | `VAC_SNAP_OPEN` | 无 | `APP::vac_claw.OpenSnap()` |
| `0x11` | `VAC_SNAP_CLOSE` | 无 | `APP::vac_claw.CloseSnap()` |
| `0x12` | `VAC_SNAP_CLOSE_HARD` | 无 | `APP::vac_claw.CloseSnap_Hard()` |
| `0x13` | `VAC_PUMP_ON` | 无 | `APP::get_block.OpenPump()` |
| `0x14` | `VAC_PUMP_OFF` | 无 | `APP::get_block.ClosePump()` |
| `0x15` | `VAC_PRV_ON` | 无 | `APP::get_block.OpenPRV()` |
| `0x16` | `VAC_PRV_OFF` | 无 | `APP::get_block.ClosePRV()` |
| `0x20` | `LEG_FRONT_SET` | i16 mm | `APP::leg.SetFrontPos()` |
| `0x21` | `LEG_BACK_SET` | i16 mm | `APP::leg.SetBackPos()` |
| `0x22` | `LEG_BOTH_SET` | i16 front, i16 back | `SetFrontPos/SetBackPos` |
| `0x30` | `JAW_SET_POS` | i16 x_mm, i16 y_mm | `APP::jaw.SetPos()` |
| `0x31` | `JAW_PITCH_UP` | 无 | `APP::jaw.PitchUp()` |
| `0x32` | `JAW_PITCH_DOWN` | 无 | `APP::jaw.PitchDown()` |
| `0x40` | `GET_BLOCK_STORE` | 无 | `APP::get_block.Store()` |
| `0x41` | `GET_BLOCK_PUT` | 无 | `APP::get_block.Put()` |
| `0x50` | `CHASSIS_MOVE_AT` | f32 x, f32 y | `APP::chas.MoveAt()` |
| `0x51` | `CHASSIS_ROTATE_AT` | f32 yaw | `APP::chas.RotateAt()` |
| `0x52` | `CHASSIS_STOP` | 无 | `APP::chas.Move(0,0,0)` |

> 编号只是讨论稿；电控可以按机构分块，也可以一个动作一个 FuncCode。关键是**ACT 和参数要稳定**，ACK 要回 `ACT`。

### 6.3 ACK

```text
FF LEN 6F ACT STATUS
STATUS: 0=OK, 1=BUSY, 2=INVALID, 3=TIMEOUT, 4=HW_FAULT, 5=ESTOP, 6=REJECTED
```

- 先只回 `ACT + STATUS`；以后加 `SEQ`。
- 对 `JAW_OPEN/CLOSE` 这种继电器动作，MCU 可以立即回 `OK`（没有位置反馈）。
- 对 `VAC_SNAP_*`、`LEG_*`，MCU 可以等电机到位/超时后回 `OK/TIMEOUT`；但 `worklist` 已经踩过“堵转卡死”的坑，建议 MCU 内部也加超时。

---

## 7. 对 `app/Gripper.py` 的当前实现

旧帧里没有夹爪开合，所以不能直接复刻。现在的做法是：

- 把去年 `Area3::FetchBlockOnTheGround()` 当成一个**整段复合动作**；
- 上位机只发一条命令：`FA A5 43`（`0xA5` 动作入口，`0x43` 取块整段时序，均为暂定值）；
- 发出后用固定时间等待（当前 15s），返回 `True` 只表示“已发出并等完”，不代表 MCU 确认完成；
- 电控确认后只需要改 `PICK_BLOCK_GROUND_PAYLOAD`，调用方不用动。

如果以后要拆成原子动作（`snap_open`、`pump_on`、`prv_on`），再单独加函数；不要用一个 `open()` 掩盖复合动作。

---

## 8. 待确认

1. 今年新车的“夹爪”是 `Jaw` 那种继电器，还是 `VacuumClaw` 那种电机+泵？
2. 电控是否愿意加一个 `0xA5 + ACT` 的动作分发入口？
3. ACK 是先 `ACT + STATUS`，还是直接上 `SEQ + ACT + STATUS`？
4. `0xA1` 到点是否要补 MCU handler，还是上位机继续用 `0xBB` + 位姿闭环？
5. `Lift_Leg` 的 `SetFrontPos/SetBackPos` 是否允许上位机直接设 mm，还是只允许固定档位？

---

## 9. 证据索引

- `ReactorLibs/Users/Chassis/Chassis.cpp:52`：注册 `ctrl_spd`
- `ReactorLibs/Users/Chassis/Chassis.cpp:403-418`：只处理 `0xBB`
- `ReactorLibs/Users/CommCenter/CommCenter.cpp:47-49`：注册 `slam_pos`、`field_area_feedback`
- `ReactorLibs/Users/Jaw/Jaw.cpp:50-51`：`SOF=0xFA, Func=0xB1`，注册 `dock_err`
- `ReactorLibs/Users/Jaw/Jaw.cpp:286-294`：`OpenClaw`/`CloseClaw` 继电器
- `ReactorLibs/Users/VacuumClaw/VacuumClaw.cpp:402-434`：`OpenSnap/CloseSnap/CloseSnap_Hard`
- `ReactorLibs/Users/Get_Block/Get_Block.cpp:510-532`：泵/阀
- `ReactorLibs/Users/SemiAutoGr/Area3_Act.cpp:13-110`：真空爪取块完整序列
- `ReactorLibs/Users/Lift_Leg/Lift_Leg.hpp:40-70`：腿的位置接口
- `worklist.md`：爪子关闭堵转、定位延迟等已知问题
