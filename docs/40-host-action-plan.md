# 40 上位机动作控制现状与补齐计划

> 目标：在不改 RPC、不改旧串口帧的前提下，判断当前 `MainLogic` 是否足以控制去年那台车做单动作/动作链，并列出上位机还缺什么、要和电控对接什么。
> 电控侧证据见 [`30-2026-electronics-control-notes.md`](30-2026-electronics-control-notes.md)。

---

## 1. 结论

- **作为“骨架”：已经完备。** 入口、串口、ROS 桥、odom/SICK 解析、速度发送、位姿融合都在，速度测试也过了。
- **作为“控制整车/动作链”：不完备。** 目前上位机只有“发速度 + 收 odom/SICK + 发 SLAM/场地/矛偏移”，没有动作协议、动作客户端、机构动作模块、动作链和 mock。
- **最关键的缺口不是目录，而是“动作接口”**：去年 MCU 没把夹爪/升降/泵/阀/腿暴露给上位机，所以上位机现在无法直接调用这些动作。必须先和电控定一个动作服务（旧帧 `FA + FuncCode + payload` 即可），上位机侧才能继续写。

---

## 2. 现状完整性矩阵

| 能力 | 现有文件 | 状态 |
|---|---|---|
| 启动/进程 | `Main.py`、`MAIN/R2n_Main.py` | ✅ 速度测试已停用，改为发一次取块整段时序 |
| 串口收发 | `link/my_serial.py`、`link/serial_node.py` | ✅ |
| ROS 话题桥 | `link/ros_bridge_node.py` | ✅ |
| 字节打包 | `link/bytes.py` | ✅ 基础够用 |
| 输入解析 | `link/globalCallback.py` | ⚠️ 只有 odom/SICK/纠偏；无动作 ACK |
| 底盘速度 | `app/Move.py` | ✅ `send_test_speed`、`continuous_move` |
| 底盘到点 | `app/Move.py` | ⚠️ 有 `A1` 发送，但 MCU 没实现 `0xA1` |
| 夹爪 | `app/Gripper.py` | ⚠️ 已改为“整段取块”固定延时动作；命令码占位 `A5 43` |
| 升降/腿 | 无 | ❌ |
| 泵/阀/取块 | 无 | ❌ |
| 位姿融合 | `core/tf_manager.py` | ✅ 有，但和 `link` 仍有耦合 |
| 动作协议 | 无 | ❌ |
| 动作客户端（等完成/超时/重试） | 无 | ❌ |
| 动作链 | 无 | ❌ |
| 机构状态 | 无 | ❌ |
| 配置 | 无 | ❌ 串口/超时/ACT 散落 |
| mock/单动作调试 | 无 | ❌ |

一句话：**“能发一条速度”已经有了，“能发一个动作并知道它完成”还没有。**

---

## 3. 上位机还缺什么

### P0：没有这些就无法做动作链

1. **动作协议 `link/protocol.py`**
   - ACT 编号表；
   - 请求编码：`FA A5 ACT ARGS...`；
   - 应答解码：`FF LEN 6F ACT STATUS`；
   - `ActionResult(status, detail, elapsed)`。
2. **动作客户端 `link/action_client.py`**
   - 发命令；
   - 挂 `Future`，按 `ACT`（以后加 `SEQ`）等 ACK；
   - 超时、重试、取消；
   - 没有 ACK 的动作支持“固定延时/外部条件”模式。
3. **ACK 回调**
   - 把 `0x6F` 解析出来，交给 `ActionClient`；
   - 找不到等待者/重复回包/过期回包只记日志，不崩。
4. **机构动作模块 `app/`**
   - `Chassis`：速度、停止、到点（`BB` + 位姿闭环，或等 MCU 加 `MoveAt`）；
   - `Jaw`：矛头夹爪开/合、`SetPos(x,y)`、`PitchUp/Down`；
   - `VacuumClaw`：`OpenSnap/CloseSnap/CloseSnap_Hard`、`StretchOut/Back`、`PitchDown/Origin`；
   - `Pump`/`PRV`：`OpenPump/ClosePump`、`OpenPRV/ClosePRV`；
   - `LiftLeg`：前后腿 `SetFrontPos/SetBackPos`、慢速到位；
   - `GetBlock`：`Store/Put`（如果保留 MCU 复合动作）。
5. **mock 下位机 `tools/mock_mcu.py`**
   - 不接车也能测动作链：收到 `A5` 后按 ACT 回 `6F OK/TIMEOUT`；
   - 可以故意丢包/延迟/回错误，测超时和重试。
6. **第一条动作链 `MAIN/sequences.py`**
   - 先用普通 `async/await` 串起来，不急着上框架。

### P1：有了这些才好维护

7. **机构状态 `state/mechanism.py`**
   - 最近一次动作、是否 busy、故障码、腿/夹爪位置；
   - 如果每个动作都返回 `ActionResult`，第一版可以先不做完整 state。
8. **配置 `config/robot.yaml`**
   - 串口、波特率、超时、ACT 编号、动作限幅；
   - 不要再写死在入口和模块里。
9. **日志**
   - 每个动作：ACT、参数、耗时、结果；
   - 出错能看出是哪一步。

### P2：安全/工程化（你说后面补）

10. 心跳/看门狗、急停、断线停车；
11. 单元测试、协议编解码测试、CI；
12. 文档与代码同步。

---

## 4. 建议的目标结构（最小版）

```text
MainLogic/
├── MAIN/
│   ├── R2n_Main.py          # 入口：只组装
│   └── sequences.py         # 动作链：pick_block / score / ...
├── app/
│   ├── Chassis.py           # 速度、停止、到点
│   ├── Jaw.py               # 矛头夹爪 + SetPos/Pitch
│   ├── VacuumClaw.py        # snap/stretch/pitch
│   ├── Pump.py              # 泵 + 泄压阀（也可并入 VacuumClaw）
│   ├── LiftLeg.py           # 前后腿
│   └── GetBlock.py          # Store/Put（如果 MCU 保留复合）
├── link/
│   ├── protocol.py          # ACT、帧、ActionResult
│   ├── action_client.py     # 发送/等待/超时/重试
│   ├── globalCallback.py    # ACK/odom/SICK 分发
│   └── ...
├── core/
│   └── tf_manager.py        # 位姿融合（以后拆 state/）
├── state/
│   └── mechanism.py         # 可选，第一版可先不做
├── config/
│   └── robot.yaml
└── tools/
    ├── mock_mcu.py
    └── action_cli.py        # 单动作调试按钮
```

---

## 5. 动作控制的两级模型（关键建议）

不要把去年 MCU 的每一个电机步骤都搬到上位机。建议分两级：

### 5.1 复合动作（留在 MCU）

适合“时序紧、依赖传感器、已经调好”的流程：

- `PICK_BLOCK_GROUND`：取地上的块（关 PRV、抬腿、抬升、翻爪、前进、合爪、开泵、再前进、夹紧、后退、摩擦带、伸出、收回、开爪、关泵、开 PRV、存块）。
- `STORE_BLOCK`：`GetBlock::Store()`。
- `PUT_BLOCK`：`GetBlock::Put()`。
- `TAKE_SPEAR` / `MATE_SPEAR`：矛头相关。

上位机只发一个 ACT，等一个 ACK。
**好处**：复用去年已经调好的时序，风险最低。

### 5.2 原子动作（逐步开放）

适合调试、微调、组合：

- `JAW_OPEN/CLOSE`
- `VAC_SNAP_OPEN/CLOSE/CLOSE_HARD`
- `PUMP_ON/OFF`、`PRV_ON/OFF`
- `LEG_FRONT_SET/BACK_SET`
- `JAW_SET_POS`、`JAW_PITCH_UP/DOWN`
- `CHASSIS_STOP`、`CHASSIS_MOVE_AT/ROTATE_AT`（如果 MCU 加）

上位机可以用原子动作做“单步调试”，也可以在做复合动作前先摆好姿态。

### 5.3 上位机负责什么

- 决定“去哪、取哪个、放哪、失败怎么办”；
- 调用复合/原子动作；
- 处理超时、重试、换桩、换区；
- 不关心电机 PID、编码器、泵的时序。

---

## 6. 动作链怎么写

### 第一版：普通 `async/await`

```python
async def fetch_one_block():
    await chassis.move_to(2.0, 4.2, 0.0)
    await actions.pick_block_ground()      # MCU 复合动作，等 ACK
    await chassis.move_to(1.0, 2.0, 1.57)
    await actions.store_block()            # MCU 复合动作
```

简单、直观，足够跑通第一条链。

### 第二版：小 `Sequence` 辅助（需要时再加）

```python
await run_steps([
    Step("move_to", chassis.move_to(2.0, 4.2, 0.0), timeout=10),
    Step("pick", actions.pick_block_ground(), timeout=20, retries=1),
    Step("store", actions.store_block(), timeout=10),
])
```

- 每一步有名字、超时、重试、失败处理；
- 日志按步骤打印；
- 不需要一上来就写状态机。

### 状态机放哪

- 只用于“换区/决策”的分支，不放 `app/`，放在 `MAIN/zone_decision.py`（之前确认的 A 方案）；
- 动作链本身用 `async` 顺序 + 条件判断即可。

---

## 7. 需要和电控对接的清单

| # | 事项 | 要确认/实现什么 |
|---|---|---|
| 1 | 完整 `HostPC.hpp` / FuncCode 表 | 仓库里没提交，需要电控给全量枚举和保留码 |
| 2 | 动作清单与粒度 | 哪些是 MCU 复合动作，哪些开放原子动作 |
| 3 | 动作帧 | `FA A5 ACT ARGS...` 是否接受；参数宽度/字节序 |
| 4 | ACK | `FF LEN 6F ACT STATUS`；先 `ACT+STATUS`，以后加 `SEQ` |
| 5 | ACK 语义 | `OK` 是“已接受”还是“已完成”；`BUSY` 怎么回；超时/故障码 |
| 6 | 反馈 | 至少动作完成；最好有 fault/腿位置/夹爪状态/泵状态 |
| 7 | 单位与符号 | mm 还是 m；腿前后正负；Jaw x/y 正负；snap 位置 |
| 8 | `0xA1` / `MoveAt` | 是否补 MCU handler，还是上位机用 `0xBB` + 位姿闭环 |
| 9 | `0x78` payload | 修 `payload[2]` 越界；确认 field/area/start_at 三个字节 |
| 10 | 安全 | 急停/心跳/断线停车；后面补，但协议要预留 |
| 11 | 旧车机构清单 | 去年车实际有 Jaw/VacuumClaw/Lift_Leg/GetBlock 中的哪些 |
| 12 | 串口/波特率/频率 | `/dev/serial_ch340` @ 921600 是否继续；动作帧发送频率 |

---

## 8. 建议实施顺序

1. **定动作表**：和电控确认复合/原子动作、ACT 编号、ACK 格式（先不写代码）。
2. **写 `link/protocol.py` + `tools/mock_mcu.py`**：不接车，先把编解码和 ACK 跑通。
3. **写 `link/action_client.py`**：超时/重试/等待；用 mock 测丢包、延迟、错误。
4. **写 `app/Chassis.py`**：速度、停止；到点先用 `0xBB` + 位姿闭环。
5. **写 `app/Jaw.py` / `app/VacuumClaw.py` / `app/LiftLeg.py`**：先原子动作，接一台机构调一台。
6. **写 `app/GetBlock.py` / 复合动作**：把去年调好的时序包成 ACT。
7. **写 `MAIN/sequences.py`**：第一条动作链（取块→存块）。
8. **补 `config/`、日志、状态**；最后再做安全/心跳/急停。

---

## 9. 验收标准（上位机侧）

- [ ] 不接车，`mock_mcu` 能跑通一条完整动作链。
- [ ] 每个动作都有：ACT、参数、超时、重试、日志、返回值。
- [ ] 丢包/延迟/错误回包时，动作链能停在安全状态，不会卡死。
- [ ] 真实车上，单动作按钮（`tools/action_cli.py`）能逐个动。
- [ ] 一条动作链（例如“去取块→存块”）能重复跑 10 次不卡死。
- [ ] 动作代码里不再出现 `0xNN`、`writeBytes`、`/dev/tty`。
