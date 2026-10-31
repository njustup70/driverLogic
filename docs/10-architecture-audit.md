# 10 现状结构与代码欠缺审计

> 基线：`b2e43a2`（2026-10-30，分支 `driverLogic_2027`）。
> 方法：静态阅读 + `python -m compileall` + 全历史 `git grep`（含已删除的 `app/`、`core/nav/`、`core/zone2_model/`）。
> 本文只记录事实与建议，不改业务代码；协议细节见 `20-single-action-protocol.md`。
> 本次审计同时更新了 `docs/`、根 `README.md`、`src/MainLogic/README.md`、`src/MainLogic/AGENTS.md` 与 `.gitignore`；问题清单描述的是基线 `b2e43a2` 的代码状态，不因文档更新而消失。

---

## 1. 目录与体量（当前实际存在）

```
src/MainLogic/
├── Main.py                 103 行   总入口：ROS 初始化 + asyncio 线程 + 动态加载 MAIN
├── globalCallback.py       186 行   串口/ROS 回调大杂烩（含重复 docstring 与半截 import）
├── core/
│   ├── ros_bridge_node.py  124 行   ROS2 桥接节点 + 串口收发 + TF 发布
│   ├── serial_node.py       72 行   串口子进程节点
│   ├── my_serial.py        229 行   异步串口 + 帧解析（含示例/残留代码）
│   ├── tf_manager.py       452 行   TFManager + TFOdin 两套单例（重复实现）
│   ├── Move.py             115 行   底盘动作（MPC 桩 + A1/BB 发送）
│   └── icp.py              117 行   标定脚本（cv2/matplotlib，写死路径）
├── MAIN/
│   ├── R2n_Main.py          65 行   速度测试入口（当前唯一可导入）
│   ├── R2_Main.py           83 行   引用已删除的 app/、move_to、zone2_model_api
│   ├── R1_Main.py           23 行   引用已删除的 app/climb_manager
│   ├── testMain.py         101 行   引用已删除的 core/nav、app/actions
│   └── R2_build_spear.py    37 行   引用已删除的 app/actions
└── Lib/
    ├── odomVec.py          282 行   Odom / SE3
    ├── AsyncTools.py       194 行   AsyncVariable / async_property
    ├── Visual.py           306 行   Foxglove + PathVisual（模块顶层 import foxglove）
    ├── bytes.py             38 行   turn_to_bytes
    ├── CheckActions.py      12 行   动作完成等待（模块级 async_property 误用）
    └── decorder.py          27 行   计时装饰器（拼写错误，当前无引用）
```

- 总计约 2570 行 Python；`core/tf_manager.py` 与 `Lib/Visual.py` 占了 1/3。
- 已删除但仍在被引用的目录：`app/`、`core/nav/`、`core/zone2_model/`（提交 `071fe4c` 删除，共 5122 行）。
- 没有 `tests/`、没有 `pyproject.toml`/`setup.py`、没有 Python lint/类型检查配置。

---

## 2. 当前运行链路（按代码实际行为）

```
python -m MainLogic.Main --main-module R2n_Main
        │
        ├─ multiprocessing.set_start_method('spawn')        Main.py:17
        ├─ rclpy.init()                                     Main.py:57
        ├─ 后台线程跑 asyncio loop                           Main.py:61-63
        ├─ RosBridgeNodeInstance.init() + SingleThreadedExecutor
        │       └─ 订阅 serial_rx / 发布 serial_tx          ros_bridge_node.py:19-21
        ├─ 动态 import MainLogic.MAIN.<module>               Main.py:30-38
        └─ executor.spin()（主线程）                          Main.py:86
                 │
                 └─ async_main()
                       ├─ start_serial_process() → 子进程 SerialNode
                       │       └─ AsyncSerial_t（自己开线程+loop）
                       │              └─ 串口字节 → serial_rx 话题
                       ├─ register_serial_sub(cb, code)
                       ├─ register_ros2_sub(...)
                       ├─ TFManagerInstance.register_tf_chain(...)
                       └─ asyncio.create_task(tf_update_loop())
                              ├─ 10ms: odom_10ms() → A0 回传 + /state/base_link_odom
                              └─ 100ms: slam_100ms() → 读 TF、发静态 TF、画 path
```

关键事实：

1. **串口在子进程**，主进程只通过 `serial_rx` / `serial_tx` 两个 ROS 话题与它通信；子进程用 `spawn`，不能共享内存单例。
2. **回调在 ROS executor 线程执行**，业务协程在另一个 asyncio 线程；两者靠 `AsyncVariable` 和 `run_coroutine_threadsafe` 连接。
3. **帧解析在 `my_serial._parser_loop`**：物理帧为 `0xFF | LEN | CODE | BODY`，`LEN = 1 + len(BODY)`；解析后回调收到 `0xFF + CODE + BODY`，`ros_bridge_node` 再剥掉前两字节，把 `BODY` 交给按 `CODE` 注册的回调。
4. **下行帧只有 `0xFA + CODE + BODY`**，没有长度、没有校验、没有序号（`writeBytes`，`ros_bridge_node.py:51-55`）。
5. 当前默认入口是 `R2n_Main`（`Main.py:41`），它只注册 `0xAA` 里程计并周期发 `0xBB` 速度，不建 TF 链、不跑融合。

---

## 3. 分层评估

| 层 | 现状 | 评价 |
|---|---|---|
| 入口/进程 | `Main.py` 同时管进程、ROS、事件循环、动态加载、退出 | 职责过多，但基本可用；缺生命周期抽象 |
| 传输 | `my_serial` + `serial_node` + `ros_bridge_node` 三处都懂协议 | 传输与协议耦合，替换 eRPC 会牵动三处 |
| 协议 | opcode 散落在 6 个文件，发送端只补 `0xFA` | 无单一真源，收发不对称 |
| 控制 | `Move.py` + `tf_manager` 直接 `writeBytes` | 控制层被融合层/业务层穿透 |
| 融合/状态 | `TFManager` / `TFOdin` 两套 450 行单例 | 重复、私有字段互借、初始化顺序脆弱 |
| 决策 | `app/` 删除后没有替代；`MAIN/` 里直接写流程 | 决策层实际上缺位 |
| 感知适配 | `globalCallback` 直接解析视觉/定位 | 回调巨石，算法模块难替换 |
| 工具 | `odomVec` 质量较好；`bytes`/`CheckActions` 粗糙 | 数学层可保留，动作同步机制要重写 |
| 工程化 | 无测试、无 CI、文档陈旧、提交垃圾 | 与"为后辈铺路"目标冲突 |

---

## 4. 问题清单

### P0 — 当前直接跑不通/会挂死

#### P0-01 四个入口里三个无法导入
- `R2_Main.py:7` `from MainLogic.core.tf_manager import move_to` —— `move_to` 已在 `d28f343` 被删除（提交说明："干了 tf 中的 move to，会导致 main 函数报错"）。
- `R2_Main.py:8` `MainLogic.app.climb_manager` —— `app/` 已删除。
- `R2_Main.py:68/78/80` `zone2_model_api` —— 未定义、未导入；`Odom` 也未导入（第 28-31 行直接使用）。
- `testMain.py:15/17/23` `MainLogic.core.nav.observer`、`core.nav.mpc`、`app.actions` —— 均已删除。
- `R1_Main.py:8` `MainLogic.app.climb_manager` —— 已删除。
- `R2_build_spear.py:9-15` `MainLogic.app.actions` —— 已删除。

**影响**：除 `R2n_Main` 外，`python -m MainLogic.Main --main-module ...` 全部在 import 阶段失败；`R1.launch.py` 还指向不存在的 `slamMain`。
**建议**：短期把这几个入口标记为 `WIP` 并移出可启动集合；中期按新架构重写，不要在原文件上继续打补丁。

#### P0-02 `register_serial_sub` 调用签名不一致
- 定义：`ros_bridge_node.py:46` `register_serial_sub(self, callback, code)`。
- 缺 `code` 的调用：`R1_Main.py:21-22`、`R2_Main.py:26-27/36`。
- `R2_Main.py:27/36` 同一个 `climb_type_callback` 注册两次；字典覆盖，无告警（`ros_bridge_node.py:48-50`）。

**影响**：`TypeError: missing 1 required positional argument: 'code'`；重复注册掩盖配置错误。
**建议**：`code` 改为必填并做类型校验；重复注册直接抛错；注册动作集中到一个 `register_all_callbacks()`。

#### P0-03 `globalCallback` 缺少入口引用的符号
- `R2_Main.py:27/36` `gcb.climb_type_callback` —— 已删除。
- `R2_Main.py:38`、`testMain.py:74`、`R2_build_spear.py:33` `gcb.spear_callback` —— 已删除。
- `testMain.py:70`、`R2_build_spear.py:31` `gcb.serial_action_return_callback` —— 当前只剩注释（`globalCallback.py:164-167`）。

**影响**：`AttributeError`；即使入口能导入，注册阶段也会挂。

#### P0-04 双 TF 单例 + `from ... import` 绑定，导致 `testMain` 永远等不到 SICK
- `globalCallback.py:6` `from MainLogic.core.tf_manager import TFManagerInstance`：模块加载时把名字绑定到 `TFManager` 单例。
- `testMain.py:46` `tf_manager.TFManagerInstance = tf_manager.TFOdin()`：只改了 `tf_manager` 模块里的名字。
- `gcb.sick_callback` 仍然写旧的 `TFManager`（`globalCallback.py:113`），而 `testMain.py:85` 在等 `TFOdin.sick_buffer`。
- `serial_correct_callback`（`globalCallback.py:146-147`）同时调用旧 `TFManagerInstance` 和模块级 `TFOdinInstance`，而 `testMain` 新建的 `TFOdin()` 又是第三个实例。

**影响**：SICK 数据进错对象，`testMain` 卡在 `while len(...sick_buffer)==0`；纠偏作用在错误实例上。
**建议**：取消"替换模块级单例"的用法；用显式依赖注入（`TFManager` 实例作为参数传入回调/节点），或至少提供 `set_active_manager()` 并让所有引用走同一入口。

#### P0-05 `CheckActions` 的模块级 `async_property` 根本不工作
- `CheckActions.py:5` `serial_action_finish = async_property(bytes)`：`AsyncProperty` 是描述器，必须放在类体里才会触发 `__set_name__`；模块级只是普通对象。
- `check_finish()` 用 `serial_action_finish != action_type` 比较"描述器对象 vs bytes"，永远为真；`timeout=500` 配 `sleep(0.01)` 实际约 5 s，注释却写 100 ms。
- 没有任何代码写 `serial_action_finish`。

**影响**：所有依赖 `check_finish()` 的动作（取矛、对矛）要么超时、要么静默继续。
**建议**：改用 `asyncio.Future` / `AsyncVariable[ActionResult]` + `(cmd, seq)` 关联；超时用 `asyncio.wait_for` 表达。

#### P0-06 单动作 ACK 链路是断的
- 物理帧 `FF 01 6F` 经解析器变成 `FF 6F`，但 `ros_bridge_node._serial_rx_callback`（`:57-73`）把前两字节都剥掉，交给回调的是 `b''`。
- `globalCallback.action_callback`（`:11-16`）检查 `data[0:2] == b'\xFF\x6F'`，永远不成立；且它只 `print`，没有通知任何等待者。
- `zone2_sender`（已删）原本用 `threading.Event` 等 ACK，但 `action_callback` 并没有 set 它；`R2_Main` 也没有注册 `0x6F`。
- 历史 `serial_action_return_callback` 检查 `data[0:2] == b'\xFF\xFF'`，与当前剥帧规则同样对不上。

**影响**：任何"发一条、等完成"的动作都无法闭环；只能靠固定 `sleep`。
**建议**：在协议层定义 ACK 语义（带命令字/序号/状态），在 `device/action_client.py` 统一管理 pending 表。

#### P0-07 `testMain` 的标定路径写死且依赖 `foxglove`
- `testMain.py:54` `npy_path='/home/Elaina/ros2_ws/src/MainLogic/SE_Trans.npy'`；`icp.py:115` 同样写死并 `np.save`。
- `tf_manager.py:18` 顶层 `from MainLogic.Lib.Visual import PathVisualInstance`；`Visual.py:1` 顶层 `import foxglove`。
- 也就是说：**只要 import 主逻辑，就必须装 foxglove**，即使不跑可视化。

**影响**：环境少一个包就整体起不来；换机器/换用户路径失效；标定文件被 `.gitignore` 的 `*.npy` 忽略，无法复现。
**建议**：可视化改成可选依赖（延迟 import）；标定矩阵放 `config/calibration/` 并入库（小文件）或用生成脚本 + 校验和。
另：`icp.main()` 用 5 个点做 3D 刚体 SVD，其中 4 个参考点 z≈0，滚转/俯仰几乎不可观，roll/pitch 误差会耦合进平移；建议只标 2D（x,y,yaw），或增加明显离面的标定点。

---

### P1 — 结构/协议层面，影响后续扩展

#### P1-01 `Main.py` 职责过多，事件循环写法有隐患
- `asyncioEventLoop` 先 `new_event_loop()`（`:21`）又 `get_event_loop()`（`:26`），前者被丢弃；Python 3.12+ 对 `get_event_loop()` 的隐式创建行为收紧，3.14 会直接抛 `RuntimeError`（仓库里已有 `cpython-314` 的 pycache，说明有人用 3.14 跑过）。
- `multiprocessing.set_start_method('spawn', force=True)` 在模块顶层执行（`:17`），作为库被 import 时会污染宿主进程。
- 退出时先 `loop.stop()` 再关 executor，没有 cancel 未完成任务。
**建议**：拆成 `bootstrap.py`（进程/信号/生命周期）+ `runtime.py`（ROS 节点）+ `Main.py`（薄 CLI）；显式 `asyncio.new_event_loop()` 并 `set_event_loop`。

#### P1-02 传输/协议/业务三层互相穿透
- `serial_node.py:29` 认识 `0xFA/0xB1/0xBB`（虽然只是打印分支）。
- `ros_bridge_node.py:57-73` 认识 `CODE` 的剥帧规则并做分发。
- `tf_manager.py:210/442` 融合层直接 `writeBytes(b'\xA0'...)`。
- `Move.py:96/113` 控制层直接拼 `A1/BB`。
- `globalCallback.py` 同时解析协议、写状态、发 ROS 话题、触发纠偏。
**建议**：`transport` 只搬字节；`protocol` 只编解码；`device` 只暴露语义动作；`state` 只存状态；`tasks` 只做决策。

#### P1-03 下行帧无长度/CRC/序号，与上行不对称
- 上行：`FF LEN CODE BODY`，有长度但**无 CRC**，靠固定长度/首尾相等校验。
- 下行：`FA CODE BODY`，无长度、无 CRC、无序号；MCU 只能靠命令字推断长度。
- 没有 ACK 关联字段，无法区分"上一条的完成"和"这一条的完成"。
- `Lib/bytes.turn_to_bytes` 对 `int` 宽度做隐式推断：`0-255` 编 1 字节，其余编 4 字节（`bytes.py:19-24`）。同一个"高度"字段，200 和 400 会得到不同长度，协议字段必须显式指定宽度。
**建议**：短期保持旧车兼容，只给 ACK 增加 `CMD/SEQ/STATUS`；新车主控直接上 `LEN + CRC` 或 eRPC（见协议文档）。

#### P1-04 命令码冲突/复用
- `0xA0`：当前是融合位姿回传（`tf_manager`），旧 `serial_dispose` 里却是 `cmd_vel`。
- `0xA3`：既是"对矛"下行命令（历史 `actions.py`），又被 `testMain.py:70` 注册为上行回调 code。
- `0xB1`：既是"矛头偏移"（历史 `actions.py`，当前 `globalCallback.py:68` 仍保留常量），又是"攀爬腿高度"（历史 `climb_manager`）。
- `0x69`：下行是 `TURN_CW_90`（历史 `zone2_encoder`），上行是"一/三区重试"（历史 `zone_retry_callback`）。
- `0x78`：上行是"红/蓝场"（历史 `field_color_callback`），下行是 R1_new `tf_manager` 的 `[field_color_flag, zone_retry_flag]` 配置帧。
- `0x64-0x6D` 只在已删除的 `zone2_encoder` 里有定义，当前没有任何模块引用，属于"协议孤儿"。
- `src/serial_dispose/` 还有一套并行协议：下行 `A0=cmd_vel`、`A1=location`，上行 `AA=odom`、`AB=exec_callback`；与 MainLogic 的 `A0/A1` 语义冲突，且 `src/perception/package.xml` 仍 `exec_depend` 它，但该目录没有 `package.xml`。
**建议**：建立 opcode 分配表 + 保留区间；删除/迁移冲突码；旧码只读兼容一个赛季。

#### P1-05 决策层缺位
`app/` 删除后，`statemachine.py`、`climb_manager.py`、`actions.py`、`zone2_model_api.py` 全部消失；`MAIN/` 里又直接写 `move_to`、`climb`、`zone2_model_api.send_mcu_action_frame_to_mcu`（后者根本不存在）这类细节。
**影响**：比赛流程没有可复用的状态机；后辈只能从入口文件里"考古"。
**建议**：新增 `tasks/`，用显式状态机（状态、信号、handler 表）表达比赛流程；入口只负责组装和启动。

#### P1-06 `globalCallback.py` 是巨石且文件结构损坏
- 第 1-57 行定义回调，第 59-61 行又出现一份模块 docstring，第 62-65 行第二次 import；同一文件两个"开头"。
- `meilin_map_frame_callback` 解析完只 `print`，结果丢弃（`:39-57`）；docstring 写"14 字节含 FF A2"，实际长度检查是 12 字节（桥接层已剥帧头）。
- `ros_qr_callback` 算出 `states` 后直接 `return`，没有存储/发布（`:171-186`）。
- `sick_callback` 解析 4 个 float 只用 2 个，且每个回调都 `print`（100 Hz 级别）。
**建议**：按"输入协议 → 领域事件"拆成多个 `callbacks/*.py`；解析结果写入 `state/`；日志用 `logging` + 限频。

#### P1-07 融合层私有字段被外部依赖
- `slam_100ms` 直接读 `self.rosBridge._tfBuffer`（`tf_manager.py:223`）；`TFOdin.odom_10ms` 同样（`:394`）。
- `Move.py:96` 直接读 `TFManagerInstance.rosBridge`；`PathVisual` 直接持有 `RosBridgeNodeInstance`。
**影响**：无法 mock、无法单测；换 ROS 桥接实现要改多处。
**建议**：`RosBridge` 暴露 `lookup_transform()` 等公开方法；`TFManager` 构造时注入 bridge；可视化依赖注入或事件订阅。
另：`slam_100ms` 每 100 ms 用 `StaticTransformBroadcaster` 重发 `slam_init->odom`、`map->slam_init`（`tf_manager.py:241-242`）。静态 TF 应只发一次；变化的部分应走动态 TF，否则语义混淆且浪费带宽。

#### P1-08 单例/初始化顺序脆弱
- `RosBridgeNodeInstance = rosBridgeNode()` 在 import 时创建，但 `__init__` 不调用 `super().__init__()`，真正初始化在 `init()`；忘记调用时 `_pubDict` 不存在，报错信息难懂。
- `TFManagerInstance`/`TFOdinInstance` 在 import 时创建，此时 `rosBridge` 还没注册。
- `Main.py` 先 `register_event_loop` 再动态加载入口，顺序正确但没有任何断言保护。
**建议**：用 `bootstrap()` 显式创建并注入；或改用 `lazy singleton` + `initialized` 检查，失败时给出明确提示。

#### P1-09 业务常量硬编码
| 常量 | 位置 |
|---|---|
| 串口/波特率 | `R2_Main.py:19-20`、`testMain.py:33-34`、`R1_Main.py:15-16`、`R2n_Main.py:34-35` |
| 外参 | `R2_Main.py:28-31`、`testMain.py:51-53`、`tf_manager.py:52-57` 默认值 |
| 标定文件 | `testMain.py:54`、`icp.py:115` |
| 话题名 | 分散在 `tf_manager.py`、`Move.py`、`globalCallback.py`、入口文件 |
| 投放/梅林坐标 | 历史 `climb_manager.py`（已删） |
| 场地宽 6 m / 12 cm 偏置 | `tf_manager.py:158/164`（`6.0-0.39`）、`tf_manager.py:373`（`6.0-sick_y-0.12`） |
| 环境变量配置 | `R2_Main.py:14-16` 定义了 `SERIAL_PORT`/`SERIAL_BAUDRATE`，但 `:19-20` 又硬编码，配置实际无效 |
**建议**：`config/robot.yaml` + dataclass 校验；入口只读配置，不再出现魔数。

#### P1-10 `Move.py` 名不副实
- `mpc_move_to_point`/`mpc_by_path_move`/`mpc_move_to_baseodom`/`mpc_control_loop` 全是 `🔴不可用🔴` 桩，`mpc_control_loop` 只 sleep，不计算控制量（`Move.py:13-67`）。
- `send_test_speed` 是测试函数，却被 `R2n_Main` 当作正式速度控制使用（`R2n_Main.py:45-52`）。
- `_listen_tf` 无超时、无取消，odom 断流时协程永久挂起（`Move.py:71-87`）。
**建议**：拆成 `device/chassis.py`（move_to/velocity/stop/等待到位）与 `navigation/`（MPC）；测试函数放 `tools/`。

---

### P2 — 工程化与可维护性

#### P2-01 文档与代码严重脱节
- 基线时 `src/MainLogic/README.md` 仍在讲 `slamMain`、`asyncMain.py`、`app/`、`core/nav/`、`Lib/rosSerialNode.py`、`Lib/rosBridgeNode.py`，这些都不存在（本轮已重写）。
- 基线时 `src/MainLogic/AGENTS.md` 同样按 `app/`/`nav/` 描述，并声称 `globalCallback` 有 `climb_type_callback`/`spear_callback`（已删；本轮已重写）。
- 基线时根 `README.md` 只有一句"本地项目文档全部过时了"（本轮已重写）。
**建议**：以 `docs/` 为唯一真源；`AGENTS.md` 重写为当前事实；README 给"3 分钟跑起来"的最短路径。

#### P2-02 无测试/CI/类型检查/包元数据
- 没有 `tests/`，没有 mock 串口；协议解析无法回归。
- `.github/workflows` 只有 Docker 构建和飞书机器人，没有 Python lint/test。
- 没有 `pyproject.toml`/`setup.py`，`MainLogic` 不是可安装包，只能靠 `PYTHONPATH`。
**建议**：先加 `pytest` + `tests/test_protocol.py`（纯字节，不依赖 ROS）；再加 `ruff`；最后考虑 `ament_python` 或 `pyproject`。

#### P2-03 提交垃圾
- 4 个 `.DS_Store` 被 git 跟踪：根目录、`src/`、`src/MainLogic/`、`src/MainLogic/core/`。
- `__pycache__` 目录存在（未跟踪），`.gitignore` 基线时只忽略 `*.pyc`，没有忽略目录（本轮已补）。
**建议**：`git rm --cached` 清掉；`.gitignore` 加 `.DS_Store`、`__pycache__/`。

#### P2-04 命名/拼写
`rosBridgeNode`（类名小写开头）、`chassic_move_to`、`decorder.py`、`TFOdin`/`TFManager` 混用、`R2n` 含义不明、`R2_build_spear` 与 `R2n_Main` 命名风格不一致。
**建议**：新代码统一风格；旧名保留别名过渡，不直接重命名入口（避免 launch 断链）。

#### P2-05 日志/性能
- `mcu_transmit_callback` 每帧 `print`（100 Hz）；`sick_callback` 每帧两次 `print`。
- `TFOdin.odom_10ms` 在 `mapToOdinInit != 0` 时每 10 ms `print`（`tf_manager.py:435-436`）。
- `slam_100ms` 每 100 ms 把最多 2000 个 `PoseStamped` 的整条 `nav_msgs/Path` 重新序列化发布（`Lib/Visual.py:268-305`），是隐性带宽/CPU 开销。
- `Move.send_test_speed` 每次调用都 `print`，`R2n_Main` 以 50 Hz 调用。
- 串口 921600 下，print 到 stdout（Docker + tty）会明显抖动。
**建议**：`logging` + 限频/采样；高频数据只在调试开关打开时打印。

#### P2-06 异常处理
- `tf_update_loop` 的 `except Exception as e: print(e)`（`tf_manager.py:257-258`）会吞掉 TF 断裂、类型错误等关键故障。
- `my_serial._receiver_loop` 的 `except (asyncio.TimeoutError, Exception)` 冗余，且 `continue` 后没有退避。
- `ros_qr_callback` 裸 `except: return`。
**建议**：区分"可恢复/不可恢复"；记录计数与最后错误；连续失败触发降级/告警。

#### P2-07 `my_serial.py` 库文件里塞了示例和残留
第 151-229 行是 `__main__` 示例、`mcu_transmit_callback`、`my_data_callback`，还重复 `import time,struct`；`_receiver_loop` 里局部变量叫 `bytes`，遮蔽内建类型。
**建议**：示例移到 `examples/`；库文件只保留 `AsyncSerial_t`。

#### P2-08 串口解析健壮性
- `_parser_loop` 只认 `0xFF` 帧头，遇到噪声逐字节丢弃；`LEN` 没有上限校验，损坏的 `LEN=255` 会阻塞到 257 字节。
- 无 CRC；SICK 帧靠"首尾相等 + 和校验"，里程计帧完全无校验。
- 未知 `CODE` 会广播给所有回调（`ros_bridge_node.py:70-72`），可能把错误长度的数据喂给解析器。
**建议**：加最大帧长、CRC8/CRC16、显式未知帧计数；广播行为删除。

---

## 5. 入口可运行性矩阵（静态判断）

| 入口 | import | 运行 | 主要阻断 |
|---|---|---|---|
| `R2n_Main` | ✅ | ⚠️ 仅速度测试 | 无 TF 链；`send_test_speed` 为测试函数；无 SICK |
| `R2_Main` | ❌ | ❌ | `move_to`、`app.*`、`zone2_model_api`、`Odom` |
| `testMain` | ❌ | ❌ | `core.nav.*`、`app.actions`、`serial_action_return_callback`、SICK 实例错位 |
| `R1_Main` | ❌ | ❌ | `app.climb_manager`、`register_serial_sub` 缺 code |
| `R2_build_spear` | ❌ | ❌ | `app.actions` |
| `R2.launch.py` | — | ❌ | 指向 `R2_Main` |
| `test.launch.py` | — | ❌ | 指向 `testMain` |
| `R1.launch.py` | — | ❌ | 指向不存在的 `slamMain` |
| `R2n.launch.py` | — | ⚠️ | 指向可导入的 `R2n_Main`，但只测速度 |

---

## 6. 协议现状摘要

详见 `20-single-action-protocol.md`。一句话：

- **下行**：`0xFA | CODE | BODY`，无长度/CRC/序号；当前真正在用的只有 `A0`（位姿回传）和 `BB`（速度），`A1`（到点）代码在但入口不可用。
- **上行**：`0xFF | LEN | CODE | BODY`，只有 `AA`（odom）、`B3`（SICK）、`B2`（纠偏）三条链路是通的。
- **单动作 ACK**：`0x6F` 设计存在，但链路断裂（P0-06）。
- **夹爪开合**：全历史 `git grep` 无任何实现。
- **机构升降**：只在已删除的 `zone2_encoder` 里有固定档位码 `0x64-0x67`，且只能作为 `0x91` 序列的一部分发送，不能单独调用。

---

## 7. 建议的目标结构

```
src/MainLogic/
├── Main.py                 # 只做 CLI / 启动 / 退出
├── config/
│   ├── robot.yaml          # 串口、外参、限幅、话题
│   └── schema.py           # dataclass + 校验
├── transport/              # 只搬字节
│   ├── base.py             # Transport 协议
│   ├── serial_transport.py # 现 AsyncSerial_t 演进
│   └── erpc_transport.py   # 未来
├── protocol/               # 唯一协议真源
│   ├── opcodes.py
│   ├── frames.py           # 编解码 + CRC
│   └── types.py            # ActionResult / Status
├── device/                 # 控制层：语义化单动作
│   ├── action_client.py    # seq / pending / timeout / ACK
│   ├── chassis.py
│   ├── lift.py
│   ├── gripper.py
│   └── spear.py
├── state/                  # 共享状态（位姿、机构、场地）
├── perception/             # 视觉/定位适配器
├── navigation/             # 路径/MPC（按需）
├── tasks/                  # 决策层：状态机/比赛流程
├── Lib/                    # 纯数学/工具
├── MAIN/                   # 薄入口：只组装
└── tests/                  # 协议/动作/mock transport
```

原则：

1. `MAIN/` 不允许出现 `b'\xNN'`、`0xNN`、串口路径、外参。
2. `tasks/` 不允许直接 `writeBytes`，只调用 `device/*`。
3. `device/*` 不允许知道帧格式，只调用 `protocol/*` + `transport/*`。
4. `protocol/*` 不允许 import ROS、串口、业务。
5. `transport/*` 不允许知道 opcode。

---

## 8. 迁移路线（建议，不一次性推倒）

### 阶段 0：止血（1-2 天）
- [ ] 把 `R2_Main.py`、`testMain.py`、`R1_Main.py`、`R2_build_spear.py` 标记 `WIP` 或移到 `MAIN/_wip/`，保证"能启动的入口都是能跑的"。
- [ ] 修 `register_serial_sub` 调用与 `globalCallback` 缺失符号（或直接删掉失效入口）。
- [ ] 修 `testMain` 的实例错位（或标记 WIP）。
- [ ] `.gitignore` 加 `.DS_Store`/`__pycache__/`，清 tracked 垃圾。
- [ ] `README`/`AGENTS` 标注"以 `docs/` 为准"。

### 阶段 1：协议层收口（3-5 天）
- [ ] 新建 `protocol/opcodes.py`、`frames.py`、`types.py`，把 `A0/A1/BB/AA/B3/B2` 收进来。
- [ ] `writeBytes` 改为 `send(frame)`，禁止业务层拼字节。
- [ ] 单测：给定字节流 → 解析结果；给定动作 → 期望帧。
- [ ] 加 `ActionClient`：`(cmd, seq)` pending 表 + `asyncio.Future` + 超时。
- [ ] 下行 ACK 增加 `CMD/SEQ/STATUS`（先只在新命令用，旧车兼容）。

### 阶段 2：控制层封装（1 周）
- [ ] `device/chassis.py`：`move_to/velocity/stop/wait_arrive`，替代 `Move.py`。
- [ ] `device/lift.py`、`device/gripper.py`、`device/spear.py`：每个单动作一个 async 方法。
- [ ] `state/pose.py`：把 `TFManager` 的共享状态抽出来；`TFManager` 只做融合。
- [ ] 回调拆分为 `perception/` + `state/` 写入，不再直接改业务对象。

### 阶段 3：决策层（1 周）
- [ ] `tasks/` 状态机：`IDLE → ... → FINISH`，handler 表显式化。
- [ ] 入口只做"读配置 → 建对象 → 启动状态机"。
- [ ] 用 mock transport 跑通整场流程（无硬件）。

### 阶段 4：eRPC / 工程化（按新车进度）
- [ ] `transport/erpc_transport.py` 替换串口；`protocol/*` 退化为 IDL 生成物。
- [ ] CI：`compileall` + `ruff` + `pytest`。
- [ ] 配置/标定入库；旧代码删除。

---

## 9. eRPC 适配要点（现在就要留的口子）

1. **不要双重封帧**：eRPC `FramedTransport` 自带 6 字节头（`crcHeader(u16) + messageLength(u16) + crcBody(u16)`，CRC16）。迁移后 `protocol/frames.py` 的自定义帧头要删掉，只保留 IDL 结构体。
2. **IDL 是单一真源**：`erpcgen` 生成 MCU server 与 Python client；opcode 变成 function ID，ACK 变成返回值/异常，`0x6F` 不再需要。
3. **Python client 是阻塞的**：`SerialTransport` 用 `serial.read()` 阻塞；必须放在独立线程/`run_in_executor`，再桥回 asyncio，不能直接在事件循环里调用。
4. **通知（notification）代替上行帧**：odom/SICK 这类高频数据用 eRPC notification，或保留一条自定义串口通道；不要用 RPC 请求-响应去轮询 100 Hz 数据。
5. **带宽核算**：921600 8N1 ≈ 92 KB/s；odom 100 Hz × (6B 头 + ~16B) ≈ 2.2 KB/s，SICK 同量级，余量足够；但要避免每个控制周期都做一次同步 RPC。
6. **错误模型统一**：eRPC 的 `eRPCError` / 自定义 status 映射到 `ActionResult(status)`，决策层不感知传输。
7. **保留 `Transport` 抽象**：测试用 `MockTransport` 注入预设回包，保证没有硬件也能跑状态机。

---

## 10. 验收清单（重构完成的定义）

- [ ] `python -m compileall src/MainLogic` 无 warning（除第三方）。
- [ ] 每个 `MAIN/*.py` 都能 `python -c "import ..."`。
- [ ] 业务/决策代码中 grep 不到 `0x[0-9A-Fa-f]`、`b'\x`、`/dev/tty`、`writeBytes`。
- [ ] 每个单动作都有：请求帧、ACK、超时、重试策略、单元测试。
- [ ] 拔插串口能自动恢复；连续 30 min 无内存/协程泄漏。
- [ ] `R2n_Main` 能在旧车上完成"前进/横移/自转/停止"。
- [ ] `testMain` 能在旧车上完成"odom + SICK + 融合位姿 + TF"。
- [ ] 文档中的路径、入口、话题与实际一致（用脚本校验）。
- [ ] 新人按 README 30 min 内能启动仿真/mock 并看到位姿。

---

## 11. 附录：本次审计用到的检查

```bash
# 语法/导入（只能查语法，不能查 ROS 依赖）
python3 -m compileall -q src/MainLogic

# 找出所有直接写字节的地方
grep -rnE "0x[0-9A-Fa-f]{2}|b'\\\\x" --include=*.py src/MainLogic

# 找出所有串口写入口
grep -rn "writeBytes\|serial_tx" --include=*.py src/MainLogic

# 找出已删除但仍被引用的模块
grep -rn "MainLogic\.app\|MainLogic\.core\.nav\|zone2_model" --include=*.py src/MainLogic

# 对比历史协议实现
git show 071fe4c^:src/MainLogic/app/actions.py
git show 071fe4c^:src/MainLogic/app/climb_manager.py
git show 071fe4c^:src/MainLogic/core/zone2_model/zone2_encoder.py
git show 071fe4c^:src/MainLogic/core/zone2_model/zone2_sender.py
```
