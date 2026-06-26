# MainLogic AGENTS Guide

本文档面向进入 `/home/hjw/logic2/driverLogic/src/MainLogic` 的协作者与智能体，按当前仓库中的源码状态整理结构、运行链路、接口约定与风险点。

如果文档与代码冲突，以代码为准。

## 1. 项目快照

- `MainLogic` 是放在 `driverLogic/src` 下的 Python 包，依赖 `PYTHONPATH` 导入，不是完整的 `ament_python` 打包结构。
- 当前代码大体分成五层：
  - 入口与进程管理：`Main.py`
  - ROS2 / 串口桥接：`core/ros_bridge_node.py`、`core/serial_node.py`、`core/my_serial.py`
  - 位姿与 TF 融合：`core/tf_manager.py`
  - 上层任务流程：`MAIN/`、`app/`
  - 工具与数学基础：`Lib/`、`core/nav/`
- 基础设施层已经比较成型；业务动作层和部分入口还处在迁移中，存在明显运行风险。
- 当前目录下没有 `R3_Main.py`、`tests/`、`app/TFManager.py` 这类旧文档里提到的文件。

## 2. 运行方式

先设置导入路径：

```bash
export PYTHONPATH=/home/hjw/logic2/driverLogic/src:$PYTHONPATH
```

默认入口由 `Main.py` 决定：

```bash
python -m MainLogic.Main --main-module R2_Main --main-func async_main
```

也可以显式切换到导航演示入口：

```bash
python -m MainLogic.Main --main-module testMain --main-func async_main
```

补充说明：

- `Main.py` 默认读取环境变量 `MAIN_MODULE` / `MAIN_FUNC`，否则落到 `R2_Main.async_main`。
- 运行环境除了 ROS2 相关依赖外，还会用到 `pyserial`、`numpy`、`scipy`、`do_mpc`、`casadi`，可视化路径相关代码还会触发 `foxglove` 导入。
- 入口文件中写死了串口设备与波特率：
  - `MAIN/R2_Main.py`：`/dev/ttyUSB0`，`921600`
  - `MAIN/testMain.py`：`/dev/ttyUSB0`，`115200`
  - `MAIN/R1_Main.py`：`/dev/ttyACM0`，`115200`

## 3. 主运行链路

当前主链路按下面的顺序工作：

1. `Main.py` 设置多进程启动方式为 `spawn`，初始化 `rclpy`，创建后台 `asyncio` 事件循环线程。
2. `Main.py` 调用 `RosBridgeNodeInstance.init()`，把全局单例初始化成主进程 ROS2 节点，并注册到 `SingleThreadedExecutor`。
3. `Main.py` 动态导入 `MainLogic.MAIN.<main_module>`，把 `async_main()` 投递到后台 `asyncio` 线程。
4. `MAIN/*.py` 入口协程调用 `core.serial_node.start_serial_process()` 拉起串口子进程。
5. 串口子进程里的 `SerialNode` 使用 `AsyncSerial_t` 读写真实串口，并桥接 `serial_rx` / `serial_tx`。
6. 主进程里的 `RosBridgeNodeInstance` 订阅 `serial_rx`，收到字节后分发给已注册的串口回调。
7. `globalCallback.py` 解析 MCU / SICK / 爬墙 / QR / spear 等输入，并把结果写入 `TFManagerInstance`、`ClimbManagerInstance` 或业务状态。
8. `core/tf_manager.py::tf_update_loop()` 周期融合 `odom`、SLAM TF 和 SICK 修正，持续发布 TF、回传融合位姿并更新共享状态。
9. 业务层和导航层调用 `move_to()`、`climb()`、`mpc_loop()`、`observer_update()` 等上层动作。

## 4. 目录与职责

### `Main.py`

- 项目总入口。
- 负责解析 `--main-module` 与 `--main-func`。
- 动态加载 `MainLogic.MAIN.<module>`。
- 初始化 `RosBridgeNodeInstance`。
- 创建后台 `asyncio` 线程，主线程交给 ROS2 executor。
- 退出时停止事件循环、终止子进程并关闭 ROS2。

### `MAIN/`

- `R2_Main.py`
  - 当前默认入口。
  - 启动串口子进程，注册 `mcu_transmit_callback`、`climb_type_callback`、`qr_detection_result`、`spear_status`。
  - 配置 `TFManagerInstance.register_tf_chain(...)`，启动 `tf_update_loop()`。
  - 目前主体逻辑只有一次 `await check_types()`，其后的比赛动作基本都还停留在注释阶段。
- `testMain.py`
  - 更像导航 / 控制演示入口。
  - 启动串口子进程、TF 更新、`mpc_loop()` 和 `observer_update()`。
  - 直接构造一条硬编码路径并设置 MPC 目标点，然后常驻。
- `R1_Main.py`
  - 更老的入口样例。
  - 结构接近旧版代码，和当前 `TFManager`、动作层接口不完全对齐。

### `core/`

- `ros_bridge_node.py`
  - 主进程 ROS2 桥接节点。
  - 管理 `serial_tx` 发布与 `serial_rx` 订阅。
  - 维护发布器字典、订阅器字典、串口回调列表。
  - 提供动态 / 静态 TF 发布接口。
  - `writeBytes(data)` 会自动在业务数据前补一个 `0xFA` 帧头。
  - `publish_ros2(topic, msg)` 在话题未注册时会按 `type(msg)` 自动建 publisher。
- `serial_node.py`
  - 串口子进程里的 ROS2 节点。
  - 把串口读到的字节发布到 `serial_rx`。
  - 把 `serial_tx` 写回串口。
  - `start_serial_process()` 会做“已在运行则直接返回”的保护。
- `my_serial.py`
  - 真实串口异步封装。
  - 自己开线程和事件循环，做串口重连、缓存与回调分发。
  - 当前读取逻辑不是按协议切帧，而是等 `in_waiting` 稳定后把当前缓存整块读出。
- `tf_manager.py`
  - 位姿融合核心。
  - 管理 `map`、`slam_init`、`odom`、`base_link`、`camera_init`、`aft_mapped` 等坐标关系。
  - `register_tf_chain(sick2Base, map2BaseInit, laser2Base)` 会注册硬编码外参并发布 `map -> slam_init` 静态 TF。
  - `odom_10ms()` 发布 `odom -> base_link`，融合得到 `map -> base_link`，回传 `A0`，并发布 `/state/base_link_odom`。
  - `slam_100ms()` 从 TF buffer 读取 `camera_init -> aft_mapped`，推导 `slam_init -> odom`。
  - `move_to()` 通过 `A1` 指令给下位机发目标位姿，并轮询 `baseLinkOdom` 直到到达或超时。
- `nav/mpc.py`
  - 基于 `do_mpc` 的路径跟随控制器。
  - `MPCPathFollowerInstance` 当前实例按 `omni` 模型创建。
  - `mpc_loop()` 从 `TFManagerInstance.baseLinkOdom` 取当前姿态，计算控制量，发布 `/state/mpc_control`，并通过 `BB` 串口命令同步下发。
- `nav/observer.py`
  - 基于简化卡尔曼模型，从位姿估计车体系速度。
  - `observer_update()` 发布 `/state/velocity_observation`。
- `nav/path_generate.py`
  - 样条路径生成与按弧长插值工具，被 MPC 使用。

### `app/`

- `climb_manager.py`
  - 梅林攀爬流程控制器。
  - 负责从格位坐标计算攀爬目标点、朝向和腿部高度编码。
  - 对外暴露 `climb()`、`climb_move()`、`climb_arm_act()` 等协程接口。
  - 当前源码里能看出完整意图，但内部状态类型和回调数据格式并不完全一致，后文有风险说明。
- `actions.py`
  - 放置取矛、对矛、二维码触发相关动作。
  - 当前实现与 `ros_bridge_node.publish_ros2()`、`CheckActions.check_finish()` 的接口并未完全对齐。
- `meilin_climb.py`
  - 内容与 `actions.py` 高度重复。
  - 当前代码树里没有其他模块引用它，更像历史残留副本。

### `Lib/`

- `AsyncTools.py`
  - 项目里最重要的共享状态基础设施。
  - `AsyncVariable[T]` 支持 `await var` 等待下一次更新，也支持通过 `.value` 或属性代理写入。
  - `async_property(factory)` 设计目标是“类属性描述器”，适合写在类体里，不适合直接放在模块级做全局变量。
- `odomVec.py`
  - 二维位姿对象 `Odom`。
  - 实现了位姿组合、逆变换、欧式距离、四元数转换、ROS TF 互转等功能。
- `bytes.py`
  - 把 `bool` / `int` / `float` / `list` / `tuple` 序列化成字节流。
- `CheckActions.py`
  - 动作完成等待辅助函数 `check_finish(action_type, timeout=500)`。
- `Visual.py`
  - 一部分是 Foxglove JSON 日志封装。
  - 当前主链路更常用的是 `PathVisualInstance`，它通过 ROS `nav_msgs/Path` 发布轨迹和目标路径。

### `globalCallback.py`

- 串口与 ROS 输入汇聚层。
- `mcu_transmit_callback(data)`
  - 识别 14 字节 `FF AA` 里程计帧。
  - 识别 20 字节 SICK 帧。
  - 识别 4 字节 `FF B2 B2 FF` 纠偏触发帧。
- `climb_type_callback(data)`
  - 识别 4 字节 `FF B1` 攀爬状态帧，并写入 `ClimbManagerInstance`。
- `ros_qr_callback(msg)`
  - 期望 `msg.data` 是 8 位十六进制字符串，按每 2 bit 解码成 12 个状态值。
- `spear_callback(msg)`
  - 期望 `msg.data` 提供 spear 排序状态，但当前对接的数据容器实现并不正确。

## 5. 关键单例与共享状态

- `RosBridgeNodeInstance`
  - 定义在 `core/ros_bridge_node.py`。
  - 模块导入时就已经实例化，`Main.py` 再调用 `.init()` 完成 ROS2 节点初始化。
- `SerialProcess`
  - 定义在 `core/serial_node.py`。
  - 保存串口子进程句柄。
- `TFManagerInstance`
  - 定义在 `core/tf_manager.py`。
  - 维护全局 TF、融合位姿与导航动作接口。
- `ClimbManagerInstance`
  - 定义在 `app/climb_manager.py`。
  - 保存攀爬相关状态与动作流程。
- `QRRecogInstance`
  - 定义在 `app/actions.py`。
  - 保存二维码识别结果字符串。
- `PathVisualInstance`
  - 定义在 `Lib/Visual.py`。
  - 发布 `/state/base_link_path`、`/state/target_path` 等路径话题。

需要特别注意：

- `AsyncVariable` 才是当前项目里真正稳定的“可等待共享状态容器”。
- `async_property()` 只有在类体中作为描述器使用时才符合设计意图。

## 6. ROS2 接口

当前代码里实际出现的主要话题如下：

- `serial_tx`：`UInt8MultiArray`，主进程发给串口子进程。
- `serial_rx`：`UInt8MultiArray`，串口子进程发给主进程。
- `qr_detection_result`：`String`，视觉模块回传二维码结果。
- `spear_status`：`UInt8MultiArray`，spear 状态输入。
- `/update_exec_req`：预期是 `String`，用于通知视觉或执行模块切换任务。
- `location`：`String`，在 `R1_Main.py` / `R2_Main.py` 中注册，但当前未见实际使用。
- `/state/base_link_odom`：`geometry_msgs.msg.Vector3`，由 `TFManager` 发布融合位姿。
- `/state/mpc_control`：`geometry_msgs.msg.Vector3`，由 `mpc_loop()` 发布控制量。
- `/state/velocity_observation`：`geometry_msgs.msg.Vector3Stamped`，由 `observer_update()` 发布观测速度。
- `/state/base_link_path`：`nav_msgs.msg.Path`，由 `PathVisualInstance` 维护轨迹。
- `/state/target_path`：`nav_msgs.msg.Path`，由 `MPCPathFollowerInstance.set_path()` 发布目标路径。

TF 相关主要坐标系：

- `map`
- `slam_init`
- `odom`
- `base_link`
- `camera_init`
- `aft_mapped`

## 7. 串口协议概览

### 下位机上报帧

- 里程计帧
  - 前缀：`FF AA`
  - 总长度：14 字节
  - 负载：`<fff`，表示 `x, y, yaw`
- SICK 帧
  - 总长度：20 字节
  - 通过“首尾相等 + 校验和”规则做合法性检查
  - 最终由 `globalCallback.mcu_transmit_callback()` 提取出侧向距离并写入 `TFManagerInstance.sick()`
- 纠偏触发帧
  - 固定格式：`FF B2 B2 FF`
  - 触发 `TFManagerInstance.apply_sick_initial_yaw_correction()`
- 攀爬状态帧
  - 前缀：`FF B1`
  - 当前实现要求总长度恰好 4 字节
  - 第 3、4 字节分别被写入 `climb_type` 与 `climb_arm`

### 上位机下发帧

所有业务层发送最终都建议走 `RosBridgeNodeInstance.writeBytes(data)`，因为它会自动补一个 `0xFA` 帧头。

当前代码里明确出现的命令包括：

- `A0`
  - `tf_manager.py` 周期性回传融合后的 `map -> base_link`
- `A1`
  - `move_to()` 发送目标位姿
- `A2`
  - `take_spear_head()` 触发取矛
- `A3`
  - `build_spear()` 触发对矛
- `B1`
  - `climb_manager.py` 用于腿部抬升 / 回收控制
- `BB`
  - `mpc_loop()` 下发车体系速度控制量 `[vx, vy, vw]`

## 8. 推荐阅读顺序

如果只是想先抓住主干，建议按这个顺序看：

1. `Main.py`
2. `core/ros_bridge_node.py`
3. `core/serial_node.py`
4. `core/my_serial.py`
5. `core/tf_manager.py`
6. `globalCallback.py`
7. `MAIN/R2_Main.py` 或 `MAIN/testMain.py`
8. `app/climb_manager.py`
9. `core/nav/mpc.py` 与 `core/nav/observer.py`

如果你要改比赛业务，优先看 `R2_Main.py + climb_manager.py + actions.py`。

如果你要改导航控制，优先看 `testMain.py + tf_manager.py + core/nav/*`。

## 9. 协作约定

- 新代码统一使用 `MainLogic.*` 绝对导入。
- `writeBytes(data)` 已经自动补 `0xFA`，上层不要再手动补帧头。
- `publish_ros2(topic, msg)` 需要传 ROS 消息实例；不要把 Python 原生 `str`、`list`、`tuple` 直接当作 ROS 消息发出去。
- `register_tf_chain()` 当前必须显式传入三个 `Odom` 外参：`sick2Base`、`map2BaseInit`、`laser2Base`。
- 外参并不集中在配置文件里，而是散落在各个入口模块中；改车体安装参数时要同步检查 `R2_Main.py` 与 `testMain.py`。
- 需要共享异步状态时，优先用 `AsyncVariable`，或者把 `async_property()` 放进类体里使用；不要在模块级直接声明 `async_property(...)` 当全局变量。
- 如果你要改串口协议，通常需要联动检查：
  - `globalCallback.py`
  - `core/tf_manager.py`
  - `app/climb_manager.py`
  - `core/nav/mpc.py`

## 10. 当前高风险问题

下面这些问题是按当前源码静态阅读得到的，修改前建议先确认：

- `README.md` 仍然写着 `slamMain`、`asyncMain.py` 以及 `app/TFManager.py`，和当前代码树不一致。
- 当前目录下没有旧文档提到的 `R3_Main.py`、`tests/mock_integration_harness.py` 等文件；如果看到旧说明，基本可以直接忽略。
- `R2_Main.py` 虽然是默认入口，但它会调用 `await check_types()`；而 `ClimbManager.check_type()` 内部调用 `_trans_type(current_type)` 时参数数量不对，按源码判断这里大概率会直接报错。
- `app/climb_manager.py` 内部状态格式前后不一致：
  - 回调里把 `climb_type`、`climb_arm` 写成整数
  - `_trans_climb_type()` 却把输入当作 `bytes`
  - `check_arm_state()` 又把 `climb_arm` 当成可下标访问的两元素状态
  - 这意味着当前攀爬主流程还不能直接视为可运行
- `app/actions.py` 与 `app/meilin_climb.py` 基本是重复实现，而且两者都把 `async_property(...)` 直接放在模块级使用；这不符合 `AsyncTools.py` 的设计方式。
- `globalCallback.spear_callback()` 试图写 `order_spear.value = msg.data`，但 `order_spear` 不是 `AsyncVariable`，这条链路按当前实现并不成立。
- `Lib/CheckActions.py` 的 `serial_action_finish = async_property(bytes)` 也是同类问题，因此 `check_finish()` 这套等待逻辑当前并不可靠。
- `actions.py` / `meilin_climb.py` 调用 `check_finish()` 时没有传 `action_type`，与 `CheckActions.check_finish(action_type, timeout=500)` 的签名不匹配。
- `actions.py` / `meilin_climb.py` 用 `publish_ros2('/update_exec_req', msg)` 发送的是 Python 字符串，不是 `std_msgs.msg.String`；如果该话题已按 `String` 注册，会触发类型不匹配。
- `core/my_serial.py` 当前不会按协议拆帧，只会把“当前串口缓存的一整块字节”回调出去；而 `globalCallback.py` 大量逻辑都要求“每次回调恰好是一帧固定长度数据”，因此连续帧合包时会被直接丢弃。
- `R1_Main.py` 仍然调用了不带参数的 `TFManagerInstance.register_tf_chain()`，和当前 `tf_manager.py` 的函数签名不一致。
- `testMain.py` 所依赖的导航模块会引入 `do_mpc`、`casadi`、`scipy`、`foxglove`；环境不完整时，即使基础桥接层没问题，也会在导入或运行阶段失败。

## 11. 一句话总结

`MainLogic` 现在是一套“`Main.py` 统一入口 + `core/` 提供桥接与位姿融合 + `MAIN/` / `app/` 负责任务流程 + `core/nav/` 负责导航控制”的 ROS2 Python 主控包；基础骨架已经形成，但动作层和部分入口仍有明显迁移痕迹，接手时要优先核对接口一致性。
