# MainLogic

2027 赛季主逻辑包。当前基线 `b2e43a2`，结构与协议说明以仓库根目录 `docs/` 为准：

- [`docs/00-project-context.md`](../../docs/00-project-context.md)：背景、长期目标、架构方向
- [`docs/10-architecture-audit.md`](../../docs/10-architecture-audit.md)：现状结构、问题清单、迁移路线
- [`docs/20-single-action-protocol.md`](../../docs/20-single-action-protocol.md)：单动作清单、协议草案

## 启动

```bash
export PYTHONPATH=<repo>/src:$PYTHONPATH
python -m MainLogic.Main --main-module R2n_Main --main-func async_main
```

`Main.py` 默认读取 `MAIN_MODULE` / `MAIN_FUNC`，否则落到 `R2n_Main.async_main`。

> 当前只有 `R2n_Main`（速度测试）可导入；`R2_Main`、`testMain`、`R1_Main`、`R2_build_spear` 引用了已删除的 `app/`、`core/nav/`，处于 WIP，详见审计文档 P0-01。

## 目录

- `Main.py`：包入口，初始化 ROS2 并动态加载 `MainLogic.MAIN.<module>`。
- `MAIN/`：入口协程（薄层，逐步收敛为"只组装"）。
- `core/`：ROS 桥接、串口、TF 融合、底盘动作。
- `Lib/`：`Odom`/`SE3`、`AsyncVariable`、字节序列化、可视化工具。
- `globalCallback.py`：串口/ROS 回调适配（待拆分）。

## 导入约定

- 统一使用 `MainLogic.*` 绝对导入。
- `writeBytes(data)` 会自动补 `0xFA` 帧头，上层不要再手动补。
- 下行协议目前只有 `0xFA | CODE | BODY`，没有长度/CRC/序号；单动作 ACK 链路未打通。
- 协议常量不要写进业务代码；后续收敛到 `protocol/`（见迁移路线）。

## 已过时的旧说明

本文件旧版提到的 `slamMain`、`asyncMain.py`、`app/`、`core/nav/`、`Lib/rosSerialNode.py`、`Lib/rosBridgeNode.py` 均不存在，已删除。
