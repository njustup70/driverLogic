# driverLogic

ROBOCON 战队主逻辑库（2027 赛季）。

## 文档

- 飞书：`算法文档 / driverLogic仓库`（比赛相关、对外）
- 仓库内（以这里为准，和代码同步）：
  - [`docs/00-project-context.md`](docs/00-project-context.md)：背景、长期目标、架构方向、约定
  - [`docs/10-architecture-audit.md`](docs/10-architecture-audit.md)：现状结构、代码欠缺、迁移路线
  - [`docs/20-single-action-protocol.md`](docs/20-single-action-protocol.md)：单动作回调清单、协议现状、v0 草案、电控确认表

## 快速开始

```bash
export PYTHONPATH=$PWD/src:$PYTHONPATH
python -m MainLogic.Main --main-module R2n_Main --main-func async_main
```

Docker：

```bash
cd docker
docker compose -f R2n_compose.yml up          # 或 R2n_odin.yml / test-compose.yml
```

## 当前状态

- 基线 `b2e43a2`，`driverLogic_2027` 分支。
- 仅 `R2n_Main` 可导入运行（速度测试）；其余入口引用了已删除模块，详见审计文档。
- 通信层未来可能切换 eRPC，但当前仍为自定义串口帧；架构上已要求业务层不直接拼字节。

## 目录

```
src/MainLogic/   主逻辑包（core / MAIN / Lib）
src/rc_bringup/  launch 启动文件
src/my_driver/   传感器/底盘驱动
src/perception/  定位/SLAM 脚本
docker/          运行环境 compose
docs/            架构与协议文档
```
