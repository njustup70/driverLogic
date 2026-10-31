#!/usr/bin/env python3
"""R2n 单车入口：只启动主逻辑，用于速度发送/底盘联调。

说明：
- 不拉雷达、相机、foxglove、rosbag，减少联调时的失败点。
- 串口和波特率目前写在 MAIN/R2n_Main.py 里：/dev/serial_ch340 @ 921600。
- 仓库根目录按容器约定放在 ~/ros2_ws（src 在 ~/ros2_ws/src）。

用法：
    ros2 launch rc_bringup R2n.launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    # 容器里仓库根是 ~/ros2_ws；直接从源码目录 launch 时用 launch/../.. 回退。
    ws_src = os.path.expanduser("~/ros2_ws/src")
    source_src = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    src_path = ws_src if os.path.isdir(os.path.join(ws_src, "MainLogic")) else source_src

    r2n_main = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            (
                f"export PYTHONPATH={src_path}:$PYTHONPATH; "
                "export PYTHONUNBUFFERED=1; "
                "exec python3 -m MainLogic.Main "
                "--main-module R2n_Main --main-func async_main"
            ),
        ],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([r2n_main])
