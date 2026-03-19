# MainLogic

MainLogic is a Python package. All imports use absolute package paths such as `MainLogic.core.xxx`.

## Startup

Recommended startup command:

```bash
python -m MainLogic.Main --main-module slamMain --main-func async_main
```

## Environment Variable

Set `PYTHONPATH` so Python can resolve `MainLogic` from any working directory.

Example:

```bash
export PYTHONPATH=/home/Elaina/ros2_ws/src:$PYTHONPATH
```

## Package Layout

- `Main.py`: package entrypoint, initializes ROS2 and dynamically loads `MainLogic.MAIN.<module>`.
- `MAIN/`: async workflow entry modules (`asyncMain.py`, `slamMain.py`).
- `core/`: core and common business modules (`ros_bridge_node`, `serial_node`, `tf_manager`).
- `app/`: application-level behavior orchestration.
- `Lib/`: shared infrastructure and utility modules.
- `globalCallback.py`: ROS/serial callback adapters used by entry workflows.

## Core Responsibilities

- `core/ros_bridge_node.py`: main ROS2 bridge node, topic bridge, TF publish helpers.
- `core/serial_node.py`: ROS2 serial bridge process entry and node wrapper.
- `core/tf_manager.py`: shared TF fusion manager and move helper.

## Lib Responsibilities

- `Lib/AsyncTools.py`: async helper primitives (for example `async_property`).
- `Lib/odomVec.py`: odometry pose math and transform helpers.
- `Lib/bytes.py`: bytes serialization utilities for controller payloads.
- `Lib/mySerial.py`: serial communication wrapper.
- `Lib/rosSerialNode.py`: compatibility shim that forwards to `MainLogic.core.serial_node`.
- `Lib/rosBridgeNode.py`: compatibility shim that forwards to `MainLogic.core.ros_bridge_node`.
- `Lib/CheckActions.py`: action completion wait/check helpers.
- `Lib/decorder.py`: timing utility decorator (`time_print`).

## Notes

- `Lib/decorder.py` name is historical; current content is a timing decorator helper.
- `app/TFManager.py` is now a compatibility shim forwarding to `MainLogic.core.tf_manager`.
- If needed later, it can be renamed in a separate change to avoid mixed concerns.
