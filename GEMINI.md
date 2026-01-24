# Project Context: driverLogic

## Overview
This is a ROS 2 workspace (`driverLogic`) focused on robot drivers, perception, and bringup logic. It manages various hardware sensors including LiDARs (Mid360, MS200), IMUs, and Cameras (Realsense, Orbbec).

## Architecture & Structure

### ROS 2 Packages (`src/`)
- **`rc_bringup`**: The main entry point for the robot. Contains top-level launch files.
    - Key Launch Files: `R2.launch.py`, `driver_with_utils.launch.py`.
- **`my_driver`**: Hardware interface package.
    - Handles: IMU (CH040), LiDAR (Mid360, MS200), Cameras (Realsense, Orbbec, USB).
- **`perception`**: Perception algorithms.
- **`serial_dispose`**: Handling of serial communication.
- **`performance_test`**: Testing utilities.
- **`python_pkg`**: Python-based utilities or nodes.
- **`autostart`**: Scripts for system startup/deployment.

### External Dependencies (`packages/`)
- **`librealsense`**, **`orbbecSDK`**: Vendor SDKs for cameras.
- **`protocol_lib`**: Custom protocol library (Python).
- **`ros-bridge`**: Bridges for ROS 1/ROS 2 communication or external bridges.

### Environment
- **Devcontainer**: Configured in `.devcontainer` using `docker-compose.yml`.
- **Workspace Path**: `/home/Elaina/ros2_ws` (inside container).

## Development Workflow

### Build
The project uses `colcon` for building.
```bash
colcon build --symlink-install
```

### Source Environment
After building, source the setup script:
```bash
source install/setup.bash
```

### Running
Launch files are typically found in `rc_bringup`.
```bash
ros2 launch rc_bringup R2.launch.py
# or
ros2 launch rc_bringup driver_with_utils.launch.py
```

## Documentation
- Local `README.md` indicates that documentation is primarily hosted on Feishu.
- Code style follows standard ROS 2 conventions (ament_cmake, setup.py for Python).
