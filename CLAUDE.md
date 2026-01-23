# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS2 (Robot Operating System 2) hardware driver repository with Docker containerization. It integrates multiple sensors (IMUs, LiDARs, cameras) and provides automated deployment and development workflows.

## Architecture

### Key Components
- **my_driver** (`src/my_driver/`): Hardware driver integration package supporting:
  - IMUs: ch040_imu, hfi_imu, wheel_imu
  - LiDARs: mid360 (Livox), ms200, robosense airy
  - Cameras: orbbec, realsense, D435i
  - Gamepad: joy control
- **rc_bringup** (`src/rc_bringup/`): Unified launch system with:
  - `driver_with_utils.launch.py`: Hardware drivers + utility nodes
  - `utils_bringup.launch.py`: Utility nodes only (rosbridge, foxglove, etc.)
  - `rosbag_with_utils.launch.py`: Bag playback + utilities
- **serial_dispose**: Serial communication handling
- **python_pkg**: Python utilities (image bridge, rosbag recording, etc.)

### Container Architecture
- **Development**: VS Code Dev Containers (`.devcontainer/`)
- **Production**: Docker Compose (`docker/docker-compose.yml`)
- **Shared volumes**: Code synchronization between host and containers
- **Hardware access**: `/dev` mounted with privileged mode

## Development Commands

### Environment Setup
```bash
# Set up device permissions (run once on host)
sudo ./udev_init.bash

# Start development container (VS Code will do this automatically)
# Or manually:
docker-compose -f .devcontainer/docker-compose.yml up
```

### Building ROS2 Packages
```bash
# Clean build artifacts
rm -rf build install

# Build all packages with colcon
colcon build --symlink-install --cmake-args -GNinja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=1

# Build specific package
colcon build --packages-select my_driver

# VS Code tasks available:
# - "ROS2:清理" (clean)
# - "ROS2:编译整个项目" (build)
# - "ROS2:清理并编译整个项目" (clean and build)
```

### Running and Testing
```bash
# Source the workspace
source install/setup.bash

# Launch hardware drivers with utilities
ros2 launch rc_bringup driver_with_utils.launch.py

# Launch utilities only
ros2 launch rc_bringup utils_bringup.launch.py

# Test specific sensor
ros2 launch my_driver mid360_bringup.launch.py
```

### Debugging
- **GDB debugging**: Use "ROS2:GDB调试" launch configuration in VS Code
- **LLDB debugging**: Use "ROS2:LLDB调试" launch configuration
- **Python debugging**: Standard debugpy configuration available

## Production Deployment

```bash
# Start production services
docker-compose -f docker/docker-compose.yml up

# Services automatically start:
# 1. driverLogic_service: Hardware drivers with utilities
# 2. voxel_service: SLAM system (depends on driver service)
```

## Key Configuration Files

### Device Management
- `udev_init.bash`: Sets udev rules for all hardware devices
- Device permissions required for: librealsense, orbbec cameras, serial devices (CH340, CP210x, etc.)

### ROS2 Configuration
- **Launch files**: `src/*/launch/` directories
- **Config files**: `src/*/config/` directories (YAML/JSON)
- **RViz configs**: `src/*/rviz2/` directories

### Container Configuration
- `.devcontainer/devcontainer.json`: VS Code development container settings
- `.devcontainer/docker-compose.yml`: Development container compose
- `docker/docker-compose.yml`: Production container compose

## Workflow Notes

1. **Development**: Use VS Code with Dev Containers for consistent environment
2. **Hardware Access**: Always run `udev_init.bash` on host before development
3. **Build System**: Uses colcon with CMake for C++ and setuptools for Python
4. **Dependencies**: Third-party SDKs in `packages/` (librealsense, orbbecSDK)
5. **Data Management**: Automatic rosbag recording with size limits (2GB max, keep last 5)
6. **Network**: Uses host networking mode for ROS2 communication

## Important Paths
- Workspace root: `/home/Elaina/ros2_ws` (inside containers)
- Source code: `src/` (mounted to container workspace)
- Build output: `build/` and `install/` directories
- Bag files: `bag_play/` for playback, automatic recording to `data/`

## Sensor-Specific Notes
- **Mid360 LiDAR**: Requires network IP configuration (see README)
- **Robosense Airy**: Requires host IP configuration matching radar
- **USB devices**: Check serial port permissions and device names
- **Cameras**: Orbbec and RealSense have separate SDK dependencies