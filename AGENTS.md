# Repository Guidelines

## Project Structure

This repository is a ROS 2 workspace for hardware drivers and bringup logic.

- `src/`: ROS 2 packages (primary code)
  - `rc_bringup/`: top-level launch files (entry points)
  - `my_driver/`: sensor/driver integration
  - `perception/`, `python_pkg/`, `serial_dispose/`, `performance_test/`, `autostart/`
- `packages/`: vendored SDKs and helpers (contains `COLCON_IGNORE`; not built by `colcon`)
- `build/`, `install/`, `log/`: `colcon` artifacts (safe to delete)
- `docker/`, `.devcontainer/`: runtime + devcontainer definitions
- `bag_play/`, `rosbag_record/`: rosbag playback/recording utilities

## Build, Test, and Development Commands

```bash
# One-time (host): device permissions + udev rules (writes to /etc)
sudo ./udev_init.bash

# Build (common local workflow)
rm -rf build install log
colcon build --symlink-install --cmake-args -GNinja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=1
source install/setup.bash

# Run
ros2 launch rc_bringup driver_with_utils.launch.py
```

VS Code tasks in `.vscode/tasks.json` mirror the build/clean flow (e.g. `ROS2:编译整个项目`).

## Coding Style & Naming

- C++: format with `.clang-format` (LLVM-based, 4-space indent, `ColumnLimit: 100`, `c++20`).
- Python: keep `flake8` + `pep257` happy (tests run via `ament_flake8` / `ament_pep257`).
- ROS conventions: package names are `snake_case`; launch files are `*.launch.py`; runtime configs live in `config/`.

## Testing Guidelines

Run tests via `colcon` (primarily linters in this workspace):

```bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

If you touch hardware-facing code, include a short note on what device(s) you tested and how to reproduce (launch file + key params).

## Commit & Pull Request Guidelines

Recent history commonly uses Conventional Commit-style prefixes (e.g. `fix:`, `docs:`, `refactor:`) and may include Chinese summaries.

- Prefer: `type(scope): summary` (scope is optional), e.g. `fix(my_driver): handle ms200 reconnect`.
- PRs: describe changes, link related issues/PRs, and include logs/screenshots when behavior is visible (RViz/Foxglove/rosbag).

## Security & Configuration Notes

- `udev_init.bash` installs/removes packages and writes udev rules; review before running.
- Docker configs run privileged, mount `/dev`, and may mount the Docker socket—avoid adding secrets to images or repo files.
