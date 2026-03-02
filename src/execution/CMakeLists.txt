cmake_minimum_required(VERSION 3.8)
project(execution)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# 安装 Python 可执行脚本
install(
  PROGRAMS
    scripts/doing_exec.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()