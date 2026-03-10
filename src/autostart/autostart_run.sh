#!/usr/bin/env bash
# 宿主机启动逻辑中心
# 这个脚本负责读取 YAML 配置并对应启动容器

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"

CONFIG_FILE="$DIR/autostart.yaml"

# 使用 Python 解析 YAML 并带颜色输出
python3 <<EOF
import yaml
import os
import subprocess
import time

# 颜色定义
RED = "\033[1;31m"      # 停止/错误
YELLOW = "\033[1;33m"   # 警告/执行中
GREEN = "\033[1;32m"    # 通行/成功
BLUE = "\033[1;34m"     # 提示/辅助信息
RESET = "\033[0m"

def log_info(msg):
    print(f"{YELLOW}[Autostart] {msg}{RESET}")

def log_success(msg):
    print(f"{GREEN}[Autostart] {msg}{RESET}")

def log_error(msg):
    print(f"{RED}[Autostart] ❌ {msg}{RESET}")

def log_hint(msg):
    print(f"{BLUE}[Autostart] ℹ️ {msg}{RESET}")

def run_cmd(cmd):
    try:
        subprocess.run(cmd, shell=True, check=True)
        return True
    except subprocess.CalledProcessError as e:
        log_error(f"执行失败: {e}")
        return False
    except Exception as e:
        log_error(f"发生未知错误: {e}")
        return False

# 读取配置
if not os.path.exists('$CONFIG_FILE'):
    log_error(f"配置文件不存在: $CONFIG_FILE")
    exit(1)

log_hint(f"加载配置文件: $CONFIG_FILE")

with open('$CONFIG_FILE', 'r') as f:
    config = yaml.safe_load(f)

# 自动获取宿主机非 root 用户路径 (针对 /home/XXX/...)
def get_real_home():
    # 获取脚本所在绝对路径，如 /home/yutou/njustup70/...
    path_parts = os.path.abspath('$DIR').split('/')
    if len(path_parts) > 2 and path_parts[1] == 'home':
        return f"/home/{path_parts[2]}"
    return os.path.expanduser('~')

REAL_HOME = get_real_home()

def expand_path(path):
    if not path: return path
    return path.replace('~', REAL_HOME)

# 1. 检查总开关
if not config.get('enabled', True):
    log_info("⚠️ 自动启动已关闭 (enabled: false)，跳过执行。")
    exit(0)

# 2. 遍历并启动容器
containers = config.get('containers', [])
log_hint(f"共发现 {len(containers)} 个待启动项。")

for item in containers:
    name = item.get('name', 'Unknown')
    compose_dir = expand_path(item.get('compose_dir'))

    print(f"\n{GREEN}========================================{RESET}")
    log_info(f"正在拉起项目: {name}")
    print(f"{GREEN}========================================{RESET}")
    
    # 优先启动 Compose 目录
    if compose_dir and os.path.exists(compose_dir):
        log_hint(f"工作目录: {compose_dir}")
        log_info(f"正在执行 Docker Compose...")
        if run_cmd(f"cd {compose_dir} && docker compose up -d"):
            log_success(f"{name} 启动指令已完成。")
    else:
        log_error(f"找不到路径: {compose_dir}")
        log_hint(f"尝试通过名称直接启动容器: {name}")
        if run_cmd(f"docker start {name}"):
            log_success(f"容器 {name} 已拉起。")
    
    time.sleep(1)

print("")
log_success("✅ 所有启动流程已完成。")
EOF
