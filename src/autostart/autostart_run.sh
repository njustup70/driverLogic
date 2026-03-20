#!/usr/bin/env bash
# 宿主机启动逻辑中心
# 这个脚本负责读取 YAML 配置并对应启动容器

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"

CONFIG_FILE="$DIR/autostart.yaml"

# 开启 lo 口的多播支持 (部分容器通信需要)
echo "[Autostart] 正在开启 lo 口多播支持..."
sudo ip link set lo multicast on

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

with open('$CONFIG_FILE', 'r') as f:
    config = yaml.safe_load(f)

# 自动获取宿主机非 root 用户路径 (针对 /home/XXX/...)
def get_real_home():
    path_parts = os.path.abspath('$DIR').split('/')
    if len(path_parts) > 2 and path_parts[1] == 'home':
        return f"/home/{path_parts[2]}"
    return os.path.expanduser('~')

REAL_HOME = get_real_home()

def expand_path(path):
    if not path: return path
    return path.replace('~', REAL_HOME)

# --- 配置日志文件 ---
LOG_PATH = expand_path(config.get('log_file', 'autostart.log'))
log_dir = os.path.dirname(LOG_PATH)
if log_dir and not os.path.exists(log_dir):
    os.makedirs(log_dir, exist_ok=True)

# 每次运行脚本时，先清空（覆盖）历史日志
with open(LOG_PATH, 'w') as lf:
    pass

def write_to_log(msg):
    # 去掉颜色代码再写入文件
    import re
    clean_msg = re.sub(r'\033\[[0-9;]*m', '', msg)
    with open(LOG_PATH, 'a') as lf:
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        lf.write(f"[{timestamp}] {clean_msg}\n")

log_hint(f"加载配置文件: $CONFIG_FILE")
log_hint(f"日志将同步写入: {LOG_PATH}")

def log_info(msg):
    full_msg = f"{YELLOW}[Autostart] {msg}{RESET}"
    print(full_msg)
    write_to_log(msg)

def log_success(msg):
    full_msg = f"{GREEN}[Autostart] {msg}{RESET}"
    print(full_msg)
    write_to_log(msg)

def log_error(msg):
    full_msg = f"{RED}[Autostart] ❌ {msg}{RESET}"
    print(full_msg)
    write_to_log(f"❌ {msg}")

def log_hint(msg):
    full_msg = f"{BLUE}[Autostart] ℹ️ {msg}{RESET}"
    print(full_msg)
    write_to_log(f"ℹ️ {msg}")

def run_cmd(cmd):
    try:
        # 将命令执行结果也捕获并记录到日志（可选，这里为了简洁只记录关键状态）
        result = subprocess.run(cmd, shell=True, check=True, capture_output=True, text=True)
        if result.stdout: write_to_log(f"CMD Output: {result.stdout.strip()}")
        return True
    except subprocess.CalledProcessError as e:
        log_error(f"执行失败: {e}")
        if e.stderr: write_to_log(f"CMD Error: {e.stderr.strip()}")
        return False
    except Exception as e:
        log_error(f"发生未知错误: {e}")
        return False

# 获取命令行参数
import sys
FORCE_RUN = "--force" in sys.argv

# 1. 路径预检 (无论是否开启自动启动，都执行预检，方便手动调试)
containers = config.get('containers', [])
log_hint(f"共发现 {len(containers)} 个待处理项，正在进行路径校验...")

path_errors = 0
for item in containers:
    name = item.get('name', 'Unknown')
    compose_dir = expand_path(item.get('compose_dir'))
    
    if compose_dir:
        if os.path.exists(compose_dir):
            log_success(f"✅ 路径检查通过 [{name}]: {compose_dir}")
        else:
            log_error(f"❌ 路径不存在 [{name}]: {compose_dir}")
            path_errors += 1
    else:
        log_error(f"⚠️ 未配置路径 [{name}]")
        path_errors += 1

if path_errors > 0:
    log_info(f"注意：共有 {path_errors} 个路径检查未通过。")

# 2 & 3. 检查容器开关并遍历启动容器
if config.get('enabled', True) or FORCE_RUN:
    if FORCE_RUN:
        log_info("🚀 检测到 --force 参数，正在强制执行启动流程...")

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
else:
    print("")
    log_info("⚠️ 容器自动启动已关闭 (enabled: false)。已跳过容器拉起。")
    log_hint("提示: 若要强制启动容器，请手动运行: ./autostart_run.sh --force")

# 4. 独立拉起 Sunshine 进程
sunshine_enabled = config.get('sunshine_enabled', False)
if sunshine_enabled or FORCE_RUN:
    print(f"\n{GREEN}========================================{RESET}")
    log_info("正在拉起项目: Sunshine")
    print(f"{GREEN}========================================{RESET}")
    try:
        # 使用 Popen 将 sunshine 放入后台运行，避免阻塞主脚本的退出
        subprocess.Popen("nohup sunshine > /tmp/autostart_sunshine.log 2>&1 &", shell=True)
        log_success("Sunshine 启动指令已完成 (后台运行)。")
    except Exception as e:
        log_error(f"Sunshine 启动报错: {e}")
else:
    print("")
    log_hint("Sunshine 自动启动配置为关闭 (sunshine_enabled: false)，已跳过。")

print("")
log_success("✅ 所有启动流程已完成。")
EOF
