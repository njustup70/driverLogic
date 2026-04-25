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
python3 - "$@" <<EOF
import yaml
import os
import subprocess
import time
import shutil
import pwd

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
REAL_USER = os.path.basename(REAL_HOME) if REAL_HOME.startswith('/home/') else os.getenv('SUDO_USER') or os.getenv('USER', 'root')

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
args = set(sys.argv[1:])
FORCE_RUN = "--force" in args

FORCE_CONTAINERS_ON = "--containers-on" in args
FORCE_CONTAINERS_OFF = "--containers-off" in args

FORCE_SUNSHINE_ON = "--sunshine-on" in args
FORCE_SUNSHINE_OFF = "--sunshine-off" in args

CLEANUP_CONTAINERS = "--cleanup-containers" in args
CLEANUP_CONTAINERS_OFF = "--no-cleanup-containers" in args
CLEANUP_VOLUMES = "--cleanup-volumes" in args
CLEANUP_ONLY = "--cleanup-only" in args

if FORCE_CONTAINERS_ON and FORCE_CONTAINERS_OFF:
    log_error("参数冲突: --containers-on 与 --containers-off 不能同时使用")
    sys.exit(2)

if FORCE_SUNSHINE_ON and FORCE_SUNSHINE_OFF:
    log_error("参数冲突: --sunshine-on 与 --sunshine-off 不能同时使用")
    sys.exit(2)

if CLEANUP_CONTAINERS and CLEANUP_CONTAINERS_OFF:
    log_error("参数冲突: --cleanup-containers 与 --no-cleanup-containers 不能同时使用")
    sys.exit(2)

# 默认行为：清理容器后再启动；可通过 --no-cleanup-containers 显式关闭
cleanup_containers_enabled = CLEANUP_CONTAINERS or not CLEANUP_CONTAINERS_OFF

if CLEANUP_VOLUMES and not cleanup_containers_enabled:
    log_error("参数使用错误: --cleanup-volumes 需要与 --cleanup-containers 一起使用")
    sys.exit(2)

if CLEANUP_ONLY and not cleanup_containers_enabled:
    log_error("参数使用错误: --cleanup-only 需要与 --cleanup-containers 一起使用")
    sys.exit(2)

if FORCE_CONTAINERS_ON or FORCE_CONTAINERS_OFF or FORCE_SUNSHINE_ON or FORCE_SUNSHINE_OFF or CLEANUP_CONTAINERS or CLEANUP_CONTAINERS_OFF or CLEANUP_VOLUMES or CLEANUP_ONLY:
    log_hint("检测到运行时开关参数，将覆盖 YAML 对应项")

# 1. 路径预检 (无论是否开启自动启动，都执行预检，方便手动调试)
containers = config.get('containers', [])
log_hint(f"共发现 {len(containers)} 个待处理项，正在进行路径校验...")

path_errors = 0
for item in containers:
    name = item.get('name', 'Unknown')
    container_enabled = item.get('enabled', True)
    compose_dir = expand_path(item.get('compose_dir'))

    if not container_enabled:
        log_hint(f"容器已禁用 [{name}]，跳过路径检查。")
        continue
    
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
containers_enabled_cfg = config.get('containers_enabled', config.get('enabled', True))

if FORCE_CONTAINERS_ON:
    containers_should_run = True
elif FORCE_CONTAINERS_OFF:
    containers_should_run = False
elif FORCE_RUN:
    containers_should_run = True
else:
    containers_should_run = containers_enabled_cfg

def should_handle_container(item):
    if FORCE_RUN:
        return True
    return item.get('enabled', True)

def cleanup_containers(items, remove_volumes=False):
    if not items:
        log_hint("没有可清理的容器项，跳过清理。")
        return

    log_info("正在执行容器清理流程...")
    for item in items:
        name = item.get('name', 'Unknown')
        compose_dir = expand_path(item.get('compose_dir'))

        print(f"\n{GREEN}----------------------------------------{RESET}")
        log_info(f"正在清理项目: {name}")
        print(f"{GREEN}----------------------------------------{RESET}")

        down_cmd = "docker compose down --remove-orphans"
        if remove_volumes:
            down_cmd += " -v"

        if compose_dir and os.path.exists(compose_dir):
            log_hint(f"工作目录: {compose_dir}")
            log_info(f"正在执行: {down_cmd}")
            if run_cmd(f"cd {compose_dir} && {down_cmd}"):
                log_success(f"{name} 清理完成。")
            else:
                log_hint(f"{name} Compose 清理失败，尝试按容器名强制删除。")
                if run_cmd(f"docker rm -f {name}"):
                    log_success(f"容器 {name} 已强制删除。")
        else:
            log_hint(f"未找到 Compose 路径，尝试按容器名清理: {name}")
            if run_cmd(f"docker rm -f {name}"):
                log_success(f"容器 {name} 已强制删除。")

        time.sleep(0.5)

containers_for_action = [item for item in containers if should_handle_container(item)]

if cleanup_containers_enabled:
    if not CLEANUP_CONTAINERS and not CLEANUP_CONTAINERS_OFF:
        log_hint("未指定清理参数，使用默认行为：先清理容器再启动。")
    cleanup_containers(containers_for_action, remove_volumes=CLEANUP_VOLUMES)

if CLEANUP_ONLY:
    print("")
    log_success("✅ 清理流程已完成（cleanup-only 模式，不执行启动）。")
    sys.exit(0)

if containers_should_run:
    if FORCE_RUN:
        log_info("🚀 检测到 --force 参数，正在强制执行启动流程...")

    for item in containers:
        name = item.get('name', 'Unknown')
        container_enabled = item.get('enabled', True)
        compose_dir = expand_path(item.get('compose_dir'))

        if not container_enabled and not FORCE_RUN:
            log_hint(f"容器已禁用 [{name}]，本次跳过启动。")
            continue

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
    log_info("⚠️ 容器自动启动已关闭，已跳过容器拉起。")
    log_hint("提示: 可用 --containers-on / --containers-off 在本次运行覆盖 YAML 配置")

# 4. 独立拉起 Sunshine 进程
sunshine_enabled_cfg = config.get('sunshine_enabled', False)

if FORCE_SUNSHINE_ON:
    sunshine_should_run = True
elif FORCE_SUNSHINE_OFF:
    sunshine_should_run = False
elif FORCE_RUN:
    sunshine_should_run = True
else:
    sunshine_should_run = sunshine_enabled_cfg

if sunshine_should_run:
    print(f"\n{GREEN}========================================{RESET}")
    log_info("正在拉起项目: Sunshine")
    print(f"{GREEN}========================================{RESET}")

    sunshine_cmd = shutil.which("sunshine")
    if not sunshine_cmd:
        log_error("未找到 sunshine 可执行程序，请先安装并确保在 PATH 中")
    else:
        target_user = None
        launch_cmd = [sunshine_cmd]
        launch_env = os.environ.copy()

        if os.geteuid() == 0 and REAL_USER != "root":
            target_user = REAL_USER
            try:
                user_info = pwd.getpwnam(REAL_USER)
                launch_cmd = ["runuser", "-u", REAL_USER, "--", sunshine_cmd]
                launch_env["HOME"] = REAL_HOME
                launch_env["USER"] = REAL_USER
                launch_env["LOGNAME"] = REAL_USER
                launch_env.setdefault("XDG_RUNTIME_DIR", f"/run/user/{user_info.pw_uid}")
                launch_env.setdefault("DISPLAY", ":0")
                log_hint(f"检测到当前以 root 运行，Sunshine 将降权为用户 {REAL_USER} 启动。")
            except KeyError:
                log_error(f"无法解析系统用户 {REAL_USER}，Sunshine 启动已跳过。")
                target_user = None

        check_cmd = ["pgrep", "-x", "sunshine"]
        if target_user:
            check_cmd = ["pgrep", "-x", "-u", target_user, "sunshine"]

        check_running = subprocess.run(check_cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)

        if check_running.returncode == 0 and check_running.stdout.strip():
            pid_list = check_running.stdout.split()
            pids = ", ".join(pid_list)
            if len(pid_list) > 1:
                log_error(f"检测到多个 Sunshine 进程同时运行 (PID: {pids})，本次不再重复启动。")
            else:
                if target_user:
                    log_hint(f"检测到用户 {target_user} 的 Sunshine 已在运行 (PID: {pids})，本次跳过重复启动。")
                else:
                    log_hint(f"检测到 Sunshine 已在运行 (PID: {pids})，本次跳过重复启动。")
        else:
            try:
                sunshine_log = "/tmp/autostart_sunshine.log"
                with open(sunshine_log, "a") as sf:
                    subprocess.Popen(
                        launch_cmd,
                        stdout=sf,
                        stderr=subprocess.STDOUT,
                        start_new_session=True,
                        env=launch_env
                    )

                time.sleep(1)
                recheck_running = subprocess.run(check_cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
                if recheck_running.returncode == 0 and recheck_running.stdout.strip():
                    pids = ", ".join(recheck_running.stdout.split())
                    if target_user:
                        log_success(f"Sunshine 已以用户 {target_user} 启动并在后台运行 (PID: {pids})。")
                    else:
                        log_success(f"Sunshine 已启动并在后台运行 (PID: {pids})。")
                else:
                    log_error("Sunshine 启动命令已执行，但未检测到运行进程，请检查 /tmp/autostart_sunshine.log")
            except Exception as e:
                log_error(f"Sunshine 启动报错: {e}")
else:
    print("")
    log_hint("Sunshine 自动启动已关闭，已跳过。")

print("")
log_success("✅ 所有启动流程已完成。")
EOF
