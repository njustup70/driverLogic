#!/bin/bash

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 1. 检查 root 权限
if [ "$EUID" -ne 0 ]; then
  echo -e "${RED}❌ 错误: 请使用 sudo 权限运行此脚本${NC}"
  echo "用法: sudo $0 [install|uninstall]"
  exit 1
fi

WORK_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SERVICE_NAME="autostart_ros2.service"
TARGET_SERVICE_PATH="/etc/systemd/system/$SERVICE_NAME"

function install_service() {
    echo -e "${BLUE}ℹ️ 正在安装自启动服务...${NC}"
    
    # 准备执行权限
    chmod +x "$WORK_DIR/autostart_run.sh"

    # 更新并拷贝服务文件
    sed "s|/PLACEHOLDER/PATH|$WORK_DIR|g" "$WORK_DIR/$SERVICE_NAME" > "$TARGET_SERVICE_PATH"

    # 注册并启用
    systemctl daemon-reload
    systemctl enable "$SERVICE_NAME"
    
    echo -e "${GREEN}✅ 注册成功！服务已设置为开机自启。${NC}"
    echo -e "${YELLOW}💡 提示: 您可以编辑 $WORK_DIR/autostart.yaml 来配置容器。${NC}"
    echo -e "${BLUE}执行 'sudo systemctl start $SERVICE_NAME' 立即启动测试。${NC}"
}

function uninstall_service() {
    echo -e "${YELLOW}⚠️ 正在注销并停止服务...${NC}"
    
    if [ -f "$TARGET_SERVICE_PATH" ]; then
        systemctl stop "$SERVICE_NAME"
        systemctl disable "$SERVICE_NAME"
        rm "$TARGET_SERVICE_PATH"
        systemctl daemon-reload
        echo -e "${RED}🛑 服务已注销并从系统中移除。${NC}"
    else
        echo -e "${BLUE}ℹ️ 系统中未发现已安装的服务。${NC}"
    fi
}

# 参数解析
case "$1" in
    -install)
        install_service
        ;;
    -uninstall)
        uninstall_service
        ;;
    *)
        echo -e "${BLUE}用法:${NC} sudo $0 {-install|-uninstall}"
        echo -e "  ${GREEN}-install${NC}   : 注册自启动服务"
        echo -e "  ${RED}-uninstall${NC} : 注销自启动服务"
        exit 1
esac
