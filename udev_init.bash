#!/bin/bash
# 飞书文档🔗:https://tcnpd0yows2w.feishu.cn/wiki/E3o5wEuHnijR69k8f4EcxHmanXe
# 获取脚本的绝对路径
SCRIPT_DIR=$(dirname "$(realpath "$0")")
sudo apt-get install v4l-utils
#煞笔盲文挤占ch340
sudo apt-get remove brltty 
#先remove自己的udev规则，避免冲突
sudo rm -f /etc/udev/rules.d/my_dev.rules

# 进入脚本同目录下的 librealsense 目录
cd "$SCRIPT_DIR/packages/librealsense"
echo "当前目录: $(pwd)"
./scripts/setup_udev_rules.sh

# 安装奥比中光规则
cd "$SCRIPT_DIR/packages/orbbecSDK/misc/scripts"
echo "当前目录: $(pwd)"
sudo ./install_udev_rules.sh
# sudo ./wheeltec_udev.sh
#增加对轮趣imu (fdilink_arhs)的支持
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0003", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_FDI_IMU_GNSS"' >/etc/udev/rules.d/my_dev.rules

# 添加QinHeng Electronics USB Single Serial规则 (ID 1a86:55d4)
# echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", MODE:="0777", GROUP:="dialout", SYMLINK+="qinheng"' >> /etc/udev/rules.d/my_dev.rules
# echo 'KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", MODE:="0777", GROUP:="dialout", SYMLINK+="qinheng"' >> /etc/udev/rules.d/my_dev.rules
echo 'KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", ATTRS{serial}=="5954002901",MODE:="0777", GROUP:="dialout", SYMLINK+="serial_qh"' >> /etc/udev/rules.d/my_dev.rules
# 添加 CH340 规则 (针对没有唯一序列号的设备)
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", GROUP:="dialout", SYMLINK+="ch340"' >> /etc/udev/rules.d/my_dev.rules
echo 'KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", GROUP:="dialout", SYMLINK+="ch340"' >> /etc/udev/rules.d/my_dev.rules

# 立即
# CP210x 规则（精确匹配序列号 "0001"）
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", MODE:="0777", GROUP:="dialout", SYMLINK+="cp210x"' >> /etc/udev/rules.d/my_dev.rules

#安装ms_200规则
# 设置设备别名并设置权限
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", MODE:="0777", GROUP:="dialout", SYMLINK+="ms200"' >> /etc/udev/rules.d/my_dev.rules
echo 'KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", MODE:="0777", GROUP:="dialout", SYMLINK+="ms200"' >> /etc/udev/rules.d/my_dev.rules
# echo 'KERNEL=="tty*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", ATTRS{serial}=="597B009268", MODE:="0777", GROUP:="dialout", SYMLINK+="ms200"' >> /etc/udev/rules.d/my_dev.rules

# 为 MS200 设备（如果有不同的序列号）创建 /dev/ms200
# 下位机串口
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0777", GROUP:="dialout", SYMLINK+="serial_x64"' >> /etc/udev/rules.d/my_dev.rules
echo 'KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0777", GROUP:="dialout", SYMLINK+="serial_x64"' >> /etc/udev/rules.d/my_dev.rules

echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", GROUP:="dialout", SYMLINK+="serial_ch340"' >> /etc/udev/rules.d/my_dev.rules
echo 'KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", GROUP:="dialout", SYMLINK+="serial_ch340"' >> /etc/udev/rules.d/my_dev.rules

#添加ch040imu规则
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
    ATTRS{serial}=="26454afeb1ebed1181ec429aa88ea882", \
    MODE:="0777", GROUP:="dialout", SYMLINK+="ch040_imu"' >> /etc/udev/rules.d/my_dev.rules
    
echo 'KERNEL=="ttyACM*", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="6666", MODE:="0777", GROUP:="dialout", SYMLINK+="serial_sick"' >> /etc/udev/rules.d/my_dev.rules
#增加对hik_camera的支持
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="2bdf", ATTRS{idProduct}=="0001", MODE="0777", SYMLINK+="hik_camera"' >> /etc/udev/rules.d/my_dev.rules
# 增加odin支持
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2207", ATTR{idProduct}=="0019", MODE="0777", GROUP="plugdev",SYMLINK+="odin"' >> /etc/udev/rules.d/my_dev.rules
#增加对新的qh串口支持
echo 'KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d3", MODE:="0777", GROUP:="dialout", SYMLINK+="serial_qh"' >> /etc/udev/rules.d/my_dev.rules
service udev reload
sleep 2
service udev restart
#archlinux系统需要执行以下命令来重新加载 udev 规则并重启 udev 服务：
sudo udevadm control --reload-rules && sudo udevadm trigger