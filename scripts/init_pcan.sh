#!/bin/bash

# PEAK PCAN-USB 配置脚本
# 此脚本用于配置PEAK PCAN-USB适配器，设置波特率为1Mbps

echo "=== PEAK PCAN-USB 配置脚本 ==="

# 检查是否以root权限运行
if [ "$(id -u)" -ne 0 ]; then
    echo "错误: 请使用sudo或以root权限运行此脚本"
    exit 1
fi

# 检查can0接口是否存在
if ! ip link show can0 &> /dev/null; then
    echo "错误: 未找到can0接口！"
    echo "请检查PEAK PCAN-USB适配器是否正确连接。"
    echo "执行以下命令检查设备识别状态："
    echo "  dmesg | grep -i peak"
    exit 1
fi

# 检查peak_usb模块是否加载
if ! lsmod | grep -q peak_usb; then
    echo "警告: peak_usb模块可能未加载，尝试加载..."
    modprobe peak_usb
    if [ $? -ne 0 ]; then
        echo "错误: 无法加载peak_usb模块！"
        exit 1
    fi
    echo "peak_usb模块已加载"
fi

echo "正在配置PCAN接口..."

# 配置CAN接口
# 1. 先关闭接口（如果正在运行）
ip link set can0 down
if [ $? -ne 0 ]; then
    echo "警告: 关闭can0接口失败，可能接口已经关闭"
fi

# 2. 设置波特率为1Mbps
echo "设置波特率为1Mbps..."
ip link set can0 type can bitrate 1000000
if [ $? -ne 0 ]; then
    echo "错误: 无法设置CAN波特率！"
    exit 1
fi

# 3. 启动CAN接口
echo "启动CAN接口..."
ip link set can0 up
if [ $? -ne 0 ]; then
    echo "错误: 无法启动CAN接口！"
    exit 1
fi

# 4. 显示CAN接口状态
echo "\n=== CAN接口状态信息 ==="
ip -s link show can0

echo "\n=== CAN接口配置完成！==="
echo "可以使用以下命令测试CAN通信："
echo "  candump can0         # 接收CAN消息"
echo "  cansend can0 123#1122334455667788  # 发送CAN消息"

exit 0