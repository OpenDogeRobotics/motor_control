#!/usr/bin/env python3
"""
设置电机CAN ID的Python脚本。
基于C++ motor_control_cpp.cpp中的set_motor_can_id函数实现，并参考robstride_ros_sample中的Set_CAN_ID。
"""

import socket
import struct
import time
import sys
import argparse

# CAN常量
try:
    PF_CAN = socket.PF_CAN
except AttributeError:
    PF_CAN = 29

try:
    SOCK_RAW = socket.SOCK_RAW
except AttributeError:
    SOCK_RAW = 3

try:
    CAN_RAW = socket.CAN_RAW
except AttributeError:
    CAN_RAW = 1

CAN_EFF_FLAG = 0x80000000  # 扩展帧标志

# 通信类型定义（来自motor_control_cpp.h）
COMMUNICATION_TYPE_CAN_ID = 0x07

def init_can(interface="can0"):
    """初始化CAN套接字"""
    try:
        sock = socket.socket(PF_CAN, SOCK_RAW, CAN_RAW)
    except OSError as e:
        print(f"创建CAN套接字失败: {e}")
        sys.exit(1)
    
    # 绑定到指定接口
    try:
        sock.bind((interface,))
    except OSError as e:
        print(f"无法绑定到CAN接口 {interface}: {e}")
        print("请确保CAN接口已启动且存在。")
        sock.close()
        sys.exit(1)
    
    return sock

def send_extended_frame(sock, arbitration_id, data, block_receive=True, timeout=1.0):
    """
    发送扩展CAN帧并可选接收响应。
    
    参数:
        sock: CAN套接字
        arbitration_id: 仲裁ID（不含标志）
        data: 字节列表，长度<=8
        block_receive: 是否等待响应
        timeout: 接收超时（秒）
    
    返回:
        (成功标志, 接收到的数据)
        成功标志: 0表示成功，-1表示失败
        接收到的数据: 字节列表（如果block_receive为True且成功）
    """
    # 确保数据长度不超过8
    if len(data) > 8:
        data = data[:8]
    elif len(data) < 8:
        data = data + [0] * (8 - len(data))
    
    # 构建can_frame结构
    # struct can_frame {
    #     canid_t can_id;  # 32位
    #     __u8    can_dlc; # 数据长度
    #     __u8    __pad;   # 填充
    #     __u8    __res0;  # 保留
    #     __u8    __res1;  # 保留
    #     __u8    data[8];
    # };
    can_id = arbitration_id | CAN_EFF_FLAG
    can_dlc = len(data)
    frame_format = "=IBBxx8s"
    frame_data = struct.pack(frame_format, can_id, can_dlc, 0, bytes(data))
    
    # 发送帧
    try:
        sent = sock.send(frame_data)
        if sent != len(frame_data):
            print("错误: 发送CAN消息不完整")
            return -1, []
    except OSError as e:
        print(f"发送CAN消息错误: {e}")
        return -1, []
    
    if not block_receive:
        return 0, []
    
    # 设置接收超时
    sock.settimeout(timeout)
    
    # 接收响应
    try:
        rx_frame = sock.recv(16)  # can_frame大小为16字节
    except socket.timeout:
        print(f"超时: 在{timeout}秒内未收到来自仲裁ID {hex(arbitration_id)}的响应")
        return -1, []
    except OSError as e:
        print(f"接收CAN消息错误: {e}")
        return -1, []
    
    # 解析接收到的帧
    if len(rx_frame) < 16:
        print("错误: 接收到的帧长度不足")
        return -1, []
    
    rx_can_id, rx_can_dlc, _, rx_data = struct.unpack("=IBBxx8s", rx_frame)
    # 检查是否是扩展帧
    if not (rx_can_id & CAN_EFF_FLAG):
        print("错误: 接收到标准帧而非扩展帧")
        return -1, []
    
    # 提取数据
    rx_data_bytes = list(rx_data[:rx_can_dlc])
    return 0, rx_data_bytes

def set_motor_can_id(interface, master_id, motor_id, new_can_id):
    """
    设置电机CAN ID
    
    参数:
        interface: CAN接口名称，如'can0'
        master_id: 主机ID (0-255)
        motor_id: 当前电机ID (0-255)
        new_can_id: 新的CAN ID (0-255)
    
    返回:
        0表示成功，-1表示失败
    """
    print(f"设置电机CAN ID: 接口={interface}, 主机ID={master_id}, 当前电机ID={motor_id}, 新CAN ID={new_can_id}")
    
    # 初始化CAN
    sock = init_can(interface)
    
    # 构造仲裁ID (参考Set_CAN_ID实现)
    arbitration_id = (COMMUNICATION_TYPE_CAN_ID << 24) | (new_can_id << 16) | (master_id << 8) | motor_id
    
    # 数据全为零
    data = [0] * 8
    
    # 发送帧并等待响应
    result, rx_data = send_extended_frame(sock, arbitration_id, data, block_receive=True)
    
    sock.close()
    
    if result == 0:
        print(f"成功: 电机ID {motor_id} 的CAN ID已设置为 {new_can_id}")
        return 0
    else:
        print(f"失败: 无法设置电机ID {motor_id} 的CAN ID")
        return -1

def main():
    parser = argparse.ArgumentParser(description="设置电机CAN ID")
    parser.add_argument("--interface", default="can0", help="CAN接口 (默认: can0)")
    parser.add_argument("--master", type=int, default=0xfd, help="主机ID (默认: 0xfd)")
    parser.add_argument("motor_id", type=int, help="当前电机ID (0-255)")
    parser.add_argument("new_can_id", type=int, help="新的CAN ID (0-255)")
    
    args = parser.parse_args()
    
    # 验证参数范围
    for name, val in [("motor_id", args.motor_id), ("new_can_id", args.new_can_id), ("master", args.master)]:
        if val < 0 or val > 255:
            print(f"错误: {name} 必须在0-255范围内")
            sys.exit(1)
    
    ret = set_motor_can_id(args.interface, args.master, args.motor_id, args.new_can_id)
    sys.exit(0 if ret == 0 else 1)

if __name__ == "__main__":
    main()