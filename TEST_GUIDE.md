# 电机控制接口测试指南

## 1. 准备工作

### 1.1 硬件连接
- 连接PEAK PCAN-USB适配器到计算机的USB端口
- 确保CAN总线两端的终端电阻正确连接（120Ω）
- 连接电机到CAN总线

### 1.2 安装依赖
```bash
# 安装CAN工具
sudo apt-get install can-utils

# 安装ros2_control相关包
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
```

## 2. CAN总线初始化

### 2.1 检查CAN接口
```bash
dmesg | grep -i peak  # 检查PEAK设备是否被识别
ip link show can0     # 检查can0接口是否存在
```

### 2.2 运行初始化脚本
执行项目提供的CAN总线初始化脚本：

```bash
cd /home/ffd/ros2_ws/src/motor_control
chmod +x init_pcan.sh  # 确保脚本可执行
sudo ./init_pcan.sh     # 以root权限运行
```

该脚本会：
- 检查并加载peak_usb模块
- 设置CAN波特率为1Mbps
- 启动can0接口

### 2.3 验证CAN接口状态
```bash
ip -s link show can0
```

预期输出应显示can0接口状态为UP。

## 3. 编译项目

```bash
cd /home/ffd/ros2_ws
colcon build --packages-select motor_control_interface
```

## 4. 测试方法

### 4.1 方法1：直接运行测试程序

```bash
source install/setup.bash
./install/motor_control_interface/lib/motor_control_interface/test_motor_hardware
```

测试程序会：
- 初始化MotorHardware接口
- 配置CAN通信和关节参数
- 设置电机位置命令
- 读取电机状态
- 输出测试结果

### 4.2 方法2：使用ROS 2启动文件

```bash
source install/setup.bash
ros2 launch motor_control motor_control_launch.py
```

该启动文件会启动：
- controller_manager节点（ros2_control_node）
- 关节状态发布器（joint_state_broadcaster）
- 位置控制器（position_controller）

## 5. 发送电机控制命令

当使用启动文件运行时，可以通过ROS 2话题发送位置命令：

```bash
# 向关节1发送0.5弧度的位置命令
ros2 topic pub --once /position_controller/commands std_msgs/msg/Float64MultiArray "{
  data: [0.5]
}"
```

## 6. 监控电机状态

### 6.1 查看关节状态
```bash
ros2 topic echo /joint_states
```

### 6.2 查看控制器状态
```bash
ros2 control list_controllers
```

### 6.3 监控CAN总线通信
```bash
candump can0  # 实时查看CAN总线消息
```

## 7. 常见问题排查

### 7.1 CAN总线通信失败
- 检查CAN接口是否已正确初始化：`ip -s link show can0`
- 验证终端电阻是否正确连接
- 检查波特率设置是否与电机匹配

### 7.2 电机无响应
- 检查电机ID是否与配置文件中的设置一致
- 确保电机已供电
- 检查电机控制器是否处于激活状态

### 7.3 控制器启动失败
- 查看启动日志：`ros2 launch motor_control motor_control_launch.py --log-level debug`
- 检查控制器配置文件：`/home/ffd/ros2_ws/src/motor_control/config/motor_controller_config.yaml`

## 8. 配置文件说明

### 8.1 启动配置 (`launch/motor_control_launch.py`)
- `can_interface`: CAN接口名称 (默认: can0)
- `master_id`: 主控制器ID (默认: 253)
- `joints`: 关节配置列表

### 8.2 控制器配置 (`config/motor_controller_config.yaml`)
- `position_controller`: 位置控制器参数
- `joints`: 受控关节列表
- `gains`: PID控制器增益参数

## 9. 停止和清理

### 9.1 停止控制器
```bash
ros2 control stop controllers position_controller
```

### 9.2 关闭CAN接口
```bash
sudo ip link set can0 down
```
