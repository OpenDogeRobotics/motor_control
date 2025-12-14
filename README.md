# 电机控制接口项目

## 项目概述
本项目基于ROS 2和ros2_control框架实现电机硬件控制接口，支持通过CAN总线与电机通信，提供位置控制功能，并可通过ROS 2话题进行远程控制。

## 项目依赖
- ROS 2 Humble Hawksbill (或其他兼容版本)
- ros2_control
- ros2_controllers
- can-utils (用于CAN总线调试)

## 安装说明
### 1. 安装ROS 2
请按照[ROS 2官方文档](https://docs.ros.org/en/humble/Installation.html)安装对应版本的ROS 2。

### 2. 创建工作空间
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### 3. 克隆仓库
```bash
cd ~/ros2_ws/src
git clone <仓库URL>  # 替换为实际仓库地址
```

### 4. 安装依赖
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

## 编译步骤
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 项目结构
```
ros2_ws/src/
├── motor_control/
│   ├── CMakeLists.txt
│   ├── config/
│   ├── launch/
│   ├── urdf/
│   ├── motor_hardware_interface.cpp
│   ├── motor_hardware_interface.h
│   ├── control_motor_via_topic.py
│   └── test_motor_hardware.cpp
└── motor_control_interface/
    ├── CMakeLists.txt
    └── ...
```

## 核心功能
- 基于ros2_control的硬件接口实现
- CAN总线通信支持
- 电机位置控制
- 实时状态反馈（位置、速度）
- 通过ROS 2话题进行远程控制

## 配置说明

### 控制器配置
控制器配置文件：`config/motor_controller_config.yaml`
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    position_controller:
      type: position_controllers/JointGroupPositionController
      joints:
        - joint1
      interface_name_mapping:
        joint1:
          position: position
          velocity: velocity
          effort: effort
      joint1:
        pid:
          p: 100.0
          i: 0.0
          d: 1.0
          i_clamp: 0.0
```

### URDF配置
URDF文件用于描述机器人模型和硬件接口配置，定义了关节和电机之间的映射关系。

## 启动方法

### 启动电机控制节点
```bash
cd /home/ffd/ros2_ws
source install/setup.bash
ros2 launch motor_control_interface motor_control_launch.py
```

启动后会自动：
1. 加载电机硬件接口
2. 启动控制器管理器
3. 激活joint_state_broadcaster
4. 激活position_controller

## 测试方法

### 运行硬件测试程序
```bash
cd /home/ffd/ros2_ws
source install/setup.bash
./install/motor_control_interface/lib/motor_control_interface/test_motor_hardware
```

### 通过话题控制电机
使用提供的Python脚本发送位置命令：
```bash
cd /home/ffd/ros2_ws
source install/setup.bash
python3 src/motor_control/control_motor_via_topic.py
```

## 话题接口

### 命令话题
- **话题名**：`/position_controller/commands`
- **消息类型**：`std_msgs/msg/Float64MultiArray`
- **描述**：发送电机位置命令，单位为弧度
- **示例数据**：`data: [0.5]` 表示将电机位置设置为0.5弧度

### 状态话题
- **话题名**：`/joint_states`
- **消息类型**：`sensor_msgs/msg/JointState`
- **描述**：反馈电机当前状态，包括位置、速度和力矩

## 控制器信息
- **控制器类型**：`position_controllers/JointGroupPositionController`
- **控制关节**：joint1
- **更新频率**：100Hz

## 硬件接口
- **插件名称**：`motor_control::MotorControlHardware`
- **通信方式**：CAN总线
- **支持接口**：
  - 命令接口：position
  - 状态接口：position、velocity、effort

## 故障排除

### 常见问题
1. **CAN接口占用**：关闭测试程序时可能出现"Bad file descriptor"错误，这是因为CAN接口被占用，可通过重启系统或重新初始化CAN接口解决。

2. **控制器激活失败**：检查控制器配置文件中的关节名称和接口映射是否正确。

3. **电机无响应**：确认CAN总线连接正常，电机ID配置正确，且电机已正确使能。

## 使用示例

### 发送位置命令
通过ROS 2话题工具发送位置命令：
```bash
ros2 topic pub -1 /position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5]"
```

### 查看关节状态
```bash
ros2 topic echo /joint_states
```

## 注意事项
- 位置命令单位为弧度
- 电机控制范围根据实际电机参数确定
- 启动前确保CAN接口已正确初始化
- 测试程序运行时请勿手动操作电机