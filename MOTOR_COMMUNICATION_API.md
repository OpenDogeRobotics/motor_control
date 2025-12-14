# 电机通讯API说明手册

## 1. 概述

本手册详细介绍电机控制系统的通讯API，基于CAN总线实现电机与控制器之间的通信。API提供了完整的电机控制功能，包括初始化、使能、失能、位置控制、参数设置等操作。

## 2. 数据类型定义

### 2.1 电机状态结构体
```cpp
typedef struct {
    float position;     // 电机位置 (单位: 弧度)
    float torque;       // 电机扭矩 (单位: N·m)
    float speed;        // 电机速度 (单位: 弧度/秒)
    float temperature;  // 电机温度 (单位: °C)
} MotorState;
```

## 3. 常量说明

### 3.1 电机参数范围
```cpp
// 扭矩范围
const float P_MIN = -12.5f;     // 扭矩下限 (N·m)
const float P_MAX = 12.5f;      // 扭矩上限 (N·m)

// 角度范围
const float T_MIN = -12.0f;     // 角度下限 (弧度)
const float T_MAX = 12.0f;      // 角度上限 (弧度)

// 速度范围
const float V_MIN = -50.0f;     // 速度下限 (弧度/秒)
const float V_MAX = 50.0f;      // 速度上限 (弧度/秒)

// 位置环比例增益范围
const float KP_MIN = 0.0f;      // KP下限
const float KP_MAX = 500.0f;    // KP上限

// 位置环微分增益范围
const float KD_MIN = 0.0f;      // KD下限
const float KD_MAX = 5.0f;      // KD上限
```

### 3.2 通信类型定义
```cpp
// 通信类型ID
const uint8_t Communication_Type_Get_ID = 0x00;          // 获取设备ID和64位MCU唯一标识符
const uint8_t Communication_Type_MotionControl = 0x01;   // 运控模式发送控制指令
const uint8_t Communication_Type_MotorRequest = 0x02;    // 反馈电机运行状态
const uint8_t Communication_Type_MotorEnable = 0x03;     // 电机使能运行
const uint8_t Communication_Type_MotorStop = 0x04;       // 电机停止运行
const uint8_t Communication_Type_SetPosZero = 0x06;      // 设置电机机械零位
const uint8_t Communication_Type_Can_ID = 0x07;          // 更改当前电机CAN_ID
const uint8_t Communication_Type_Control_Mode = 0x12;    // 设置电机模式
```

## 4. 类定义与方法说明

### 4.1 MotorController类

#### 4.1.1 构造函数与析构函数
```cpp
// 构造函数
MotorController(const std::string& can_interface, uint8_t master_id = 0xfd);

// 析构函数
~MotorController();
```

**参数说明**：
- `can_interface`: CAN总线接口名称 (如 "can0")
- `master_id`: 主机ID，默认为0xfd

#### 4.1.2 CAN总线初始化与关闭
```cpp
// CAN总线初始化
int init_can();

// 关闭CAN总线
void close_can();
```

**返回值**：
- `init_can()`: 成功返回0，失败返回-1

#### 4.1.3 电机控制方法

##### 设置电机零角度
```cpp
int set_motor_angle_zero(uint8_t motor_id);
```

**功能**：设置电机当前位置为机械零位

**参数**：
- `motor_id`: 电机ID

**返回值**：成功返回0，失败返回-1

##### 设置电机CAN ID
```cpp
int set_motor_can_id(uint8_t motor_id, uint8_t new_can_id);
```

**功能**：修改电机的CAN ID

**参数**：
- `motor_id`: 当前电机ID
- `new_can_id`: 新的电机ID

**返回值**：成功返回0，失败返回-1

##### 设置电机主动上报
```cpp
int set_motor_report(uint8_t motor_id, uint8_t report_type);
```

**功能**：设置电机是否主动上报状态

**参数**：
- `motor_id`: 电机ID
- `report_type`: 上报类型 (0: 关闭, 1: 开启)

**返回值**：成功返回0，失败返回-1

##### 设置运动模式
```cpp
int set_motion_mode(uint8_t motor_id);
```

**功能**：将电机设置为运动控制模式

**参数**：
- `motor_id`: 电机ID

**返回值**：成功返回0，失败返回-1

##### 设置电机使能
```cpp
int set_motion_enable(uint8_t motor_id);
```

**功能**：使能电机运动

**参数**：
- `motor_id`: 电机ID

**返回值**：成功返回0，失败返回-1

##### 设置电机失能
```cpp
int set_motion_disable(uint8_t motor_id);
```

**功能**：失能电机运动

**参数**：
- `motor_id`: 电机ID

**返回值**：成功返回0，失败返回-1

##### 设置运动参数
```cpp
int leg_set_motion_parameter(uint8_t motor_id, float torque, float radian, float speed, float kp, float kd);
```

**功能**：设置电机运动参数（位置、速度、扭矩、PID参数）

**参数**：
- `motor_id`: 电机ID
- `torque`: 扭矩限制 (范围: -12.5 ~ 12.5 N·m)
- `radian`: 目标位置 (范围: -12.0 ~ 12.0 弧度)
- `speed`: 速度限制 (范围: -50.0 ~ 50.0 弧度/秒)
- `kp`: 位置环比例增益 (范围: 0.0 ~ 500.0)
- `kd`: 位置环微分增益 (范围: 0.0 ~ 5.0)

**返回值**：成功返回0，失败返回-1

#### 4.1.4 数据解析方法

##### 解析电机数据
```cpp
MotorState parse_motor_data(const std::vector<uint8_t>& data);
```

**功能**：解析从电机接收到的原始数据

**参数**：
- `data`: 接收到的8字节原始数据

**返回值**：解析后的电机状态结构体

## 5. 通讯协议详解

### 5.1 CAN帧格式

系统采用扩展CAN帧格式，仲裁ID为29位：

```
| 28 27 26 25 24 | 23 22 21 20 19 18 17 16 | 15 14 13 12 11 10 09 08 | 07 06 05 04 03 02 01 00 |
|----------------|------------------------|------------------------|------------------------|
| 通信类型       | 参数1                  | 参数2                  | 电机ID                 |
```

### 5.2 数据转换机制

系统使用16位无符号整数(0-65535)表示各种浮点型电机参数，转换公式如下：

**浮点型转16位无符号整数**：
```cpp
uint16_t = (float_data - float_data_min) / (float_data_max - float_data_min) * 65535.0f
```

**16位无符号整数转浮点型**：
```cpp
float = ((uint16_data - 32767.0f) / 65535.0f) * (float_data_max - float_data_min)
```

### 5.3 控制帧数据格式

控制帧数据长度为8字节，格式如下：

```
| Byte 0-1 | Byte 2-3 | Byte 4-5 | Byte 6-7 |
|----------|----------|----------|----------|
| 位置参数 | 速度参数 | KP参数   | KD参数   |
```

**说明**：
- 每个参数占2字节，高位在前(大端模式)
- 位置参数范围：-12.0 ~ 12.0 弧度
- 速度参数范围：-50.0 ~ 50.0 弧度/秒
- KP参数范围：0.0 ~ 500.0
- KD参数范围：0.0 ~ 5.0

### 5.4 状态帧数据格式

状态帧数据长度为8字节，格式如下：

```
| Byte 0-1 | Byte 2-3 | Byte 4-5 | Byte 6-7 |
|----------|----------|----------|----------|
| 位置数据 | 速度数据 | 扭矩数据 | 温度数据 |
```

**说明**：
- 每个参数占2字节，高位在前(大端模式)
- 位置数据范围：-12.57 ~ 12.57 弧度
- 速度数据范围：-50.0 ~ 50.0 弧度/秒
- 扭矩数据范围：-6.0 ~ 6.0 N·m
- 温度数据：单位为0.1°C (实际温度 = 数据值 / 10)

## 6. 使用示例

### 6.1 基本使用流程

```cpp
#include "motor_control_cpp.h"

int main() {
    // 创建电机控制器实例
    MotorController controller("can0", 0xfd);
    
    // 初始化CAN总线
    if (controller.init_can() != 0) {
        std::cerr << "Failed to initialize CAN bus" << std::endl;
        return -1;
    }
    
    // 设置运动模式
    controller.set_motion_mode(1);
    
    // 使能电机
    controller.set_motion_enable(1);
    
    // 设置运动参数 (扭矩限制, 目标位置, 速度限制, KP, KD)
    controller.leg_set_motion_parameter(1, 5.0f, 1.0f, 10.0f, 100.0f, 1.0f);
    
    // 等待1秒
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // 失能电机
    controller.set_motion_disable(1);
    
    // 关闭CAN总线
    controller.close_can();
    
    return 0;
}
```

### 6.2 电机状态读取示例

```cpp
#include "motor_control_cpp.h"

int main() {
    MotorController controller("can0", 0xfd);
    
    if (controller.init_can() != 0) {
        std::cerr << "Failed to initialize CAN bus" << std::endl;
        return -1;
    }
    
    // 设置电机主动上报
    controller.set_motor_report(1, 1);
    
    // 创建接收缓冲区
    std::vector<uint8_t> rx_data(8, 0);
    
    // 循环读取电机状态
    for (int i = 0; i < 10; ++i) {
        // 读取CAN消息
        // (此处需要根据实际应用实现CAN消息接收逻辑)
        
        // 解析电机数据
        MotorState state = controller.parse_motor_data(rx_data);
        
        // 打印电机状态
        std::cout << "Position: " << state.position << " rad" << std::endl;
        std::cout << "Speed: " << state.speed << " rad/s" << std::endl;
        std::cout << "Torque: " << state.torque << " N·m" << std::endl;
        std::cout << "Temperature: " << state.temperature << " °C" << std::endl;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    controller.close_can();
    
    return 0;
}
```

## 7. 错误处理

### 7.1 常见错误代码

| 错误类型 | 错误代码 | 说明 |
|---------|---------|------|
| CAN套接字创建失败 | -1 | socket()调用失败 |
| CAN接口获取失败 | -1 | ioctl()调用失败 |
| CAN接口绑定失败 | -1 | bind()调用失败 |
| 发送CAN消息失败 | -1 | write()调用失败 |
| 接收CAN消息超时 | -1 | 1秒内未收到响应 |
| 接收CAN消息失败 | -1 | read()调用失败 |

### 7.2 错误排查建议

1. **CAN总线连接问题**：
   - 检查CAN接口是否正确连接
   - 确认CAN总线终端电阻是否安装
   - 使用`ip link`命令检查CAN接口状态

2. **CAN接口配置问题**：
   - 确认CAN接口已启用：`sudo ip link set can0 up type can bitrate 500000`
   - 检查CAN接口波特率设置是否正确

3. **电机ID问题**：
   - 确认电机ID配置与代码中使用的ID一致
   - 使用`candump can0`命令监听CAN总线上的消息

4. **权限问题**：
   - 确保程序以足够权限运行，能够访问CAN接口
   - 可以尝试以root权限运行程序

## 8. 系统要求

- 操作系统：Linux
- CAN总线接口：支持SocketCAN的CAN适配器
- 编译器：支持C++11及以上标准
- 依赖库：标准C++库

## 9. 更新日志

| 版本 | 日期 | 更新内容 |
|------|------|---------|
| 1.0  | YYYY-MM-DD | 初始版本 |

## 10. 联系方式

如有问题或建议，请联系：
- 技术支持：[邮箱地址]
- 文档更新：[文档更新链接]

---

**注意**：本手册基于当前代码版本编写，如有代码更新，请以最新代码实现为准。