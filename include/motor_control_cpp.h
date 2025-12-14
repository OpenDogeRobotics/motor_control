#ifndef MOTOR_CONTROL_CPP_H
#define MOTOR_CONTROL_CPP_H

#include <iostream>
#include <string>
#include <vector>
#include <cstring>
#include <chrono>
#include <thread>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

// 常量定义
const float pai = 3.1415926f;

// 不可改动
const float P_MIN = -12.5f;     //扭矩下限范围
const float P_MAX = 12.5f;      //扭矩上限范围
const float T_MIN = -12.0f;     //角度(弧度)下限范围
const float T_MAX = 12.0f;      //角度(弧度)上限范围
const float V_MIN = -50.0f;     //速度下限范围
const float V_MAX = 50.0f;      //速度上限范围
const float KP_MIN = 0.0f;      //KP下限范围
const float KP_MAX = 500.0f;    //KP上限范围
const float KD_MIN = 0.0f;      //KD下限范围
const float KD_MAX = 5.0f;      //KD上限范围

// 通信地址定义
const uint8_t Communication_Type_Get_ID = 0x00;  //获取设备的ID和64位MCU唯一标识符
const uint8_t Communication_Type_MotionControl = 0x01;  //运控模式用来向主机发送控制指令
const uint8_t Communication_Type_MotorRequest = 0x02;  //用来向主机反馈电机运行状态
const uint8_t Communication_Type_MotorEnable = 0x03;  //电机使能运行
const uint8_t Communication_Type_MotorStop = 0x04;  //电机停止运行
const uint8_t Communication_Type_SetPosZero = 0x06;  //设置电机机械零位
const uint8_t Communication_Type_Can_ID = 0x07;  //更改当前电机CAN_ID
const uint8_t Communication_Type_Control_Mode = 0x12;  //设置电机模式

// 电机状态结构体
typedef struct {
    float position;
    float torque;
    float speed;
    float temperature;
} MotorState;

// 电机控制类
class MotorController {
private:
    int can_socket_;
    std::string can_interface_;
    uint8_t master_id_;
    
    // 发送扩展CAN帧
    int send_extended_frame(uint32_t arbitration_id, const std::vector<uint8_t>& data, bool block_receive, std::vector<uint8_t>& rx_data);
    
    // 数据类型转换函数
    uint16_t float_to_uint16(float float_data, float float_data_min, float float_data_max);
    float uint16_to_float(uint16_t uint16_data, float float_data_min, float float_data_max);
    
public:
    // 构造函数和析构函数
    MotorController(const std::string& can_interface, uint8_t master_id = 0xfd);
    ~MotorController();
    
    // CAN总线初始化
    int init_can();
    
    // 关闭CAN总线
    void close_can();
    
    // 电机控制函数
    int set_motor_angle_zero(uint8_t motor_id);
    int set_motor_can_id(uint8_t motor_id, uint8_t new_can_id);
    int set_motor_report(uint8_t motor_id, uint8_t report_type);
    int set_motion_mode(uint8_t motor_id);
    int set_motion_enable(uint8_t motor_id);
    int set_motion_disable(uint8_t motor_id);
    int leg_set_motion_parameter(uint8_t motor_id, float torque, float radian, float speed, float kp, float kd);
    
    // 电机数据解析
    MotorState parse_motor_data(const std::vector<uint8_t>& data);
};

#endif // MOTOR_CONTROL_CPP_H