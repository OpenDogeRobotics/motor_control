#include "include/motor_control_cpp.h"

// 构造函数
MotorController::MotorController(const std::string& can_interface, uint8_t master_id)
    : can_interface_(can_interface), master_id_(master_id), can_socket_(-1) {
}

// 析构函数
MotorController::~MotorController() {
    close_can();
}

// CAN总线初始化
int MotorController::init_can() {
    struct ifreq ifr;
    struct sockaddr_can addr;

    // 创建CAN套接字
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
        perror("Error creating CAN socket");
        return -1;
    }

    // 设置CAN接口
    strcpy(ifr.ifr_name, can_interface_.c_str());
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
        perror("Error getting CAN interface index");
        close(can_socket_);
        can_socket_ = -1;
        return -1;
    }

    // 绑定CAN接口
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_socket_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("Error binding CAN socket");
        close(can_socket_);
        can_socket_ = -1;
        return -1;
    }

    return 0;
}

// 关闭CAN总线
void MotorController::close_can() {
    if (can_socket_ >= 0) {
        close(can_socket_);
        can_socket_ = -1;
    }
}

// 发送扩展CAN帧
int MotorController::send_extended_frame(uint32_t arbitration_id, const std::vector<uint8_t>& data, bool block_receive, std::vector<uint8_t>& rx_data) {
    struct can_frame frame;
    
    // 初始化接收数据
    rx_data.resize(8, 0);
    
    // 设置CAN帧
    frame.can_id = arbitration_id | CAN_EFF_FLAG;  // 设置扩展帧标志
    frame.can_dlc = data.size();
    
    // 复制数据
    for (int i = 0; i < frame.can_dlc; i++) {
        frame.data[i] = data[i];
    }
    
    // 发送CAN消息
    if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Error sending CAN message");
        return -1;
    }
    
    // 如果需要阻塞接收
    if (block_receive) {
        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        
        // 设置接收超时
        if (setsockopt(can_socket_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
            perror("Error setting socket timeout");
            return -1;
        }
        
        // 接收CAN消息
        if (read(can_socket_, &frame, sizeof(struct can_frame)) < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                std::cerr << "Timeout waiting for response from arbitration ID: " << arbitration_id << std::endl;
            } else {
                perror("Error receiving CAN message");
            }
            return -1;
        }
        
        // 检查是否是我们要的响应
        if (!(frame.can_id & CAN_EFF_FLAG)) {
            std::cerr << "Received standard frame instead of extended frame" << std::endl;
            return -1;
        }
        
        // 提取接收数据
        for (int i = 0; i < frame.can_dlc; i++) {
            rx_data[i] = frame.data[i];
        }
    }
    
    return 0;
}

// 有符号浮点型转十六进制(0~65536)无符号型
uint16_t MotorController::float_to_uint16(float float_data, float float_data_min, float float_data_max) {
    float float_data_s;
    
    // 限制数据范围
    if (float_data > float_data_max) {
        float_data_s = float_data_max;
    } else if (float_data < float_data_min) {
        float_data_s = float_data_min;
    } else {
        float_data_s = float_data;
    }
    
    // 转换
    return static_cast<uint16_t>((float_data_s - float_data_min) / (float_data_max - float_data_min) * 65535.0f);
}

// 十六进制(0~65535)无符号型转有符号浮点型
float MotorController::uint16_to_float(uint16_t uint16_data, float float_data_min, float float_data_max) {
    return ((static_cast<float>(uint16_data) - 32767.0f) / 65535.0f) * (float_data_max - float_data_min);
}

// 设置电机零角度
int MotorController::set_motor_angle_zero(uint8_t motor_id) {
    std::vector<uint8_t> data = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint32_t arbitration_id = 0x06000000 | (master_id_ << 8) | motor_id;
    std::vector<uint8_t> rx_data;
    
    int result = send_extended_frame(arbitration_id, data, true, rx_data);
    if (result == 0) {
        std::cout << "Motor ID: " << static_cast<int>(motor_id) << " set angle zero OK" << std::endl;
    } else {
        std::cout << "Motor ID: " << static_cast<int>(motor_id) << " set angle zero error" << std::endl;
    }
    
    return result;
}

// 设置电机CAN ID
int MotorController::set_motor_can_id(uint8_t motor_id, uint8_t new_can_id) {
    std::vector<uint8_t> data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint32_t arbitration_id = 0x07000000 | (master_id_ << 8) | motor_id;
    std::vector<uint8_t> rx_data;
    
    int result = send_extended_frame(arbitration_id, data, true, rx_data);
    if (result == 0) {
        std::cout << "Motor ID: " << static_cast<int>(motor_id) << " set CAN ID: " << static_cast<int>(new_can_id) << " OK" << std::endl;
    } else {
        std::cout << "Motor ID: " << static_cast<int>(motor_id) << " set CAN ID: " << static_cast<int>(new_can_id) << " error" << std::endl;
    }
    
    return result;
}

// 设置电机主动上报
int MotorController::set_motor_report(uint8_t motor_id, uint8_t report_type) {
    std::vector<uint8_t> data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, (report_type == 0) ? 0x00 : 0x01, 0x00};
    uint32_t arbitration_id = 0x18000000 | (master_id_ << 8) | motor_id;
    std::vector<uint8_t> rx_data;
    
    int result = send_extended_frame(arbitration_id, data, true, rx_data);
    if (result == 0) {
        std::cout << "Motor ID: " << static_cast<int>(motor_id) << " set report type: " << static_cast<int>(report_type) << " OK" << std::endl;
    } else {
        std::cout << "Motor ID: " << static_cast<int>(motor_id) << " set report type: " << static_cast<int>(report_type) << " error" << std::endl;
    }
    
    return result;
}

// 设置运动模式
int MotorController::set_motion_mode(uint8_t motor_id) {
    std::vector<uint8_t> data(8, 0);
    uint32_t arbitration_id = 0x12000000 | (master_id_ << 8) | motor_id;
    std::vector<uint8_t> rx_data;
    
    int result = send_extended_frame(arbitration_id, data, true, rx_data);
    if (result == 0) {
        std::cout << "Motor ID: " << static_cast<int>(motor_id) << " set motion mode OK" << std::endl;
    } else {
        std::cout << "Motor ID: " << static_cast<int>(motor_id) << " set motion mode error" << std::endl;
    }
    
    return result;
}

// 设置电机使能
int MotorController::set_motion_enable(uint8_t motor_id) {
    std::vector<uint8_t> data(8, 0);
    uint32_t arbitration_id = 0x03000000 | (master_id_ << 8) | motor_id;
    std::vector<uint8_t> rx_data;
    
    int result = send_extended_frame(arbitration_id, data, true, rx_data);
    if (result == 0) {
        std::cout << "Motor ID: " << static_cast<int>(motor_id) << " motion enable OK" << std::endl;
    } else {
        std::cout << "Motor ID: " << static_cast<int>(motor_id) << " motion enable error" << std::endl;
    }
    
    return result;
}

// 设置电机失能
int MotorController::set_motion_disable(uint8_t motor_id) {
    std::vector<uint8_t> data(8, 0);
    uint32_t arbitration_id = 0x04000000 | (master_id_ << 8) | motor_id;
    std::vector<uint8_t> rx_data;
    
    int result = send_extended_frame(arbitration_id, data, true, rx_data);
    if (result == 0) {
        std::cout << "Motor ID: " << static_cast<int>(motor_id) << " motion disable OK" << std::endl;
    } else {
        std::cout << "Motor ID: " << static_cast<int>(motor_id) << " motion disable error" << std::endl;
    }
    
    return result;
}

// 设置运动参数
int MotorController::leg_set_motion_parameter(uint8_t motor_id, float torque, float radian, float speed, float kp, float kd) {
    std::vector<uint8_t> data(8, 0);
    
    // 计算仲裁ID（包含扭矩参数）
    uint16_t torque_uint16 = float_to_uint16(torque, P_MIN, P_MAX);
    uint32_t arbitration_id = (Communication_Type_MotionControl << 24) | (torque_uint16 << 8) | motor_id;
    
    // 设置角度参数
    uint16_t radian_uint16 = float_to_uint16(radian, T_MIN, T_MAX);
    data[0] = radian_uint16 >> 8;
    data[1] = radian_uint16 & 0x00ff;
    
    // 设置速度参数
    uint16_t speed_uint16 = float_to_uint16(speed, V_MIN, V_MAX);
    data[2] = speed_uint16 >> 8;
    data[3] = speed_uint16 & 0x00ff;
    
    // 设置KP参数
    uint16_t kp_uint16 = float_to_uint16(kp, KP_MIN, KP_MAX);
    data[4] = kp_uint16 >> 8;
    data[5] = kp_uint16 & 0x00ff;
    
    // 设置KD参数
    uint16_t kd_uint16 = float_to_uint16(kd, KD_MIN, KD_MAX);
    data[6] = kd_uint16 >> 8;
    data[7] = kd_uint16 & 0x00ff;
    
    std::vector<uint8_t> rx_data;
    
    return send_extended_frame(arbitration_id, data, false, rx_data);
}

// 解析电机数据
MotorState MotorController::parse_motor_data(const std::vector<uint8_t>& data) {
    MotorState state;
    
    // 解析位置
    uint16_t position_uint16 = (data[0] << 8) | data[1];
    state.position = -12.57f + (static_cast<float>(position_uint16) / 65535.0f) * 25.14f;
    
    // 解析速度
    uint16_t speed_uint16 = (data[2] << 8) | data[3];
    state.speed = -50.0f + (static_cast<float>(speed_uint16) / 65535.0f) * 100.0f;
    
    // 解析扭矩
    uint16_t torque_uint16 = (data[4] << 8) | data[5];
    state.torque = -6.0f + (static_cast<float>(torque_uint16) / 65535.0f) * 12.0f;
    
    // 解析温度
    uint16_t temp_uint16 = (data[6] << 8) | data[7];
    state.temperature = static_cast<float>(temp_uint16) / 10.0f;
    
    return state;
}