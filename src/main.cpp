#include "include/motor_control_cpp.h"

int main() {
    // 创建电机控制器实例
    MotorController motor_controller("can0");
    
    // 初始化CAN总线
    if (motor_controller.init_can() < 0) {
        std::cerr << "Failed to initialize CAN bus" << std::endl;
        return 1;
    }
    
    std::cout << "CAN bus initialized successfully" << std::endl;
    
    try {
        // 测试单个电机
        uint8_t motor_id = 1;
        // uint8_t motor_id2 = 7;
        
        // 设置电机零角度
        motor_controller.set_motor_angle_zero(motor_id);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // 设置电机运动模式
        motor_controller.set_motion_mode(motor_id);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // 使能电机
        motor_controller.set_motion_enable(motor_id);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // // 设置第二个电机
        // motor_controller.set_motor_angle_zero(motor_id2);
        // std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // motor_controller.set_motion_mode(motor_id2);
        // std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // 电机来回旋转测试
        float red = 0.0f;           // 起始角度
        int fx = 0;                 // 方向标志位
        float red_max = 1.2f;       // 最大角度
        float time_delay = 0.002f;  // 速率延时
        
        std::cout << "Starting motor rotation test..." << std::endl;
        
        while (true) {
            // 更新角度
            if (fx == 0 && red < red_max) {
                red += 0.01f;
            } else if (fx == 0 && red >= red_max) {
                fx = 1;
                std::this_thread::sleep_for(std::chrono::seconds(1));
            } else if (fx == 1 && red > 0) {
                red -= 0.01f;
            } else if (fx == 1 && red <= 0) {
                fx = 0;
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            
            // 控制电机
            motor_controller.leg_set_motion_parameter(motor_id, 0, red, 0, 10, 0.5);
            // motor_controller.leg_set_motion_parameter(motor_id2, 0, red, 0, 10, 0.5);
            
            // 延时
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(time_delay * 1000)));
            

        }
        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "Unknown exception" << std::endl;
    }
    
    // 关闭CAN总线
    motor_controller.close_can();
    
    return 0;
}