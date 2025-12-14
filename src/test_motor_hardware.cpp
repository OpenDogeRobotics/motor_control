#include <memory>
#include <thread>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "include/motor_hardware_interface.h"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto motor_hardware = std::make_shared<MotorHardware>();
  
  // 配置硬件接口
  hardware_interface::HardwareInfo info;
  info.hardware_parameters["can_interface"] = "can0";
  info.hardware_parameters["master_id"] = "253";
  
  // 添加一个关节
  hardware_interface::ComponentInfo joint_info;
  joint_info.name = "joint1";
  joint_info.parameters["motor_id"] = "1";
  
  hardware_interface::InterfaceInfo position_command;
  position_command.name = "position";
  joint_info.command_interfaces.push_back(position_command);
  
  hardware_interface::InterfaceInfo position_state;
  position_state.name = "position";
  joint_info.state_interfaces.push_back(position_state);
  
  hardware_interface::InterfaceInfo velocity_state;
  velocity_state.name = "velocity";
  joint_info.state_interfaces.push_back(velocity_state);
  
  hardware_interface::InterfaceInfo effort_state;
  effort_state.name = "effort";
  joint_info.state_interfaces.push_back(effort_state);
  
  info.joints.push_back(joint_info);
  
  // 配置硬件
  if (motor_hardware->on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    RCLCPP_FATAL(rclcpp::get_logger("TestMotorHardware"), "Failed to configure hardware");
    return -1;
  }
  
  // 启动硬件
  if (motor_hardware->on_activate(rclcpp_lifecycle::State()) != hardware_interface::CallbackReturn::SUCCESS) {
    RCLCPP_FATAL(rclcpp::get_logger("TestMotorHardware"), "Failed to start hardware");
    return -1;
  }
  
  // 获取状态和命令接口
  auto state_interfaces = motor_hardware->export_state_interfaces();
  auto command_interfaces = motor_hardware->export_command_interfaces();
  
  // 创建时钟对象
  rclcpp::Clock clock;
  
  // 测试电机控制
  for (int i = 0; i < 10; i++) {
    // 设置命令位置
    double position_command = i * 0.1;
    command_interfaces[0].set_value(position_command);
    
    // 获取当前时间
    rclcpp::Time time = clock.now();
    rclcpp::Duration period = rclcpp::Duration::from_seconds(0.1);
    
    // 写入命令
    if (motor_hardware->write(time, period) != hardware_interface::return_type::OK) {
      RCLCPP_ERROR(rclcpp::get_logger("TestMotorHardware"), "Failed to write command");
      break;
    }
    
    // 读取状态
    if (motor_hardware->read(time, period) != hardware_interface::return_type::OK) {
      RCLCPP_ERROR(rclcpp::get_logger("TestMotorHardware"), "Failed to read state");
      break;
    }
    
    // 获取并打印状态
    double position = state_interfaces[0].get_value();
    double velocity = state_interfaces[1].get_value();
    double effort = state_interfaces[2].get_value();
    
    RCLCPP_INFO(rclcpp::get_logger("TestMotorHardware"), 
               "Command: %.2f, Position: %.2f, Velocity: %.2f, Effort: %.2f", 
               position_command, position, velocity, effort);
    
    // 等待100毫秒
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  
  // 停止硬件
  if (motor_hardware->on_deactivate(rclcpp_lifecycle::State()) != hardware_interface::CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("TestMotorHardware"), "Failed to stop hardware");
  }
  
  rclcpp::shutdown();
  return 0;
}
