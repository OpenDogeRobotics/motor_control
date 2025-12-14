#ifndef MOTOR_HARDWARE_INTERFACE_H
#define MOTOR_HARDWARE_INTERFACE_H

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "include/motor_control_cpp.h"

using hardware_interface::return_type;
using hardware_interface::CallbackReturn;

class MotorHardware : public hardware_interface::SystemInterface
{
public:
  MotorHardware();
  ~MotorHardware();

    // 硬件接口实现
    CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    
    return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // 电机控制器实例
  std::unique_ptr<MotorController> motor_controller_;
  
  // 电机状态和命令
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> joint_efforts_;
  std::vector<double> joint_commands_;
  
  // 电机ID列表
  std::vector<uint8_t> motor_ids_;
  
  // 电机配置参数
  std::string can_interface_;
  uint8_t master_id_;
  
  // 电机使能状态
  std::vector<bool> motor_enabled_;
};

#endif // MOTOR_HARDWARE_INTERFACE_H
