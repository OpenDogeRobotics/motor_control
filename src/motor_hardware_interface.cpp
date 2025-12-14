#include "include/motor_hardware_interface.h"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

MotorHardware::MotorHardware()
  : motor_controller_(nullptr)
{}

MotorHardware::~MotorHardware()
{
  if (motor_controller_) {
    // 禁用所有电机
    for (size_t i = 0; i < motor_ids_.size(); i++) {
      motor_controller_->set_motion_disable(motor_ids_[i]);
    }
    motor_controller_->close_can();
  }
}

CallbackReturn MotorHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  // 保存硬件信息
  info_ = info;

  // 初始化状态和命令向量
  joint_positions_.resize(info.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_velocities_.resize(info.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_efforts_.resize(info.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_commands_.resize(info.joints.size(), std::numeric_limits<double>::quiet_NaN());
  
  // 初始化电机ID列表和使能状态
  motor_ids_.resize(info.joints.size());
  motor_enabled_.resize(info.joints.size(), false);

  // 获取CAN接口配置
  auto can_interface_it = info.hardware_parameters.find("can_interface");
  if (can_interface_it != info.hardware_parameters.end()) {
    can_interface_ = can_interface_it->second;
  } else {
    RCLCPP_FATAL(rclcpp::get_logger("MotorHardware"), "Missing 'can_interface' parameter");
    return CallbackReturn::ERROR;
  }

  auto master_id_it = info.hardware_parameters.find("master_id");
  if (master_id_it != info.hardware_parameters.end()) {
    master_id_ = std::stoi(master_id_it->second);
  } else {
    RCLCPP_FATAL(rclcpp::get_logger("MotorHardware"), "Missing 'master_id' parameter");
    return CallbackReturn::ERROR;
  }

  // 检查电机配置
  for (size_t i = 0; i < info_.joints.size(); i++) {
    const hardware_interface::ComponentInfo & joint = info_.joints[i];
    
    // 检查是否为位置控制器
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(rclcpp::get_logger("MotorHardware"),
                   "Joint '%s' has %zu command interfaces found. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("MotorHardware"),
                   "Joint '%s' have %s command interfaces found. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    // 检查状态接口
    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(rclcpp::get_logger("MotorHardware"),
                   "Joint '%s' has %zu state interface. 3 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("MotorHardware"),
                   "Joint '%s' have '%s' as first state interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(rclcpp::get_logger("MotorHardware"),
                   "Joint '%s' have '%s' as second state interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
                   hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(rclcpp::get_logger("MotorHardware"),
                   "Joint '%s' have '%s' as third state interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[2].name.c_str(),
                   hardware_interface::HW_IF_EFFORT);
      return CallbackReturn::ERROR;
    }

    // 获取电机ID
    auto motor_id_it = joint.parameters.find("motor_id");
    if (motor_id_it != joint.parameters.end()) {
      uint8_t motor_id = std::stoi(motor_id_it->second);
      motor_ids_[i] = motor_id;
    } else {
      RCLCPP_FATAL(rclcpp::get_logger("MotorHardware"), "Missing 'motor_id' parameter for joint '%s'", joint.name.c_str());
      return CallbackReturn::ERROR;
    }
  }

  // 创建电机控制器实例
  motor_controller_ = std::make_unique<MotorController>(can_interface_, master_id_);

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MotorHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &joint_positions_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &joint_velocities_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name,
      hardware_interface::HW_IF_EFFORT,
      &joint_efforts_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MotorHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &joint_commands_[i]));
  }

  return command_interfaces;
}

CallbackReturn MotorHardware::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  // 初始化CAN总线
  if (motor_controller_->init_can() != 0) {
    RCLCPP_FATAL(rclcpp::get_logger("MotorHardware"), "Failed to initialize CAN bus");
    return CallbackReturn::ERROR;
  }

  // 初始化所有电机
  for (size_t i = 0; i < motor_ids_.size(); i++) {
    uint8_t motor_id = motor_ids_[i];
    
    // 设置运动模式
    if (motor_controller_->set_motion_mode(motor_id) != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("MotorHardware"), "Failed to set motion mode for motor %d", motor_id);
      return CallbackReturn::ERROR;
    }
    
    // 使能电机
    if (motor_controller_->set_motion_enable(motor_id) != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("MotorHardware"), "Failed to enable motor %d", motor_id);
      return CallbackReturn::ERROR;
    }
    
    motor_enabled_[i] = true;
    
    // 初始化位置
    joint_positions_[i] = 0.0;
    joint_velocities_[i] = 0.0;
    joint_efforts_[i] = 0.0;
    joint_commands_[i] = 0.0;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn MotorHardware::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  // 禁用所有电机
  for (size_t i = 0; i < motor_ids_.size(); i++) {
    if (motor_enabled_[i]) {
      motor_controller_->set_motion_disable(motor_ids_[i]);
      motor_enabled_[i] = false;
    }
  }
  
  // 关闭CAN总线
  motor_controller_->close_can();

  return CallbackReturn::SUCCESS;
}

return_type MotorHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // 这里需要实现从电机读取状态的逻辑
  // 由于当前的MotorController没有提供轮询读取电机状态的方法，
  // 我们需要根据实际情况实现
  
  // 暂时使用模拟数据进行测试
  for (size_t i = 0; i < joint_positions_.size(); i++) {
    // 模拟电机位置向命令值靠近
    joint_positions_[i] = joint_positions_[i] * 0.9 + joint_commands_[i] * 0.1;
    joint_velocities_[i] = (joint_positions_[i] - joint_positions_[i] * 0.9) * 10.0;
    joint_efforts_[i] = 0.0;
  }

  return return_type::OK;
}

return_type MotorHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // 向所有电机发送命令
  for (size_t i = 0; i < joint_commands_.size(); i++) {
    if (motor_enabled_[i]) {
      // 设置电机运动参数
      // 这里使用位置控制模式，扭矩设置为0，速度限制为5.0，KP和KD设置为适当的值
      RCLCPP_INFO(rclcpp::get_logger("MotorHardware"), "Setting motor %d to position %f", motor_ids_[i], joint_commands_[i]);
      if (motor_controller_->leg_set_motion_parameter(
            motor_ids_[i], 0.0, joint_commands_[i], 5.0, 50.0, 1.0) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("MotorHardware"), "Failed to set motion parameters for motor %d", motor_ids_[i]);
        return return_type::ERROR;
      }
      
    }
  }

  return return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(MotorHardware, hardware_interface::SystemInterface)
