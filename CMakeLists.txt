cmake_minimum_required(VERSION 3.10)
project(motor_control_interface)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_manager REQUIRED)
find_package(ros2_control REQUIRED)

# 创建硬件接口库
add_library(motor_hardware_interface SHARED
  src/motor_hardware_interface.cpp
  src/motor_control_cpp.cpp
)

# 添加可执行文件
add_executable(motor_control_node src/main.cpp)
add_executable(test_motor_hardware src/test_motor_hardware.cpp)

# 包含头文件目录
target_include_directories(motor_control_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(test_motor_hardware PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(motor_hardware_interface PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# 链接依赖
target_link_libraries(motor_control_node pthread motor_hardware_interface)
target_link_libraries(test_motor_hardware pthread motor_hardware_interface)
target_link_libraries(motor_hardware_interface pthread)

ament_target_dependencies(motor_control_node
  rclcpp
  hardware_interface
)

ament_target_dependencies(test_motor_hardware
  rclcpp
  hardware_interface
)

ament_target_dependencies(motor_hardware_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  ros2_control
)

# 安装目标
install(TARGETS motor_control_node test_motor_hardware
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS motor_hardware_interface
  DESTINATION lib
)

# 注释掉头文件安装步骤，因为当前项目结构不使用单独的include目录
# install(DIRECTORY include/
#   DESTINATION include
# )

# 安装launch文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 安装配置文件
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 安装URDF文件
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# 导出硬件接口插件
pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)

ament_package()