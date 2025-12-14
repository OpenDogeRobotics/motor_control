import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取参数文件路径
    package_name = 'motor_control_interface'
    config_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'motor_controller_config.yaml'
    )
    
    # 获取URDF文件路径
    urdf_file = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        'motor_control.urdf'
    )
    
    # 读取URDF文件内容
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # 创建启动描述
    ld = LaunchDescription()
    
    # 添加控制器管理器节点
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            config_file
        ],
        output='screen'
    )
    
    # 添加位置控制器节点
    position_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['position_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # 添加关节状态发布器节点
    joint_state_publisher_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # 将节点添加到启动描述
    ld.add_action(controller_manager_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(position_controller_node)
    
    return ld
