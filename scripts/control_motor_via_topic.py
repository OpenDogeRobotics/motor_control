#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        # 创建发布者，发布到position_controller/commands话题
        self.publisher = self.create_publisher(
            Float64MultiArray, '/position_controller/commands', 10)
        
        self.get_logger().info('电机控制器节点已启动，使用话题 /position_controller/commands 控制电机')
    
    def set_position(self, position):
        """设置电机位置
        
        参数:
            position: 目标位置，单位为弧度
        """
        # 创建Float64MultiArray消息
        msg = Float64MultiArray()
        # 设置数据，由于只有一个关节，所以数组只有一个元素
        msg.data = [position]
        
        # 发布消息
        self.publisher.publish(msg)
        self.get_logger().info(f'已发送位置命令: {position} 弧度')

def main(args=None):
    # 初始化ROS 2
    rclpy.init(args=args)
    
    # 创建电机控制器节点
    controller = MotorController()
    
    try:
        # 演示如何控制电机
        positions = [0.0, 0.5, 1.0, 0.0, -0.5, -1.0, 3.14]
        
        for pos in positions:
            controller.set_position(pos)
            time.sleep(1.0)  # 等待1秒
    except KeyboardInterrupt:
        # 捕获键盘中断
        controller.get_logger().info('控制程序已终止')
    finally:
        # 关闭节点
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
