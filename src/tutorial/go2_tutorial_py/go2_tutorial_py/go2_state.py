"""
需求：
订阅里程计消息，当机器人位移超出指定阀值，输出机器人此刻的坐标。

实现流程：
    1. 订阅里程计消息；
    2. 在回调函数中，计算当前机器人位置与上一个记录点的距离；
        如果距离大于阀值，就输出坐标，并更新记录点；
    3. 第一个记录点在第一次接收到里程计消息时生成。
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class Go2StatePy(Node):

    def __init__(self):
        super().__init__('go2_state_py')
        self.get_logger().info('Go2StatePy 创建')

        # 是否第一次接收到里程计
        self.is_first = True
        self.last_x = 0.0
        self.last_y = 0.0

        # 订阅里程计
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        
        # 声明参数：距离阀值
        self.declare_parameter('distance', 0.5)
        
    def odom_cb(self, odom: Odometry):
        # 获取当前坐标
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y

        # 第一次接收到消息，初始化记录点
        if self.is_first:
            self.last_x = x
            self.last_y = y
            self.is_first = False
            self.get_logger().info(f'起点坐标：({self.last_x:.2f}, {self.last_y:.2f})')
            return

        # 计算位移
        dis_x = x - self.last_x
        dis_y = y - self.last_y
        distance = math.sqrt(dis_x ** 2 + dis_y ** 2)

        # 获取阀值参数
        # threshold = self.get_parameter('distance').value  # 推荐
        threshold = self.get_parameter('distance').get_parameter_value().double_value
        # 判断是否超过阀值
        if distance > threshold:
            self.get_logger().info(f'坐标：({x:.2f}, {y:.2f})')
            # 更新记录点
            self.last_x = x
            self.last_y = y


def main(args=None):
    rclpy.init(args=args)
    node = Go2StatePy()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    