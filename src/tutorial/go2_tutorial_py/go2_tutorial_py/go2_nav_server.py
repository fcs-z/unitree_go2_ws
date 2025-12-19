"""
导航服务端
需求:
    1.处理客户端请求
        提交的请求为浮点数（前进距离）
        如果大于0，那么控制机器人往前运动，否则认为数据不合法
    2.处理客户端取消请求
        当客户端发送取消请求，那么需要让机器人停止运动
    3.产生连续反馈和最终响应
        根据机器人当前坐标，结合起点坐标，结合前进距离计算剩余距离并周期性反馈
        当机器人到达目标点，响应机器人坐标
    实现:
        1.创建action服务端，执行相关操作；
        2.需要向πgo2ctrl注入参数控制机器人运动或停止；
        3.订阅机器人里程计获取坐标。
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from go2_tutorial_inter.action import Nav
from nav_msgs.msg import Odometry
from .sport_model import ROBOT_SPORT_API_IDS
from geometry_msgs.msg import Point
from unitree_api.msg import Request
import json

from rclpy.action.server import CancelResponse, GoalResponse, ServerGoalHandle
import time
import math
from rclpy.executors import MultiThreadedExecutor


class Go2NavServerPy(Node):

    def __init__(self):
        super().__init__('go2_nav_server_py')
        self.get_logger().info('Go2NavServerPy 创建')

        # Action Server
        self._action_server = ActionServer(self, Nav, 'nav',
            execute_callback=self.execute_cb, # 主逻辑的处理函数
            goal_callback=self.goal_cb,       # 
            cancel_callback=self.cancel_cb
        )

        # 订阅里程计
        self.current_point = Point()
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)

        # 设置巡航参数
        self.declare_parameter('x', 0.1)
        self.declare_parameter('error', 0.2)
        
        # 创建速度控制的发布对象
        self.api_id = ROBOT_SPORT_API_IDS['BALANCESTAND']
        self.req_pub = self.create_publisher(Request, '/api/sport/request', 10)
        self.timer = self.create_timer(0.1, self.on_timer) 

        # 位姿
        self.current_point = Point()
        self.start_point = Point()

    def execute_cb(self, goal_handle: ServerGoalHandle):
        self.get_logger().info('开始执行任务')
        
        feedback = Nav.Feedback()
        result = Nav.Result()
        self.start_point = self.current_point

        # 生成连续反馈
        while rclpy.ok():
            # 生成剩余距离 --- 获取当前坐标点和起点 --- 剩余距离 = 目标距离 - 已行进距离
            # 已行进距离
            dis_x = self.current_point.x - self.start_point.x
            dis_y = self.current_point.y - self.start_point.y
            dis = math.sqrt(math.pow(dis_x, 2) + math.pow(dis_y, 2))
            # 剩余距离 = 目标距离 - 已行进距离
            distance = goal_handle.request.goal - dis
            # 发布剩余距离
            feedback.distance = distance
            goal_handle.publish_feedback(feedback)

            # 处理取消
            if goal_handle.is_cancel_requested:
                self.get_logger().info('任务被取消')
                self.api_id = ROBOT_SPORT_API_IDS['STOPMOVE']
                result.point = self.current_point
                goal_handle.canceled()
                return result

            # 判断完成
            if distance <= self.get_parameter('error').value:
                self.get_logger().info('任务完成')
                self.api_id = ROBOT_SPORT_API_IDS['STOPMOVE']
                break

            time.sleep(0.5)
        
        # 生成最终结果
        goal_handle.succeed()
        result = Nav.Result()
        result.point = self.current_point
        return result
    # Action 回调
    def goal_cb(self, goal_request: Nav.Goal):
        goal_dis = goal_request.goal

        if goal_dis > 0.0:
            self.start_point = self.current_point
            self.get_logger().info(f'前进 {goal_dis:.2f} 米')
            self.api_id = ROBOT_SPORT_API_IDS['MOVE']
            return GoalResponse.ACCEPT
        else:
            self.get_logger().error(f'前进 {goal_dis:.2f} 米, 距离小于0, 非法数据!')
            self.api_id = ROBOT_SPORT_API_IDS['STOPMOVE']
            return GoalResponse.REJECT
    
    # 无条件取消
    def cancel_cb(self, cancel_request: CancelResponse):
        self.get_logger().info('取消请求')
        self.api_id = ROBOT_SPORT_API_IDS['STOPMOVE']
        return CancelResponse.ACCEPT

    def odom_cb(self, odom: Odometry):
        self.current_point = odom.pose.pose.position
    
    def on_timer(self):
        req = Request()
        # 设置数据
        req.header.identity.api_id = self.api_id
        # 设置参数
        js = {
            # "x": self.get_parameter('x').get_parameter_value().double_value,
            "x": self.get_parameter('x').value,     # 推荐
            "y": 0.0,
            "z": 0.0
        }
        req.parameter = json.dumps(js)
        self.req_pub.publish(req)

def main(args=None):
    rclpy.init(args=args)
    
    node = Go2NavServerPy()
    # rclpy.spin(node)
    # 使用多线程执行器运行节点
    executor = MultiThreadedExecutor()
    # 把节点添加进执行器
    executor.add_node(node)
    # 调用执行器的spin方法
    executor.spin()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
