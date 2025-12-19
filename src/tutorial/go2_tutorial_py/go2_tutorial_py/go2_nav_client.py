"""
导航客户端
需求：客户端要发送数据到服务端并处理服务端的响应结果。
    流程：
    1.判断程序执行时参数个数是否合法；
    2.初始化ROS2;
    3.创建action客户端对象；
    4.连接服务端；
    5.连接成功后，发送请求数据，并处理响应结果。连接失败，直接退出；
    6.调用spin函数，并传入节点对象指针。
    7.释放资源。
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from go2_tutorial_inter.action import Nav

from rclpy.action.client import ClientGoalHandle
import sys


class Go2NavClientPy(Node):

    def __init__(self):
        super().__init__('go2_nav_client_py')
        self.get_logger().info('Go2NavClient 创建')

        self.client_ = ActionClient(self, Nav, 'nav')

    def connect_server(self):
        while not self.client_.wait_for_server(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('等待服务端时被中断')
                return False
            self.get_logger().info('服务端连接中...')
        self.get_logger().info('成功连接到服务端!') 
        return True

    def send_request(self, x: float):
        # 组织目标数据
        goal_msg = Nav.Goal()
        goal_msg.goal = x
        # 发送数据，并处理响应
        future: rclpy.task.Future = self.client_.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        # 处理连续反馈
        feedback: Nav.Feedback = feedback_msg.feedback
        self.get_logger().info(f'距离目标点还有: {feedback.distance:.2f} 米')
    # 回调函数
    def goal_response_callback(self, future : rclpy.task.Future):
        # goal_handle: ClientGoalHandle = future.result()
        goal_handle: rclpy.action.client.ClientGoalHandle = future.result()

        if goal_handle.accepted:
            self.get_logger().info('目标请求被接受!')
            # 注册最终结果回调
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.result_callback)
        else:
            self.get_logger().error('目标值非法!')
            # rclpy.shutdown() # 此行注释，则与cpp执行一致，否则，输入负数时，会直接退出

    def result_callback(self, future: rclpy.task.Future):
        result: Nav.Result = future.result().result
        self.get_logger().info(f'机器人的停止坐标: ({result.point.x:.2f}, {result.point.y:.2f})')
        
        # code = result.status
        # if code == result.Status.STATUS_SUCCEEDED:
        #     point = result.result.point
        #     self.get_logger().info(f'机器人的停止坐标: ({point.x:.2f}, {point.y:.2f})')
        # elif code == result.Status.STATUS_CANCELED:
        #     self.get_logger().error('当前任务被取消!')
        # else:
        #     self.get_logger().error('未知异常!')
        # rclpy.shutdown()


def main(args=None):
    if len(sys.argv) != 2:
        print('请输入一个浮点类型的数据!')
        return

    rclpy.init(args=args)
    node = Go2NavClientPy()

    # 连接服务端
    if not node.connect_server():
        node.get_logger().error('连接服务端失败!')
        return

    # 发送请求
    node.send_request(float(sys.argv[1]))

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
