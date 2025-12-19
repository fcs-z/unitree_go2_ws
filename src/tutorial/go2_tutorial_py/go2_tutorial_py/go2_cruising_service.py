"""
需求： 处理客户端提交的数据(0 或 非0)
    - 0  ：停止巡航
    - 非0: 开始巡航
    无论提交什么数据，都需要响应机器人的位置信息
"""

import rclpy
from rclpy.node import Node
from go2_tutorial_inter.srv import Cruising
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from unitree_api.msg import Request
from .sport_model import ROBOT_SPORT_API_IDS
import json


class Go2CruisingServicePy(Node):

    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.get_logger().info('Go2CruisingServicePy 创建')

        # 创建服务端
        self.service = self.create_service(Cruising, 'cruising', self.cru_cb)
        
        # 订阅里程计
        self.current_point = Point()
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
         
        # 创建速度控制的发布对象
        self.api_id = ROBOT_SPORT_API_IDS['BALANCESTAND']
        self.req_pub = self.create_publisher(Request, '/api/sport/request', 10)
        self.timer = self.create_timer(0.1, self.on_timer) 

        # 本地参数
        self.declare_parameter('x', 0.1)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.5)

    # 服务回调函数
    def cru_cb(self, request: Cruising.Request, response: Cruising.Response):
        flag = request.flag

        # 判断巡航状态
        if flag != 0:
            api_id = ROBOT_SPORT_API_IDS['MOVE']
            self.get_logger().info('开始巡航...')
        else:
            api_id = ROBOT_SPORT_API_IDS['STOPMOVE']
            self.get_logger().info('停止巡航...')

        # 响应机器人的当前位置信息
        response.point = self.current_point
        return response

    # 里程计回调
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
            "y": self.get_parameter('y').value,
            "z": self.get_parameter('z').value
        }
        req.parameter = json.dumps(js)
        self.req_pub.publish(req)
    
def main(args=None):
    rclpy.init(args=args)

    node = Go2CruisingServicePy('go2_crusing_service_py')
    rclpy.spin(node)   
    rclpy.shutdown()


if __name__ == '__main__':
    main()
