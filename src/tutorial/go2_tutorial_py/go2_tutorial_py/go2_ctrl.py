"""
需求：以动态传参的方式控制 go2 运动
"""

import rclpy
from rclpy.node import Node
from unitree_api.msg import Request

import json
from .sport_model import ROBOT_SPORT_API_IDS


class Go2CtrlPy(Node):

    def __init__(self):
        super().__init__('go2_ctrl_py')
        self.get_logger().info('go2_ctrl_py 创建')

        # 声明参数
        self.declare_parameter('sport_api_id', ROBOT_SPORT_API_IDS["BALANCESTAND"])
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)

        self.req_pub = self.create_publisher(Request, '/api/sport/request', 10)
        self.timer = self.create_timer(0.1, self.on_timer)

    def on_timer(self):
        request = Request()

        # 读取参数
        api_id = self.get_parameter('sport_api_id').get_parameter_value().integer_value
        request.header.identity.api_id = api_id
        if api_id == ROBOT_SPORT_API_IDS["MOVE"]:
            js = {
                "x": self.get_parameter('x').get_parameter_value().double_value,
                "y": self.get_parameter('y').get_parameter_value().double_value,
                "z": self.get_parameter('z').get_parameter_value().double_value
            }
            request.parameter = json.dumps(js)

        self.req_pub.publish(request)


def main(args=None):
    rclpy.init(args=args)
    node = Go2CtrlPy()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
