import rclpy
from rclpy.node import Node
from unitree_api.msg import Request
from geometry_msgs.msg import Twist

import json
from .sport_model import ROBOT_SPORT_API_IDS

class TwistBridgePy(Node):
    def __init__(self):
        super().__init__("twist_brige_py")
        self.get_logger().info("TwistBridgePy创建成功")
         # 1. 创建一个Request的发布对象
        self.request_pub = self.create_publisher(Request, '/api/sport/request', 10)

        # 2. 创建twist订阅对象
        self.twist_sub = self.create_subscription(Twist, 'cmd_vel', self.twist_cb, 10)

    def twist_cb(self, twist):
        # 3. 在回调函数中实现消息的转换以及发布
        request_msg = Request()

        # 获取 twist 消息的线速度和角速度
        x = twist.linear.x
        y = twist.linear.y
        z = twist.angular.z
        # 默认api_id 平衡站立
        api_id = ROBOT_SPORT_API_IDS["BALANCESTAND"]
        # 运动状态
        if x != 0 or y != 0 or z != 0:
            api_id = ROBOT_SPORT_API_IDS["MOVE"]
            # 设置参数 -- 字符串样式的速度指令
            js = {
                'x': x,
                'y': y,
                'z': z
            }
            parameter = json.dumps(js)
        request_msg.header.identity.api_id = api_id
        
        request_msg.parameter = parameter

        # 发布Request消息
        self.request_pub.publish(request_msg)
        self.get_logger().info('Published request message')

def main(args=None):
    rclpy.init(args=args)
    twist_bridge = TwistBridgePy()
    rclpy.spin(twist_bridge)
    rclpy.shutdown()

if __name__=='__main__':
   main()
