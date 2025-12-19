"""
需求：客户端向服务端发送整型数据，并接收服务端的响应结果。
流程：
    1. 判断提交的数据是否合法；
    2. ROS2 初始化；
    3. 创建自定义节点类对象；
    4. 连接服务端；
    5. 连接成功后向服务端发送数据；
    6. 处理响应结果；
    7. 资源释放。
"""

import rclpy
from rclpy.node import Node
from go2_tutorial_inter.srv import Cruising
import sys
from rclpy.task import Future

class Go2CruisingClientPy(Node):

    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.cru_client = self.create_client(Cruising, 'cruising')

    def connect_service(self) -> bool:
        """等待并连接服务端"""
        while not self.cru_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                # self.get_logger().error('等待服务时被中断')
                rclpy.logging.get_logger("rclpy").error('等待服务时被中断')
                return False
            self.get_logger().info('服务端连接中...')
        self.get_logger().info('成功连接到服务端!')
        return True

    def send_request(self, flag: int) -> Future: # 返回Future类型
        """发送请求"""
        req = Cruising.Request()
        req.flag = int(flag)
        return self.cru_client.call_async(req)


def main(args=None):
    # 1. 参数检查
    if len(sys.argv) != 2:
        print('参数不正确! 需要一个整数参数, 0表示停止巡航, 非0表示开始巡航')
        return

    # 2. ROS2 初始化
    rclpy.init(args=args)

    # 3. 创建节点
    node = Go2CruisingClientPy('go2_cruising_client_py')

    # 4. 连接服务端
    flag = node.connect_service()
    if not flag:
        # rclpy.shutdown()
        return

    # 5. 发送请求
    future = node.send_request(sys.argv[1])

    # 6. 等待并处理响应
    rclpy.spin_until_future_complete(node, future)

    if future.done():
        response : Cruising.Response = future.result()
        node.get_logger().info(f'响应成功! 机器人坐标: x={response.point.x:.2f}, y={response.point.y:.2f}, z={response.point.z:.2f}'
        )
    else:
        node.get_logger().error('响应失败!')

    # 7. 资源释放
    rclpy.shutdown()


if __name__ == '__main__':
    main()
