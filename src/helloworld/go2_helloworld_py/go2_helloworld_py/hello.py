import rclpy
from rclpy.node import Node
from unitree_api.msg import Request
 
class HelloWorld(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.pub_ = self.create_publisher(Request, '/api/sport/request', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("Hello World node has been started.")
    
    def timer_callback(self):
        request = Request()
        request.header.identity.api_id = 1016  # 打招呼的协议码
        self.pub_.publish(request)
        
def main():
    rclpy.init()
    node = HelloWorld("hello_world_py")
    rclpy.spin(node)
    rclpy.shutdown()