"""
需求：该节点启动时，需要实现三个主要功能。
    1.发布里程计消息。
    2.广播里程计相关坐标变换；
    3.发布关节状态信息。
  分析1：发布里程计消息。
    ros2 interface show nav_msgs/msg/Odometry
    ros2 interface show unitree_go/msg/SportModeState 
    1.先了解里程计消息的字段；
    2.这些数据从哪获取？机器人已经发布了相关话题了；
    3.实现上，可以先订阅状态话题，然后解析转换成里程计消息，最后发布即可。
  分析2：广播里程计相关坐标变换；
    ros2 interface show nav_msgs/msg/Odometry
    ros2 interface show geometry_msgs/msg/TransformStamped
    1.需要发布机器人基坐标系与odom坐标系的相对关系；
    2.这些相对关系与里程计数据类似；
    3.最后发布即可。
  分析3：发布关节状态信息。
    ros2 interface show sensor_msgs/msg/JointState
    ros2 interface show unitree_go/msg/LowState
    1.先了解关节状态信息；
    2.怎么获取这些数据？机器人已经发布了相关话题了；
    3.实现上，可以先订阅低层状态话题，然后解析转换成关节消息，最后发布即可。
"""

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState

from tf2_ros import TransformBroadcaster

from unitree_go.msg import SportModeState, LowState


class DriverPy(Node):
    def __init__(self):
        super().__init__('driver_py')
        self.get_logger().info('DriverPy 创建')

        # 参数
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base')
        self.declare_parameter('publish_tf', True)
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        # 里程计,创建SportModeState订阅对象，创建Odometry发布对象
        self.mode_sub = self.create_subscription(SportModeState, '/lf/sportmodestate', self.mode_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        # TF 广播
        self.tf_bro = TransformBroadcaster(self)
        # 关节状态,创建JointState发布对象，创建LowState订阅对象
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.low_state_sub = self.create_subscription(LowState, '/lf/lowstate', self.low_state_cb, 10)

    # 订阅 LowState → 发布 JointState
    def low_state_cb(self, low_state: LowState):
        # 获取并发布关节状态
        joint_state = JointState()
        # 组织数据
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = [
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"
        ]

        for i in range(12):
            motor = low_state.motor_state[i]
            joint_state.position.append(motor.q)

        self.joint_state_pub.publish(joint_state)

    # 订阅 SportModeState → 发布 Odometry + TF
    def mode_cb(self, mode: SportModeState):
        # 解析生成odom对象
        odom = Odometry()

        # -------- header --------
        # odom.header.stamp.sec = mode.stamp.sec
        # odom.header.stamp.nanosec = mode.stamp.nanosec
        odom.header.stamp = self.get_clock().now().to_msg() # 时间戳
        odom.header.frame_id = self.odom_frame              # 原点坐标系
        odom.child_frame_id = self.base_frame               # 机器人基坐标系
        # -------- pose --------
        # 位置
        odom.pose.pose.position.x = float(mode.position[0])
        odom.pose.pose.position.y = float(mode.position[1])
        odom.pose.pose.position.z = float(mode.position[2])
        # 姿态
        odom.pose.pose.orientation.w = float(mode.imu_state.quaternion[0])
        odom.pose.pose.orientation.x = float(mode.imu_state.quaternion[1])
        odom.pose.pose.orientation.y = float(mode.imu_state.quaternion[2])
        odom.pose.pose.orientation.z = float(mode.imu_state.quaternion[3])
        # -------- twist --------
        # 速度
        odom.twist.twist.linear.x = float(mode.velocity[0])
        odom.twist.twist.linear.y = float(mode.velocity[1])
        odom.twist.twist.linear.z = float(mode.velocity[2])
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = float(mode.yaw_speed)

        self.odom_pub.publish(odom)

        # -------- TF --------
        if not self.publisher_tf:
            return
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        # 设置偏移量
        transform.transform.translation.x = odom.pose.pose.position.x
        transform.transform.translation.y = odom.pose.pose.position.y
        transform.transform.translation.z = odom.pose.pose.position.z

        transform.transform.rotation = odom.pose.pose.orientation

        self.tf_bro.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = DriverPy()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
