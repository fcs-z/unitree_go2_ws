/*
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
*/
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <unitree_go/msg/low_state.hpp>


class Driver: public rclcpp::Node{
public:
    Driver(const std::string &node_name):Node(node_name){
      RCLCPP_INFO(this->get_logger(), "Driver 创建");

      // 声明参数
      this->declare_parameter("odom_frame", "odom");
      this->declare_parameter<std::string>("base_frame", "base");
      this->declare_parameter("publish_tf", true);
      // 获取参数
      odom_frame = this->get_parameter("odom_frame").as_string();
      base_frame = this->get_parameter("base_frame").as_string();
      publish_tf = this->get_parameter("publish_tf").as_bool();

      // 里程计,订阅与发布
      mode_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
          "/lf/sportmodestate", 10, std::bind(&Driver::mode_cb, this, std::placeholders::_1)
      );
      odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

      // 坐标变换广播器
      tf_bro_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      // 关节状态,订阅与发布
      joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
      low_state_sub_ = this->create_subscription<unitree_go::msg::LowState>(
          "/lf/lowstate", 10, std::bind(&Driver::low_state_cb, this, std::placeholders::_1)
      );
    }

private:
    // 订阅底层状态,获取关节状态,组织消息并发布
    void low_state_cb(const unitree_go::msg::LowState::SharedPtr low_state){
      sensor_msgs::msg::JointState joint_state;
      // 组织数据
      joint_state.header.stamp = this->now();
      joint_state.name = {
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", 
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"
      }; // 12个关节名称
      // 遍历底层状态信息中的关节数据
      for(size_t i=0; i<12; i++){
        auto motor = low_state->motor_state[i]; // 获取某个关节的状态信息
        // 获取旋转角度,添加进关节状态消息
        joint_state.position.push_back(motor.q);
      }
      
      joint_state_pub_->publish(joint_state);
    }

    // 里程计,回调函数
    void mode_cb(const unitree_go::msg::SportModeState::SharedPtr mode){
        // RCLCPP_INFO(this->get_logger(), "收到go2状态消息，mode=%d", mode->mode);
        // 转换并发布里程计
        // nav_msgs/msg/Odometry --- unitree_go/msg/SportModeState
        nav_msgs::msg::Odometry odom;  // auto odom = nav_msgs::msg::Odometry();  auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();

        // 组织数据
        // 头
        // odom.header.stamp.sec = mode->stamp.sec;
        // odom.header.stamp.nanosec = mode->stamp.nanosec;
        odom.header.stamp = this->now();
        odom.header.frame_id = odom_frame; // 里程计坐标系
        odom.child_frame_id = base_frame;  // 机器人基坐标系
        // 位姿
        odom.pose.pose.position.x = mode->position[0];
        odom.pose.pose.position.y = mode->position[1];
        odom.pose.pose.position.z = mode->position[2];
        odom.pose.pose.orientation.w = mode->imu_state.quaternion[0];
        odom.pose.pose.orientation.x = mode->imu_state.quaternion[1];
        odom.pose.pose.orientation.y = mode->imu_state.quaternion[2];
        odom.pose.pose.orientation.z = mode->imu_state.quaternion[3];
        // 速度
        odom.twist.twist.linear.x = mode->velocity[0];
        odom.twist.twist.linear.y = mode->velocity[1];
        odom.twist.twist.linear.z = mode->velocity[2];
        odom.twist.twist.angular.x = 0;
        odom.twist.twist.angular.y = 0;
        odom.twist.twist.angular.z = mode->yaw_speed;

        odom_pub_->publish(odom);
        /////////////////////////////////////////////////////////////////////////
        // 是否发布坐标变换
        if(!publish_tf){
          return;
        }
        // 发布一次坐标变换
        // geometry_msgs/msg/TransformStamped --- nav_msgs/msg/Odometry
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = odom_frame; // 里程计坐标系
        transform.child_frame_id = base_frame;  // 机器人基坐标系
        // 设置偏移量
        transform.transform.translation.x = odom.pose.pose.position.x;
        transform.transform.translation.y = odom.pose.pose.position.y;
        transform.transform.translation.z = odom.pose.pose.position.z;
        // 左边：geometry_msgs::msg::Vector3   右边：geometry_msgs::msg::Point
        // 错误写法
        // transform.transform.translation = odom.pose.pose.position;  

        // 设置旋转角度
        transform.transform.rotation = odom.pose.pose.orientation;
        // 左边：geometry_msgs::msg::Quaternion  右边：geometry_msgs::msg::Quaternion
        
        tf_bro_ -> sendTransform(transform);
        /////////////////////////////////////////////////////////////////////////

    }

    // 订阅go2的状态,发布里程计
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr mode_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    // 动态调整参数
    std::string odom_frame, base_frame;
    // 广播坐标变换
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_bro_;
    // 是否发布坐标变换
    bool publish_tf;
    // 订阅低层状态,发布关节状态
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
};
 
int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Driver>("driver");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}